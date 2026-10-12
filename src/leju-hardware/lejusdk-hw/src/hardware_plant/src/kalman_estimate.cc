#include "kalman_estimate.h"

KalmanEstimate::KalmanEstimate(uint32_t na, Eigen::Vector3d b_a, Eigen::Vector3d imu_in_toros) : na_{na},
                                                                   b_a_{b_a},r_imu_in_toros(imu_in_toros)
{
    
    double com_Q[] = {1, 1, 1, 1, 1, 1};
    double com_R[] = {1, 1, 1, 1, 1, 1};
    com_kf = std::make_unique<Kalman>(6, 6);
    estimate_com_.resize(3);
    estimate_com_.setZero();

    double base_Q[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    double base_R[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    base_kf = std::make_unique<Kalman>(15, 15);
    setBaseQR(base_Q, base_R);

    double joint_Q[] = {1, 1, 1};
    double joint_R[] = {1, 1, 1};
    for (uint32_t i = 0; i < na_; i++)
    {
        joint_kf.push_back(std::make_unique<Kalman>(3, 3));
        joint_kf[i]->setQR(joint_Q, joint_R);
    }
    double hr_Q[] = {1e-3, 1e-3, 1};
    double hr_R[] = {1e2, 1e2, 1};
    hiproll_kf = std::make_unique<Kalman>(3, 3);
    hiproll_kf->setQR(hr_Q, hr_R);
    estimate_roll_.resize(3);
    estimate_roll_.setZero();

    estimate_base_.resize(15);
    estimate_base_.setZero();
    estimate_base_.segment(12, 3) = b_a_;
    predict_states.px = estimate_base_.segment(0,3);
    predict_states.pq = rpyToQuat(estimate_base_.segment(3, 3));
    predict_states.pv = estimate_base_.segment(6, 3);
    predict_states.pw = estimate_base_.segment(9,3);
    predict_states.b_a= estimate_base_.segment(12,3);

    update_state_.px = estimate_base_.segment(0, 3);
    update_state_.pq = rpyToQuat(estimate_base_.segment(3, 3));
    update_state_.pv = estimate_base_.segment(6, 3);
    update_state_.pw = estimate_base_.segment(9, 3);
    update_state_.b_a = estimate_base_.segment(12, 3);
    decoupled_P.resize(15, 15);
    decoupled_P.setIdentity();
}

KalmanEstimate::~KalmanEstimate()
{
}
void KalmanEstimate::setBaseQR(Eigen::VectorXd Q, Eigen::VectorXd R)
{
    base_kf->setQR(Q, R);
}
void KalmanEstimate::setBaseQR(double *Q, double *R)
{
    base_kf->setQR(Q, R);
}

void KalmanEstimate::setBaseState(Eigen::VectorXd &qv)
{
    Eigen::VectorXd measure(15);
    Eigen::Quaterniond quat(qv[0], qv[1], qv[2], qv[3]);
    Eigen::Vector3d eulerAngle = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    measure << qv.segment(4, 3), eulerAngle, qv.segment(7 + na_ + 3, 3), qv.segment(7 + na_, 3), estimate_base_.segment(12, 3);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x(measure.size(), 1);
    x = measure;
    estimate_base_ = x;
    base_kf->setState(x);
}

void KalmanEstimate::setBaseState_(Eigen::VectorXd &qv)
{
    Eigen::VectorXd measure(16);
    measure << qv.segment(4, 3), qv.segment(0, 4), qv.segment(7 + na_ + 3, 3), qv.segment(7 + na_, 3), update_state_.b_a;
    Eigen::Quaterniond quat(qv[0], qv[1], qv[2], qv[3]);

    Eigen::Vector3d eulerAngle = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x(measure.size(), 1);
    predict_states.px = qv.segment(4, 3);
    predict_states.pq = qv.segment(0, 4);
    predict_states.pv = qv.segment(7 + na_ + 3, 3);
    predict_states.pw = qv.segment(7 + na_, 3);
    predict_states.b_a= update_state_.b_a;
    update_state_ = predict_states;
    decoupled_P.setIdentity();
    x = measure;
    base_kf->setState(x);
}

void KalmanEstimate::setcomQR(double *Q, double *R)
{
    com_kf->setQR(Q, R);
}

void KalmanEstimate::setcomState(Eigen::VectorXd &r_vec)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x(r_vec.size(), 1);
    x = r_vec;
    estimate_com_ = x;
    com_kf->setState(x);
}

bool KalmanEstimate::setJointQR(uint8_t index, double *Q, double *R)
{
    if (index > joint_kf.size())
    {
        return false;
    }

    joint_kf[index]->setQR(Q, R);
    return true;
}

void KalmanEstimate::setJointState(Eigen::VectorXd &qv, Eigen::VectorXd &vdot,
                                   uint32_t nq, uint32_t nv,
                                   uint32_t nq_f, uint32_t nv_f)
{
    for (uint32_t i = 0; i < joint_kf.size(); i++)
    {
        Eigen::VectorXd measure(3);
        measure << qv[nq_f + i], qv[nq + nv_f + i], vdot[nv_f + i];
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x(measure.size(), 1);
        x = measure;
        joint_kf[i]->setState(x);
    }
}

Eigen::MatrixXd KalmanEstimate::getBaseF(Eigen::Vector3d gyro_w, Eigen::Vector3d acc, Eigen::Vector3d b_a, Eigen::Vector4d pq, double dt)
{
    Eigen::Matrix3d Identity3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d zero3d = Eigen::MatrixXd::Zero(3, 3);

    Eigen::Vector3d v = dt * gyro_w;
    Eigen::Matrix3d m1;
    m1 = skew(v);
    m1 = Identity3d + m1 * sin(v.norm()) + m1 * m1 * (1 - cos(v.norm()));

    Eigen::Matrix3d R_T = quaternionToRotationMatrix(pq).transpose();
    Eigen::Vector3d g = Eigen::Vector3d(0., 0., gravity_);
    Eigen::Vector3d v1 = R_T * (acc - b_a) - g;
    Eigen::Matrix3d m2;
    m2 = skew(v1);

    Eigen::MatrixXd F(15, 15);
    F << Identity3d, zero3d, dt * Identity3d, zero3d, zero3d,
        zero3d, Identity3d, zero3d, dt * Identity3d, zero3d,
        zero3d, zero3d, Identity3d, zero3d, -dt * R_T,
        zero3d, zero3d, zero3d, Identity3d, zero3d,
        zero3d, zero3d, zero3d, zero3d, Identity3d;

    return F;
}

Decoupled_states KalmanEstimate::baseFiltering(Eigen::VectorXd &qv, Eigen::Vector3d &acc, double dt)
{
    
    Eigen::Quaterniond quat(qv[0], qv[1], qv[2], qv[3]);
    Vec3 rpy;
    Vec4 quat_;
    quat_<< qv[0], qv[1], qv[2], qv[3];
    rpy = quatToRPY(quat_);
    Eigen::Matrix3d R_T = quat.toRotationMatrix();
    Eigen::Vector3d g = Eigen::Vector3d(0., 0., gravity_);
    static Eigen::Vector3d gyro_w_pre;
    Eigen::Vector3d gyro_w = qv.segment(7 + na_, 3);
    predict_states.px = update_state_.px + update_state_.pv * dt;
    Eigen::Matrix4d G_quatrate;
    G_quatrate = quat_rate(update_state_.pw);
    Vec4 delta_pq;
    delta_pq = 0.5 * G_quatrate * update_state_.pq * dt;
    predict_states.pq = update_state_.pq + delta_pq;
    predict_states.pv = update_state_.pv + (R_T * (acc - update_state_.b_a) - g) * dt;
    predict_states.pw = update_state_.pw + gyro_w - gyro_w_pre;
    predict_states.b_a = update_state_.b_a;
    gyro_w_pre = gyro_w;
    Vec3 rpy_predict;
    rpy_predict = quatToRPY(predict_states.pq);
    Eigen::VectorXd measure(15);
    measure << qv.segment(4, 3), rpy, qv.segment(7 + na_ + 3, 3), qv.segment(7 + na_, 3), update_state_.b_a;
    Eigen::MatrixXd F = getBaseF(update_state_.pw, acc, update_state_.b_a, update_state_.pq, dt);
    Eigen::MatrixXd H(15, 15);
    H.setIdentity();

    Eigen::VectorXd x_(15);
    x_ << predict_states.px, rpy_predict, predict_states.pv, predict_states.pw, predict_states.b_a;
    Eigen::VectorXd deltaX = base_kf->updateData_(F ,H, decoupled_P, x_, measure);
    Vec3 deltaphi;
    deltaphi = deltaX.segment(3,3);
    Vec4 deltapq = QuaterniondExp(deltaphi);
    update_state_.pq = quat_;
    update_state_.px = x_.segment(0,3) + deltaX.segment(0,3);
    update_state_.pw = x_.segment(9,3) + deltaX.segment(9,3);
    update_state_.pv = x_.segment(6,3) + deltaX.segment(6,3);
    update_state_.b_a = x_.segment(12,3) + deltaX.segment(12,3);

    qv.segment(0, 4) = update_state_.pq;
    qv.segment(4, 3) = update_state_.px;
    qv.segment(7 + na_, 3) = update_state_.pw;
    qv.segment(7 + na_ + 3, 3) = update_state_.pv - update_state_.pw.cross(r_imu_in_toros);
    Vec3 rpy_update = quatToRPY(update_state_.pq);

    return update_state_;
}

void KalmanEstimate::jointFiltering(Eigen::VectorXd &qv, Eigen::VectorXd &vdot, double dt)
{
    Eigen::VectorXd measure(3);
    Eigen::VectorXd estimate(3);
    Eigen::MatrixXd F(3, 3);
    F << 1.0, dt, dt * dt / 2.0,
        0.0, 1.0, dt,
        0.0, 0.0, 1.0;
    Eigen::MatrixXd H = Eigen::Matrix3d::Identity();

    for (uint32_t i = 0; i < na_; i++)
    {
        measure << qv[7 + i], qv[7 + na_ + 6 + i], vdot[6 + i];
        joint_kf[i]->update_F_H(F, H);
        estimate = joint_kf[i]->updateData(measure);
        qv[7 + i] = estimate[0];
        qv[7 + na_ + 6 + i] = estimate[1];
        vdot[6 + i] = estimate[2];
    }
}

Eigen::MatrixXd KalmanEstimate::getBaseF2(Eigen::Vector3d gyro_w, Eigen::Vector3d acc, Eigen::Vector3d b_a, Eigen::Quaterniond quat, double dt)
{
    Eigen::Matrix3d Identity3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d zero3d = Eigen::MatrixXd::Zero(3, 3);

    Eigen::Vector3d v = dt * gyro_w;
    Eigen::Matrix3d m1;
    m1 << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    m1 = Identity3d + m1 * sin(v.norm()) + m1 * m1 * (1 - cos(v.norm()));

    Eigen::Matrix3d R_T = quat.toRotationMatrix();
    Eigen::Vector3d g = Eigen::Vector3d(0., 0., gravity_);
    Eigen::Vector3d v1 = R_T * (acc - b_a) - g;
    Eigen::Matrix3d m2;
    m2 << 0, -v1(2), v1(1),
        v1(2), 0, -v1(0),
        -v1(1), v1(0), 0;

    Eigen::MatrixXd F(15, 15);
    F << Identity3d, zero3d, dt * Identity3d, zero3d, zero3d,
        zero3d, Identity3d, zero3d, dt * Identity3d, zero3d,
        zero3d, zero3d, Identity3d, zero3d, -dt * R_T,
        zero3d, zero3d, zero3d, Identity3d, zero3d,
        zero3d, zero3d, zero3d, zero3d, Identity3d;

    return F;
}

void KalmanEstimate::baseFiltering2(Eigen::VectorXd &qv, Eigen::Vector3d &acc, double dt)
{

    

    Eigen::Quaterniond quat(qv[0], qv[1], qv[2], qv[3]);
    Eigen::Vector3d eulerAngle = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    eulerAngle(0) = eulerAngle(0) > M_PI / 2 ? eulerAngle(0) - M_PI : (eulerAngle(0) < -M_PI / 2 ? eulerAngle(0) + M_PI : eulerAngle(0));
    eulerAngle(1) = eulerAngle(1) > M_PI / 2 ? M_PI - eulerAngle(1) : (eulerAngle(1) < -M_PI / 2 ? -M_PI - eulerAngle(1) : eulerAngle(1));
    eulerAngle(2) = 0;
    Eigen::Matrix3d R_T = quat.toRotationMatrix();
    Eigen::Vector3d g = Eigen::Vector3d(0., 0., gravity_);

    Eigen::Vector3d px, pq, pv, pw, b_a;
    px = estimate_base_.segment(0, 3);
    pq = estimate_base_.segment(3, 3);
    pv = estimate_base_.segment(6, 3);
    pw = estimate_base_.segment(9, 3);
    b_a = estimate_base_.segment(12, 3);

    Eigen::VectorXd measure(15);
    Eigen::Vector3d mx, mq, mv, mw, mb_a;
    mx = qv.segment(4, 3);
    mq = eulerAngle;
    mw = qv.segment(7 + na_, 3);
    mv = qv.segment(7 + na_ + 3, 3) + mw.cross(r_imu_in_toros);
    mb_a = estimate_base_.segment(12, 3);
    measure << qv.segment(4, 3), eulerAngle, qv.segment(7 + na_ + 3, 3), qv.segment(7 + na_, 3), estimate_base_.segment(12, 3);

    Eigen::MatrixXd F = getBaseF2(measure.segment(9, 3), acc, estimate_base_.segment(12, 3), quat, dt);
    Eigen::MatrixXd H(15, 15);
    H.setIdentity();
    base_kf->update_F_H(F, H);

    static Eigen::Vector3d gyro_w_pre;
    Eigen::Vector3d gyro_w = qv.segment(7 + na_, 3);
    Eigen::VectorXd predict(15);
    predict << px + pv * dt,
        pq + pw * dt,
        pv + (R_T * (acc - b_a) - g) * dt,
        pw + gyro_w - gyro_w_pre,
        b_a;
    gyro_w_pre = gyro_w;

    estimate_base_ = base_kf->updateData(predict, measure);

    // std::cout<<"b_a "<<b_a.transpose()<<std::endl;
    // std::cout<<"_K "<<base_kf->_K<<std::endl;
    // std::cout<<std::endl;

    // eulerAngle = estimate_base_.segment(3, 3);
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
    // quat = yawAngle * pitchAngle * rollAngle;
    // qv.segment(0, 4) = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());

    Eigen::Vector3d est_x, est_q, est_v, est_w;
    est_x = estimate_base_.segment(0, 3);
    est_q = estimate_base_.segment(3, 3);
    est_v = estimate_base_.segment(6, 3);
    est_w = estimate_base_.segment(9, 3);

    qv.segment(4, 3) = est_x;
    qv.segment(7 + na_, 3) = est_w;
    qv.segment(7 + na_ + 3, 3) = est_v - est_w.cross(r_imu_in_toros);
}

void KalmanEstimate::comFiltering(Eigen::VectorXd &r_vec_est, Eigen::VectorXd &r_vec, Eigen::Vector3d &p_st, double dt)
{
    Eigen::Matrix3d Identity3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d zero3d = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd F(6, 6);
    F << Identity3d, dt * Identity3d,
        zero3d, Identity3d;
    Eigen::MatrixXd H(6, 6);
    H.setIdentity();
    com_kf->update_F_H(F, H);

    static Eigen::Vector3d rd_pre;
    Eigen::Vector3d r, rd, rdd;
    r = estimate_com_.segment(0, 3);
    rd = estimate_com_.segment(3, 3);
    rdd = (r - p_st) * gravity_ / (r_vec[2] - p_st[2]);
    rdd[2] = 0; // no z dir predict

    Eigen::VectorXd predict(6);
    predict << r + rd * dt,
        rd + rdd * dt;
    rd_pre = r_vec.segment(3, 3);
    estimate_com_ = com_kf->updateData(predict, r_vec);
    r_vec_est = estimate_com_;
}

void KalmanEstimate::HipRollFiltering(Eigen::VectorXd &mhiproll, Eigen::VectorXd &hiproll, double v_torso, double v_foot, double l, double dt)
{
    Eigen::Matrix3d I3d = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d zero3d = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd F(3, 3);
    F << 1, dt, 0,
        0, 1, 0,
        0, 0, 1;
    Eigen::MatrixXd H(3, 3);
    H.setIdentity();
    hiproll_kf->update_F_H(F, H);

    Eigen::VectorXd predict(3);
    double p_hrv = -(v_torso - v_foot) / l;
    double p_hr = estimate_roll_(0) + p_hrv * dt;
    predict << p_hr, p_hrv, 0;
    Eigen::VectorXd meas(3);
    meas = mhiproll;

    estimate_roll_ = hiproll_kf->updateData(predict, meas);
    hiproll = estimate_roll_;
}
