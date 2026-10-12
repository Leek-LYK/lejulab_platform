#include "Kalman.h"

Kalman::Kalman()
{
}

Kalman::Kalman(int state_dim, int measure_dim)
{
    state_dim_ = state_dim;
    measure_dim_ = measure_dim;

    x.resize(state_dim_, 1);

    Q.resize(state_dim_, state_dim_);
    R.resize(measure_dim_, measure_dim_);

    _F.resize(state_dim_, state_dim_);
    _P.resize(state_dim_, state_dim_);
    _H.resize(measure_dim_, state_dim_);
    _K.resize(state_dim_, measure_dim_);

    x.setZero();

    Q.setIdentity();
    R.setIdentity();

    _F.setIdentity();
    _P.setIdentity();
    _H.setIdentity();
    _K.setIdentity();
}

Kalman::Kalman(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &x_init, int measure_dim)
{
    state_dim_ = x_init.rows();
    measure_dim_ = measure_dim;

    x.resize(state_dim_, 1);

    Q.resize(state_dim_, state_dim_);
    R.resize(measure_dim_, measure_dim_);

    _F.resize(state_dim_, state_dim_);
    _P.resize(state_dim_, state_dim_);
    _H.resize(measure_dim_, state_dim_);
    _K.resize(state_dim_, measure_dim_);

    x = x_init;

    Q.setIdentity();
    R.setIdentity();

    _F.setIdentity();
    _P.setIdentity();
    _H.setIdentity();
    _K.setIdentity();
}
void Kalman::setQR(Eigen::VectorXd q, Eigen::VectorXd r)
{
    Q = Eigen::MatrixXd(q.asDiagonal());
    R = Eigen::MatrixXd(r.asDiagonal());
}
void Kalman::setQR(double *input_q, double *input_r)
{
    Eigen::VectorXd q(state_dim_);
    for (int i = 0; i < state_dim_; ++i)
        q(i) = input_q[i];
    Eigen::VectorXd r(measure_dim_);
    for (int i = 0; i < measure_dim_; ++i)
        r(i) = input_r[i];

    Q = Eigen::MatrixXd(q.asDiagonal());
    R = Eigen::MatrixXd(r.asDiagonal());
}

void Kalman::setState(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &x_init)
{
    x = x_init;
}

void Kalman::update_F_H(Eigen::MatrixXd &F, Eigen::MatrixXd &H)
{
    _F = F;
    _H = H;
}

Eigen::MatrixXd Kalman::updateData(Eigen::MatrixXd z)
{
    x = _F * x;
    _P = _F * _P * _F.transpose() + Q;

    auto _S = _H * _P * _H.transpose() + R;
    _K = _P * _H.transpose() * _S.inverse();

    x += _K * (z - _H * x);

    Eigen::MatrixXd I = _P;
    I.setIdentity();
    _P = (I - _K * _H) * _P;

    return x;
}

Eigen::MatrixXd Kalman::updateData(Eigen::MatrixXd x, Eigen::MatrixXd z)
{
    _P = _F * _P * _F.transpose() + Q;

    auto _S = _H * _P * _H.transpose() + R;
    _K = _P * _H.transpose() * _S.inverse();

    x += _K * (z - _H * x);

    Eigen::MatrixXd I = _P;
    I.setIdentity();
    _P = (I - _K * _H) * _P;

    return x;
}

Eigen::VectorXd Kalman::updateData_(const Eigen::MatrixXd &F, const Eigen::MatrixXd &H, Eigen::MatrixXd &P, const Eigen::VectorXd x, const Eigen::VectorXd z)
{
    P = F * P * F.transpose() + Q;
    auto _S = _H * P * H.transpose() + R;
    auto K = P * H.transpose() * _S.inverse();

    Eigen::VectorXd deltaX;
    deltaX = K * (z - H * x);

    Eigen::MatrixXd I = P;
    I.setIdentity();
    P = (I - K * H) * P;

    return deltaX;
}
