#include "lejuclaw_controller.h"
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <iostream>

namespace eef_controller {

static bool CreateDirectoriesWithPermissions(const std::filesystem::path &dir_path) {
    try {
        if(std::filesystem::create_directories(dir_path)) {
            auto perms_rwx_xr_r = 
                std::filesystem::perms::owner_all  | 
                std::filesystem::perms::group_exec | std::filesystem::perms::group_read |
                std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加目录的可执行和读权限
            std::filesystem::permissions(dir_path, perms_rwx_xr_r, std::filesystem::perm_options::add);
            
            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[LejuClawController] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }

    return false;   
}

static bool CopyFileWithPermissions(const std::filesystem::path &src_path, 
                        const std::filesystem::path &dst_path, 
                        std::filesystem::copy_options opt = std::filesystem::copy_options::skip_existing) 
{
    try {
        if(std::filesystem::copy_file(src_path, dst_path, opt)) {
            auto perms_rw_rw_rw = 
                std::filesystem::perms::owner_write | std::filesystem::perms::owner_read |
                std::filesystem::perms::group_write | std::filesystem::perms::group_read |
                std::filesystem::perms::others_write | std::filesystem::perms::others_read;
            
            // 给其他用户组和其他人添加读写权限
            std::filesystem::permissions(dst_path, perms_rw_rw_rw, std::filesystem::perm_options::add);

            return true;
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        std::cerr << "[LejuClawController] CreateDirectoriesWithPermissions exception:" << ex.what() << '\n';
    }
    return false;  
}

std::string GetHomePath()
{
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user != nullptr) {
        passwd *pw = getpwnam(sudo_user);
        if (pw != nullptr){
            std::string path = pw->pw_dir;
            std::cout << "sudo user home path: " << path << std::endl;
            return path;
        }
    }
    else
    {
        uid_t uid = getuid();
        passwd *pw = getpwuid(uid);
        if (pw != nullptr){
            std::string path = pw->pw_dir;
            return path;
        }
    }
    return "/home/lab";
}

bool CopyOrUpdateLejuClawConfigFile(const std::string &src_file)
{   
    auto config_file_path = std::filesystem::path(GetHomePath()) / ".config/lejuconfig/config.yaml";
    if(!std::filesystem::exists(config_file_path)) {
        std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml not exist:" << config_file_path.string() << std::endl;
        /* config.yaml not exist, copy src_file to ~/.config/lejuconfig/config.yaml  */
        if(CreateDirectoriesWithPermissions(config_file_path.parent_path())) {
            return CopyFileWithPermissions(src_file, config_file_path);
        }
        else {
            std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile CreateDirectoriesWithPermissions failed" << std::endl;
            return false;
        }
    }

    /* config.yaml exist, then check params, */
    bool need_updated = false;
    try {
        std::cout << " [LejuClawController]  old config path: " << config_file_path << std::endl;
        std::cout << " [LejuClawController]  new config path: " << src_file << std::endl;

        YAML::Node old_config = YAML::LoadFile(config_file_path.string());
        YAML::Node new_config = YAML::LoadFile(src_file);

        std::vector<std::string> item_names{"address", "online", "parameter"};
        std::vector<std::string> claw_names{"Claw_joint_left", "Claw_joint_right"};

        for(auto &item_name : item_names) {
            if(!old_config[item_name]) {
                std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `" << item_name << "` not exist, updated it." << std::endl;
                old_config[item_name] = new_config[item_name];
                need_updated = true;
                continue;
            }

            for(auto &claw_name : claw_names) {
                if(!old_config[item_name][claw_name]) {
                    need_updated = true;
                    old_config[item_name][claw_name] = new_config[item_name][claw_name];
                    std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `" << claw_name << "` not exist, updated it." << std::endl;
                }
            }
        }

        /* checkouut ratio item */
        if (!old_config["ratio"]) {
            old_config["ratio"] = new_config["ratio"];
            need_updated = true;
            std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile config.yaml `ratio` not exist, updated it." << std::endl;
        }

        /* need to update config.yaml */
        if (need_updated) {
            /*  backup the config.yaml to config.yaml.for_lejuclaw.bak and update the config.yaml */
            auto bak_file_path = config_file_path.parent_path() / "config.yaml.for_lejuclaw.bak";
            std::cout << "[LejuClawController] CopyOrUpdateLejuClawConfigFile backup config.yaml to " << bak_file_path.string() << std::endl;
            if(!CopyFileWithPermissions(config_file_path, bak_file_path)) {
                std::cerr << "[LejuClawController] CopyOrUpdateLejuClawConfigFile backup config.yaml failed \n";
            }
            
            /* update the config.yaml */ 
            std::ofstream fout(config_file_path);
            fout << old_config;
        }
        return true;
    }
    catch (const YAML::Exception &e) {
        std::cout << "[LejuClawController] YAML Exception: " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cout <<  "[LejuClawController] Error: " <<  e.what()  << std::endl;
        return false;
    }
    
    return false;
}

ClawState GetClawState(const LejuClawControllerPtr &controller_ptr)
{
    ClawState state;
    if(controller_ptr) {
        auto pos = controller_ptr->get_positions();
        auto tor = controller_ptr->get_torque();
        auto vel = controller_ptr->get_velocity();
        auto claw_state = controller_ptr->get_state();
        
        state.data.name = {kLeftGripperName, kRightGripperName};
        state.data.position = pos.empty() ? std::vector<double>{0.0, 0.0} : pos;
        state.data.velocity = vel.empty() ? std::vector<double>{0.0, 0.0} : vel;
        state.data.effort = tor.empty() ? std::vector<double>{0.0, 0.0} : tor;
        state.state = {static_cast<int8_t>(claw_state[0]), static_cast<int8_t>(claw_state[1])};
    }
    return state;
}

bool LejuClawController::controlGripper(ControlClawRequest &req, ControlClawResponse &res)
{
    res.success = false;
   
    if (this->execute(req.data, res.message)) {
        res.success = true;
    }
    return res.success;
}

void  LejuClawController::command(const lejuClawCommand &msg)
{
    std::string err_msg;
    this->execute(msg, err_msg);
}

bool LejuClawController::execute(const lejuClawCommand &data, std::string &err_msg)
{
    // 更新VR检测
    update_vr_detection();
    
    // Check params
    if (data.position.empty()||data.name.empty() || data.position.size() != data.name.size()) {
        err_msg = "Invaild Data: position.size() and name.size() must be equal!";
        std::cout << err_msg << std::endl;
        return false;
    }

    // Check name
    int right_claw_idx = -1, left_claw_idx = -1;
    for (size_t i = 0; i < data.name.size(); i++) {
        if (data.name[i] != kRightGripperName && data.name[i] != kLeftGripperName) {
            err_msg = "Invaild Data: name must be `left_claw` or `right_claw`!";
            return false;
        }
        else {
            if (data.name[i] == kRightGripperName) {
                right_claw_idx = i;
            }
            else {
                left_claw_idx = i;
            }
        }
    }

    const double kDefaultTorque = 1.0;    // 1.0A
    const double kDefaultVelocity = 50;   // 50%
    std::vector<double> pos{0.0, 0.0}, vel{kDefaultVelocity, kDefaultVelocity}, tor{kDefaultTorque, kDefaultTorque};
    for(int i = 0; i<2; i++) {
        if(i == right_claw_idx) {
            pos[1] = data.position[i];
            if(data.velocity.size() > i) {
                vel[1] = data.velocity[i] == 0.0 ? kDefaultVelocity : data.velocity[i]; // if velocity is 0, use default velocity
            }
            if (data.effort.size() > i) {
                tor[1] = data.effort[i] == 0.0 ? kDefaultTorque : data.effort[i];
            }
        }
        else if(i == left_claw_idx){
            pos[0] = data.position[i];
            if(data.velocity.size() > i) {
                vel[0] = data.velocity[i] == 0.0 ? kDefaultVelocity : data.velocity[i]; // if velocity is 0, use default velocity
            }
            if (data.effort.size() > i) {
                tor[0] = data.effort[i] == 0.0 ? kDefaultTorque : data.effort[i];
            }
        }
    }

    // VR模式下允许覆盖正在执行的任务，非VR模式下夹爪正在执行则丢弃
    if (claw_is_executing_.load() && !is_vr_control_mode_) {
        err_msg = " claw is executing, please send again later.";
        return false;
    }
    
    // VR模式下强制覆盖当前任务
    if (is_vr_control_mode_ && claw_is_executing_.load()) {
        // std::cout << "[LejuClawController] VR模式：强制覆盖当前执行任务" << std::endl;
    }

    // add task
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        current_task_ = [this, positions = pos, velocity=vel, torque=tor](){
            this->move_paw(positions, velocity, torque);
        };
    }
    condition_.notify_all(); // 通知工作线程有新任务

    return true;
}


void LejuClawController::updateState(std::array<State, 2> new_state) {
    gripper_state_ = new_state;
}

LejuClawController::~LejuClawController(){
    this->close();
}

void LejuClawController::workerThread()
{
    while (!stop_worker_) {
        TaskFunc task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            condition_.wait(lock, [this] { return current_task_.has_value() || stop_worker_; });
            if (stop_worker_ && !current_task_.has_value()) {
                return;
            }

            // 取出当前任务并清空任务变量
            if (current_task_.has_value()) {
                task = std::move(*current_task_);
                current_task_.reset();
            }
        }

        // 执行任务
        if (task) {
            task();
        }
    }
}

bool LejuClawController::initialize(bool init_bmapilib)
{
    claw_is_executing_ = false;
    is_vr_control_mode_ = false;
    last_command_time_ = std::chrono::steady_clock::now();
    
    int ret = 1;
    if (is_can_protocol_) {
        claw_can_ptr_ = new lejuclaw_can::LeJuClawCan();
        ret = claw_can_ptr_->initialize();
    }
    else {
        claw_ptr_ = new LeJuClaw();
        ret = claw_ptr_->initialize(init_bmapilib);
    }
    if (ret != 0) {
        std::cout << "[LejuClawController]: claw init failed, retcode:" << ret << std::endl;
        return false;
    }

    worker_ = std::thread(&LejuClawController::workerThread, this);

    return true;
}

void LejuClawController::close()
{
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        stop_worker_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }

    if (is_can_protocol_) {
        if (claw_can_ptr_ != nullptr) {
            std::cout << "[LejuClawController]: claw close" << std::endl;
            claw_can_ptr_->close();
            delete claw_can_ptr_;
            claw_can_ptr_ = nullptr;
        }
    }
    else {
        if (claw_ptr_ != nullptr) {
            std::cout << "[LejuClawController]: claw close" << std::endl;
            claw_ptr_->close();
            delete claw_ptr_;
            claw_ptr_ = nullptr;
        }
    }
}

std::vector<double> LejuClawController::get_positions()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_positions();
    }
    else {
        return claw_ptr_->get_positions();
    }
}

std::vector<double> LejuClawController::get_torque()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_torque();
    }
    else {
        return claw_ptr_->get_torque();
    }
}

std::vector<double> LejuClawController::get_velocity()
{
    if (is_can_protocol_) {
        return claw_can_ptr_->get_velocity();
    }
    else {
        return claw_ptr_->get_velocity();
    }
}

std::array<LejuClawController::State, 2> LejuClawController::move_paw(const std::vector<double> &positions, 
const std::vector<double> &velocity,
const std::vector<double> &torque)
{
    // update moving state
    this->updateState({State::kMoving, State::kMoving});
    
    // mark claw is executing
    claw_is_executing_.store(true);


    // FIXME: 由于夹爪存在 bug , 这里需要将位置范围设为 5.0 ~ 95.0
    // qa: https://www.lejuhub.com/zhanglongbo/leju_claw_driver/-/issues/1
    auto temp_pos = positions;
    // temp_pos[0] = std::min(95.0, std::max(temp_pos[0], 5.0));
    // temp_pos[1] = std::min(95.0, std::max(temp_pos[1], 5.0));
    temp_pos[0] = std::min(100.0, std::max(temp_pos[0], 0.0));
    temp_pos[1] = std::min(100.0, std::max(temp_pos[1], 0.0));
    // 
    // std::cout << "[LejuClawController] 调用底层move_paw，VR模式: " << (is_vr_control_mode_ ? "是" : "否") << std::endl;
    LeJuClaw::PawMoveState ret_state;
    if (is_can_protocol_) {
        auto can_ret = claw_can_ptr_->move_paw(temp_pos, velocity, torque, is_vr_control_mode_);
        ret_state = static_cast<LeJuClaw::PawMoveState>(static_cast<int8_t>(can_ret));
    }
    else {
        ret_state = claw_ptr_->move_paw(temp_pos, velocity, torque, is_vr_control_mode_);
    }
    std::array<State, 2> new_state{State::kUnknown, State::kUnknown};

    static const std::map<LeJuClaw::PawMoveState, std::array<State, 2>> kStateMap = {
        {LeJuClaw::PawMoveState::ERROR, {State::kError, State::kError}},
        {LeJuClaw::PawMoveState::LEFT_GRABBED_RIGHT_GRABBED, {State::kGrabbed, State::kGrabbed}},
        {LeJuClaw::PawMoveState::LEFT_GRABBED_RIGHT_REACHED, {State::kGrabbed, State::kReached}},
        {LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_GRABBED, {State::kReached, State::kGrabbed}},
        {LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED, {State::kReached, State::kReached}}
        };

    if (kStateMap.find(ret_state) != kStateMap.end()) {
        new_state = kStateMap.at(ret_state);
    }

    // update state
    this->updateState(new_state);
    // mark claw is executing
    claw_is_executing_.store(false);

    return new_state;
}

void LejuClawController::update_vr_detection()
{
    auto current_time = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_command_time_).count();
    
    // VR检测逻辑：若时间间隔小于等于VR控制检测间隔，认为是VR控制模式
    if (time_diff <= VR_CONTROL_DETECTION_INTERVAL_MS) {
        is_vr_control_mode_ = true;
        // std::cout << "[LejuClawController] 检测到VR控制模式 (time_diff=" << time_diff << "ms <= " << VR_CONTROL_DETECTION_INTERVAL_MS << "ms)" << std::endl;
    } else {
        is_vr_control_mode_ = false;
        // std::cout << "[LejuClawController] 检测到非VR控制模式 (time_diff=" << time_diff << "ms > " << VR_CONTROL_DETECTION_INTERVAL_MS << "ms)" << std::endl;
    }
    
    // 更新最后命令时间
    last_command_time_ = current_time;
}

} // namespace eef_controller    