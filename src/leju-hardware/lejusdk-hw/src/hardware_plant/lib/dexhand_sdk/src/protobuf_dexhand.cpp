#include "protobuf_dexhand.h"

#include <unistd.h>
#include <malloc.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <memory>
#include <iostream>
#include <unordered_map>

#include "stark_wrapper.h"
#include "libserialport.h"

#define PRINT_SEND_DATA 0
#define PRINT_RECEIVE_DATA 0

class ProtobufDexhandImpl;
class SerialPort;
using namespace dexhand;
/********************************************Helper functions******************************************************** */

std::ostream& operator<<(std::ostream& os, const StarkFingerStatus& status) {
    os << "StarkFingerStatus: ";
    os << "\n\tPositions: ";
    for(int i = 0; i < 6; i++) {
        os << static_cast<int>(status.finger_positions[i]) << " ";
    }   
    os << "\n\tSpeeds:";
    for (int i = 0; i < 6; i++) {
        os << static_cast<int>(status.finger_speeds[i]) << " ";
    }
    os << "\n\tCurrents:";
    for (int i = 0; i < 6; i++) {
        os << static_cast<int>(status.finger_currents[i]) << " ";
    }
    os << "\n\tStates:";
    for (int i = 0; i < 6; i++) {
        os << static_cast<int>(status.finger_states[i]) << " ";
    }
    os << "\n";
    return os;
}
/**************************************** Stark SDK Callback functions ********************************************/
// 这里声明 stark getter 函数的回调
 

void _on_stark_log(int level, const char *msg);
void _on_error(const char *device_id, int error);
void _on_motorboard_info(const char *device_id, MotorboardInfo *info);
void _on_force_level(const char *device_id, int force_level);
void _on_turbo_mode(const char *device_id, int turbo_mode);
void _on_turbo_config(const char *device_id, int turbo_interval, int turbo_duration);
void _on_finger_status(const char *device_id, FingerStatusData *finger_status);
void receive_data(ProtobufDexhandImpl* impl);

/*********************************************************************************************************************/
// 使用 map 管理 device_id 和 ProtobufDexhandImpl 的映射关系
std::unordered_map<std::string, ProtobufDexhandImpl*> g_device_map;
std::mutex g_device_map_mutex;

// 全局 StarkWrapper 实例
static StarkWrapper* g_stark_wrapper = nullptr;

ProtobufDexhandImpl* get_impl_by_deviceid(const char* device_id) {
    if (device_id == NULL) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_device_map_mutex);
    if (g_device_map.find(device_id) == g_device_map.end()) {
        return nullptr;
    }
    
    return g_device_map[device_id];
}

void add_impl_by_deviceid(const char* device_id, ProtobufDexhandImpl* device) {
    if (device_id == NULL) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_device_map_mutex);
    g_device_map[device_id] = device;
}

void remove_impl_by_deviceid(const char* device_id) {
    std::lock_guard<std::mutex> lock(g_device_map_mutex);
    g_device_map.erase(device_id);
}

/*********************************************************************************************************************/
class SerialPort {
public:
    SerialPort(const std::string& serial_port, int baudrate): 
    port_(serial_port), baudrate_(baudrate), sp_port_(nullptr), rx_event_set_(nullptr) {
    }

    /**
     * @brief Open serial port.
     * 
     * @return bool true: success, false: failed.
     */
    bool open_serial() {
        bool open_success = false;
        sp_return sp_ret = SP_ERR_FAIL;
        /* Open and configure port. */
        do {
            #define CHECK_SP_OPT(sp_ret, func) \
                sp_ret = func; \
                if (sp_ret != SP_OK) { \
                    break; \
                }
            /* Open and configure port. */
            CHECK_SP_OPT(sp_ret, sp_get_port_by_name(port_.c_str(), &sp_port_));
            CHECK_SP_OPT(sp_ret, sp_open(sp_port_, SP_MODE_READ_WRITE));
            /* Set baudrate, bits, parity, stopbits, flowcontrol */
            CHECK_SP_OPT(sp_ret, sp_set_baudrate(sp_port_, baudrate_));
            CHECK_SP_OPT(sp_ret, sp_set_bits(sp_port_, 8));
            CHECK_SP_OPT(sp_ret, sp_set_parity(sp_port_, SP_PARITY_NONE));
            CHECK_SP_OPT(sp_ret, sp_set_stopbits(sp_port_, 1));
            CHECK_SP_OPT(sp_ret, sp_set_flowcontrol(sp_port_, SP_FLOWCONTROL_NONE));
            /* Adding port RX event to event set*/
            CHECK_SP_OPT(sp_ret, sp_new_event_set(&rx_event_set_));
            CHECK_SP_OPT(sp_ret, sp_add_port_events(rx_event_set_, sp_port_, SP_EVENT_RX_READY));
            #undef CHECK_SP_OPT
            open_success = true;
            sp_ret = SP_OK;
        } while (false);

        if (!open_success) {
            char *err_msg = sp_last_error_message();
            printf("\033[31mError: Open serial port: [%s], baudrate:[%d], failed: %s, sp_ret=%d\033[0m\n", port_.c_str(), baudrate_, err_msg, sp_ret);
            sp_free_error_message(err_msg);
            close_serial();
        }

        return open_success;
    }

    /**
     * @brief Close serial port.
     */
    void close_serial() {
        if(rx_event_set_) {
            sp_free_event_set(rx_event_set_);
        }
        if(sp_port_) {
            sp_close(sp_port_);
            sp_free_port(sp_port_);
        }

        sp_port_ = nullptr;
        rx_event_set_ = nullptr;
    }
    ~SerialPort() {
        close_serial();
    }

    const std::string& port() const {
        return port_;
    }
    int baudrate() const {
        return baudrate_;
    }
    sp_port* get_sp_port() {
        return sp_port_;
    }
    sp_event_set *get_rx_event_set() {
        return rx_event_set_;
    }
private:
    std::string port_;  // serial port name "stark_serial_R"
    int baudrate_;      // 115200
    sp_port* sp_port_;  // serial port handle
    sp_event_set *rx_event_set_; // serial port event set
};

class ProtobufDexhandImpl {
public:
    ProtobufDexhandImpl(std::shared_ptr<SerialPort> serial_port, const std::string& uuid, int slave_id_):
    serial_port_(serial_port),  uuid_(uuid), slave_id_(slave_id_) {
        // 使用 wrapper 创建设备和设置回调
        if (g_stark_wrapper && g_stark_wrapper->create_serial_device) {
            handle_ = g_stark_wrapper->create_serial_device(uuid.c_str(), slave_id_);
            if (handle_ && g_stark_wrapper->set_error_callback) {
                g_stark_wrapper->set_error_callback(handle_, _on_error);
            }
        }
        raw_motorboard_info_ = MotorboardInfo();
        raw_motorboard_info_.hand_type = static_cast<int>(DexHandType::SKU_TYPE_NONE);
        raw_motorboard_info_.sn[0] = '\0';
        raw_motorboard_info_.fw_version[0] = '\0';
    }

    ~ProtobufDexhandImpl() {
        handle_ = nullptr;
    }
    
    const std::string& device_id() const {return uuid_;}
    std::shared_ptr<SerialPort> serial_port() {return serial_port_;}
    StarkDevice* handle() {return handle_;}
    /****************************************************************************
     * Callbacks: 添加回调函数在这里
     * ***************************************************************************/
    void on_motorboard_info(MotorboardInfo *info) {
        if (info) {
            raw_motorboard_info_ = *info;
        }
        is_motorboard_info_received_ = true;
    }

    void on_force_level(int force_level) {
        raw_force_level_ = static_cast<StarkForceLevel>(force_level);
        is_force_level_received_ = true;
    }

    void on_finger_status(FingerStatusData *finger_status) {
        if (finger_status->n_finger_status > 0) {
                if (finger_status->finger_status && finger_status->finger_status[0]) {
                    // update raw_finger_status_
                    raw_finger_status_ = *(finger_status->finger_status[0]);
                if (PRINT_RECEIVE_DATA) {
                    std::cout <<"[cb on_finger_status]: device_id: " << uuid_ << raw_finger_status_ << std::endl;
                }
            }
        }
        is_finger_status_received_ = true;
    }

    void on_turbo_mode(int turbo_mode) {
        raw_turbo_mode_ = turbo_mode;
        is_turbo_mode_received_ = true;
    }

    void on_turbo_config(int turbo_interval, int turbo_duration) {
        raw_turbo_config_.interval = turbo_interval;
        raw_turbo_config_.duration = turbo_duration;
        is_turbo_config_received_ = true;
    }
    /****************************************************************************
     * Getters
     * ***************************************************************************/
    bool getDeviceInfo(DeviceInfo_t& info) {
        info.sku_type = DexHandType::SKU_TYPE_NONE;
        info.serial_number = "";
        info.firmware_version = "";
        bool success = false;

        try {
            if (g_stark_wrapper && g_stark_wrapper->get_motorboard_info) {
                g_stark_wrapper->get_motorboard_info(handle_, _on_motorboard_info);
                wait_for_trigger_callback(is_motorboard_info_received_);

                info.sku_type = static_cast<DexHandType>(raw_motorboard_info_.hand_type);
                info.serial_number = raw_motorboard_info_.sn;
                info.firmware_version = raw_motorboard_info_.fw_version;
                success = true;
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] getDeviceInfo exception: " << e.what() << std::endl;
        }
        return success;
    }

    bool getFingerStatus(FingerStatus& status) {
        status.clear(); // Clear data before getting new values
        bool success = false;
        try {
            if (g_stark_wrapper && g_stark_wrapper->get_finger_status) {
                g_stark_wrapper->get_finger_status(handle_, _on_finger_status);
                wait_for_trigger_callback(is_finger_status_received_);

                std::copy(std::begin(raw_finger_status_.finger_positions), std::end(raw_finger_status_.finger_positions), std::begin(status.positions));
                std::copy(std::begin(raw_finger_status_.finger_speeds), std::end(raw_finger_status_.finger_speeds), std::begin(status.speeds));
                std::copy(std::begin(raw_finger_status_.finger_currents), std::end(raw_finger_status_.finger_currents), std::begin(status.currents));
                std::copy(std::begin(raw_finger_status_.finger_states), std::end(raw_finger_status_.finger_states), std::begin(status.states));

                for (int i = 0; i < 6; i++) {
                    status.positions[i] = std::max<uint16_t>(0, std::min<uint16_t>(100, status.positions[i]));
                    status.speeds[i] = std::max<int16_t>(-100, std::min<int16_t>(100, status.speeds[i]));
                    status.currents[i] = std::max<int16_t>(-100, std::min<int16_t>(100, status.currents[i]));
                    status.states[i] = std::max<uint16_t>(0, std::min<uint16_t>(100, status.states[i]));
                }
                success = true;
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] getFingerStatus exception: " << e.what() << std::endl;
        }
        return success;
    }

    GripForce getGripForce() {
        GripForce force_level = GripForce::FORCE_LEVEL_NORMAL;
        try {
            if (g_stark_wrapper && g_stark_wrapper->get_force_level) {
                g_stark_wrapper->get_force_level(handle_, _on_force_level);
                wait_for_trigger_callback(is_force_level_received_);
                force_level = static_cast<GripForce>(raw_force_level_);
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] getGripForce exception: " << e.what() << std::endl;
        }
        return force_level;
    }

    TurboConfig_t getTurboConfig() {
        TurboConfig_t config = {0, 0};
        try {
            if (g_stark_wrapper && g_stark_wrapper->get_turbo_conf) {
                g_stark_wrapper->get_turbo_conf(handle_, _on_turbo_config);
                wait_for_trigger_callback(is_turbo_config_received_);
                config = raw_turbo_config_;
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] getTurboConfig exception: " << e.what() << std::endl;
        }
        return config;
    }

    bool getTurboModeEnabled() {
        bool enabled = false;
        try {
            if (g_stark_wrapper && g_stark_wrapper->get_turbo_mode) {
                g_stark_wrapper->get_turbo_mode(handle_, _on_turbo_mode);
                wait_for_trigger_callback(is_turbo_mode_received_);
                enabled = raw_turbo_mode_ == 1;
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] getTurboModeEnabled exception: " << e.what() << std::endl;
        }
        return enabled;
    }
    /****************************************************************************
     * Setters
     * ***************************************************************************/
    void setFingerPositions(const UnsignedFingerArray &positions) {
        int finger_positions[6] = {};
        for (int i = 0; i < 6; i++) {
            finger_positions[i] = std::max<int>(0, std::min<int>(100, positions[i]));
        }
        try {
            if (g_stark_wrapper && g_stark_wrapper->set_finger_positions) {
                g_stark_wrapper->set_finger_positions(handle_, finger_positions);
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] setFingerPositions exception: " << e.what() << std::endl;
        }
    }

    void setFingerSpeeds(const FingerArray &speeds) {
        int finger_speeds[6] = {};
        for (int i = 0; i < 6; i++) {
            finger_speeds[i] = std::max<int>(-100, std::min<int>(100, speeds[i]));
        }
        try {
            if (g_stark_wrapper && g_stark_wrapper->set_finger_speeds) {
                g_stark_wrapper->set_finger_speeds(handle_, finger_speeds);
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] setFingerSpeeds exception: " << e.what() << std::endl;
        }
    }

    void setGripForce(GripForce level) {
        try {
            if (g_stark_wrapper && g_stark_wrapper->set_force_level) {
                g_stark_wrapper->set_force_level(handle_, static_cast<int>(level));
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] setGripForce exception: " << e.what() << std::endl;
        }
    }

    void setTurboModeEnabled(bool enabled) {
        try {
            if (g_stark_wrapper && g_stark_wrapper->set_turbo_mode) {
                g_stark_wrapper->set_turbo_mode(handle_, enabled ? 1 : 0);
                raw_turbo_mode_ = enabled ? 1 : 0;
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] setTurboModeEnabled exception: " << e.what() << std::endl;
        }
    }

    void runActionSequence(ActionSequenceId_t seq_id) {
        try {
            if (g_stark_wrapper && g_stark_wrapper->run_action_sequence) {
                g_stark_wrapper->run_action_sequence(handle_, seq_id);
            }
        }catch(std::exception &e) {
            std::cerr << "[ProtobufDexhand] runActionSequence exception: " << e.what() << std::endl;
        }
    }

    bool setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) {
        if (seq_id < ACTION_SEQUENCE_ID_CUSTOM_GESTURE1 || seq_id > ACTION_SEQUENCE_ID_CUSTOM_GESTURE6) {
            std::cerr << "[TouchDexhand] Invalid sequence id: " << seq_id << std::endl;
            return false;
        }

        /// - `0`: 动作序列索引
        /// - `2000`: 动作序列持续时间，单位毫秒
        /// - `0, 0, 100, 100, 100, 100`: 6 个手指位置
        /// - `10, 20, 30, 40, 50, 60`: 6 个手指速度
        /// - `5, 10, 15, 20, 25, 30`: 6 个手指力量
        int action_num = sequences.size();
        uint16_t* sequences_data = new uint16_t[action_num * 20];

        for (int i = 0; i < sequences.size(); i++) {
            int start_index = i * 20;
            sequences_data[start_index++] = i; // index
            sequences_data[start_index++] = sequences[i].duration_ms;

            for (int j = 0; j < 6; j++) {
                sequences_data[start_index++] = static_cast<uint16_t>(std::min<uint16_t>(100, std::max<uint16_t>(-100, sequences[i].positions[j])));
            }
            for (int j = 0; j < 6; j++) {
                sequences_data[start_index++] = static_cast<uint16_t>(std::min<uint16_t>(100, std::max<uint16_t>(-100, sequences[i].speeds[j])));
            }
            for (int j = 0; j < 6; j++) {
                sequences_data[start_index++] = static_cast<uint16_t>(std::min<uint16_t>(100, std::max<uint16_t>(-100, sequences[i].forces[j])));
            }
        }
        
        // std::cout << "[TouchDexhand] set action sequence, data length:" << sequences_data.size() << std::endl;
        // std::cout << "[TouchDexhand] set action sequence, length:" << sequences.size() << std::endl;
        try {
            if (g_stark_wrapper && g_stark_wrapper->transfer_action_sequence) {
                g_stark_wrapper->transfer_action_sequence(handle_, seq_id, action_num, &sequences_data);
                delete[] sequences_data;
                sequences_data = nullptr;
            }
        }catch(std::exception &e) {
            std::cerr << "[TouchDexhand] setActionSequence exception: " << e.what() << std::endl;
            delete[] sequences_data;
            sequences_data = nullptr;
            return false;
        }

        return true;
    }
private:
    void wait_for_trigger_callback(std::atomic<bool> &is_received) {
        is_received = false;
        receive_data(this);        
        int count = 0;
        // cppcheck-suppress knownConditionTrueFalse
        while (!is_received && count < 10) {
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
            count++;
        }

        // if (count >= 10) {
        //     std::cerr << "[ProtobufDexhand] wait_for_trigger_callback timeout, device_id: " << uuid_ << std::endl;
        // }
    }
private:
    int slave_id_;
    std::string uuid_;
    std::shared_ptr<SerialPort> serial_port_;
    StarkDevice* handle_;

    /* data */
    // finger status
    StarkFingerStatus raw_finger_status_;    
    std::atomic<bool> is_finger_status_received_{false};
    // motorboard info
    MotorboardInfo raw_motorboard_info_;
    std::atomic<bool> is_motorboard_info_received_{false};
    // force level
    StarkForceLevel raw_force_level_= StarkForceLevel::STARK_FORCE_LEVEL_NORMAL;
    std::atomic<bool> is_force_level_received_{false};
    // turbo mode
    std::atomic<bool> is_turbo_mode_received_{false};
    int raw_turbo_mode_= 0;
    // turbo config
    std::atomic<bool> is_turbo_config_received_{false};
    TurboConfig_t raw_turbo_config_{0, 0};
};

/********************************************************************************
 * Stark SDK callbacks
 ********************************************************************************/
void printData(const uint8_t *data, size_t size, bool format) {
    for (int i = 0; i < size; ++i) {
        if (format) {
            printf("0x%02X", data[i]); // 打印当前字节的十六进制表示
        } else {
            printf("%02X", data[i]); // 打印当前字节的十六进制表示
        }
        if (i != size - 1) {
            if (format) printf(","); // 除了最后一个字节外，每个字节后面加逗号
        } else {
            printf("\n"); // 最后一个字节后面加换行符
        }
    }
}
/**
 * @brief Send data to the serial port.
 * 
 * @param device_id Device ID.
 * @param data Data to send.
 * @param size Size of the data to send.
 * @return int 0: success, -1: port is NULL, -2: send data failed, -3: timed out.
 */
int _send_data(const char* device_id, const uint8_t *data, int size) {
    /* Send data. */
    if (PRINT_SEND_DATA) {
        printData(data, size, true);
    }

    if(device_id == NULL) {
        printf("[ProtobufDexhand] Error: Send data failed, device_id is NULL.\n");
        return -1;
    }

    ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id);
    if (impl == nullptr) {
        printf("[ProtobufDexhand] Error: Send data failed, device_id: %s,impl is NULL.\n", device_id);
        return -1;
    }

    auto serial_port = impl->serial_port();
    if (serial_port == nullptr) {
        printf("[ProtobufDexhand] Error: Send data failed, device_id: %s, serial_port is NULL.\n", device_id);
        return -1;
    }

    /* We'll allow a 1 second timeout for send. */
	unsigned int timeout = 1000;
    sp_return sp_ret = sp_blocking_write(serial_port->get_sp_port(), data, size, timeout);
    if (sp_ret == SP_ERR_ARG || sp_ret == SP_ERR_FAIL || sp_ret == SP_ERR_MEM) {
        printf("[ProtobufDexhand] Error: Send data failed, sp_ret=%d\n", sp_ret);
        return -2;
    }

	/* Check whether we sent all of the data. */
	if (sp_ret != size) {
        return -3; // time out
    }

    return 0;
}

void receive_data(ProtobufDexhandImpl* impl) {
    if (impl == nullptr) {
        printf("[ProtobufDexhand] Error: Receive data failed, impl is NULL.\n");
        return;
    }

    auto serial_port = impl->serial_port();
    if (serial_port == nullptr) {
        printf("[ProtobufDexhand] Error: Receive data failed, serial_port is NULL.\n");
        return;
    }

    usleep(100000); // wait 100ms for return data

    /* Now we can call sp_wait() to await any event in the set.
    * It will return when an event occurs, or the timeout elapses. */
    sp_return sp_ret = sp_wait(serial_port->get_rx_event_set(), 1000);
    if (sp_ret != SP_OK) {
        printf("[ProtobufDexhand] Error: Receive data failed, sp_ret=%d\n", sp_ret);
        return;
    }

    int bytes_waiting = sp_input_waiting(serial_port->get_sp_port());
    if (bytes_waiting <= 0) {
        if (PRINT_RECEIVE_DATA) {
            printf("[ProtobufDexhand] No data waiting, result:%d.\n", bytes_waiting);
        }
        return;
    }

    if (PRINT_RECEIVE_DATA) {
        printf("[ProtobufDexhand] Receiving %d bytes data.\n", bytes_waiting);
    }

    /* Allocate a buffer to receive data. */
    int size = bytes_waiting;
	uint8_t *buf = (uint8_t *)malloc(size);

    /* We'll allow a 100ms timeout for receive. */
    unsigned int timeout = 100;
    sp_ret = sp_blocking_read(serial_port->get_sp_port(), buf, size, timeout);
    if (sp_ret == SP_ERR_ARG || sp_ret == SP_ERR_FAIL || sp_ret == SP_ERR_MEM) {
        printf("[ProtobufDexhand] Error: Receive data failed, sp_ret=%d\n", sp_ret);
        free(buf);
        return;
    }

	/* Check whether we received the number of bytes we wanted. */
	if (sp_ret > 0) {
        if (PRINT_RECEIVE_DATA) {
            printf("Received %d bytes successfully.\n", sp_ret);
            // printData(buf, sp_ret, true);
        }
        if (g_stark_wrapper && g_stark_wrapper->did_receive_data) {
            g_stark_wrapper->did_receive_data(impl->handle(), buf, sp_ret);
        }
    } else {
        printf("[ProtobufDexhand] Timed out, none data received, result: %d\n", sp_ret);
    }

	/* Free receive buffer. */
	free(buf);
    buf = nullptr;
}
/**************************************** Stark SDK Callback functions ******************************************* */

void _on_stark_log(int level, const char *msg) {
    printf("%s\n", msg);
}

void _on_error(const char *device_id, int error) {
    if (device_id == NULL) {
        printf("[cb on_error], device_id is NULL, error: %d\n", error);
        return;
    }
    printf("[cb on_error] device_id: %s, error: %d\n", device_id, error);
}

void _on_motorboard_info(const char *device_id, MotorboardInfo *info) {
    if (device_id == NULL) {
        printf("[cb on_motorboard_info], device_id is NULL\n");
        return;
    }
    if (PRINT_RECEIVE_DATA) {
        printf("[cb on_motorboard_info] device_id: %s, hand_type: %d, sn: %s, fw_version: %s\n", 
            device_id, static_cast<int>(info->hand_type), info->sn, info->fw_version);
    }

    // 调用 ProtobufDexhandImpl
    if (ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id)) {
        impl->on_motorboard_info(info);
    }
}

void _on_force_level(const char *device_id, int force_level) {
    if (device_id == NULL) {
        printf("[cb on_force_level], device_id is NULL\n");
        return;
    }
    if (PRINT_RECEIVE_DATA) {
        printf("[cb on_force_level] device_id: %s, force_level: %d\n", device_id, force_level);
    }

    if (ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id)) {
        impl->on_force_level(force_level);
    }
}

void _on_turbo_mode(const char *device_id, int turbo_mode) {
    if (device_id == NULL) {
        printf("[cb on_turbo_mode], device_id is NULL\n");
        return;
    }
    if (PRINT_RECEIVE_DATA) {
        printf("[cb on_turbo_mode] device_id: %s, turbo_mode: %d\n", device_id, turbo_mode);
    }
    if (ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id)) {
        impl->on_turbo_mode(turbo_mode);
    }
}

void _on_turbo_config(const char *device_id, int turbo_interval, int turbo_duration) {
    if (device_id == NULL) {
        printf("[cb on_turbo_config], device_id is NULL\n");
        return;
    }
    if (PRINT_RECEIVE_DATA) {
        printf("[cb on_turbo_config] device_id: %s, turbo_interval: %d, turbo_duration: %d\n", device_id, turbo_interval, turbo_duration);
    }
    if (ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id)) {
        impl->on_turbo_config(turbo_interval, turbo_duration);
    }
}

void _on_finger_status(const char *device_id, FingerStatusData *finger_status) {
    if (device_id == NULL) {
        printf("[cb on_finger_status], device_id is NULL\n");
        return;
    }
    if (PRINT_RECEIVE_DATA) {
        printf("[cb on_finger_status] device_id: %s \n", device_id);
    }
    if (ProtobufDexhandImpl* impl = get_impl_by_deviceid(device_id)) {
        impl->on_finger_status(finger_status);
    }
}
/**************************************** Callbacks End ******************************************* */

/**************************************** Class ProtobufDexhand ******************************************* */

std::once_flag init_flag;
namespace dexhand {
std::unique_ptr<ProtobufDexhand> ProtobufDexhand::Connect(
        const std::string& serial_port,
        const std::string& device_id,
        uint8_t slave_id /* = 10 */,
        uint32_t baudrate /* = 115200 */
) {
    // Initialize stark wrapper **Only Once**
    std::call_once(init_flag, []() {
        g_stark_wrapper = stark_wrapper_init("libstark.so");

        // Initialize stark protobuf SDK **Only Once**
        if (g_stark_wrapper) {
            printf("\033[32m[ProtobufDexhand] StarkSDK version: v%s\033[0m\n", g_stark_wrapper->get_sdk_version());
            g_stark_wrapper->set_log_level(LOG_LEVEL_ERROR);
            g_stark_wrapper->set_log_callback(_on_stark_log);
            g_stark_wrapper->set_write_data_callback(_send_data);
        } else {
            printf("\033[31m[ProtobufDexhand] Failed to initialize Stark SDK wrapper\033[0m\n");
        }
    });

    // Open serial port
    std::shared_ptr<SerialPort> sp_serial_port = std::make_shared<SerialPort>(serial_port, baudrate);
    if (!sp_serial_port->open_serial()) {
        printf("\033[31m[ProtobufDexhand] Open serial port failed, serial_port: %s, baudrate: %u\033[0m\n", serial_port.c_str(), baudrate);
        return nullptr;
    }

    // Create ProtobufDexhandImpl device
    ProtobufDexhandImpl* impl = nullptr;
    std::lock_guard<std::mutex> lock(g_device_map_mutex);
    if (g_device_map.find(device_id) != g_device_map.end()) {
        printf("\033[33m[ProtobufDexhand] device_id: %s, already exists.\033[0m\n", device_id.c_str());
        impl = g_device_map[device_id];
    } else {
        impl = new ProtobufDexhandImpl(sp_serial_port, device_id, slave_id);
        g_device_map[device_id] = impl;
        printf("[ProtobufDexhand] Create ProtobufDexhandImpl, uuid: %s, slave_id: %d\n", device_id.c_str(), slave_id);
    }
    return std::unique_ptr<ProtobufDexhand>(new ProtobufDexhand(impl));
}

ProtobufDexhand::ProtobufDexhand(ProtobufDexhandImpl* impl): DexHandBase(), impl_(impl) {
}

ProtobufDexhand::~ProtobufDexhand() {
    if (impl_) {
        std::lock_guard<std::mutex> lock(g_device_map_mutex);
        g_device_map.erase(impl_->device_id());
        delete impl_;
    }
}
/**************************************** Getters ******************************************* */
DexHandFwType ProtobufDexhand::getDexHandFwType() {
    return DexHandFwType::RS485_PROTOBUF;
}

bool ProtobufDexhand::getDeviceInfo(DeviceInfo_t& info) {
    return impl_->getDeviceInfo(info);
}

bool ProtobufDexhand::getFingerStatus(FingerStatus& status) {
    return impl_->getFingerStatus(status);
}

GripForce ProtobufDexhand::getGripForce() {
    return impl_->getGripForce();
}

bool ProtobufDexhand::isTurboModeEnabled() {
    return impl_->getTurboModeEnabled();
}

TurboConfig_t ProtobufDexhand::getTurboConfig() {
    return impl_->getTurboConfig();
}

/**************************************** Setters ******************************************* */
void ProtobufDexhand::setFingerPositions(const UnsignedFingerArray &positions) {
    impl_->setFingerPositions(positions);
}

void ProtobufDexhand::setFingerSpeeds(const FingerArray &speeds) {
    impl_->setFingerSpeeds(speeds);
}

void ProtobufDexhand::setGripForce(GripForce level) {
    impl_->setGripForce(level);
}

void ProtobufDexhand::setTurboModeEnabled(bool enabled) {
    impl_->setTurboModeEnabled(enabled);
}

void ProtobufDexhand::runActionSequence(ActionSequenceId_t seq_id) {
    impl_->runActionSequence(seq_id);
}

bool ProtobufDexhand::setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences) {
    return impl_->setActionSequence(seq_id, sequences);
}

// 清理函数，可以在程序退出时调用
void CleanupProtobufDexhand() {
    if (g_stark_wrapper) {
        stark_wrapper_cleanup(g_stark_wrapper);
        g_stark_wrapper = nullptr;
    }
}

} // namespace dexhand
/**************************************** Class ProtobufDexhand ******************************************* */
/*
Getter: 获取数据的流程
    call getXX 调用获取函数接口
      --> stark_get_XX 调用 stark sdk 的获取函数
      --> wait_for_trigger_callback 等待回调
            --> receive_data 接收串口数据
              --> stark_did_receive_data 处理串口数据
                --> on_XX callback sdk内部触发回调函数
                    --> impl->on_XX callback 回调函数触发impl函数处理数据
      --> return 返回数据
*/

