/**
 * 全身电机磨线程序
 *
 * 驱动腿/腰(CANFD广播) + 手臂/头(CAN单帧) 电机执行重复性关键帧动作，
 * 用于新电机的线束磨合（break-in）。
 *
 * 总线布局 (Roban v14):
 *   bcan0: 左腿 joint 1-6     (CANFD 广播, 500Hz)
 *   bcan1: 右腿 joint 1-6 + 腰 joint 7  (CANFD 广播, 500Hz)
 *   bcan2: 左臂 joint 1-4 + 头 joint 9-10 (CAN 单帧, 250Hz)
 *   bcan3: 右臂 joint 5-8     (CAN 单帧, 250Hz)
 *
 * 用法:
 *   sudo ./canfd_breakin --config <yaml> [选项]
 *
 * 选项:
 *   --config <file>      YAML 配置文件路径 (必须)
 *   --duration <秒>      运行总时长 (默认 600)
 *   --rounds <次>        运行轮数 (与 --duration 二选一)
 *   --log <file.csv>     CSV 日志路径
 *   --no-log             不记录日志
 *   --no-waist           不控制腰部
 *   --no-arm             不控制手臂/头部
 *   --leg-only           只控制腿部 (等价于 --no-waist --no-arm)
 *   --arm-only           只控制手臂/头部 (不控制腿/腰)
 *   --kp <值>            覆盖腿部所有 kp
 *   --kd <值>            覆盖腿部所有 kd
 *   --hz <频率>          腿部控制频率 (默认 500)
 *   --dry-run            只打印配置
 */

#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <memory>
#include <time.h>
#include <cstring>
#include <cstdio>

// ============================================================
// 数据结构
// ============================================================
struct JointGain { double kp = 150.0; double kd = 2.0; };
struct MirrorRule { int from_joint; bool negate; };

struct BreakinConfig {
    // --- 腿部 CAN FD ---
    std::string left_leg_interface = "bcan0";
    int left_leg_motor_count = 6;
    std::vector<int> left_leg_motor_ids = {1,2,3,4,5,6};

    std::string right_leg_interface = "bcan1";
    int right_leg_motor_count = 7;
    std::vector<int> right_leg_motor_ids = {1,2,3,4,5,6,7};

    // 腰部总线归属:
    //   空           -> 沿用 v14: 腰挂右腿总线, 靠 (id > left_leg_motor_count) 识别
    //   == 左腿接口  -> v17: 腰挂左腿总线(bcan0), 由左腿循环用 waist_motor_id 驱动
    std::string waist_interface;
    int waist_motor_id = 0;

    int frequency_hz = 500;
    double default_kp = 150.0;
    double default_kd = 2.0;
    std::map<int, JointGain> joint_gains;          // 手写 yaml 的增益(按 id), v14 用
    // 按总线分的增益(消除同 id 跨总线冲突). 由 joint_gains 复制或被 device_config 覆盖
    std::map<int, JointGain> left_gains;
    std::map<int, JointGain> right_gains;
    JointGain waist_gain;
    // 臂/头增益(按 MotorevoActuator 索引 0-3左臂,4-5头,6-9右臂), 由 device_config 读;
    // 空或某项 kp<0 时回退 arm_kp/head_kp
    std::vector<JointGain> arm_gains;

    // 部署设备配置(~/.config/lejuconfig/canbus_device_cofig.yaml): 若指定则
    // 从中按总线读取 腿/腰 的电机 id 与 kp/kd, 覆盖上面手写的 canbus/joint_gains
    std::string device_config;

    double frame_duration_sec = 1.0;
    std::vector<std::vector<double>> left_leg_keyframes;   // [joint][frame], 度
    std::vector<std::vector<double>> right_leg_keyframes;  // [joint][frame], 度; 非空则右腿独立, 否则镜像左腿
    std::map<int, MirrorRule> right_leg_mirror;
    std::vector<std::vector<double>> waist_keyframes;      // [0][frame], 度

    // --- 手臂/头 CAN 单帧 ---
    std::string arm_canbus_config;    // MotorevoActuator 配置文件
    int arm_frequency_hz = 250;
    double arm_frame_duration_ms = 1900;

    // left_arm_keyframes[frame][4joints], 弧度
    std::vector<std::vector<double>> left_arm_keyframes;
    std::map<int, MirrorRule> right_arm_mirror;
    // head_keyframes[frame][2joints], 弧度
    std::vector<std::vector<double>> head_keyframes;

    double arm_kp = 20.0, arm_kd = 1.0;
    double head_kp = 10.0, head_kd = 1.0;

    // --- 安全 ---
    std::map<int, double> max_offset_deg;
    double arm_max_offset_rad = 3.0;
    double head_max_offset_rad = 1.5;
    int stale_threshold = 50;
    int max_temperature = 80;
};

// ============================================================
// 工具函数
// ============================================================
static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

static std::vector<double> parseYamlArray(const std::string& line) {
    std::vector<double> r;
    size_t a = line.find('['), b = line.rfind(']');
    if (a == std::string::npos || b == std::string::npos) return r;
    std::istringstream iss(line.substr(a+1, b-a-1));
    std::string tok;
    while (std::getline(iss, tok, ',')) {
        std::string t = trim(tok);
        if (!t.empty()) { try { r.push_back(std::stod(t)); } catch(...) {} }
    }
    return r;
}

// 解析 { key: val, ... } 中的值
static std::string extractBraceVal(const std::string& inner, const std::string& key) {
    size_t pos = inner.find(key + ":");
    if (pos == std::string::npos) return "";
    size_t start = pos + key.size() + 1;
    size_t comma = inner.find(',', start);
    return trim((comma != std::string::npos) ? inner.substr(start, comma-start) : inner.substr(start));
}

// ============================================================
// YAML 配置加载
// ============================================================
static bool loadConfig(const std::string& path, BreakinConfig& cfg) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "\033[31m无法打开配置文件: " << path << "\033[0m\n";
        return false;
    }

    std::string line;
    std::string section, subsection, array_target;

    while (std::getline(file, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;

        // 顶层 section
        if (t == "canbus:")  { section = "canbus"; subsection.clear(); array_target.clear(); continue; }
        if (t == "control:") { section = "control"; subsection.clear(); array_target.clear(); continue; }
        if (t == "action:")  { section = "action"; subsection.clear(); array_target.clear(); continue; }
        if (t == "arm:")     { section = "arm"; subsection.clear(); array_target.clear(); continue; }
        if (t == "safety:")  { section = "safety"; subsection.clear(); array_target.clear(); continue; }

        size_t colon = t.find(':');
        std::string key = (colon != std::string::npos) ? trim(t.substr(0, colon)) : "";
        std::string val = (colon != std::string::npos) ? trim(t.substr(colon+1)) : "";
        // 去除引号
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
            val = val.substr(1, val.size()-2);

        // ---- 顶层标量 ----
        if (section.empty() && key == "device_config") { cfg.device_config = val; continue; }

        // ---- canbus ----
        if (section == "canbus") {
            if (t == "left_leg:")  { subsection = "left_leg"; continue; }
            if (t == "right_leg:") { subsection = "right_leg"; continue; }
            if (t == "waist:")     { subsection = "waist"; continue; }
            if (subsection == "left_leg") {
                if (key == "interface") cfg.left_leg_interface = val;
                else if (key == "motor_count") cfg.left_leg_motor_count = std::stoi(val);
                else if (key == "motor_ids") { auto v = parseYamlArray(t); cfg.left_leg_motor_ids.clear(); for(auto x:v) cfg.left_leg_motor_ids.push_back((int)x); }
            } else if (subsection == "right_leg") {
                if (key == "interface") cfg.right_leg_interface = val;
                else if (key == "motor_count") cfg.right_leg_motor_count = std::stoi(val);
                else if (key == "motor_ids") { auto v = parseYamlArray(t); cfg.right_leg_motor_ids.clear(); for(auto x:v) cfg.right_leg_motor_ids.push_back((int)x); }
            } else if (subsection == "waist") {
                if (key == "interface") cfg.waist_interface = val;
                else if (key == "motor_id") cfg.waist_motor_id = std::stoi(val);
            }
        }

        // ---- control ----
        if (section == "control") {
            if (t == "joint_gains:") { subsection = "joint_gains"; continue; }
            if (subsection.empty()) {
                if (key == "frequency_hz") cfg.frequency_hz = std::stoi(val);
                else if (key == "default_kp") cfg.default_kp = std::stod(val);
                else if (key == "default_kd") cfg.default_kd = std::stod(val);
            }
            if (subsection == "joint_gains") {
                size_t ba = t.find('{'), bb = t.find('}');
                if (ba != std::string::npos && bb != std::string::npos) {
                    int jid = std::stoi(trim(t.substr(0, t.find(':'))));
                    std::string inner = t.substr(ba+1, bb-ba-1);
                    JointGain g = {cfg.default_kp, cfg.default_kd};
                    std::string s;
                    s = extractBraceVal(inner, "kp"); if (!s.empty()) g.kp = std::stod(s);
                    s = extractBraceVal(inner, "kd"); if (!s.empty()) g.kd = std::stod(s);
                    cfg.joint_gains[jid] = g;
                }
            }
        }

        // ---- action ----
        if (section == "action") {
            if (t[0] == '-' && !array_target.empty()) {
                auto v = parseYamlArray(t);
                if (!v.empty()) {
                    if (array_target == "left_leg_keyframes") cfg.left_leg_keyframes.push_back(v);
                    else if (array_target == "right_leg_keyframes") cfg.right_leg_keyframes.push_back(v);
                    else if (array_target == "waist_keyframes") cfg.waist_keyframes.push_back(v);
                }
                continue;
            }
            if (colon != std::string::npos && t[0] != '-') {
                if (key == "frame_duration_sec") { cfg.frame_duration_sec = std::stod(val); array_target.clear(); }
                else if (key == "left_leg_keyframes") { array_target = "left_leg_keyframes"; subsection.clear(); continue; }
                else if (key == "right_leg_keyframes") { array_target = "right_leg_keyframes"; subsection.clear(); continue; }
                else if (key == "waist_keyframes") { array_target = "waist_keyframes"; subsection.clear(); continue; }
                else if (key == "right_leg_mirror") { subsection = "right_leg_mirror"; array_target.clear(); continue; }
                else array_target.clear();
            }
            if (subsection == "right_leg_mirror") {
                size_t ba = t.find('{'), bb = t.find('}');
                if (ba != std::string::npos && bb != std::string::npos) {
                    int jid = std::stoi(trim(t.substr(0, t.find(':'))));
                    std::string inner = t.substr(ba+1, bb-ba-1);
                    MirrorRule rule = {jid, false};
                    std::string s;
                    s = extractBraceVal(inner, "from"); if (!s.empty()) rule.from_joint = std::stoi(s);
                    s = extractBraceVal(inner, "negate"); rule.negate = (s.find("true") != std::string::npos);
                    cfg.right_leg_mirror[jid] = rule;
                }
            }
        }

        // ---- arm ----
        if (section == "arm") {
            if (t[0] == '-' && !array_target.empty()) {
                auto v = parseYamlArray(t);
                if (!v.empty()) {
                    if (array_target == "left_arm_keyframes") cfg.left_arm_keyframes.push_back(v);
                    else if (array_target == "head_keyframes") cfg.head_keyframes.push_back(v);
                }
                continue;
            }
            if (colon != std::string::npos && t[0] != '-') {
                if (key == "canbus_config") { cfg.arm_canbus_config = val; array_target.clear(); }
                else if (key == "frequency_hz") { cfg.arm_frequency_hz = std::stoi(val); array_target.clear(); }
                else if (key == "frame_duration_ms") { cfg.arm_frame_duration_ms = std::stod(val); array_target.clear(); }
                else if (key == "left_arm_keyframes") { array_target = "left_arm_keyframes"; subsection.clear(); continue; }
                else if (key == "head_keyframes") { array_target = "head_keyframes"; subsection.clear(); continue; }
                else if (key == "right_arm_mirror") { subsection = "right_arm_mirror"; array_target.clear(); continue; }
                else if (key == "arm_kp") { cfg.arm_kp = std::stod(val); array_target.clear(); }
                else if (key == "arm_kd") { cfg.arm_kd = std::stod(val); array_target.clear(); }
                else if (key == "head_kp") { cfg.head_kp = std::stod(val); array_target.clear(); }
                else if (key == "head_kd") { cfg.head_kd = std::stod(val); array_target.clear(); }
                else array_target.clear();
            }
            if (subsection == "right_arm_mirror") {
                size_t ba = t.find('{'), bb = t.find('}');
                if (ba != std::string::npos && bb != std::string::npos) {
                    int idx = std::stoi(trim(t.substr(0, t.find(':'))));
                    std::string inner = t.substr(ba+1, bb-ba-1);
                    MirrorRule rule = {idx, false};
                    std::string s;
                    s = extractBraceVal(inner, "from"); if (!s.empty()) rule.from_joint = std::stoi(s);
                    s = extractBraceVal(inner, "negate"); rule.negate = (s.find("true") != std::string::npos);
                    cfg.right_arm_mirror[idx] = rule;
                }
            }
        }

        // ---- safety ----
        if (section == "safety") {
            if (t == "max_offset_deg:") { subsection = "max_offset_deg"; continue; }
            if (subsection == "max_offset_deg" && !val.empty()) {
                try { cfg.max_offset_deg[std::stoi(key)] = std::stod(val); } catch(...) {}
            } else {
                if (key == "stale_threshold") cfg.stale_threshold = std::stoi(val);
                else if (key == "max_temperature") cfg.max_temperature = std::stoi(val);
                else if (key == "arm_max_offset_rad") cfg.arm_max_offset_rad = std::stod(val);
                else if (key == "head_max_offset_rad") cfg.head_max_offset_rad = std::stod(val);
            }
        }
    }
    return true;
}

// ============================================================
// 全局
// ============================================================
std::atomic<bool> g_running(true);
void signalHandler(int) {
    std::cout << "\n\033[33m收到停止信号, 正在安全停止...\033[0m\n";
    g_running = false;
}

// Minimum-jerk: s(t) = 10t³ - 15t⁴ + 6t⁵
static double mjInterp(double a, double b, double t) {
    t = std::max(0.0, std::min(1.0, t));
    double s = 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t;
    return a + (b - a) * s;
}
// Minimum-jerk 速度前馈: s'(t)/T
static double mjVel(double a, double b, double T, double t) {
    t = std::max(0.0, std::min(1.0, t));
    double ds = 30*t*t - 60*t*t*t + 30*t*t*t*t;
    return (b - a) * ds / T;
}

static std::string getTimestamp() {
    time_t now = time(nullptr); struct tm buf; localtime_r(&now, &buf);
    char s[32]; strftime(s, sizeof(s), "%Y%m%d_%H%M%S", &buf); return s;
}
static std::string formatTime(double sec) {
    int h = (int)sec/3600, m = ((int)sec%3600)/60, s = (int)sec%60;
    char buf[32]; snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s); return buf;
}

// 打印 CAN 接口状态/错误计数, 用于排查总线是否进入 error-warn / bus-off
// (区分"总线位错误/bus-off" 与 "电机不响应": 前者 berr/bus-off 会涨, 后者计数为 0)
static void dumpCanState(const std::string& tag, const std::vector<std::string>& ifaces) {
    for (const auto& ifc : ifaces) {
        if (ifc.empty()) continue;
        std::string cmd = "ip -details -statistics link show " + ifc +
            " 2>/dev/null | grep -iE 'state |bus-off|berr|error-warn|RX:|TX:|^[[:space:]]+[0-9]'";
        std::cout << "\033[35m[CAN-" << tag << "] " << ifc << "\033[0m\n";
        FILE* fp = popen(cmd.c_str(), "r");
        if (!fp) continue;
        char buf[300];
        while (fgets(buf, sizeof(buf), fp)) std::cout << "    " << buf;
        pclose(fp);
    }
}

// 展开路径首部的 ~ / $HOME 为绝对路径 (C++ 文件 API 不会自动展开)
static std::string expandHome(const std::string& p) {
    if (p.empty()) return p;
    const char* home = getenv("HOME");
    if (!home) return p;
    if (p == "~") return home;
    if (p.rfind("~/", 0) == 0) return std::string(home) + p.substr(1);
    if (p.rfind("$HOME/", 0) == 0) return std::string(home) + p.substr(5);
    return p;
}

// 取设备名尾部数字后缀 (Lleg_joint_03 -> 3); 无则 -1
static int nameSuffix(const std::string& n) {
    size_t p = n.find_last_not_of("0123456789");
    return (p != std::string::npos && p + 1 < n.size()) ? std::stoi(n.substr(p + 1)) : -1;
}

// ============================================================
// 主程序 (统一: 全身走单个 MotorevoActuator, 与 hardware_node 同一套驱动)
//   - 不再自写 ControlLoop / 直接用 RevoMotorControl
//   - 腿/腰/臂/头全部由一个 MotorevoActuator(读部署 canbus 配置)驱动
//   - cali_buses 为空: 不重写零点(仅用已标定零点回零)
// ============================================================
static void printUsage() {
    std::cout << "用法: sudo ./canfd_breakin --config <yaml> --device-config <canbus_device_cofig.yaml> [选项]\n"
              << "  --device-config <yaml>  部署 canbus 配置(总线/电机 id/kp-kd 来源, 必须)\n"
              << "  --duration <秒>  运行时长 (默认 600)\n"
              << "  --rounds <次>    运行轮数\n"
              << "  --log <csv>      日志路径   --no-log 不记录\n"
              << "  --no-waist / --no-arm / --leg-only / --arm-only\n"
              << "  --kp/--kd <值>   覆盖所有关节增益    --hz <频率> 腿部频率(默认500)\n"
              << "  --dry-run        只打印配置\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string config_path, device_config_cli;
    double duration = 600.0;
    int rounds = -1;
    std::string log_file;
    bool no_log=false, no_waist=false, no_arm=false, arm_only=false, dry_run=false;
    double override_kp=-1, override_kd=-1;
    int override_hz=-1;

    for (int i=1;i<argc;++i){
        std::string a=argv[i];
        if      (a=="--config" && i+1<argc) config_path=argv[++i];
        else if (a=="--device-config" && i+1<argc) device_config_cli=argv[++i];
        else if (a=="--duration" && i+1<argc) duration=std::stod(argv[++i]);
        else if (a=="--rounds" && i+1<argc) rounds=std::stoi(argv[++i]);
        else if ((a=="--log"||a=="--csv") && i+1<argc) log_file=argv[++i];
        else if (a=="--no-log") no_log=true;
        else if (a=="--no-waist") no_waist=true;
        else if (a=="--no-arm") no_arm=true;
        else if (a=="--leg-only"){ no_waist=true; no_arm=true; }
        else if (a=="--arm-only") arm_only=true;
        else if (a=="--dry-run") dry_run=true;
        else if (a=="--kp" && i+1<argc) override_kp=std::stod(argv[++i]);
        else if (a=="--kd" && i+1<argc) override_kd=std::stod(argv[++i]);
        else if (a=="--hz" && i+1<argc) override_hz=std::stoi(argv[++i]);
        else if (a=="-h"||a=="--help"){ printUsage(); return 0; }
    }
    if (config_path.empty()){ std::cerr << "\033[31m错误: 请指定 --config <yaml>\033[0m\n"; return 1; }

    BreakinConfig cfg;
    if (!loadConfig(config_path, cfg)) return 1;

    // device_config: CLI 优先, 其次 yaml; v17 必须
    std::string dev = !device_config_cli.empty() ? device_config_cli : cfg.device_config;
    if (dev.empty()){ std::cerr << "\033[31m错误: 需要 --device-config 或 yaml 中 device_config\033[0m\n"; return 1; }
    dev = expandHome(dev);

    int leg_hz = override_hz>0 ? override_hz : cfg.frequency_hz;

    // ---- 运动时序 ----
    int leg_nf = !cfg.left_leg_keyframes.empty() ? (int)cfg.left_leg_keyframes[0].size() : 1;
    int leg_nj = (int)cfg.left_leg_keyframes.size();
    double leg_cycle = leg_nf * cfg.frame_duration_sec;
    bool have_arm_kf = !cfg.left_arm_keyframes.empty();
    int arm_nf = have_arm_kf ? (int)cfg.left_arm_keyframes.size() : 1;
    int arm_nj = have_arm_kf ? (int)cfg.left_arm_keyframes[0].size() : 0;
    int head_nj = !cfg.head_keyframes.empty() ? (int)cfg.head_keyframes[0].size() : 0;
    double arm_frame_sec = cfg.arm_frame_duration_ms / 1000.0;
    double arm_cycle = arm_nf * arm_frame_sec;
    double cycle_time = std::max(leg_cycle, (have_arm_kf && !no_arm) ? arm_cycle : 0.0);
    if (arm_only) cycle_time = arm_cycle;
    if (cycle_time <= 0) cycle_time = leg_cycle > 0 ? leg_cycle : 1.0;
    if (rounds > 0) duration = rounds * cycle_time;
    int total_rounds = (int)std::ceil(duration / cycle_time);

    // ---- 打印计划 ----
    std::cout << "\033[32m" << std::string(60,'=') << "\n  全身电机磨线 (统一 MotorevoActuator)\n" << std::string(60,'=') << "\033[0m\n\n";
    std::cout << "配置文件:   " << config_path << "\n";
    std::cout << "设备配置:   " << dev << "\n";
    std::cout << "频率:       腿/腰 " << leg_hz << "Hz, 臂/头 " << leg_hz/2 << "Hz\n";
    std::cout << "腿动作:     " << leg_nf << "帧×" << cfg.frame_duration_sec << "s = " << leg_cycle << "s/轮\n";
    if (have_arm_kf) std::cout << "臂动作:     " << arm_nf << "帧×" << arm_frame_sec << "s = " << arm_cycle << "s/轮\n";
    std::cout << "范围:       " << (arm_only?"仅臂/头":(no_arm?"腿+腰(--no-arm)":"全身")) << (no_waist?" 不含腰":"") << "\n";
    std::cout << "运行时长:   " << formatTime(duration) << " (" << total_rounds << " 轮)\n";
    if (!no_log && log_file.empty()) log_file = "breakin_" + getTimestamp() + ".csv";
    if (!no_log) std::cout << "CSV 日志:   " << log_file << "\n";
    std::cout << "\n\033[33m请确保机器人已吊起, 各关节处于零位, 周围无障碍物; Ctrl+C 安全停止\033[0m\n\n";

    if (dry_run) { std::cout << "\033[36m[dry-run] 配置检查完毕 (未读取部署配置/未驱动电机)\033[0m\n"; return 0; }

    // ---- 从 device_config 读各电机 kp/kd (按设备名) ----
    std::map<std::string, JointGain> gain_by_name;
    {
        canbus_sdk::ConfigParser parser;
        try {
            if (!parser.parseFromFile(dev)) { std::cerr << "\033[31m设备配置解析失败: " << dev << "\033[0m\n"; return 1; }
        } catch (const std::exception& e) { std::cerr << "\033[31m设备配置解析异常: " << e.what() << "\033[0m\n"; return 1; }
        for (const auto& bus : parser.getCanBusConfigs())
            for (const auto& d : parser.getDevices(bus.name)) {
                double kp = d.motor_params.size()>1 ? d.motor_params[1] : cfg.default_kp;
                double kd = d.motor_params.size()>2 ? d.motor_params[2] : cfg.default_kd;
                gain_by_name[d.name] = {kp, kd};
            }
    }

    // ============================================================
    // 创建统一 actuator (读部署配置, 全身; 空 cali_buses = 不重写零点)
    // ============================================================
    dumpCanState("init前", {"bcan0","bcan1","bcan2","bcan3"});
    motorevo::MotorevoActuator actuator(dev, std::set<std::string>{}, leg_hz);
    int ret = actuator.init();
    dumpCanState("init后", {"bcan0","bcan1","bcan2","bcan3"});
    if (ret != 0) { std::cerr << "\033[31mactuator 初始化失败 (code=" << ret << ")\033[0m\n"; return 1; }

    auto names = actuator.getMotorRefNames();
    int N = (int)names.size();

    // ---- 为每个电机索引建立运动计划 ----
    enum Type { NONE, LLEG, RLEG, WAIST, LARM, RARM, HEAD };
    struct Plan { Type type=NONE; int j=0; bool neg=false; double kp=0, kd=0; double max_off=1e9; std::string name; };
    std::vector<Plan> plan(N);
    auto degLimit = [&](int joint1based)->double {
        auto it = cfg.max_offset_deg.find(joint1based);
        return it != cfg.max_offset_deg.end() ? it->second * M_PI / 180.0 : 1e9;
    };
    for (int i=0;i<N;++i){
        Plan pl; pl.name = names[i];
        auto git = gain_by_name.find(names[i]);
        pl.kp = git!=gain_by_name.end() ? git->second.kp : cfg.default_kp;
        pl.kd = git!=gain_by_name.end() ? git->second.kd : cfg.default_kd;
        if (override_kp>=0) pl.kp = override_kp;
        if (override_kd>=0) pl.kd = override_kd;
        const std::string& nm = names[i];
        if (nm.rfind("Lleg_joint",0)==0)       { int x=nameSuffix(nm); pl.type=LLEG;  pl.j=x-1; pl.max_off=degLimit(x); }
        else if (nm.rfind("Rleg_joint",0)==0)  { int x=nameSuffix(nm); pl.type=RLEG;  pl.j=x-1; pl.max_off=degLimit(x);
            auto m=cfg.right_leg_mirror.find(x); pl.neg = m!=cfg.right_leg_mirror.end() ? m->second.negate : false; }
        else if (nm.rfind("Waist",0)==0)       { pl.type=WAIST; pl.max_off=degLimit(7); }
        else if (nm.rfind("Larm_joint",0)==0)  { int x=nameSuffix(nm); pl.type=LARM;  pl.j=x-1; pl.max_off=cfg.arm_max_offset_rad; }
        else if (nm=="Head_yaw")               { pl.type=HEAD; pl.j=0; pl.max_off=cfg.head_max_offset_rad; }
        else if (nm=="Head_pitch")             { pl.type=HEAD; pl.j=1; pl.max_off=cfg.head_max_offset_rad; }
        else if (nm.rfind("Rarm_joint",0)==0)  { int x=nameSuffix(nm); pl.type=RARM;  pl.j=x-1; pl.max_off=cfg.arm_max_offset_rad;
            auto m=cfg.right_arm_mirror.find(x-1); pl.neg = m!=cfg.right_arm_mirror.end() ? m->second.negate : false; }
        plan[i] = pl;
    }
    std::cout << "\033[36m电机映射 (共 " << N << " 个):\033[0m\n";
    for (int i=0;i<N;++i)
        std::cout << "  [" << i << "] " << plan[i].name << "  kp=" << plan[i].kp << " kd=" << plan[i].kd << "\n";

    std::vector<double> init_pos = actuator.getPositions();
    if ((int)init_pos.size() < N) init_pos.resize(N, 0.0);

    // ---- CSV ----
    std::ofstream csv;
    if (!no_log && !log_file.empty()) {
        csv.open(log_file);
        if (csv.is_open()) {
            csv << "time_s,round";
            for (int i=0;i<N;++i) csv << "," << plan[i].name << "_tgt," << plan[i].name << "_act";
            csv << "\n";
        }
    }
    int log_iv = std::max(1, leg_hz/10);

    // ============================================================
    // 磨线主循环 (setTargets 更新目标; actuator 控制线程负责发帧)
    // ============================================================
    std::cout << "\n\033[32m开始磨线! " << formatTime(duration) << " (" << total_rounds << " 轮)\033[0m\n\n";
    auto start = std::chrono::steady_clock::now();
    int cur_round = 0; long counter = 0;
    std::vector<double> last_pos; int global_stale = 0;
    int comm_thr = std::max(cfg.stale_threshold, leg_hz);  // 全部冻结≥1s 才判总线掉线

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double el = std::chrono::duration<double>(now - start).count();
        if (el >= duration) { std::cout << "\033[32m磨线时长已到\033[0m\n"; break; }
        int rd = (int)(el / cycle_time);
        if (rd > cur_round) { cur_round = rd;
            std::cout << "\033[36m[" << formatTime(el) << "] 第 " << cur_round << "/" << total_rounds
                      << " 轮 (" << std::fixed << std::setprecision(1) << el/duration*100 << "%)\033[0m\n"; }

        double lp = std::fmod(el, leg_cycle);
        int fi = (int)(lp / cfg.frame_duration_sec); int fin = (fi+1)%std::max(1,leg_nf);
        double ft = (lp - fi*cfg.frame_duration_sec) / cfg.frame_duration_sec;
        double acy = arm_cycle>0 ? arm_cycle : 1.0;
        double ap = std::fmod(el, acy);
        int afi = (int)(ap / arm_frame_sec); int afin = (afi+1)%std::max(1,arm_nf);
        double aft = (ap - afi*arm_frame_sec) / arm_frame_sec;

        auto legDeg = [&](int j)->double {
            if (j<0 || j>=leg_nj) return 0.0;
            return mjInterp(cfg.left_leg_keyframes[j][fi], cfg.left_leg_keyframes[j][fin], ft);
        };
        // 右腿: 有独立关键帧用之, 否则镜像左腿(按 negate)
        auto rightLegDeg = [&](int j, bool neg)->double {
            if (j>=0 && j<(int)cfg.right_leg_keyframes.size()
                && fin<(int)cfg.right_leg_keyframes[j].size())
                return mjInterp(cfg.right_leg_keyframes[j][fi], cfg.right_leg_keyframes[j][fin], ft);
            double d = legDeg(j); return neg ? -d : d;
        };
        auto armRad = [&](int j)->double {
            if (j<0 || j>=arm_nj) return 0.0;
            return mjInterp(cfg.left_arm_keyframes[afi][j], cfg.left_arm_keyframes[afin][j], aft);
        };

        std::vector<motorevo::RevoMotorCmd_t> tgt(N);
        bool trip = false;
        for (int i=0;i<N && !trip;++i){
            const Plan& pl = plan[i];
            double off = 0.0;  // rad, 关节空间偏移
            switch (pl.type) {
                case LLEG:  if (!arm_only) off = legDeg(pl.j) * M_PI/180.0; break;
                case RLEG:  if (!arm_only) off = rightLegDeg(pl.j, pl.neg) * M_PI/180.0; break;
                case WAIST: if (!arm_only && !no_waist && !cfg.waist_keyframes.empty()) {
                                double d = mjInterp(cfg.waist_keyframes[0][fi], cfg.waist_keyframes[0][fin], ft);
                                off = d*M_PI/180.0; } break;
                case LARM:  if (!no_arm && have_arm_kf) off = armRad(pl.j); break;
                case RARM:  if (!no_arm && have_arm_kf) { double o = armRad(pl.j); if (pl.neg) o=-o; off = o; } break;
                case HEAD:  if (!no_arm && pl.j<head_nj && !cfg.head_keyframes.empty())
                                off = mjInterp(cfg.head_keyframes[afi][pl.j], cfg.head_keyframes[afin][pl.j], aft); break;
                default: off = 0.0;
            }
            if (std::fabs(off) > pl.max_off) {
                std::cerr << "\033[31m安全限位! " << pl.name << " offset=" << off << " rad > " << pl.max_off << "\033[0m\n";
                trip = true; break;
            }
            motorevo::RevoMotorCmd_t c;
            c.pos = init_pos[i] + off; c.vel = 0; c.torque = 0; c.kp = pl.kp; c.kd = pl.kd;
            tgt[i] = c;
        }
        if (trip) { g_running = false; break; }
        actuator.setTargets(tgt);

        // 通信丢失检测: 仅当【全部电机反馈同时冻结】持续 comm_thr 帧(整条总线掉线)才触发,
        // 避免磨线起步/静止关节(偏移≈0)被逐个误判
        auto pos = actuator.getPositions();
        if ((int)pos.size() == N) {
            if (last_pos.empty()) { last_pos = pos; }
            else {
                bool any_moved = false;
                for (int i=0;i<N;++i) if (std::fabs(pos[i]-last_pos[i]) > 1e-6) { any_moved = true; break; }
                if (any_moved) { global_stale = 0; last_pos = pos; }
                else if (++global_stale >= comm_thr) {
                    std::cerr << "\033[31m通信丢失! 全部电机反馈冻结 " << comm_thr << " 帧 (~"
                              << comm_thr*1000/leg_hz << "ms), 疑似总线掉线\033[0m\n";
                    g_running = false;
                }
            }
        }
        if (!g_running) break;

        if (csv.is_open() && counter%log_iv==0) {
            csv << std::fixed << std::setprecision(4) << el << "," << cur_round;
            for (int i=0;i<N;++i) csv << "," << tgt[i].pos << "," << (i<(int)pos.size()?pos[i]:0.0);
            csv << "\n";
        }
        counter++;
        std::this_thread::sleep_for(std::chrono::microseconds(1000000 / leg_hz));
    }

    // ============================================================
    // 平滑回零 (2s, 回到 init_pos) → 失能
    // ============================================================
    std::cout << "\033[33m正在回零位...\033[0m\n";
    std::vector<double> cur = actuator.getPositions();
    if ((int)cur.size() < N) cur.resize(N, 0.0);
    auto ret_start = std::chrono::steady_clock::now();
    const double ret_dur = 2.0;
    while (true) {
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - ret_start).count();
        if (t >= ret_dur) break;
        double s = mjInterp(0.0, 1.0, t/ret_dur);
        std::vector<motorevo::RevoMotorCmd_t> tg(N);
        for (int i=0;i<N;++i) {
            motorevo::RevoMotorCmd_t c;
            c.pos = cur[i] + (init_pos[i]-cur[i])*s; c.vel=0; c.torque=0; c.kp=plan[i].kp; c.kd=plan[i].kd;
            tg[i] = c;
        }
        actuator.setTargets(tg);
        std::this_thread::sleep_for(std::chrono::microseconds(1000000 / leg_hz));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::cout << "\033[33m失能电机...\033[0m\n";
    actuator.disableAll();
    actuator.reset();
    if (csv.is_open()) { csv.close(); std::cout << "\033[32mCSV: " << log_file << "\033[0m\n"; }

    std::cout << "\n\033[32m" << std::string(60,'=') << "\n  磨线结束\n" << std::string(60,'=') << "\033[0m\n";
    return 0;
}
