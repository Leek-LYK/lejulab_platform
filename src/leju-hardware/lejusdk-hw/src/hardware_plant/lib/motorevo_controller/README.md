# Motorevo Controller

Revo 系列电机控制器，提供统一的电机控制接口。

```
motorevo_controller/
├── src/                     # 实现文件
│   ├── motor_ctrl.cpp       # 电机控制实现
│   ├── motorevo_actuator.cpp # 执行器实现
│   └── motor_utils.cpp      # 工具函数
├── include/motorevo/        # 公共头文件
│   ├── motor_ctrl.h         # 电机控制接口
│   ├── motorevo_actuator.h  # 执行器接口
│   ├── motor_def.h          # 数据结构定义
│   └── motor_log.h          # 日志接口
├── examples/                # 使用示例
├── tests/                   # 测试套件
└── CMakeLists.txt           # 构建配置
```

## 功能特性

- **多种控制模式**: 支持位置-力矩混合（PTM）、速度、力矩控制
- **多 CAN 总线管理**: 支持同时管理多个 CAN 总线
- **实时控制**: 线程安全控制循环，可配置频率（默认 250Hz）
- **零点校准**: 自动化电机零点校准和偏移管理
- **故障检测**: 全面的电机故障检测和错误代码管理
- **高性能**: O(1) 索引访问、线程安全设计
- **配置驱动**: 基于 YAML 的设备和总线配置

## 快速开始

### 1. 安装依赖

```bash
sudo apt-get install libyaml-cpp-dev pkg-config libgtest-dev
```

### 2. 构建依赖

由于 motorevo_controller 依赖 canbus_sdk，需要先编译 canbus_sdk：

```bash
# 编译canbus_sdk
cd ../canbus_sdk
mkdir build && cd build
cmake ..
make
```

### 3. 构建 motorevo_controller

```bash
cd motorevo_controller
mkdir build && cd build
cmake ..
make
```

## 示例和测试

### 运行示例程序

在 `examples/` 目录中提供了可运行的示例程序：

```bash
cd build
cmake .. && make

# 运行简单执行器示例 - 基本的初始化和状态读取
./examples/motorevo_actuator_simple_test

# 运行电机控制测试 - 单个电机的控制功能演示
./examples/motor_ctrl_simple_test

# 运行多电机控制示例 - 多电机协调控制演示
./examples/motor_control_test

```

#### 示例程序说明

这3个示例展示了不同维度的API调用方式：

**1. motorevo_actuator_simple_test** - 高层接口调用
- 演示基本的执行器初始化流程
- 展示电机状态读取（位置、速度、力矩）
- 包含错误处理和资源清理
- 适用场景：快速验证电机连接和基本功能

**2. motor_ctrl_simple_test** - 中层接口调用
- 演示单个电机的详细控制功能
- 包含位置控制、速度控制、力矩控制模式
- 展示电机使能/失能操作
- 适用场景：学习电机控制 API 使用

**3. motor_control_test** - 底层接口调用
- 演示多电机协调控制
- 包含同步控制多个电机的示例
- 展示复杂控制模式（PTM 混合控制）
- 适用场景：机器人关节控制等实际应用


### 运行测试程序

`tests/` 目录中的综合测试套件：

```bash
cd build
cmake .. && make

# 运行电机控制测试 - 核心控制功能验证
./tests/motor_ctrl_test

# 运行插值测试 - 电机轨迹插值算法测试
./tests/interpolation_test

# 运行量化分析测试 - 数据精度和范围测试
./tests/test_quantization_analysis

# 运行转换调试测试 - 数据转换函数测试
./tests/test_conversion_debug
```

#### 测试程序说明

**1. motor_ctrl_test**
- 验证电机控制器的核心功能
- 测试电机使能/失能操作
- 验证不同控制模式的正确性
- 包含边界条件和异常处理测试

**2. interpolation_test**
- 测试电机轨迹插值算法
- 验证位置、速度、加速度插值
- 测试插值精度和稳定性
- 适用场景：运动控制算法开发

**3. test_quantization_analysis**
- 分析数据量化和精度损失
- 测试浮点数到整数的转换
- 验证量化误差范围
- 适用场景：控制算法精度优化

**4. test_conversion_debug**
- 调试数据转换函数
- 测试单位转换和坐标系变换
- 验证数据一致性
- 适用场景：控制系统调试

