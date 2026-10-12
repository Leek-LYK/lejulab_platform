cmake_minimum_required(VERSION 3.16)
project(lejusdk_hw_examples)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 创建示例可执行文件
add_executable(hardware_interface_example hardware_interface_example.cpp)

# 链接库
target_link_libraries(hardware_interface_example
    PRIVATE
        lejusdk_hw
)

# 添加包含目录
target_include_directories(hardware_interface_example
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# 安装示例程序
install(TARGETS hardware_interface_example
    RUNTIME DESTINATION bin
)