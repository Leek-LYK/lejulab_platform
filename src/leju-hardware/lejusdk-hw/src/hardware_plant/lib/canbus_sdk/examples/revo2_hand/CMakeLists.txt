cmake_minimum_required(VERSION 3.10)
project(revo2_hand_example)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找必需的包
find_package(Threads REQUIRED)

# 添加头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/dist/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

# 添加库文件目录
link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/dist/shared/linux
    ${CMAKE_CURRENT_SOURCE_DIR}/../../build
)

# 添加可执行文件
add_executable(revo2_can_customed revo2_can_customed.cpp)
add_executable(dual_revo2_can_customed dual_revo2_can_customed.cpp)

# 链接库文件
target_link_libraries(revo2_can_customed
    bc_stark_sdk
    canbus_sdk
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(dual_revo2_can_customed
    bc_stark_sdk
    canbus_sdk
    ${CMAKE_THREAD_LIBS_INIT}
)

# 设置RPATH以查找共享库
set_target_properties(revo2_can_customed PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/dist/shared/linux:${CMAKE_CURRENT_SOURCE_DIR}/../../build"
    BUILD_WITH_INSTALL_RPATH TRUE
)

set_target_properties(dual_revo2_can_customed PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/dist/shared/linux:${CMAKE_CURRENT_SOURCE_DIR}/../../build"
    BUILD_WITH_INSTALL_RPATH TRUE
)