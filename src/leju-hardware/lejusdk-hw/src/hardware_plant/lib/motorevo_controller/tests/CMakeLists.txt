cmake_minimum_required(VERSION 3.8)

# 查找GTest包
find_package(GTest REQUIRED)

# 设置测试源文件
set(TEST_SOURCES
    motor_ctrl_test.cpp
)

# 创建插值测试源文件
set(INTERPOLATION_TEST_SOURCES
    interpolation_test.cpp
)

# 创建测试可执行文件
add_executable(motor_ctrl_test ${TEST_SOURCES})

# 创建插值测试可执行文件
add_executable(interpolation_test ${INTERPOLATION_TEST_SOURCES})

# 包含头文件目录
target_include_directories(motor_ctrl_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)

# 包含插值测试头文件目录
target_include_directories(interpolation_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)

# 链接库
target_link_libraries(motor_ctrl_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
    canbus_sdk
    motorevo_controller
)

# 链接插值测试库
target_link_libraries(interpolation_test
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    pthread
    canbus_sdk
)

# 添加测试
gtest_discover_tests(motor_ctrl_test)

# 添加转换调试测试程序
add_executable(test_conversion_debug test_conversion_debug.cpp)
target_include_directories(test_conversion_debug PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)
target_link_libraries(test_conversion_debug motorevo_controller stdc++)

# 下肢电机故障码链路 vcan 集成测试 (配合 lowerlimb_fault_vcan_test.sh 手动执行)
add_executable(test_lowerlimb_fault_vcan test_lowerlimb_fault_vcan.cpp)
target_include_directories(test_lowerlimb_fault_vcan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)
target_link_libraries(test_lowerlimb_fault_vcan
    pthread
    canbus_sdk
    motorevo_controller
)

# 添加量化误差分析程序
add_executable(test_quantization_analysis test_quantization_analysis.cpp)
target_include_directories(test_quantization_analysis PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)
target_link_libraries(test_quantization_analysis motorevo_controller stdc++)