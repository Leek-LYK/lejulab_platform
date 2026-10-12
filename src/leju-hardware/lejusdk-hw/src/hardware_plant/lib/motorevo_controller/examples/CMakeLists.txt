cmake_minimum_required(VERSION 3.8)

# 设置示例程序源文件
set(EXAMPLE_SOURCES
    motor_ctrl_simple_test.cpp
    motor_control_test.cpp
    motorevo_actuator_simple_test.cpp
    motorevo_tool.cpp
    canfd_motor_test.cpp
    arm_sine_test.cpp
    canfd_sine_sweep_test.cpp
    canfd_breakin.cpp
)

# 创建示例可执行文件
foreach(source ${EXAMPLE_SOURCES})
    get_filename_component(exe_name ${source} NAME_WE)
    add_executable(${exe_name} ${source})

    # 包含头文件目录
    target_include_directories(${exe_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
    )

    # 链接库
    target_link_libraries(${exe_name}
        motorevo_controller
        stdc++
        pthread
        canbus_sdk
        m
    )

    # 安装可执行文件
    install(TARGETS ${exe_name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
endforeach()