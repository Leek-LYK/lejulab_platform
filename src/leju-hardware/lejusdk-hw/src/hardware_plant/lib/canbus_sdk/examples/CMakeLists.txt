# 添加revo2_hand示例
add_subdirectory(revo2_hand)

# 添加配置文件写入示例
add_executable(write_config_example write_config_example.cpp)

# 链接库
target_link_libraries(write_config_example
    canbus_sdk
)

# 设置编译选项
target_compile_options(write_config_example PRIVATE -Wall -Wextra -g)

# 添加设备更新示例
add_executable(update_device_example update_device_example.cpp)

# 链接库
target_link_libraries(update_device_example
    canbus_sdk
)

# 设置编译选项
target_compile_options(update_device_example PRIVATE -Wall -Wextra -g)