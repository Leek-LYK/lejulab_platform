# 电机固件预检独立 CLI (运维排查用, 详见 motor_preflight_check.cpp 头注释)
add_executable(motor_preflight_check motor_preflight_check.cpp)
set_property(TARGET motor_preflight_check PROPERTY CXX_STANDARD 17)
target_link_libraries(motor_preflight_check PRIVATE hardware_plant)
target_include_directories(motor_preflight_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/hardware_plant/src
)
install(TARGETS motor_preflight_check RUNTIME DESTINATION bin)
