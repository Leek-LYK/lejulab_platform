# ==============================================================================
# Project Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.10)
project(hardware_plant VERSION 1.0.0)

# ==============================================================================
# Build Settings
# ==============================================================================
# Enforce modern C++ standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# ==============================================================================
# Dependencies
# ==============================================================================
# Set include and library paths for local dependencies
set(kuavo_common_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../kuavo_common/include)
set(kuavo_common_LIBRARIES kuavo_common)
set(kuavo_solver_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../kuavo_solver/include)
set(kuavo_solver_LIBRARIES kuavo_solver)

if (CATKIN_DEVEL_PREFIX)
    find_package(catkin REQUIRED COMPONENTS lejusdk-utils)
else()
    # For non-catkin builds, assume lejusdk-utils is in the parent include dir
    set(lejusdk-utils_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../../../../../include)
    set(lejusdk-utils_LIBRARIES lejusdk-utils)
endif()

message("Using kuavo_common from ${kuavo_common_INCLUDE_DIRS}")
message("Using kuavo_solver from ${kuavo_solver_INCLUDE_DIRS}")
# ==============================================================================
# Subdirectories
# ==============================================================================
# Add all required library subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/Math)
if (CATKIN_DEVEL_PREFIX)
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/EC_Master)
endif()
if (CATKIN_DEVEL_PREFIX)
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver)
endif()
# add_subdirectory(${PROJECT_SOURCE_DIR}/lib/ruiwo_controller)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/leju_claw_driver)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/hipnuc_imu)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/dexhand_sdk)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/canbus_sdk)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/motorevo_controller)
# ==============================================================================
# Library Target
# ==============================================================================
# Define source files for the library
set(LIBRARY_SOURCES
    src/actuators_interface.cpp
    src/Kalman.cpp
    src/kalman_estimate.cc
    src/hardware_plant.cc
    src/motor_status_manager.cpp
    # ee
    src/end_effector/revo2_hand_controller.cpp
    src/end_effector/touch_hand_controller.cpp
    src/end_effector/lejuclaw_controller.cc
    # utils
    src/utils/config_utils.cpp
    src/utils/path_utils.cpp
    src/utils/file.cpp
    # motor firmware preflight
    src/motor_preflight/param_protocol.cpp
    src/motor_preflight/preflight_config.cpp
    src/motor_preflight/motor_preflight.cpp
    src/motor_preflight/socketcan_param_transport.cpp
)

# Create shared library
add_library(${PROJECT_NAME} STATIC ${LIBRARY_SOURCES})

# Add include directories
target_include_directories(hardware_plant PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../../../../include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${kuavo_solver_INCLUDE_DIRS}>
  $<BUILD_INTERFACE:${lejusdk-utils_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/hardware_plant>
  $<INSTALL_INTERFACE:include/kuavo_common>
  $<INSTALL_INTERFACE:include/lejusdk-utils>
)

# Add kuavo_solver as PRIVATE include only (not exported)
target_include_directories(hardware_plant PRIVATE
  ${kuavo_solver_INCLUDE_DIRS}
)

# Initialize hardware include directories
set(HARDWARE_INCLUDE_DIRS
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../lejusdk-hw/include>
    $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include/hardware_plant>
    $<INSTALL_INTERFACE:include/kuavo_common>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx>
    $<INSTALL_INTERFACE:include/ruiwo_controller_cxx>
)

target_include_directories(${PROJECT_NAME} PUBLIC ${HARDWARE_INCLUDE_DIRS})

# Link dependencies
set(HARDWARE_LIBRARIES
    mathtools_hw
    lejuclaw
    hipnuc_imu_receiver
    dexhand_sdk
    canbus_sdk
    motorevo_controller
    ruiwo_actuatorCXXLib
    stdc++fs)
if (CATKIN_DEVEL_PREFIX)
    list(APPEND HARDWARE_LIBRARIES xsens_mti_driver ec_master_main ec_app)
endif()

# Change PUBLIC to PRIVATE for internal dependencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${HARDWARE_LIBRARIES}
        ${kuavo_common_LIBRARIES}
        ${kuavo_solver_LIBRARIES}
)

if (CATKIN_DEVEL_PREFIX)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            ${catkin_LIBRARIES}
    )
else()
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            ${lejusdk-utils_LIBRARIES}
    )
endif()

if (CATKIN_DEVEL_PREFIX)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ENABLE_ECMASTER ENABLE_XSENS)
endif()

# ==============================================================================
# Test Executables
# ==============================================================================
# Leju claw controller test
add_executable(lejuclaw_controller_test
    ${PROJECT_SOURCE_DIR}/src/tests/lejuclaw_controller_test.cc
)
target_link_libraries(lejuclaw_controller_test
    PUBLIC
        ${PROJECT_NAME}
)
target_include_directories(lejuclaw_controller_test
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

add_executable(lejuclaw_ctrl_test src/tests/lejuclaw_controller_test.cc)
target_link_libraries(lejuclaw_ctrl_test PUBLIC ${PROJECT_NAME})
target_include_directories(lejuclaw_ctrl_test PRIVATE
  ${PROJECT_SOURCE_DIR}/src
)
install(TARGETS lejuclaw_ctrl_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


# Include IMU test script subdirectory (with ROS support)
if(CATKIN_DEVEL_PREFIX)
    # add_subdirectory(${PROJECT_SOURCE_DIR}/src/tests)
    message(STATUS "Adding IMU test script subdirectory with ROS support")
else()
    message(STATUS "Skipping IMU test script - not in catkin environment")
endif()

# motor_preflight 单元测试 (纯逻辑, 不依赖硬件)
if(BUILD_TESTS)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(test_motor_preflight src/tests/test_motor_preflight.cpp)
        target_link_libraries(test_motor_preflight PRIVATE
            ${PROJECT_NAME}
            GTest::gtest
            GTest::gtest_main
        )
        target_include_directories(test_motor_preflight PRIVATE ${PROJECT_SOURCE_DIR}/src)
        add_test(NAME test_motor_preflight COMMAND test_motor_preflight)
    else()
        message(STATUS "GTest not found, skipping test_motor_preflight")
    endif()
endif()


# ==============================================================================
# Installation
# ==============================================================================

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/tests/
#     DESTINATION tests
# )
# Install all the subdirectory libraries
install(TARGETS
    mathtools_hw
    lejuclaw
    hipnuc_imu_receiver
    dexhand_sdk
    canbus_sdk
    motorevo_controller
    ruiwo_actuatorCXXLib
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
if (CATKIN_DEVEL_PREFIX)
    install(TARGETS
        xsens_mti_driver
        xsens-lcm-types-lib
        ec_master_main
        ec_app
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
endif()

# Install public headers
install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

set(hardware_plant_INCLUDE_DIRS ${HARDWARE_INCLUDE_DIRS})
set(hardware_plant_LIBRARIES ${PROJECT_NAME})

# Generate the config file
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
# Install the config files
install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the export set
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)
