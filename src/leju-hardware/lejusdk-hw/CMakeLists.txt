# ==============================================================================
# Project Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.16)
project(lejusdk_hw VERSION 1.0.0)

# ==============================================================================
# Build Options
# ==============================================================================
option(BUILD_CALIBRATION_TOOLS "Build calibration tools" ON)

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
# Source Files
# ==============================================================================
# Define source files for the library
file(GLOB_RECURSE LEJUSDK_HW_SOURCES
    "src/lejusdk_hw.cpp"
    "src/lejusdk_hw_impl.cc"
)

# ==============================================================================
# Dependencies
# ==============================================================================
if (CATKIN_DEVEL_PREFIX)
    find_package(catkin REQUIRED COMPONENTS lejusdk-utils)
    set(catkin_FOUND TRUE)
else()
    set(catkin_FOUND FALSE)
endif()

# ==============================================================================
# Subdirectories
# ==============================================================================
add_subdirectory(src/hardware_plant)
add_subdirectory(src/kuavo_common)
add_subdirectory(src/kuavo_solver)
add_subdirectory(examples)
# Build calibration tools only if option is enabled
if(BUILD_CALIBRATION_TOOLS)
    add_subdirectory(tools/calibration)
endif()
# 电机固件预检 CLI (motor_preflight_check)
add_subdirectory(tools/motor_preflight)

# ==============================================================================
# Library Target
# ==============================================================================
# Create shared library
add_library(${PROJECT_NAME} SHARED ${LEJUSDK_HW_SOURCES})

# Set C++ standard for the library
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${kuavo_common_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${kuavo_solver_INCLUDE_DIRS}>
)

if(catkin_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
endif()

# Link dependencies
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        hardware_plant
        pthread
        ${kuavo_common_LIBRARIES}
        ${kuavo_solver_LIBRARIES}
)

if(catkin_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
endif()

# ==============================================================================
# Installation
# ==============================================================================
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)