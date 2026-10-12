# Calibration tools CMakeLists.txt

# Calibration tool name
set(CALI_TOOL_NAME "hw_cali_tool")

# Calibration tool source files
set(CALI_TOOL_SOURCES
    hw_cali_tool.cc
    humanoid_cali.cpp
)

# Find required packages
find_package(gflags REQUIRED)

# Include parent directories for headers
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/hardware_plant
)

# Add calibration tool executable
add_executable(${CALI_TOOL_NAME} ${CALI_TOOL_SOURCES})

# Set C++ standard
set_property(TARGET ${CALI_TOOL_NAME} PROPERTY CXX_STANDARD 17)

# Link libraries
target_link_libraries(${CALI_TOOL_NAME}
    PRIVATE
        gflags
        # Link to hardware plant library if needed
        hardware_plant
        # Add other dependencies as needed
)

# Installation
install(TARGETS ${CALI_TOOL_NAME}
    RUNTIME DESTINATION bin
)