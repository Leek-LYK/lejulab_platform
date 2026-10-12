# Select architecture-specific 3rd_party directory for MuJoCo
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(MUJOCO_3RD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/aarch64")
else()
    set(MUJOCO_3RD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/x86_64")
endif()

if(NOT EXISTS "${MUJOCO_3RD_PARTY_DIR}")
    message(FATAL_ERROR "3rd_party directory for ${CMAKE_SYSTEM_PROCESSOR} not found: ${MUJOCO_3RD_PARTY_DIR}")
endif()

message(STATUS "Using MuJoCo 3rd_party for: ${CMAKE_SYSTEM_PROCESSOR} -> ${MUJOCO_3RD_PARTY_DIR}")
