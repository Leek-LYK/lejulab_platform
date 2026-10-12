cmake_minimum_required(VERSION 3.16)
project(lejusdk-recorder VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package()
endif()

# Build options
option(BUILD_RECORDER "Build the DDS recorder" ON)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Thirdparty directories
set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
set(LEJUSDK_3RD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)

# iceoryx + CycloneDDS (SHM enabled)
set(THIRD_PARTY_DIR ${LEJUSDK_3RD_PARTY})
include(${LEJUSDK_3RD_PARTY}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

# CycloneDDS depends on iceoryx
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(Threads REQUIRED)
get_target_property(_ddsc_inc CycloneDDS::ddsc INTERFACE_INCLUDE_DIRECTORIES)
list(GET _ddsc_inc 0 _first_inc)
message(STATUS "Found CycloneDDS with headers at ${_first_inc}")

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# MCAP library
add_library(mcap STATIC ${THIRDPARTY_DIR}/mcap/mcap_impl.cpp)
target_include_directories(mcap PUBLIC ${THIRDPARTY_DIR}/mcap/include)
target_compile_definitions(mcap PUBLIC MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)

# nlohmann-json (header-only)
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE ${THIRDPARTY_DIR}/nlohmann-json/include)

# optionparser (header-only)
add_library(optionparser INTERFACE)
target_include_directories(optionparser INTERFACE ${THIRDPARTY_DIR}/optionparser/include)

# spdlog (header-only)
add_library(spdlog INTERFACE)
target_include_directories(spdlog INTERFACE ${THIRDPARTY_DIR}/spdlog/include)

# Add subdirectories
add_subdirectory(common)
add_subdirectory(cyclone_dds)

if(BUILD_RECORDER)
    add_subdirectory(recorder)
endif()

#############
## Install ##
#############
if(_USING_CATKIN)
  install(TARGETS lejusdk_recorder
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS lejusdk_recorder
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

# Summary
message(STATUS "")
message(STATUS "========================================")
message(STATUS "lejusdk-recorder Configuration")
message(STATUS "========================================")
message(STATUS "Version:        ${PROJECT_VERSION}")
message(STATUS "Build Recorder: ${BUILD_RECORDER}")
message(STATUS "========================================")
message(STATUS "")
