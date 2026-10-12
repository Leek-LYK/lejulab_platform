cmake_minimum_required(VERSION 3.15)
project(canbus_sdk VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g")

include(GNUInstallDirs)

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")
set(LEJU_CAN_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../LEJU_CAN_API")

# Architecture-aware BMAPI handling
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(HAS_BMAPI OFF)
else()
    set(HAS_BMAPI ON)
    set(DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan")
endif()

set(LIB_LIST yaml-cpp stdc++fs usb-1.0 stdc++ pthread)
if(HAS_BMAPI)
    list(APPEND LIB_LIST bmapi64)
endif()

# Source files
set(CANBUS_SOURCES
    src/canbus_base.cpp
    src/leju_canbus_wrapper.cpp
    src/socketcan_wrapper.cpp
    src/bus_control_block.cpp
    src/canbus_ctrl.cpp
    src/canbus_sdk.cpp
    src/config_parser.cpp
    ${LEJU_CAN_API_DIR}/CANFD_leju.cpp
)
if(HAS_BMAPI)
    list(APPEND CANBUS_SOURCES src/bm_canbus_wrapper.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${CANBUS_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${YAML_CPP_INCLUDE_DIRS}
        ${USB1_INCLUDE_DIRS}
    PRIVATE
        ${BMAPI_DIR}/include
        ${LEJU_CAN_API_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${LIB_LIST})
if(HAS_BMAPI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_BMAPI)
    target_link_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${DEPS_DIR}>
    )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib"
    BUILD_WITH_INSTALL_RPATH FALSE
)

# Testing — respect parent BUILD_TESTS option
option(ENABLE_TESTS "Build tests" OFF)
if(ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

option(ENABLE_EXAMPLES "Build examples" OFF)
if(ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif()

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/canbus_sdkConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(TARGETS canbus_sdk
    EXPORT canbus_sdkTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT canbus_sdkTargets
    FILE canbus_sdkTargets.cmake
    NAMESPACE canbus_sdk::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canbus_sdk
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/canbus_sdkConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/canbus_sdk
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
