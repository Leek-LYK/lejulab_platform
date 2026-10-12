cmake_minimum_required(VERSION 3.12)
project(lejuclaw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(GNUInstallDirs)

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")

# Architecture-aware BMAPI handling
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(HAS_BMAPI OFF)
else()
    set(HAS_BMAPI ON)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan/)
endif()

SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 stdc++ pthread)
if(HAS_BMAPI)
    list(APPEND LIB_LIST bmapi64)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/../motorevo_controller/include
  ${PROJECT_SOURCE_DIR}/../canbus_sdk/include
  ${BMAPI_DIR}/include
)

file(GLOB_RECURSE SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
  ${PROJECT_SOURCE_DIR}/src/*.c
)

add_library(${PROJECT_NAME} STATIC ${SRC_FILE})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${BMAPI_DIR}/include>
    $<INSTALL_INTERFACE:include/leju_claw_driver>
)
target_link_libraries(${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)
if(HAS_BMAPI)
    target_link_directories(${PROJECT_NAME}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan>
        $<INSTALL_INTERFACE:lib>
    )
endif()

if(HAS_BMAPI)
    add_executable(lejuclaw_test ${PROJECT_SOURCE_DIR}/test/lejuclaw_test.cpp)
    target_link_libraries(lejuclaw_test ${LIB_NAME} ${PROJECT_NAME} ${LIB_LIST})

    add_executable(lejuclaw_can_test_single_position ${PROJECT_SOURCE_DIR}/test/lejuclaw_can_test_single_position.cpp)
    target_include_directories(lejuclaw_can_test_single_position PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(lejuclaw_can_test_single_position ${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)

    add_executable(lejuclaw_can_test_vr_control ${PROJECT_SOURCE_DIR}/test/lejuclaw_can_test_vr_control.cpp)
    target_include_directories(lejuclaw_can_test_vr_control PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(lejuclaw_can_test_vr_control ${PROJECT_NAME} ${LIB_LIST} motorevo_controller canbus_sdk)
endif()

install(DIRECTORY ${BMAPI_DIR}/include
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/leju_claw_driver
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/leju_claw_driver
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

if(HAS_BMAPI)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        set(BMAPI_ARCH_DIR "arm64")
    else()
        set(BMAPI_ARCH_DIR "unix64")
    endif()
    install(FILES ${BMAPI_DIR}/bin/${BMAPI_ARCH_DIR}/release/libbmapi64.so
      DESTINATION lib
    )
endif()
