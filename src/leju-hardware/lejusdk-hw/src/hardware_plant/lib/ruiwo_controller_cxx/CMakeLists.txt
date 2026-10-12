cmake_minimum_required(VERSION 3.12)
project(ruiwo_cxx_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(GNUInstallDirs)

set(BMAPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../BMAPI")

# Architecture-aware BMAPI handling
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(HAS_BMAPI OFF)
else()
    set(HAS_BMAPI ON)
    set(DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ruiwo_controller/python-can-3.3.4/can/interfaces/bmcan")
    link_directories("${DEPS_DIR}")
endif()

SET(LIB_LIST yaml-cpp stdc++fs usb-1.0 stdc++ pthread)
if(HAS_BMAPI)
    list(APPEND LIB_LIST bmapi64)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PROJECT_SOURCE_DIR}/src/
  ${BMAPI_DIR}/include
)

file(GLOB_RECURSE SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
  ${PROJECT_SOURCE_DIR}/src/*.c
)

set(BMAPI_STUB_SOURCES "")
if(NOT HAS_BMAPI)
    set(BMAPI_STUB_SOURCES ${BMAPI_DIR}/src/bmapi_stubs.c)
endif()

set(ruiwo_lib_name ruiwo_actuatorCXXLib)
add_library(${ruiwo_lib_name} SHARED ${SRC_FILE} ${PROJECT_SOURCE_DIR}/../leju_claw_driver/src/ruiwoSDK.c ${BMAPI_STUB_SOURCES})
target_link_libraries(${ruiwo_lib_name} PUBLIC ${LIB_LIST})
target_include_directories(${ruiwo_lib_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../leju_claw_driver/include>
  $<INSTALL_INTERFACE:include/ruiwo_controller_cxx>
  $<INSTALL_INTERFACE:include/ruiwo_controller_cxx/include>
  $<INSTALL_INTERFACE:include/leju_claw_driver>
)
if(HAS_BMAPI)
    target_link_directories(${ruiwo_lib_name} PUBLIC
      $<BUILD_INTERFACE:${DEPS_DIR}>
    )
endif()

if(HAS_BMAPI)
    add_executable(ruiwo_cxx_test ${PROJECT_SOURCE_DIR}/test/ruiwo_test.cpp)
    target_link_libraries(ruiwo_cxx_test ${ruiwo_lib_name})

    add_executable(setpos_test ${PROJECT_SOURCE_DIR}/test/setpos_test.cc)
    target_link_libraries(setpos_test ${ruiwo_lib_name})
endif()

install(TARGETS ${ruiwo_lib_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/ruiwo_controller_cxx
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
