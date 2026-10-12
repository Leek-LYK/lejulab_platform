cmake_minimum_required(VERSION 3.12)
project(dexhand_sdk)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 设置 canbus_sdk 路径
set(CANBUS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk")

# 添加lib目录 (新 SDK 内置 serialport，不再需要外部 libserialport)
SET(LIB_LIST bc_stark_sdk m pthread rt dl)

# 添加 canbus_sdk 依赖
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "release"))
  link_directories("${CANBUS_SDK_DIR}/build")
  # 添加依赖库
  list(APPEND LIB_LIST canbus_sdk)
else()
  add_definitions(-fPIC -g)
  link_directories("${CANBUS_SDK_DIR}/build")
  # 添加依赖库
  list(APPEND LIB_LIST canbus_sdk)
endif()
set(stark_modbus_sdk_DIR ${PROJECT_SOURCE_DIR}/modbus_sdk/)

# 新 SDK (v1.1.9) 的预编译 .so 按架构分目录存放:dist/shared/linux/{x86_64,aarch64}
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(STARK_ARCH "aarch64")
else()
  set(STARK_ARCH "x86_64")
endif()
set(stark_modbus_sdk_LIBDIR ${stark_modbus_sdk_DIR}/dist/shared/linux/${STARK_ARCH})

include_directories(
  ${stark_modbus_sdk_DIR}/dist/include/
  ${PROJECT_SOURCE_DIR}/
  ${CANBUS_SDK_DIR}/include
)

file(GLOB_RECURSE SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)
# protobuf_dexhand.cpp 已被 stark_dexhand.cpp 替代，不再编译
list(FILTER SRC_FILE EXCLUDE REGEX "protobuf_dexhand\\.cpp$")

# 生成库文件
add_library(${PROJECT_NAME} STATIC ${SRC_FILE})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${stark_modbus_sdk_DIR}/dist/include>
  $<BUILD_INTERFACE:${CANBUS_SDK_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:lib>
  $<BUILD_INTERFACE:${stark_modbus_sdk_LIBDIR}>
)
target_link_libraries(${PROJECT_NAME} ${LIB_LIST})


add_executable(dexhand_test tests/test.cc)
target_link_libraries(dexhand_test ${PROJECT_NAME})

install(DIRECTORY ${stark_modbus_sdk_DIR}/dist/include
  DESTINATION include/${PROJECT_NAME}
)

install(FILES
  touch_dexhand.h
  modbus.h
  stark_dexhand.h
  dexhand_base.h
  revo2_hand_can_customed.h
  DESTINATION include/${PROJECT_NAME}
)

install(FILES
  ${stark_modbus_sdk_LIBDIR}/libbc_stark_sdk.so
  DESTINATION lib
)

add_executable(revo2_can_test tests/revo2_can_test.cpp)
target_link_libraries(revo2_can_test ${PROJECT_NAME})

add_executable(revo1_can_test tests/revo1_can_test.cpp)
target_link_libraries(revo1_can_test ${PROJECT_NAME})

add_executable(stark_dexhand_test tests/protobuf_sdk_test.cc)
target_link_libraries(stark_dexhand_test ${PROJECT_NAME})

install(TARGETS dexhand_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS revo2_can_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS stark_dexhand_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
