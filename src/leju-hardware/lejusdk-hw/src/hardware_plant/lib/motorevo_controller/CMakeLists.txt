cmake_minimum_required (VERSION 3.8)
project(motorevo_controller VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(GTest QUIET)

# 设置 canbus_sdk 路径
set(CANBUS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk")

include_directories(
  ${GTEST_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../canbus_sdk/include
)

### 显式指定源文件
set(SRC_FILE
  ${PROJECT_SOURCE_DIR}/src/motor_ctrl.cpp
  ${PROJECT_SOURCE_DIR}/src/motor_utils.cpp
  ${PROJECT_SOURCE_DIR}/src/motorevo_actuator.cpp
)

# 编译选项
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "release"))
  # Release配置
else()
  add_definitions(-fPIC -g)
endif()

# 生成动态库
add_library(${PROJECT_NAME} SHARED ${SRC_FILE})

# 设置包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/motorevo_controller>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../../include>
)

# 链接canbus_sdk和标准库
target_link_libraries(${PROJECT_NAME} PRIVATE canbus_sdk stdc++ pthread)

# 添加测试子目录
if(GTest_FOUND AND BUILD_TESTING)
    add_subdirectory(tests)
endif()

# 添加示例子目录
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# 安装步骤
# 安装动态库 - 不导出以避免依赖冲突
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)