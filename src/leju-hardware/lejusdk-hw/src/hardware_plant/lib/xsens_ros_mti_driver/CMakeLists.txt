cmake_minimum_required(VERSION 3.0.0)
project(xsens_mti_driver)

set(CMAKE_BUILD_TYPE "Release")

include_directories( ${EIGEN_INCLUDE_DIRS} )
add_subdirectory(lcmtypes)

find_package( Eigen3 REQUIRED )
# 用 find_package 提供的变量,勿硬编码 /usr/include/eigen3(交叉编译时显式 -I 绝对路径
# 不被 sysroot 重定向)。
include_directories( ${EIGEN3_INCLUDE_DIR} )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  lib/xspublic
)
set(CMAKE_VERBOSE_MAKEFILE on)

message(STATUS "IMU  PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

add_custom_target(xs_public_targets
 	COMMAND echo "Building xspublic targets"
    COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
	COMMAND echo "Built xspublic targets done"
)
set(CMAKE_VERBOSE_MAKEFILE off)

aux_source_directory(./src DIR_SRCS)
list(REMOVE_ITEM DIR_SRCS ./src/imu_test.cpp)


add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})
add_dependencies(${PROJECT_NAME} xs_public_targets)
link_directories(
lib/xspublic/xscontroller 
lib/xspublic/xscommon 
lib/xspublic/xstypes 
)

target_link_libraries(
	${PROJECT_NAME}
	xscontroller
	xscommon
	xstypes
	pthread
	dl
	lcm
	xsens-lcm-types-lib
)
# target_link_directories(${PROJECT_NAME} PUBLIC lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)
# target_include_directories(${PROJECT_NAME} PUBLIC ./ ./src ./lib/xspublic)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic>
  $<INSTALL_INTERFACE:include/xsens_ros_mti_driver/src>
  $<INSTALL_INTERFACE:include/xsens_ros_mti_driver/lib/xspublic>
)

target_link_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes>
)


aux_source_directory(./src SRC_FILES)
list(REMOVE_ITEM SRC_FILES ./src/main.cpp)
add_executable(imu_test ${SRC_FILES})
target_compile_definitions(imu_test PUBLIC IMU_TEST_BUILD)
# link_directories(lib/xspublic/xscontroller lib/xspublic/xscommon lib/xspublic/xstypes)
target_link_libraries(
	imu_test
	xscontroller
	xscommon
	xstypes
	pthread
	dl
	lcm
	xsens-lcm-types-lib
)
target_include_directories(imu_test PUBLIC ./)
add_dependencies(imu_test xs_public_targets)

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include(GNUInstallDirs)

install(DIRECTORY lib/xspublic src
	DESTINATION share/hardware_plant/lib/xsens_ros_mti_driver
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY lcmtypes param urdf
	DESTINATION share/hardware_plant/lib/xsens_ros_mti_driver
)

install(DIRECTORY src
	DESTINATION include/xsens_ros_mti_driver
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY lib/xspublic
	DESTINATION include/xsens_ros_mti_driver/lib
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY lcmtypes
	DESTINATION include/xsens_ros_mti_driver
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a
  DESTINATION lib
)