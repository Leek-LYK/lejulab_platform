cmake_minimum_required(VERSION 3.0.2)
project(hipnuc_imu)

include_directories( ${EIGEN_INCLUDE_DIRS} )

find_package( Eigen3 REQUIRED )
# 用 find_package 提供的变量,勿硬编码 /usr/include/eigen3(交叉编译时显式 -I 绝对路径
# 不被 sysroot 重定向)。EIGEN3_INCLUDE_DIR 原生=/usr/include/eigen3,交叉=sysroot 内路径。
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_library(hipnuc_imu_receiver src/hipnuc_imu_receiver.cpp src/hipnuc_dec.c)
add_executable(hipimu_test src/hipimu_test.cpp)
target_link_libraries(hipimu_test hipnuc_imu_receiver pthread)
target_include_directories(hipnuc_imu_receiver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/hipnuc_imu/src>
  $<INSTALL_INTERFACE:/usr/include/eigen3>
)


install(TARGETS hipnuc_imu_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES src/hipnuc.h 
  src/hipnuc_imu_receiver.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/hardware_plant/lib/hipnuc_imu
)


