cmake_minimum_required(VERSION 3.16)
project(lejusdk-lowlevel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Set default build type to Release (CMake default)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-dds-idl lejusdk-topic-pubsub lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})

find_package(Eigen3 REQUIRED)

# iceoryx + CycloneDDS (SHM enabled)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

# CycloneDDS 依赖 iceoryx，需要先定义 iceoryx targets (静态库)
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(Threads REQUIRED)

if(_USING_CATKIN)
  # catkin package configuration
  # Note: CycloneDDS is statically linked, so we don't export it
  catkin_package(
      INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
      LIBRARIES ${PROJECT_NAME} Eigen3::Eigen
      CATKIN_DEPENDS lejusdk-dds-idl lejusdk-topic-pubsub
      DEPENDS
  )
endif()

# 保留原有的include_directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/src/communication/dds
    ${CYCLONEDDS_CXX_DIR}/include
    ${ICEORYX_DIR}/include/iceoryx/v
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# Source files
set(SOURCES
    src/communication/communication.cpp
    src/communication/dds/dds_communicator.cpp
    src/communication/dds/dds_data_converter.cpp
    src/global_robot.cpp
    src/robot_api/robot_base.cpp
    src/robot_api/roban_humanoid.cpp
    src/robot_api/kuavo_humanoid.cpp
    src/data_types_io.cpp
    src/robot_transmission/robot_transmission.cpp
    src/robot_transmission/robot_transmission.impl.cpp
    src/robot_transmission/roban_ankle_solver.cpp
    src/logger/topic_logger.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${HUMANOID_PROJECT_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)
if(_USING_CATKIN)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} lejusdk-dds-idl lejusdk-topic-pubsub lejusdk-utils)
endif()

#############
## Install ##
#############

if(_USING_CATKIN)
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  # Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )

  # Install iox-roudi binary (for lejulab setup_cyclonedds_config.sh)
  install(PROGRAMS ${ICEORYX_DIR}/bin/iox-roudi
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
else()
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )

  # Install iox-roudi binary (for lejulab setup_cyclonedds_config.sh)
  # 纯 CMake 分支同样需要 iox-roudi（aarch64 发布走纯 CMake，缺它会导致
  # CycloneDDS SHM 部署失败；对齐 catkin 分支行为）
  install(PROGRAMS ${ICEORYX_DIR}/bin/iox-roudi
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

# 添加测试子目录
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
