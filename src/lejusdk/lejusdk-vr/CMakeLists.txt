cmake_minimum_required(VERSION 3.16)
project(lejusdk-vr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加调试符号（即使是 Release 模式）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    lejusdk-dds-rpc
    lejusdk-utils
)
set(_USING_CATKIN ${catkin_FOUND})

find_package(Eigen3 REQUIRED)

# iceoryx + CycloneDDS (SHM enabled)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

# CycloneDDS dependencies
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(Threads REQUIRED)

if(_USING_CATKIN)
  catkin_package(
      INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS lejusdk-dds-idl lejusdk-topic-pubsub lejusdk-dds-rpc lejusdk-utils
      DEPENDS EIGEN3
  )
endif()

include_directories(
    include
    src
    ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# Source files
set(SOURCES
    src/vr_base.cpp
    src/kuavo_vr_api.cpp
    src/roban_vr_api.cpp
    src/communication/vr_communication.cpp
    src/communication/vr_dds_communicator.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    src
)

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
  target_link_libraries(${PROJECT_NAME} lejusdk-dds-idl lejusdk-topic-pubsub lejusdk-dds-rpc lejusdk-utils)
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

  install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
      PATTERN ".svn" EXCLUDE
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
endif()

# Tests
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Examples
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
