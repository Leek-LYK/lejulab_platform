cmake_minimum_required(VERSION 3.16)

# Default to GLVND if available.
cmake_policy(SET CMP0072 NEW)

project(leju-hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package(
    CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils
  )
endif()

# Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

###########
## Build ##
###########

set(LEJUSDK_HW_DIR ./lejusdk-hw)
set(TESTS_DIR ./tests)

add_subdirectory(${LEJUSDK_HW_DIR} "lejusdk-hw")
add_subdirectory(${TESTS_DIR} "tests")

include_directories(
    include
    ${LEJUSDK_HW_DIR}/include
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

file(GLOB HARDWARE_SOURCES "src/*.cc" "src/*.cpp")

add_library(${PROJECT_NAME}_lib STATIC
    ${HARDWARE_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_lib
    lejusdk_hw
    pthread
    Threads::Threads
)
if(_USING_CATKIN)
  target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME}_lib lejusdk-lowlevel)
endif()

add_executable(${PROJECT_NAME} src/hardware_node.cpp)

target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)
if(_USING_CATKIN)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} lejusdk-lowlevel)
endif()

#############
## Install ##
#############

if(_USING_CATKIN)
  install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

install(DIRECTORY launch/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/launch
    PATTERN "*.launch"
)

install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
)
