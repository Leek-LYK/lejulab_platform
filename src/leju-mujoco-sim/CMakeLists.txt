cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0072 NEW)
project(leju-mujoco-sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})
if(_USING_CATKIN)
  catkin_package()
endif()

find_package(Eigen3 REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/arch_select.cmake)
find_package(mujoco REQUIRED HINTS ${MUJOCO_3RD_PARTY_DIR}/mujoco-3.2.0)
find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# DDS 3rd_party paths for find_package
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk/3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(OpenCV QUIET COMPONENTS core imgproc highgui)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk/3rd_party/x86_64/cyclonedds-0.10.2/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk/3rd_party/x86_64/iceoryx/include/iceoryx/v
    include
    src/simulate
    ${CMAKE_CURRENT_SOURCE_DIR}/../leju-hardware/lejusdk-hw/src/kuavo_solver/include
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# Resolve the sibling hardware library to an absolute path.  In the unified
# workspace CMake build the binary directory may be build/src, while the
# hardware artifacts are emitted under build/leju-hardware; keeping this as a
# relative link path makes make look for build/src/leju-hardware instead.
if(EXISTS "${CMAKE_BINARY_DIR}/leju-hardware/lib")
  set(LEJU_HARDWARE_LIB_DIR "${CMAKE_BINARY_DIR}/leju-hardware/lib")
else()
  get_filename_component(LEJU_HARDWARE_LIB_DIR
    "${CMAKE_BINARY_DIR}/../leju-hardware/lib" ABSOLUTE)
endif()
if(_USING_CATKIN)
  link_directories(${LEJU_HARDWARE_LIB_DIR})
endif()

file(GLOB SIMULATE_SOURCES "src/simulate/*.cc" "src/simulate/*.cpp")
add_library(libsimulate STATIC ${SIMULATE_SOURCES})
target_link_libraries(libsimulate mujoco::mujoco Threads::Threads glfw)
add_executable(simulate src/main.cc)
target_link_libraries(simulate libsimulate mujoco::mujoco Threads::Threads glfw)

add_executable(${PROJECT_NAME} src/leju_mujoco_sim.cc)
target_link_libraries(${PROJECT_NAME} libsimulate mujoco::mujoco Threads::Threads glfw yaml-cpp CycloneDDS-CXX::ddscxx iceoryx_posh::iceoryx_posh)
if(_USING_CATKIN)
  if(TARGET kuavo_solver)
    # In a unified workspace, linking the CMake target both uses its actual
    # output directory and makes sure it is built before the simulator.
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kuavo_solver)
  else()
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
      "${LEJU_HARDWARE_LIB_DIR}/libkuavo_solver.so")
  endif()
else()
  target_link_libraries(${PROJECT_NAME} lejusdk-lowlevel kuavo_solver)
endif()
if(OpenCV_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE LEJU_HAS_OPENCV=1)
endif()

if(_USING_CATKIN)
  install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(FILES ${MUJOCO_3RD_PARTY_DIR}/mujoco-3.2.0/lib/libmujoco.so.3.2.0 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  install(CODE "execute_process(COMMAND ln -sf libmujoco.so.3.2.0 libmujoco.so.3 WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})")
  install(CODE "execute_process(COMMAND ln -sf libmujoco.so.3.2.0 libmujoco.so WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})")
else()
  install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(FILES ${MUJOCO_3RD_PARTY_DIR}/mujoco-3.2.0/lib/libmujoco.so.3.2.0
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
  install(CODE "execute_process(COMMAND ln -sf libmujoco.so.3.2.0 libmujoco.so.3
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})")
  install(CODE "execute_process(COMMAND ln -sf libmujoco.so.3.2.0 libmujoco.so
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})")
endif()
