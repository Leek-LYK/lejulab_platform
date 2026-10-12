cmake_minimum_required(VERSION 3.16)
project(lejusdk-dds-rpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-fPIC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET)
set(_USING_CATKIN ${catkin_FOUND})

option(BUILD_DDS_RPC_EXAMPLES "Build DDS-RPC example applications" OFF)

# CycloneDDS
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
  include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(include)

file(GLOB IDL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src-generated/*.cpp")

add_library(${PROJECT_NAME} STATIC ${IDL_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rpc_idl
)

target_link_libraries(${PROJECT_NAME}
    pthread
    CycloneDDS-CXX::ddscxx
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(BUILD_DDS_RPC_EXAMPLES)
    add_executable(calculator_server examples/calculator_server.cpp)
    target_link_libraries(calculator_server PRIVATE ${PROJECT_NAME})
    add_executable(calculator_client examples/calculator_client.cpp)
    target_link_libraries(calculator_client PRIVATE ${PROJECT_NAME})
    if(_USING_CATKIN)
        install(TARGETS calculator_server calculator_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    else()
        install(TARGETS calculator_server calculator_client RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()

option(BUILD_DDS_RPC_TESTS "Build DDS-RPC unit tests" ON)
if(BUILD_DDS_RPC_TESTS)
    add_subdirectory(tests)
endif()
