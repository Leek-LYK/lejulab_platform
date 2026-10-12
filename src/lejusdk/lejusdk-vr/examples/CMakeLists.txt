# Examples for lejusdk-vr

# 示例源文件（不含 controller_cli）
set(EXAMPLE_SOURCES
    vr_cmd_publisher.cpp
    vr_cmd_handler.cpp
    kuavo_arm_control_example.cpp
    roban_arm_control_example.cpp
    head_control_example.cpp
    waist_control_example.cpp
)

# 为每个示例文件创建可执行程序
foreach(SOURCE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})

    target_link_libraries(${EXEC_NAME} ${PROJECT_NAME})
    if(_USING_CATKIN)
      target_link_libraries(${EXEC_NAME} ${catkin_LIBRARIES})
    endif()
endforeach()

# controller_cli executable
add_executable(controller_cli controller_cli.cpp)
target_link_libraries(controller_cli ${PROJECT_NAME})
if(_USING_CATKIN)
  target_link_libraries(controller_cli ${catkin_LIBRARIES})
  install(TARGETS controller_cli RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
else()
  install(TARGETS controller_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

set(TACT_PLAYER_SOURCES
    tact_player/main.cpp
    tact_player/tact_parser.cpp
    tact_player/bezier_interpolator.cpp
    tact_player/trajectory_publisher.cpp
)

add_executable(tact_player_example ${TACT_PLAYER_SOURCES})

target_include_directories(tact_player_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tact_player
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lejusdk-recorder/thirdparty/nlohmann-json/include
)

target_link_libraries(tact_player_example ${PROJECT_NAME})
if(_USING_CATKIN)
  target_link_libraries(tact_player_example ${catkin_LIBRARIES})
endif()

# Tact parser test (standalone, no VR API dependency)
set(TACT_PARSER_TEST_SOURCES
    tact_player/tact_parser_test.cpp
    tact_player/tact_parser.cpp
    tact_player/bezier_interpolator.cpp
)

add_executable(tact_parser_test ${TACT_PARSER_TEST_SOURCES})

target_include_directories(tact_parser_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tact_player
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lejusdk-recorder/thirdparty/nlohmann-json/include
)

if(_USING_CATKIN)
  target_link_libraries(tact_parser_test ${catkin_LIBRARIES})
endif()
