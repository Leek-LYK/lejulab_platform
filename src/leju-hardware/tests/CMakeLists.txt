# Tests for RobotInterface

file(GLOB TEST_SOURCES "*.cpp")

set(_hw_test_targets "")
foreach(SOURCE_FILE ${TEST_SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_link_libraries(${EXEC_NAME} PUBLIC lejusdk-lowlevel)
    list(APPEND _hw_test_targets ${EXEC_NAME})
endforeach()

if(_hw_test_targets)
    install(TARGETS ${_hw_test_targets}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/tests
    )
endif()
