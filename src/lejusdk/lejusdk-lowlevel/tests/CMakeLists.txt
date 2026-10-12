cmake_minimum_required(VERSION 3.16)

# Find gtest
find_package(GTest REQUIRED)

# 测试源文件
set(TEST_SOURCES
    example_usage.cpp
    test_loop.cpp
)

# 为每个非gtest测试文件创建可执行程序
foreach(SOURCE_FILE ${TEST_SOURCES})
    # 获取不带扩展名的文件名作为可执行程序名
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_link_libraries(${EXEC_NAME} lejusdk-lowlevel pthread)
    target_compile_definitions(${EXEC_NAME} PRIVATE HAS_CYCLONEDS)
endforeach()

# gtest-based tests
add_executable(test_robot_version test_robot_version.cpp)
target_link_libraries(test_robot_version
    lejusdk-lowlevel
    GTest::gtest
    GTest::gtest_main
    pthread
)
target_compile_definitions(test_robot_version PRIVATE HAS_CYCLONEDDS)

add_executable(test_topic_logger test_topic_logger.cpp)
target_link_libraries(test_topic_logger
    lejusdk-lowlevel
    GTest::gtest
    GTest::gtest_main
    pthread
)
target_compile_definitions(test_topic_logger PRIVATE HAS_CYCLONEDDS)

# Enable testing
enable_testing()
add_test(NAME RobotVersionTest COMMAND test_robot_version)
add_test(NAME TopicLoggerTest COMMAND test_topic_logger)

install(TARGETS example_usage test_loop test_robot_version test_topic_logger
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/tests
)