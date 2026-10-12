# Tests for lejusdk-vr

# 强制添加调试符号（即使是 Release 模式）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(GTest QUIET)

if(GTest_FOUND)
    enable_testing()

    # Test executable
    add_executable(test_vr_api
        test_vr_api.cpp
    )

    target_link_libraries(test_vr_api
        ${PROJECT_NAME}
        GTest::GTest
        GTest::Main
    )

    # Add test
    add_test(NAME test_vr_api COMMAND test_vr_api)
else()
    message(STATUS "GTest not found, skipping lejusdk-vr tests")
endif()

# Stress test (standalone, no GTest dependency)
add_executable(set_mode_stress_test
    set_mode_stress_test.cpp
)

target_link_libraries(set_mode_stress_test
    ${PROJECT_NAME}
)
