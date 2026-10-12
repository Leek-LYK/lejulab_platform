find_package(GTest REQUIRED)

# 定义通用函数来添加测试
function(add_basic_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_include_directories(${test_name}
        PRIVATE
            ${YAML_CPP_INCLUDE_DIRS}
    )
    target_link_libraries(${test_name}
        PRIVATE
            canbus_sdk
            ${YAML_CPP_LIBRARIES}
            pthread
    )
endfunction()

function(add_bmcanbus_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_include_directories(${test_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../src
            ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/include
            ${YAML_CPP_INCLUDE_DIRS}
    )
    target_link_libraries(${test_name}
        PRIVATE
            canbus_sdk
            ${YAML_CPP_LIBRARIES}
            pthread
    )
endfunction()

function(add_gtest_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_include_directories(${test_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../src
            ${YAML_CPP_INCLUDE_DIRS}
    )
    target_link_libraries(${test_name}
        PRIVATE
            canbus_sdk
            ${YAML_CPP_LIBRARIES}
            GTest::gtest
            GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(${test_name})
endfunction()

# 普通测试列表（只需要canbus_sdk和pthread）
set(BASIC_TESTS
    mpsc_ring_buffer_test:test_mpsc_ring_buffer.cpp
    race_condition_test:test_race_conditions.cpp
    extreme_pressure_test:test_extreme_pressure.cpp
    bm_canbus_wrapper_test:test_bm_canbus_wrapper.cpp
    bm_canbus_wrapper_simple_test:test_bm_canbus_wrapper_simple.cpp
    leju_canbus_wrapper_test:test_leju_canbus_wrapper.cpp
    config_parser_test:test_config_parser.cpp
)
# GTest测试列表（需要GTest框架）
set(GTEST_TESTS
    canbus_control_block_test:test_canbus_control_block.cpp
    register_device_test:test_register_device.cpp
    config_parser_gtest:test_config_parser_gtest.cpp
    device_unique_id_test:test_device_unique_id.cpp
)



# 添加普通测试
foreach(test_pair ${BASIC_TESTS})
    string(REPLACE ":" ";" test_pair_list ${test_pair})
    list(GET test_pair_list 0 test_name)
    list(GET test_pair_list 1 source_file)

    # 根据测试名称判断是否需要额外包含目录
    if(${test_name} MATCHES "bm_canbus_wrapper" OR ${test_name} MATCHES "leju_canbus_wrapper")
        add_bmcanbus_test(${test_name} ${source_file})
    else()
        add_basic_test(${test_name} ${source_file})
    endif()
endforeach()

# 添加GTest测试
foreach(test_pair ${GTEST_TESTS})
    string(REPLACE ":" ";" test_pair_list ${test_pair})
    list(GET test_pair_list 0 test_name)
    list(GET test_pair_list 1 source_file)
    add_gtest_test(${test_name} ${source_file})
endforeach()