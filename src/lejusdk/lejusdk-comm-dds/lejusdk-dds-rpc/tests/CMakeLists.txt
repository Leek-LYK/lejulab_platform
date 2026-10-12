# ============================================================================
# DDS-RPC 单元测试 CMakeLists.txt
# ============================================================================

find_package(GTest REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/rpc_idl
)

# 测试可执行文件列表
set(DDS_RPC_TESTS
    test_rpc_types
    test_requester
    test_replier
    test_duplicate_service
    test_end_to_end
)

# 为每个测试创建可执行文件
foreach(TEST_NAME ${DDS_RPC_TESTS})
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME}
        ${PROJECT_NAME}
        GTest::gtest
        GTest::gtest_main
        pthread
        CycloneDDS::ddsc
        CycloneDDS-CXX::ddscxx
        iceoryx_posh::iceoryx_posh
        iceoryx_hoofs::iceoryx_hoofs
        iceoryx_binding_c::iceoryx_binding_c
    )
    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/rpc_idl
    )
endforeach()

# 启用 CTest
enable_testing()

# 注册测试
foreach(TEST_NAME ${DDS_RPC_TESTS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    # 设置测试超时时间 (DDS 发现需要时间)
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 120)
endforeach()

message(STATUS "DDS-RPC unit tests configured: ${DDS_RPC_TESTS}")

# ============================================================================
# 独立压力测试（不使用 GTest）
# ============================================================================
add_executable(rpc_first_call_test rpc_first_call_test.cpp)
target_link_libraries(rpc_first_call_test
    ${PROJECT_NAME}
    pthread
    CycloneDDS::ddsc
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    iceoryx_binding_c::iceoryx_binding_c
)
target_include_directories(rpc_first_call_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/rpc_idl
)

install(TARGETS rpc_first_call_test
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
