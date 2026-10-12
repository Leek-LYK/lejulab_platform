add_library(recorder_lib STATIC
    src/DdsRecorder.cpp
    src/McapWriter.cpp
    src/TypesCollection.cpp
    src/Configuration.cpp
)

target_include_directories(recorder_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(recorder_lib PUBLIC
    common
    cyclone_dds
    mcap
    nlohmann_json
)

# Recorder executable
add_executable(lejusdk_recorder
    app/main.cpp
)

target_link_libraries(lejusdk_recorder PRIVATE
    recorder_lib
    optionparser
    pthread
)

# Installation
install(TARGETS lejusdk_recorder
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
