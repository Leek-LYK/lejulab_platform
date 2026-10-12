add_library(common STATIC
    src/Types.cpp
    src/Config.cpp
)

target_include_directories(common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link yaml-cpp and spdlog
target_link_libraries(common PUBLIC yaml-cpp spdlog)

# Alias for consistent naming
add_library(cyclone_record_replay::common ALIAS common)
