add_library(cyclone_dds STATIC
    src/CycloneParticipant.cpp
    src/TopicDiscovery.cpp
    src/RawDataReader.cpp
    src/TypeSupport.cpp
)

target_include_directories(cyclone_dds PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cyclone_dds PUBLIC
    common
)

# Link CycloneDDS and iceoryx
target_link_libraries(cyclone_dds PUBLIC
    CycloneDDS::ddsc
    iceoryx_binding_c::iceoryx_binding_c
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

# Alias for consistent naming
add_library(cyclone_record_replay::cyclone_dds ALIAS cyclone_dds)
