cmake_minimum_required(VERSION 3.15)
project(revo2_dual_hand_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Paths ---
set(EXAMPLES_DIR      "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(STARK_SDK_DIR     "${EXAMPLES_DIR}/..")                    # stark_sdk_aarch64/
set(DEXHAND_SDK_DIR   "${STARK_SDK_DIR}/..")                   # dexhand_sdk/
set(LIB_DIR           "${DEXHAND_SDK_DIR}/..")                 # lib/
set(CANBUS_SDK_DIR    "${LIB_DIR}/canbus_sdk")
set(WORKSPACE_ROOT    "${LIB_DIR}/../../../../../..")          # repo root
get_filename_component(WORKSPACE_ROOT "${WORKSPACE_ROOT}" ABSOLUTE)

message(STATUS "[revo2_dual] STARK_SDK_DIR   = ${STARK_SDK_DIR}")
message(STATUS "[revo2_dual] CANBUS_SDK_DIR  = ${CANBUS_SDK_DIR}")
message(STATUS "[revo2_dual] WORKSPACE_ROOT  = ${WORKSPACE_ROOT}")

# --- stark_sdk_aarch64 (IMPORTED, arch-aware) ---
if(NOT TARGET stark_sdk_aarch64)
    add_subdirectory("${STARK_SDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/stark_sdk_aarch64_build")
endif()

# --- canbus_sdk: link against pre-built .so from workspace build tree ---
# canbus_sdk has external deps (yaml-cpp, libusb-1.0) — rebuilding standalone is
# brittle. Require the parent workspace to have been built first.
set(CANBUS_SDK_SEARCH_DIRS
    "${WORKSPACE_ROOT}/build/src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/canbus_sdk"
    "${WORKSPACE_ROOT}/build_cmake/src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/canbus_sdk"
    "${WORKSPACE_ROOT}/build_docker/src/leju-hardware/lejusdk-hw/src/hardware_plant/lib/canbus_sdk"
    "${CANBUS_SDK_DIR}/build"
)

find_library(CANBUS_SDK_LIB
    NAMES canbus_sdk
    HINTS ${CANBUS_SDK_SEARCH_DIRS}
    NO_DEFAULT_PATH
)

if(NOT CANBUS_SDK_LIB)
    message(FATAL_ERROR
        "[revo2_dual] libcanbus_sdk.so not found. Searched:\n"
        "  ${CANBUS_SDK_SEARCH_DIRS}\n"
        "Build the parent workspace first (so canbus_sdk gets built), or pass:\n"
        "  -DCANBUS_SDK_LIB=/path/to/libcanbus_sdk.so")
endif()

get_filename_component(CANBUS_SDK_LIB_DIR "${CANBUS_SDK_LIB}" DIRECTORY)
message(STATUS "[revo2_dual] canbus_sdk = ${CANBUS_SDK_LIB}")

# --- dexhand_base.h header path ---
# dexhand_base.h lives at lib/dexhand_sdk/dexhand_base.h
set(DEXHAND_BASE_INCLUDE_DIR "${DEXHAND_SDK_DIR}")

# --- Target ---
add_executable(revo2_dual_hand_test
    revo2_can_hand.cpp
    dual_hand_test.cpp
)

target_include_directories(revo2_dual_hand_test PRIVATE
    ${DEXHAND_BASE_INCLUDE_DIR}      # dexhand_base.h
    ${CANBUS_SDK_DIR}/include        # canbus_sdk/canbus_sdk.h
    ${CMAKE_CURRENT_SOURCE_DIR}      # revo2_can_hand.h
)

target_link_libraries(revo2_dual_hand_test PRIVATE
    stark_sdk_aarch64
    ${CANBUS_SDK_LIB}
    pthread
    dl
)

# Embed rpath so the binary finds both .so files at runtime without
# the user needing to set LD_LIBRARY_PATH manually.
set_target_properties(revo2_dual_hand_test PROPERTIES
    BUILD_RPATH "${STARK_SDK_LIB_DIR};${CANBUS_SDK_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN/../lib"
)
