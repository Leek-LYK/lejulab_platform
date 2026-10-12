cmake_minimum_required(VERSION 3.12)
#
# stark_sdk_aarch64  —  BrainCo Revo2 bc-stark-sdk v1.1.9+
#
# Directory named after the primary deployment target (aarch64 board) but the
# layout ships both aarch64 and x86_64 prebuilts — the right .so is selected
# automatically from CMAKE_SYSTEM_PROCESSOR.
#
# Exposes an IMPORTED shared-library target: `stark_sdk_aarch64`
#
# Usage (from a parent CMakeLists):
#   add_subdirectory(path/to/stark_sdk_aarch64)
#   target_link_libraries(my_target PRIVATE stark_sdk_aarch64)
#

project(stark_sdk_aarch64 LANGUAGES C CXX)

# -- Arch selection ----------------------------------------------------------
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(STARK_SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    set(STARK_SDK_ARCH "x86_64")
else()
    message(FATAL_ERROR
        "stark_sdk_aarch64: unsupported CMAKE_SYSTEM_PROCESSOR='${CMAKE_SYSTEM_PROCESSOR}'. "
        "Expected aarch64 or x86_64.")
endif()

set(STARK_SDK_ROOT    "${CMAKE_CURRENT_SOURCE_DIR}")
set(STARK_SDK_INCLUDE "${STARK_SDK_ROOT}/dist/include")
set(STARK_SDK_LIB_DIR "${STARK_SDK_ROOT}/dist/shared/linux/${STARK_SDK_ARCH}")
set(STARK_SDK_LIB     "${STARK_SDK_LIB_DIR}/libbc_stark_sdk.so")

if(NOT EXISTS "${STARK_SDK_LIB}")
    message(FATAL_ERROR
        "stark_sdk_aarch64: prebuilt library not found for arch '${STARK_SDK_ARCH}' at:\n"
        "  ${STARK_SDK_LIB}\n"
        "Run: ${STARK_SDK_ROOT}/download-lib.sh "
        "(set STARK_TARGET_ARCH=${STARK_SDK_ARCH} for cross-download)")
endif()

# -- IMPORTED target ---------------------------------------------------------
add_library(stark_sdk_aarch64 SHARED IMPORTED GLOBAL)
set_target_properties(stark_sdk_aarch64 PROPERTIES
    IMPORTED_LOCATION "${STARK_SDK_LIB}"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${STARK_SDK_INCLUDE}"
)

# Expose arch/dir so consumers can propagate rpath/install rules
set(STARK_SDK_ARCH    "${STARK_SDK_ARCH}"    CACHE INTERNAL "stark_sdk selected arch")
set(STARK_SDK_LIB_DIR "${STARK_SDK_LIB_DIR}" CACHE INTERNAL "stark_sdk prebuilt lib dir")
set(STARK_SDK_LIB     "${STARK_SDK_LIB}"     CACHE INTERNAL "stark_sdk prebuilt .so path")
set(STARK_SDK_INCLUDE "${STARK_SDK_INCLUDE}" CACHE INTERNAL "stark_sdk header dir")

message(STATUS "stark_sdk_aarch64: arch=${STARK_SDK_ARCH}  lib=${STARK_SDK_LIB}")

# -- Install (opt-in) --------------------------------------------------------
# Consumers that want to ship the .so to `lib/` can do:
#   install(FILES ${STARK_SDK_LIB} DESTINATION lib)
#   install(DIRECTORY ${STARK_SDK_INCLUDE}/ DESTINATION include/stark_sdk_aarch64)
