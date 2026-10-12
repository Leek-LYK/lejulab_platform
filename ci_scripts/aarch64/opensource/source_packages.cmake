# Generated source package list for the open-source release tree.
# release.sh regenerates this file from the closed root CMakeLists.txt and
# removes packages declared as stub/binary in opensource_packages.txt.

add_subdirectory(src/lejusdk/lejusdk-utils)
add_subdirectory(src/leju_assets)
add_subdirectory(src/leju_launch)
add_subdirectory(src/leju-controllers/leju-dummy-controller)
add_subdirectory(src/leju-controllers/leju-keyboard-controller)
if(BUILD_JOYSTICK)
    add_subdirectory(src/leju-joystick)
endif()
if(BUILD_EXAMPLES)
    add_subdirectory(src/lejusdk/examples)
endif()
add_subdirectory(src/leju-audio)
add_subdirectory(src/leju-monitor)
add_subdirectory(src/lejusdk/lejusdk-csv-recorder)
