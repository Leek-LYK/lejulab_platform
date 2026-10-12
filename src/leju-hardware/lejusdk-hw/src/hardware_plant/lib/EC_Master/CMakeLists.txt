cmake_minimum_required(VERSION 3.0.0)
project(ec_master VERSION 0.1.0)

set(CMAKE_BUILD_TYPE "Relese")

include(CTest)
enable_testing()

add_subdirectory(src)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

include(GNUInstallDirs)

get_directory_property(PARENT_DIR PARENT_DIRECTORY)
get_filename_component(PARENT_PROJECT_NAME ${PARENT_DIR} NAME)

# install(DIRECTORY config lib LinkOsLayer src
#     DESTINATION ${CMAKE_INSTALL_DATADIR}/${PARENT_PROJECT_NAME}/lib/EC_Master
#     PATTERN "*.c" EXCLUDE
#     PATTERN "*.cpp" EXCLUDE
# )