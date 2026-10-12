find_package(Eigen3 REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(SRCS
  ./MathUtilities.cpp
  ./orientation_tools.cpp
  ./savedata.cpp
)

add_library(mathtools_hw ${SRCS})

get_directory_property(PARENT_DIR PARENT_DIRECTORY)
get_filename_component(PARENT_PROJECT_NAME ${PARENT_DIR} NAME)

target_include_directories(mathtools_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/Math>
)

target_link_libraries(mathtools_hw Eigen3::Eigen)

install(TARGETS mathtools_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ./ 
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hardware/Math
  FILES_MATCHING PATTERN "*.h"
)