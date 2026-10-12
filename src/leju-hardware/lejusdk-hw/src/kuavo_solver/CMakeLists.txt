cmake_minimum_required(VERSION 3.5)
project(kuavo_solver)

# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 包含目录
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# 创建动态链接库
add_library(kuavo_solver SHARED
  src/ankle_solver.cpp
  src/roban_ankle_solver.cpp
)

# 链接 Eigen3
target_link_libraries(kuavo_solver Eigen3::Eigen)

# 设置输出目录
set_target_properties(kuavo_solver PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)