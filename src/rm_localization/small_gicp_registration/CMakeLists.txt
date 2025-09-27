cmake_minimum_required(VERSION 3.8)
project(small_gicp_registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准（small_gicp要求）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

# 添加small_gicp头文件路径
set(SMALL_GICP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../small_gicp/include)

ament_auto_find_build_dependencies()

# 检查small_gicp是否存在
if(NOT EXISTS ${SMALL_GICP_INCLUDE_DIR})
  message(FATAL_ERROR "small_gicp not found at ${SMALL_GICP_INCLUDE_DIR}")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${SMALL_GICP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# 启用OpenMP支持（如果可用）
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_OPENMP)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_localization::SmallGicpNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
