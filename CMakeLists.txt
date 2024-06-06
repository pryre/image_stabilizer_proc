cmake_minimum_required(VERSION 3.8)
project(image_stabilizer_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# ROS Dependencies (from package.xml)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  # lib
)

# Stabilizer Components
ament_auto_add_library(${PROJECT_NAME}_stabilizer_component SHARED
  src/${PROJECT_NAME}/stabilizer.cpp
)
target_compile_definitions(${PROJECT_NAME}_stabilizer_component PRIVATE "IMAGE_STABILIZER_PROC_STABILIZER_DLL")
target_link_libraries(${PROJECT_NAME}_stabilizer_component
# ${PROJECT_NAME}_common
)
rclcpp_components_register_nodes(${PROJECT_NAME}_stabilizer_component
  "${PROJECT_NAME}::Stabilizer"
)

ament_auto_add_executable(${PROJECT_NAME}_stabilizer_standalone src/stabilizer.cpp)
target_link_libraries(${PROJECT_NAME}_stabilizer_standalone ${PROJECT_NAME}_stabilizer_component)
set_target_properties(${PROJECT_NAME}_stabilizer_standalone PROPERTIES OUTPUT_NAME stabilizer)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch config
)
