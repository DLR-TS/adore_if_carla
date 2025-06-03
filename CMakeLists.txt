cmake_minimum_required(VERSION 3.8)
project(carla_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
include($ENV{SOURCE_DIRECTORY}/libraries/adore_libraries.cmake)
include(requirements.cmake)


add_executable(control_command_conversion_node src/control_command_conversion_node.cpp)
target_include_directories(control_command_conversion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_command_conversion_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
add_all_target_include_directories(control_command_conversion_node)
add_all_target_link_libraries(control_command_conversion_node)


add_executable(sensor_data_conversion_node src/sensor_data_conversion_node.cpp)
target_include_directories(sensor_data_conversion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_data_conversion_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
add_all_target_include_directories(sensor_data_conversion_node)
add_all_target_link_libraries(sensor_data_conversion_node)



install(TARGETS 
  control_command_conversion_node
  sensor_data_conversion_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()


