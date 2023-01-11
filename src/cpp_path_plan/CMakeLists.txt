cmake_minimum_required(VERSION 3.8)
project(cpp_path_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(detections REQUIRED)
find_package(geometry_msgs REQUIRED)

add_compile_options(-g)

add_executable(control_loop src/control_loop.cpp)
ament_target_dependencies(control_loop rclcpp detections geometry_msgs std_msgs)

add_compile_options(-g)

add_executable(greedy_detect_sel src/greedy_detect_sel.cpp)
ament_target_dependencies(greedy_detect_sel rclcpp detections)

install(TARGETS
  control_loop
  greedy_detect_sel
  DESTINATION lib/${PROJECT_NAME})

ament_package()
