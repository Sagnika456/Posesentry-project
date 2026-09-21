cmake_minimum_required(VERSION 3.8)
project(human_pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ground_truth_node src/ground_truth_node.cpp)

target_include_directories(ground_truth_node PUBLIC
  include
)

target_link_libraries(ground_truth_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(pose_estimation_node src/pose_estimation_node.cpp)

target_include_directories(pose_estimation_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pose_estimation_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBS}
)

install(TARGETS
  pose_estimation_node
  ground_truth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
