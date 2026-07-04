cmake_minimum_required(VERSION 3.16)
project(air_defense_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(target_sim_node src/target_sim_node.cpp)
ament_target_dependencies(target_sim_node
  rclcpp tf2 tf2_ros geometry_msgs visualization_msgs)

add_executable(radar_sensor_node src/radar_sensor_node.cpp)
ament_target_dependencies(radar_sensor_node
  rclcpp tf2 tf2_ros geometry_msgs visualization_msgs)

add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs visualization_msgs)
target_link_libraries(estimator_node Eigen3::Eigen)

add_executable(interceptor_node src/interceptor_node.cpp)
ament_target_dependencies(interceptor_node
  rclcpp tf2 tf2_ros geometry_msgs visualization_msgs)
target_link_libraries(interceptor_node Eigen3::Eigen)

install(TARGETS
  target_sim_node
  radar_sensor_node
  estimator_node
  interceptor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
