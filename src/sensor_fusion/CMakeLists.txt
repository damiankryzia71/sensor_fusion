cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pcd_fusion_node src/pcd_fusion_node.cpp)
target_include_directories(pcd_fusion_node PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(pcd_fusion_node yaml-cpp ${PCL_LIBRARIES})
ament_target_dependencies(pcd_fusion_node rclcpp sensor_msgs message_filters pcl_conversions)

add_executable(odom_tf_node src/odom_tf_node.cpp)
ament_target_dependencies(odom_tf_node rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS pcd_fusion_node odom_tf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
