cmake_minimum_required(VERSION 3.8)
project(inspection_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(
  DIRECTORY urdf worlds launch config maps rviz scripts behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/custom_teleop.py
  scripts/odom_tf_publisher.py
  scripts/initial_pose_publisher.py
  scripts/nav2_activator.py
  scripts/steering_animator.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()