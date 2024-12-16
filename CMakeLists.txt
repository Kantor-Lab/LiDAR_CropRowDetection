cmake_minimum_required(VERSION 3.5)
project(lidar_rowdetect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(test_robot_interfaces REQUIRED)
# find_package(LiDAR_CropRowDetection REQUIRED)
# Add C++ executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS cpp_node
    DESTINATION lib/${PROJECT_NAME})

# rosidl_generate_interfaces(${PROJECT_NAME}
#     "msg/Line2Pts.msg"
#     "msg/LineList.msg"
#     DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
#   )


# Install Python scripts
install(PROGRAMS
    scripts/python_node.py scripts/Lidar_FOV.py scripts/Virtual_groundplane.py
    scripts/change_odom.py scripts/Row_Detection.py scripts/Kmeans.py scripts/Ransac_fittingline.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# ament_export_dependencies(ament_cmake)
ament_package()

