cmake_minimum_required(VERSION 3.8)
project(rm_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  registration
)
# add executable file
add_executable(odom_transform_node
  src/odom_transform.cpp
  )

add_executable(rgb_depth_node
  src/rgb_depth.cpp
  )

add_executable(odom_depth_transform_node
  src/odom_depth_tf.cpp
  )

add_executable(scan_process_node
  src/scan_process.cpp
  )
# dependencies select
ament_target_dependencies(odom_transform_node 
  rclcpp                           
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  )

ament_target_dependencies(rgb_depth_node 
  rclcpp                           
  sensor_msgs
  pcl_ros
  pcl_conversions

  )

ament_target_dependencies(odom_depth_transform_node
  rclcpp                           
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  PCL
  Eigen3
  pcl_conversions
  nav_msgs
  )

ament_target_dependencies(scan_process_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  )
# include lib select
target_include_directories(
  odom_transform_node PRIVATE   
  ${CMAKE_CURRENT_SOURCE_DIR}/include          
)

target_include_directories(
  rgb_depth_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(
  odom_depth_transform_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(
  scan_process_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS
  odom_transform_node
  rgb_depth_node
  odom_depth_transform_node
  scan_process_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wasd_teleop
)
if(BUILD_TESTING)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
endif()


ament_package()
