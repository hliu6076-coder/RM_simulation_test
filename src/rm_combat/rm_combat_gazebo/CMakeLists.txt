cmake_minimum_required(VERSION 3.8)
project(rm_combat_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_combat_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(logical_shooter_plugin SHARED src/logical_shooter_plugin.cpp)
target_include_directories(logical_shooter_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(logical_shooter_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(logical_shooter_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(logical_shooter_plugin
  geometry_msgs rclcpp rm_combat_interfaces visualization_msgs)

add_library(duel_contact_plugin SHARED src/duel_contact_plugin.cpp)
target_include_directories(duel_contact_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(duel_contact_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(duel_contact_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(duel_contact_plugin rclcpp rm_combat_interfaces)

install(TARGETS logical_shooter_plugin duel_contact_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch rviz urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS test/duel_integration_probe.py DESTINATION lib/${PROJECT_NAME})

ament_package()
