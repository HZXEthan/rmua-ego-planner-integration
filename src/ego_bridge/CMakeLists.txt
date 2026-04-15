cmake_minimum_required(VERSION 3.0.2)
project(ego_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  quadrotor_msgs
  roscpp
  sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    airsim_ros
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    quadrotor_msgs
    roscpp
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)
target_link_libraries(pcd_map_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(path_to_vel_cmd src/path_to_vel_cmd.cpp)
target_link_libraries(path_to_vel_cmd ${catkin_LIBRARIES})

add_executable(position_cmd_to_vel_cmd src/position_cmd_to_vel_cmd.cpp)
target_link_libraries(position_cmd_to_vel_cmd ${catkin_LIBRARIES})

add_executable(goal_relay src/goal_relay.cpp)
target_link_libraries(goal_relay ${catkin_LIBRARIES})

add_executable(goal_segmenter src/goal_segmenter.cpp)
target_link_libraries(goal_segmenter ${catkin_LIBRARIES})

install(TARGETS
  pcd_map_publisher
  path_to_vel_cmd
  position_cmd_to_vel_cmd
  goal_relay
  goal_segmenter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
