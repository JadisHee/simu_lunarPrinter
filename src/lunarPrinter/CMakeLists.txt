cmake_minimum_required(VERSION 2.8.3)
project(lunarPrinter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)
# find_package(gazebo REQUIRED)
# find_package(Protobuf REQUIRED)
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)



# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES bingda_tutorials
#  CATKIN_DEPENDS message_runtime 
# #  DEPENDS system_lib
# )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
)

# include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
# )

# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
  # /usr/local/include # Location when using 'make system_install'
  # /usr/include       # More usual location (e.g. when installing using a package)
  # ${GAZEBO_INCLUDE_DIRS}
  # ${Protobuf_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
	script/steer_ctrl.py
  script/arm_path_planning_v3.py
  script/odom_to_tf_broadcaster.py
  script/traj_rviz_marker.py
  script/agv_pose_ctrl.py
  script/markers_hold.py
  script/foot_ctrl.py
  script/arm_joint_ctrl.py
  script/height_ctrl.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_library(trajectory_visualizer SHARED src/traj_vis.cc)
# target_link_libraries(trajectory_visualizer   ${GAZEBO_LIBRARIES} ${Protobuf_LIBRARIES})