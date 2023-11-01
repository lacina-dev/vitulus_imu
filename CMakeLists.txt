################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(vitulus_imu)


################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  roscpp
)
find_package(cmake_modules REQUIRED)
#find_package(Eigen REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy
  #DEPENDS Eigen
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
)



################################################################################
# Install
################################################################################

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/bno085_node
  nodes/bno085v2_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(nwu_to_enu src/nwu_to_enu.cpp)
#add_executable(imu_listener src/imu_listener.cpp )

add_dependencies(nwu_to_enu sensor_msgs_generate_messages_cpp)
 target_link_libraries(nwu_to_enu
   ${catkin_LIBRARIES}
   #${Eigen_LIBRARIES}
 )

################################################################################
# Test
################################################################################
