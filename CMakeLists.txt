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
  std_msgs
  vitulus_msgs
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

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/bno085_node
  nodes/bno085v2_node
  nodes/mag_heading
  tools/mag_replay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(nwu_to_enu src/nwu_to_enu.cpp)
#add_executable(imu_listener src/imu_listener.cpp )

add_dependencies(nwu_to_enu sensor_msgs_generate_messages_cpp)
 target_link_libraries(nwu_to_enu
   ${catkin_LIBRARIES}
   #${Eigen_LIBRARIES}
 )

# BNO085 UART driver in C++ (same behaviour as nodes/bno085_node; chosen by
# imu_driver:=cpp in launch/vitulus_imu.launch)
find_package(Threads REQUIRED)
add_executable(bno085_node_cpp src/bno085_node.cpp)
set_target_properties(bno085_node_cpp PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
target_compile_options(bno085_node_cpp PRIVATE -O2 -Wall -Wextra)
add_dependencies(bno085_node_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(bno085_node_cpp ${catkin_LIBRARIES} Threads::Threads)

install(TARGETS bno085_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################
# The watchdog of bno085_node_cpp alone, for test/test_bno085_recovery.py
if(CATKIN_ENABLE_TESTING)
  add_executable(bno085_supervisor_test test/bno085_supervisor_test.cpp)
  set_target_properties(bno085_supervisor_test PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
  target_link_libraries(bno085_supervisor_test Threads::Threads)
endif()
