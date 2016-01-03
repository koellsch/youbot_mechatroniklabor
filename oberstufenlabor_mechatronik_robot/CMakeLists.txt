cmake_minimum_required(VERSION 2.8.3)
project(oberstufenlabor_mechatronik_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  luh_youbot_manipulation_api
  roscpp
  roslib
  luh_youbot_msgs
  oberstufenlabor_mechatronik_vision
  luh_youbot_poses
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
  FILES
  CollectBalls.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(exercise04_collector_server
  #src/oberstufenlabor_mechatronik_robot/exercise04_collector_server.cpp
  #include/oberstufenlabor_mechatronik_robot/exercise04_collector_server.h
  src/oberstufenlabor_mechatronik_robot/exercise04_collector_server_studis.cpp
  include/oberstufenlabor_mechatronik_robot/exercise04_collector_server_studis.h
)
add_dependencies(exercise04_collector_server
  oberstufenlabor_mechatronik_vision_generate_messages_cpp
  oberstufenlabor_mechatronik_robot_generate_messages_cpp
)
target_link_libraries(exercise04_collector_server
  ${catkin_LIBRARIES}
  yaml-cpp
)
add_executable(exercise04_gripper_test
  #src/oberstufenlabor_mechatronik_robot/exercise04_gripper_test.cpp
  #include/oberstufenlabor_mechatronik_robot/exercise04_gripper_test.h
  src/oberstufenlabor_mechatronik_robot/exercise04_gripper_test_studis.cpp
  include/oberstufenlabor_mechatronik_robot/exercise04_gripper_test_studis.h
)
add_dependencies(exercise04_gripper_test
  oberstufenlabor_mechatronik_vision_generate_messages_cpp
  oberstufenlabor_mechatronik_robot_generate_messages_cpp
)
target_link_libraries(exercise04_gripper_test
  ${catkin_LIBRARIES}
  yaml-cpp
)

# collector calibration node
add_executable(gripper_calibration_node
  src/oberstufenlabor_mechatronik_robot/calibration_node.cpp
  include/oberstufenlabor_mechatronik_robot/calibration_node.h
)
add_dependencies(gripper_calibration_node
  oberstufenlabor_mechatronik_vision_generate_messages_cpp
)
target_link_libraries(gripper_calibration_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

# collector node
add_executable(exercise04_collector_node
  src/oberstufenlabor_mechatronik_robot/exercise04_collector_node.cpp
)
add_dependencies(exercise04_collector_node
  oberstufenlabor_mechatronik_robot_generate_messages_cpp
)
target_link_libraries(exercise04_collector_node
  ${catkin_LIBRARIES}
)
