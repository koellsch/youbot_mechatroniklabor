cmake_minimum_required(VERSION 2.8.3)
project(oberstufenlabor_mechatronik_vision)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  sensor_msgs
  geometry_msgs
  imes_cv
  roscpp
  std_msgs
  roslib
  message_generation
  message_runtime
  luh_youbot_manipulation_api
  luh_youbot_poses
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Circle.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetColor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(exercise02_library
  src/exercise02_library.cpp
  #src/exercise02_library_studis.cpp
  include/${PROJECT_NAME}/exercise02_library.h
)

## Declare a cpp executable
add_executable(exercise02_node src/exercise02_node.cpp)
#add_executable(exercise03 src/exercise03.cpp include/oberstufenlabor_mechatronik_vision/exercise03.h)
add_executable(exercise03 src/exercise03_studis.cpp include/oberstufenlabor_mechatronik_vision/exercise03.h)

## Specify libraries to link a library or executable target against
target_link_libraries(exercise02_node
  ${catkin_LIBRARIES}
  imes_cv
  exercise02_library
)
target_link_libraries(exercise03
  ${catkin_LIBRARIES}
  imes_cv
)
target_link_libraries(exercise02_library
  ${catkin_LIBRARIES}
  yaml-cpp
)
