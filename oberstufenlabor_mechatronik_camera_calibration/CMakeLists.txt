cmake_minimum_required(VERSION 2.8.3)
project(oberstufenlabor_mechatronik_camera_calibration)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_state_publisher
  urdf
  geometry_msgs
  image_transport
  cv_bridge
  luh_youbot_kinematics
  tf
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES oberstufenlabor_mechatronik_camera_calibration_lib
  CATKIN_DEPENDS roscpp robot_state_publisher urdf geometry_msgs image_transport cv_bridge luh_youbot_kinematics tf
  DEPENDS OpenCV
)

include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# INCLUDES
##############################################################################

set(INCLUDE_FILES include/oberstufenlabor_mechatronik_camera_calibration/cameracalibration.h
                  include/oberstufenlabor_mechatronik_camera_calibration/cameracalibration_window.h
                  include/oberstufenlabor_mechatronik_camera_calibration/cameracalibration_qtros.h)

##############################################################################
# FORMS
##############################################################################

set(QT_FORMS ui/cameracalibration_window.ui)

set(QT_MOC  include/oberstufenlabor_mechatronik_camera_calibration/cameracalibration_window.h
            include/oberstufenlabor_mechatronik_camera_calibration/cameracalibration_qtros.h)

##############################################################################
# RESOURCES
##############################################################################

set(QT_RESOURCES resources/images.qrc)
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

set(QT_SOURCES src/cameracalibration.cpp
               src/cameracalibration_qtros.cpp
               src/cameracalibration_window.cpp)

##############################################################################
# LIBRARIES
##############################################################################

add_library(oberstufenlabor_mechatronik_camera_calibration_lib ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${INCLUDE_FILES})

##############################################################################
# Binaries
##############################################################################

add_executable(cameracalibration_node src/cameracalibration_node.cpp)
target_link_libraries(oberstufenlabor_mechatronik_camera_calibration_lib ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(cameracalibration_node ${QT_LIBRARIES} oberstufenlabor_mechatronik_camera_calibration_lib)

install( TARGETS
    cameracalibration_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    oberstufenlabor_mechatronik_camera_calibration_lib
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
