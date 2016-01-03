#ifndef CALIBRATION_NODE_H
#define CALIBRATION_NODE_H

#include <ros/ros.h>
#include <ball_finder/Circle.h>
#include <ball_finder/SetColor.h>
#include <luh_youbot_manipulation_api/manipulation_api.h>
#include <yaml-cpp/yaml.h>



class CalibrationNode : public ros::NodeHandle
{
public:
    CalibrationNode();
};

#endif // CALIBRATION_NODE_H
