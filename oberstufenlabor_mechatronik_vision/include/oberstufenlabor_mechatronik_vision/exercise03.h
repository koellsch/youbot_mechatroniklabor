#ifndef OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE03_H
#define OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE03_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <imes_cv/imes_cv.h>
#include <sensor_msgs/image_encodings.h>
#include <oberstufenlabor_mechatronik_vision/Circle.h>
#include <oberstufenlabor_mechatronik_vision/SetColor.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class exercise03 : public ros::NodeHandle
{
public:
    exercise03();

protected:
    ros::Publisher circle_publisher_;
    ros::ServiceServer set_color_server_;

    image_transport::ImageTransport* image_transport_;
    image_transport::Subscriber color_subscriber_;
    image_transport::Publisher image_publisher_;

    cv::Mat C, O; // Color, Output

    // detection parameters
    double roundness_threshold_;
    int opening_kernel_size_,
    min_radius_, max_radius_;

    std::string desired_color_;

    std::string image_topic_;
    std::string filename_;
    std::string filename_morph_;

    int header_height_;
    cv::Scalar hsv_min;
    cv::Scalar hsv_max;
    int erosion_size_;
    int dilation_size_;

    oberstufenlabor_mechatronik_vision::Circle tracked_circle_;

    bool findFilterParameter(std::string color);
    bool findMorphParameter();

    void colorCallback(const sensor_msgs::ImageConstPtr &msg);

    bool setColorCallback(oberstufenlabor_mechatronik_vision::SetColor::Request &req, oberstufenlabor_mechatronik_vision::SetColor::Response &res);

    void publishCircle(oberstufenlabor_mechatronik_vision::Circle circle);

    void findBalls();

    cv::Mat filterImage(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max);
    cv::Mat morphImage(cv::Mat image, int erosion_size, int dilation_size);
    oberstufenlabor_mechatronik_vision::Circle searchForCircles(cv::Mat image);
    cv::Mat drawCircle(cv::Mat image, oberstufenlabor_mechatronik_vision::Circle circle);
};

#endif // OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE03_H
