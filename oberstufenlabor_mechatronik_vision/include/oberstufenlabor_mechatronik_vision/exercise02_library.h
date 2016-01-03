#ifndef OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE02_LIBRARY_H
#define OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE02_LIBRARY_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <imes_cv/imes_cv.h>
#include <sensor_msgs/image_encodings.h>
#include <oberstufenlabor_mechatronik_vision/Circle.h>
#include <luh_youbot_manipulation_api/manipulation_api.h>
#include <time.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

class exercise02_library : public ros::NodeHandle
{
public:
    exercise02_library();
    ~exercise02_library();

    void start();

protected:

    enum State
    {
        START,
        HSV,
        MORPH,
        DETECT,
        DONE
    }state_;

    enum color
    {
        RED,
        GREEN,
        YELLOW
    }color_;

    // user input
    int enter;
    int esc;

    // arm movement
    manipulation_api::YoubotArm arm_;

    image_transport::ImageTransport* image_transport_;
    image_transport::Subscriber color_subscriber_;

    // Color, Output
    cv::Mat C, O;
    
    cv::Mat hsv_filter_;
    cv::Mat image_morph_;
    cv::Mat image_circle_;
    oberstufenlabor_mechatronik_vision::Circle tracked_circle_;
    cv::Scalar hsv_min;
    cv::Scalar hsv_max;
    int height_offset_;

    // filter parameters
    int	filter_raw[6];
    int filter_color[3][6];
    int slider_params_[3][6];
    int curr_color;

    // window parameters
    std::string windowNames[3];
    std::string trackbarNames[8];

    // parameter file
    std::string filename_;
    std::string filename_morph_;

    bool is_active_;
    int header_height_;

    std::string image_topic_;

    void colorCallback(const sensor_msgs::ImageConstPtr &msg);
    void write(std::string text);
    void load_filter(std::string filename);
    void load_morph(std::string filename);
    void save_filter(std::string filename, int data[3][6]);
    void save_morph(std::string filename);
    cv::Mat drawCircle(cv::Mat image, oberstufenlabor_mechatronik_vision::Circle circle);
    cv::Mat filterImage(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max);
    cv::Mat morphImage(cv::Mat image, int erosion_size, int dilation_size);
    oberstufenlabor_mechatronik_vision::Circle searchForCircles(cv::Mat image);

    int erosion_size_;
    int dilation_size_;

};

#endif // OBERSTUFENLABOR_MECHATRONIK_VISION_EXERCISE02_LIBRARY_H
