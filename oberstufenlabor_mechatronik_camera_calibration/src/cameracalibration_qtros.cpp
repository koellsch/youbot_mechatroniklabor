/* *****************************************************************
 *
 * imes_youbot_pkg
 *
 * Copyright (c) 2012,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/
/**
 * @file   CameraCalibration_QtROS.cpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the CameraCalibration_QtROS class for accessing ROS
 */
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration_qtros.h>

//#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

CameraCalibration_QtROS::CameraCalibration_QtROS(int argc, char *argv[], const char* node_name) {
    ros::init(argc, argv, node_name);

    this->nh        = new ros::NodeHandle;
    this->nh_local  = new ros::NodeHandle("~");
    this->it        = new image_transport::ImageTransport(*this->nh);

    // obtain parameters
    this->getROSParameter();

    // subscribe to video stream
    this->image_sub      = this->it->subscribe(this->cameraTopicName + "/image_raw", 1, &CameraCalibration_QtROS::imageCallback, this);
    this->camerainfo_sub = this->nh->subscribe(this->cameraTopicName + "/camera_info", 1, &CameraCalibration_QtROS::cameraInfoCallback, this);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    start();
}


CameraCalibration_QtROS::~CameraCalibration_QtROS() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}


void CameraCalibration_QtROS::run() {
    ros::Rate rate(60.0);

    while(ros::ok()){


        ros::spinOnce();
        rate.sleep();
    }
    emit rosShutdown();
}


void CameraCalibration_QtROS::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        this->receivedFrame = cv_ptr->image;

        emit signalNewFrame(this->receivedFrame);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


void CameraCalibration_QtROS::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    this->imageSize = Size(cam_info->width, cam_info->height);

    camerainfo_sub.shutdown();
}


void CameraCalibration_QtROS::getROSParameter(void) {
    this->nh_local->param<std::string>("cameraTopicName", this->cameraTopicName, "/usb_cam");
    if (this->cameraTopicName[0] == '/')
        this->cameraTopicName = this->cameraTopicName.substr(1, this->cameraTopicName.length()-1);
}


Size CameraCalibration_QtROS::getImageSize(void) {
    return this->imageSize;
}


string CameraCalibration_QtROS::getCameraTopicName(void) {
    return this->cameraTopicName;
}
