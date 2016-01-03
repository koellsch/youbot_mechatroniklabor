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
 * @file   qtros.hpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the QtROS class for accessing ROS
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include "ros/ros.h"

#include <QThread>
#include <QObject>
#include <QStringListModel>
#include <QMutex>

#include <ros/package.h>

#include <string>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @class QtROS
 * @brief Class for accessing ROS
 */
class CameraCalibration_QtROS: public QThread {
    Q_OBJECT

    signals:
        /**
         * @brief this signal is emitted when the ROS main loop is left
         */
        void rosShutdown();


        /**
         * @brief This SIGNAL is emitted when a new image was received
         * @param frame received frame
         */
        void signalNewFrame(cv::Mat frame);

    public slots:


    public:
        /**
         * @brief constructor of class QtROS
         */
        CameraCalibration_QtROS(int argc, char *argv[], const char* node_name);


        /**
         * @brief destructor of class QtROS
         */
        ~CameraCalibration_QtROS();


        /**
         * @brief This method returns the current NodeHandle
         * @return NodeHandle
         */
        ros::NodeHandle getNodeHandle() {
            return *nh;
        }


        /**
         * @brief This Method returns the frame size
         * @return size of the received frames
         */
        cv::Size getImageSize(void);


        /**
         * @brief This Method returns the Name of the camera topic
         */
        std::string getCameraTopicName(void);

    private:
        /**
         * @brief This method contains the ROS event loop
         */
        void run();


        /**
         * @brief This method loads the parameter of the launchfile
         */
        void getROSParameter(void);


        /**
         * @brief callback for receiving images from ROS image stream
         * @param &msg pointer to image
         */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);


        /**
         * @brief This callback function receives the camera_info topic for a single time and shuts down the subscriber itself
         */
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);

        std::string cameraTopicName;    ///< image topic name
        cv::Size 	imageSize;          ///< image size (width x height)

        ros::NodeHandle* nh;
        ros::NodeHandle* nh_local;

        image_transport::Subscriber         image_sub;
        image_transport::ImageTransport*    it;
        ros::Subscriber                     camerainfo_sub;
        cv::Mat                             receivedFrame;
        cv_bridge::CvImagePtr               cv_ptr;
};
#endif
