/* *****************************************************************
 *
 * Oberstufenlabor Mechatronik I Aufgabe 3 - Bildverarbeitung
 * Erstellung einer Funktion zum Finden und Übermitteln eines Balls in vorgegebener Farbe in einem Bild
 * Teilaufgaben:
 *  - Überprüfen ob Bild vorhanden, Farbe vorgegeben und Parameter vorhanden
 *  - Einbindung der in Aufgabe 2 erstellten Funktionen zur Erkennung eines Balls
 *  - Publishen der Daten des gefundenen Balls
 *
 * Copyright (c) 2015,
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

#include <oberstufenlabor_mechatronik_vision/exercise03.h>

//############## CONSTRUCTOR ###########################################################################################
exercise03::exercise03() : ros::NodeHandle()
{
    // === PARAMETERS ===
    param("exercise03/image_topic", image_topic_, std::string("usb_cam/image_raw"));

    // === ROS_COMMUNICATION ===
    image_transport_ = new image_transport::ImageTransport(*this);
    color_subscriber_ = image_transport_->subscribe(image_topic_, 1, &exercise03::colorCallback, this);

    // === PARAMETERS ===
    filename_ = ros::package::getPath("oberstufenlabor_mechatronik_vision") + std::string("/config/parameter.yaml");
    filename_morph_ = ros::package::getPath("oberstufenlabor_mechatronik_vision") + std::string("/config/morph.yaml");

    // === PUBLICATIONS ===
    circle_publisher_ = advertise<oberstufenlabor_mechatronik_vision::Circle>("oberstufenlabor_mechatronik_vision/circle", 1);

    // === SERVICE SERVER ===
    set_color_server_ = advertiseService("oberstufenlabor_mechatronik_vision/set_color", &exercise03::setColorCallback, this);

    desired_color_ = "none";
    tracked_circle_.was_found = false;
}

//############## SET COLOR CALLBACK ####################################################################################
bool exercise03::setColorCallback(oberstufenlabor_mechatronik_vision::SetColor::Request &req, oberstufenlabor_mechatronik_vision::SetColor::Response &res)
{
    desired_color_ = req.name;
}

//############## FIND BALLS ############################################################################################
void exercise03::findBalls()
{
    /*************************************************************************************
         (A) Aufg:	Die Funktion "findBalls()" zum Finden eines Balls in einer vorgegebenen
                    Farbe soll erstellt werden.

                    1.  Zu Beginn sollen folgende Fälle überprüft werden:

                        - Ist die Bildmatrix C mit Daten gefüllt?
                        - Wurde in der Variablen "desired_color_" die gewünschte Farbe übergeben?
                        - Wurden die Filter-Parameter für die gewünschte Farbe sowie die Parameter
                          für die morphologischen Operationen gefunden?

                        Sollte einer dieser Fälle nicht eintreffen, soll sich die Funktion beenden.

                        Hinweise:   - Ob die Parameter für Farbfilter und morphologische Operationen
                                      vorhanden sind, kann mit den Funktionen "findFilterParameter"
                                      und "findMorphParameter" überprüft werden. Diese geben eine
                                      boolsche Variable mit dem Ergebnis zurück.
                                    - Wurde keine Farbe übergeben, dann steht in der Variablen
                                      "desired_color_" der String "none".

                    2.  Implementieren Sie die in Aufgabe 2 entwickelten Funktionen. Kopieren Sie dazu
                        den Programmcode in die gekennzeichneten Bereiche der Funktionen "filterImage",
                        "morphImage" und "searchForCircles".
                        Der gefundene Kreis soll am Ende an die Variable "tracked_circle_" übergeben
                        werden.

                        Das Einzeichnen des gefundenen Kreises in eine Bildmatrix ist nicht nötig!
                        Der gefundene Kreis kann über die Funktion "publishCircle" gepublished werden.

         Hinweise:  http://docs.opencv.org/modules/core/doc/basic_structures.html#mat
                    http://www.cplusplus.com/reference/string/string/
                    http://www.cpp-entwicklung.de/cpplinux/cpp_main/node4.html#SECTION00440000000000000000

    **************************************************************************************/
    ///<---
    if(O.empty())
        return;

    if(desired_color_.compare("none") == 0)
        return;

    if(!findFilterParameter(desired_color_) && !findMorphParameter())
        return;

    cv::Mat filtered_image = filterImage(O, hsv_min, hsv_max);
    cv::Mat morphed_image = morphImage(filtered_image, erosion_size_, dilation_size_);
    tracked_circle_ = searchForCircles(morphed_image);

    publishCircle(tracked_circle_);
    ///--->
}

//############## IMAGE FILTER ####################################################################################
cv::Mat exercise03::filterImage(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max)
{
    cv::Mat colorFilter;

    /*************************************************************************************
         Kopieren Sie den von Ihnen geschriebenen Programmcode für die Funktion "filterImage"
         in den gekennzeichneten Bereich.

    **************************************************************************************/
    ///--->
    cv::Mat tmp;

    cv::cvtColor(image, tmp, CV_BGR2HSV);
    cv::inRange(tmp, hsv_min, hsv_max, tmp);

    image.copyTo(colorFilter, tmp);
    ///<---
    return colorFilter;
}

//############## MORPHOLOGIC OPERATIONS ####################################################################################
cv::Mat exercise03::morphImage(cv::Mat image, int erosion_size, int dilation_size)
{
    /*************************************************************************************
         Kopieren Sie den von Ihnen geschriebenen Programmcode für die Funktion "morphImage"
         in den gekennzeichneten Bereich.

    **************************************************************************************/
    /// --->
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT,
                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                        cv::Point(erosion_size, erosion_size));

    cv::erode(image, image, erosion_element);

    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT,
                                    cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                    cv::Point(dilation_size, dilation_size));
    cv::dilate(image, image, dilation_element);

    /// <---
    return image;
}

//############## FIND CIRCLES ####################################################################################
oberstufenlabor_mechatronik_vision::Circle exercise03::searchForCircles(cv::Mat image)
{
    oberstufenlabor_mechatronik_vision::Circle circle;
    cv::Point ball_center;
    std::vector<std::vector<cv::Point> > contours;

    int max_area = 0;
    double min_dist = HUGE_VAL;
    double roundness_threshold_ = 0.6;
    float min_radius_ = 10;
    float max_radius_ = 100;

    /*************************************************************************************
         Kopieren Sie den von Ihnen geschriebenen Programmcode für die Funktion "searchForCircles"
         in den gekennzeichneten Bereich.

    **************************************************************************************/

    /// Aufgabenteil 1
    /// --->
    cv::cvtColor(image, image, CV_RGB2GRAY);

    cv::findContours(image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    /// <---

    for (uint i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = std::abs(cv::contourArea(contours[i]));
        double perimeter = cv::arcLength(contours[i], true);

        double roundness = 4 * M_PI * area / (perimeter * perimeter);

        if (roundness_threshold_ < roundness)
        {
            /// Aufgabenteil 2
            /// --->
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            /// <---

            if(radius > min_radius_ && radius < max_radius_)
            {

                if(tracked_circle_.was_found) // choose circle closest to last detected circle
                {
                    double dx = tracked_circle_.x - center.x;
                    double dy = tracked_circle_.y - center.y;
                    double dist = dx*dx + dy*dy;

                    if(dist < min_dist)
                    {
                        circle.x = center.x;
                        circle.y = center.y;
                        circle.r = radius;
                        circle.was_found = true;
                        min_dist = dist;
                    }
                }
                else if(area > max_area) // choose circle with greatest radius
                {
                    circle.x = center.x;
                    circle.y = center.y;
                    circle.r = radius;
                    circle.was_found = true;
                    max_area = area;
                }
            }
        }

        ball_center.x = circle.x;
        ball_center.y = circle.y;
    }
    return circle;
}

//############## FIND FILTER PARAMETER ########################################################################################
bool exercise03::findFilterParameter(std::string color)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filename_);
        if(node.IsNull())
        {
            ROS_ERROR("Couldn't read the filter parameter file...");
            return false;
        }
        for (YAML::const_iterator it=node.begin(); it!=node.end(); ++it)
        {
            if(it->first.as<std::string>() == color)
            {
                hsv_min = cv::Scalar(it->second["H_min"].as<int>(), it->second["S_min"].as<int>(), it->second["V_min"].as<int>(), 0);
                hsv_max = cv::Scalar(it->second["H_max"].as<int>(), it->second["S_max"].as<int>(), it->second["V_max"].as<int>(), 0);
            }
        }
    }
    catch(YAML::Exception e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

//############## FIND MORPH PARAMETER ########################################################################################
bool exercise03::findMorphParameter()
{
    try
    {
        YAML::Node node_morph = YAML::LoadFile(filename_morph_);
        if(node_morph.IsNull())
        {
            ROS_ERROR("Couldn't read the morph parameter file...");
            return false;
        }
        for (YAML::const_iterator it=node_morph.begin(); it!=node_morph.end(); ++it)
        {
            if(it->first.as<std::string>() == "Erosion")
                erosion_size_ = it->second["kernel_size"].as<int>();
            else if(it->first.as<std::string>() == "Dilation")
                dilation_size_ = it->second["kernel_size"].as<int>();
        }
    }
    catch(YAML::Exception e)
    {
        std::cout << e.what() << std::endl;
    }
}

//############## PUBLISH CIRCLE ########################################################################################
void exercise03::publishCircle(oberstufenlabor_mechatronik_vision::Circle circle)
{
    double center_x = 0.5 * C.cols;
    double center_y = 0.5 * C.rows;

    // normalize with image height
    circle.x = (circle.x - center_x) / C.rows;
    circle.y = (circle.y - center_y) / C.rows;
    circle.r /= C.rows;

    circle_publisher_.publish(circle);
}

//############## COLOR MAP CALLBACK ####################################################################################
void exercise03::colorCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // === READ COLOR DATA ===

    // resize image if necessary
    if(C.rows != (int)msg->height || C.cols != (int)msg->width)
    {
        C = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
        header_height_ = cvRound(0.1 * C.rows);
        O = cv::Mat::zeros(C.rows, C.cols, CV_8UC3);
    }

    // copy from msg to mat
    uint8_t* pO = O.ptr<uint8_t>(0);
    memcpy(pO, &(msg->data.front()), C.cols*C.rows*3);

    if(msg->encoding == sensor_msgs::image_encodings::RGB8)
    {
        cv::cvtColor(O, O, CV_RGB2BGR);
    }
    {
        // === READ COLOR DATA ===

        // resize image if necessary
        if(C.rows != (int)msg->height || C.cols != (int)msg->width)
        {
            C = cv::Mat::zeros(msg->height, msg->width, CV_8UC3);
            header_height_ = cvRound(0.1 * C.rows);
            O = cv::Mat::zeros(C.rows, C.cols, CV_8UC3);
        }

        // copy from msg to mat
        uint8_t* pO = O.ptr<uint8_t>(0);
        memcpy(pO, &(msg->data.front()), C.cols*C.rows*3);

        if(msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            cv::cvtColor(O, O, CV_RGB2BGR);
        }
    }
    findBalls();
}

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "exercise03");

    exercise03 exercise03;

    ros::spin();
}
