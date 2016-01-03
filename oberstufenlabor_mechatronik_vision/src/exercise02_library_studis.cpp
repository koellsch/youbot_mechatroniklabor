/* *****************************************************************
 *
 * Oberstufenlabor Mechatronik I Aufgabe 2 - Bildverarbeitung
 * Teilaufgaben:
 *  - Umwandlung des Farbraums eines Bildes von BGR in HSV
 *  - Anwendung von Farbfiltern
 *  - Anwendung von morphologischen Operationen
 *  - Konturendetektion / Erkennung eines Kreises
 *  - Berechnung von Radius und Mittelpunkt
 *  - Einzeichnen des gefundenen Kreises in eine Bildmatrix
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

#include <oberstufenlabor_mechatronik_vision/exercise02_library.h>

//############## CONSTRUCTOR ###########################################################################################
exercise02_library::exercise02_library() : ros::NodeHandle()
{
    // === PARAMETERS ===
    param("exercise02_node/image_topic", image_topic_, std::string("usb_cam/image_raw"));

    // === ROS_COMMUNICATION ===
    image_transport_ = new image_transport::ImageTransport(*this);
    color_subscriber_ = image_transport_->subscribe(image_topic_, 1, &exercise02_library::colorCallback, this);

    // === ARM_INITIALIZATION ===
    arm_.init(*this);
    arm_.moveToPose("SEARCH_HIGH");
    //arm_.moveToPose("ARM_UP");
    arm_.waitForCurrentAction();
}

//############## DESTRUCTOR ###########################################################################################
exercise02_library::~exercise02_library()
{
}

//############## START #################################################################################################
void exercise02_library::start()
{
    // initialization of window parameters
    windowNames[0] = "Red";
    windowNames[1] = "Green";
    windowNames[2] = "Yellow";

    trackbarNames[0] = "H_min";
    trackbarNames[1] = "H_max";
    trackbarNames[2] = "S_min";
    trackbarNames[3] = "S_max";
    trackbarNames[4] = "V_min";
    trackbarNames[5] = "V_max";
    trackbarNames[6] = "Erosion_size";
    trackbarNames[7] = "Dilation_size";

    height_offset_ = 60;

    // initialization of morphologic parameters
    erosion_size_ = 0;
    dilation_size_ = 0;

    // open filter parameter file
    filename_ = ros::package::getPath("oberstufenlabor_mechatronik_vision") + std::string("/config/parameter_studis.yaml");
    load_filter(filename_);

    // open morphologic parameter file
    filename_morph_ = ros::package::getPath("oberstufenlabor_mechatronik_vision") + std::string("/config/morph_studis.yaml");
    load_morph(filename_morph_);

    // user input keys
    enter = 1048586;
    esc = 1048603;
    //enter = 10;
    //esc = 27;

    // initialization of filter parameter
    for(int n = 0; n < 3; n++)
    {
        for(int idx = 0; idx < 6; idx++)
        {
            filter_raw[idx] = slider_params_[0][idx];
            filter_color[n][idx] = slider_params_[n][idx];
        }
    }

    // initialize variables for statemachine
    state_ = START;
    bool IsRunning = true;
    bool finished = false;
    int key;
    int last_color = -1;

    // wait for first image
    ROS_INFO("Waiting for images...");
    while(C.empty())
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    // statemachine
    while(IsRunning)
    {
        switch(state_)
        {
        case START:
        {
            int ans = -1;

            // menue selection
            std::cout << std::endl;
            std::cout << "Menue \n 1. HSV Farbparameter \n 2. morphologische Operationen \n 3. Kreis-Detektion \n 4. Exit \n Auswahl:";
            while(ans == -1)
                std::cin >> ans;

            // create textbox for textoutput
            cv::namedWindow("Textbox", 0);
            cv::moveWindow("Textbox", 0, 0);
            cv::resizeWindow("Textbox", C.cols, header_height_);

            // create image output window
            cv::namedWindow("Image");
            cv::moveWindow("Image", 400, header_height_ + height_offset_);

            // create slider window
            cv::namedWindow("Slider", 0);
            cv::moveWindow("Slider", 0, header_height_ + height_offset_);
            cv::resizeWindow("Slider", 330, 360);

            // create trackbars
            cv::createTrackbar(trackbarNames[0], "Slider", &filter_raw[0], 180, NULL);
            cv::createTrackbar(trackbarNames[1], "Slider", &filter_raw[1], 180, NULL);
            cv::createTrackbar(trackbarNames[2], "Slider", &filter_raw[2], 256, NULL);
            cv::createTrackbar(trackbarNames[3], "Slider", &filter_raw[3], 256, NULL);
            cv::createTrackbar(trackbarNames[4], "Slider", &filter_raw[4], 256, NULL);
            cv::createTrackbar(trackbarNames[5], "Slider", &filter_raw[5], 256, NULL);

            cv::startWindowThread();

            switch(ans)
            {
            case 1:
            {
                state_ = HSV;
                color_= RED;
                break;
            }
            case 2:
            {
                state_ = MORPH;
                break;
            }
            case 3:
            {
                state_ = DETECT;
                break;
            }
            case 4:
            {
                state_ = DONE;
                break;
            }
            }
            break;
        }
        case HSV:
        {
            while(!finished)
            {
                switch(color_)
                {
                case RED:
                {
                    write("Hold the red ball in front of the camera and press Enter.");

                    curr_color = 0;
                    key = -1;
                    while(key != enter)
                    {
                        ros::spinOnce();
                        cv::imshow("Image",O);
                        key = cv::waitKey(50);
                    }
                    break;
                }
                case GREEN:
                {
                    write("Hold the green ball in front of the camera and press Enter.");

                    curr_color = 1;
                    key = -1;
                    while(key != enter)
                    {
                        ros::spinOnce();
                        cv::imshow("Image",O);
                        key = cv::waitKey(50);
                    }
                    break;
                }
                case YELLOW:
                {
                    write("Hold the yellow ball in front of the camera and press Enter.");

                    curr_color = 2;
                    key = -1;
                    while(key != enter)
                    {
                        ros::spinOnce();
                        cv::imshow("Image",O);
                        key = cv::waitKey(50);
                    }
                    break;
                }
                }

                write("Adjust the sliders until only the ball is visible. Press Enter to continue.");

                key = -1;
                while(key != enter)
                {

                    // switch slider values dependent on current color
                    if (last_color != curr_color)
                    {
                        for(int idx = 0; idx < 6; idx++)
                        {
                            //apply last color filter parameter
                            filter_raw[idx] = filter_color[curr_color][idx];
                            // set trackbar position
                            cv::setTrackbarPos(this->trackbarNames[idx], "Slider", filter_raw[idx]);
                        }
                        last_color = curr_color;
                    }

                    // apply value of trackbars to the single-color arrays
                    for(int idx = 0; idx < 6; idx++)
                        filter_color[curr_color][idx] = filter_raw[idx];

                    // set hsv color filter thresholds
                    hsv_min = cv::Scalar(filter_color[curr_color][0], filter_color[curr_color][2], filter_color[curr_color][4], 0);
                    hsv_max = cv::Scalar(filter_color[curr_color][1], filter_color[curr_color][3], filter_color[curr_color][5], 0);

                    // apply filter values on the frozen image
                    hsv_filter_ = filterImage(O, hsv_min, hsv_max);

                    cv::imshow("Image", hsv_filter_);
                    key = cv::waitKey(50);
                }

                write("Move the ball around. If your pleased with the result, press Enter, otherwise press ESC.");

                key = -1;
                while(key != enter && key != esc)
                {
                    ros::spinOnce();

                    // apply filter values on the input image in realtime
                    hsv_filter_ = filterImage(O, hsv_min, hsv_max);

                    cv::imshow("Image", hsv_filter_);
                    key = cv::waitKey(50);
                }

                if(key == enter)
                {
                    if(color_ == RED)
                        color_ = GREEN;
                    else if (color_ == GREEN)
                        color_ = YELLOW;
                    else if(color_ == YELLOW)
                    {
                        // save filter values to yaml-file
                        save_filter(filename_, filter_color);

                        finished = true;
                        state_ = MORPH;
                    }
                }
            }
            break;
        }
        case MORPH:
        {
            write("Yellow ball: Move the slider to adjust the kernel size. Press Enter to continue, press ESC to exit.");

            key = -1;

            // initialize new windows
            cv::namedWindow("Morph");
            cv::moveWindow("Morph", 700, header_height_ + height_offset_ + 150);
            cv::moveWindow("Image", 0, header_height_ + height_offset_ + 150);

            cv::destroyWindow("Slider");
            cv::namedWindow("Slider", 0);
            cv::resizeWindow("Slider", 330, 150);
            cv::moveWindow("Slider", 0, header_height_ + height_offset_);

            cv::createTrackbar(trackbarNames[6], "Slider", &erosion_size_, 21, NULL);
            cv::createTrackbar(trackbarNames[7], "Slider", &dilation_size_, 21, NULL);
            cv::setTrackbarPos(trackbarNames[6], "Slider", erosion_size_);
            cv::setTrackbarPos(trackbarNames[7], "Slider", dilation_size_);

            hsv_min = cv::Scalar(filter_color[2][0], filter_color[2][2], filter_color[2][4], 0);
            hsv_max = cv::Scalar(filter_color[2][1], filter_color[2][3], filter_color[2][5], 0);

            while(key != enter && key != esc)
            {
                /*************************************************************************************
                     (B) Aufg:	Entfernen Sie für diesen Aufgabenteil die Kommentierungen und kommentieren
                                Sie die Zeile "key = esc;" aus.

                **************************************************************************************/
                /// --->
                /* Kommentierung entfernen
                ros::spinOnce();
                hsv_filter_ = filterImage(O, hsv_min, hsv_max);
                cv::imshow("Image", hsv_filter_);

                image_morph_ = morphImage(hsv_filter_, erosion_size_, dilation_size_);
                cv::imshow("Morph", image_morph_);

                key = cv::waitKey(50);
                */

                //Auskommentieren
                key = esc;
                /// <---
            }
            if(key == esc)
                state_ = DONE;
            if(key == enter)
            {
                // save morph values to yaml-file
                save_morph(filename_morph_);
                state_ = DETECT;
            }
            break;
        }
        case DETECT:
        {
            // initiallize new windows
            cv::destroyWindow("Image");
            cv::destroyWindow("Morph");
            cv::destroyWindow("Slider");

            cv::namedWindow("Detection");
            cv::moveWindow("Detection", 400, header_height_ + height_offset_);

            key = -1;
            oberstufenlabor_mechatronik_vision::Circle circle;

            hsv_min = cv::Scalar(filter_color[2][0], filter_color[2][2], filter_color[2][4], 0);
            hsv_max = cv::Scalar(filter_color[2][1], filter_color[2][3], filter_color[2][5], 0);

            while(key != enter && key != esc)
            {
                /*************************************************************************************
                     (C) Aufg:	Entfernen Sie für diesen Aufgabenteil die Kommentierungen und kommentieren
                                Sie die Zeile "key = esc;" aus.

                **************************************************************************************/
                /// --->
                /* Kommentierung entfernen
                ros::spinOnce();

                hsv_filter_ = filterImage(O, hsv_min, hsv_max);
                image_morph_ = morphImage(hsv_filter_, erosion_size_, dilation_size_);

                circle = searchForCircles(image_morph_);

                image_morph_.copyTo(image_circle_);
                image_circle_ = drawCircle(image_circle_, circle);

                cv::imshow("Detection", image_circle_);

                key = cv::waitKey(50);
                */

                // Auskommentieren
                key = esc;
                /// <---
            }
            if(key == esc || key == enter)
                state_ = DONE;
            break;
        }
        case DONE:
        {
            arm_.moveToPose("HOME");
            arm_.waitForCurrentAction();
            IsRunning = false;
            break;
        }
        }
    }
    ros::shutdown();
}

//############## IMAGE FILTER ####################################################################################
cv::Mat exercise02_library::filterImage(cv::Mat image, cv::Scalar hsv_min, cv::Scalar hsv_max)
{
    cv::Mat colorFilter;

    /*************************************************************************************
         (A) Aufg:	Führen Sie eine Farbraumkonvertierung der Bildmatrix "image",
                    die im BGR-Format vorliegt, in den HSV-Farbraum durch.
                    Deklarieren Sie hierzu eine Bildmatrix zur Aufnahme der HSV-Bilddaten.

                    Implementieren Sie einen Farbfilter im HSV-Farbraum. Achtung: Die
                    Schwellwerte hierfür befinden sich in den Variablen "hsv_min" und
                    "hsv_max", deren Werte mit Hilfe der bereits integrierten Schieberegler gesetzt werden.
                    Die gefilterte Matrix soll in die Bildmatrix "colorFilter" (siehe oben) kopiert werden.

         Hinweise:	http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html
                    http://docs.opencv.org/modules/core/doc/operations_on_arrays.html
                    http://docs.opencv.org/modules/core/doc/basic_structures.html

    **************************************************************************************/
    ///--->

    ///<---
    return colorFilter;
}

//############## MORPHOLOGIC OPERATIONS ####################################################################################
cv::Mat exercise02_library::morphImage(cv::Mat image, int erosion_size, int dilation_size)
{
    /************************************************************************************
         (B) Aufg:	Die Bildmatrix "image" wurde in Aufgabenteil A bereits in den HSV-Farbraum
                    konvertiert und anschließend wurde eine Farbfilterung im HSV-Farbraum durchgeführt.

                    Auf das resultierende Bild sind nun die morphologischen Operationen "Erosion"
                    und "Dilatation" nacheinander anzuwenden. Die Integervariablen "erosion_size"
                    und "dilation_size" enthalten die Kernelgrößen. Sie können später über Schieberegler
                    eingestellt werden.

                    Die Eingangsmatrix "image" soll gleichzeitig als Ausgangsmatrix dienen.

         Hinweise:	http://docs.opencv.org/doc/tutorials/imgproc/table_of_content_imgproc/table_of_content_imgproc.html
                    http://docs.opencv.org/modules/imgproc/doc/filtering.html

    **************************************************************************************/
    /// --->

    /// <---
    return image;
}

//############## FIND CIRCLES ####################################################################################
oberstufenlabor_mechatronik_vision::Circle exercise02_library::searchForCircles(cv::Mat image)
{
    oberstufenlabor_mechatronik_vision::Circle circle;
    cv::Point ball_center;
    std::vector<std::vector<cv::Point> > contours;

    int max_area = 0;
    double min_dist = HUGE_VAL;
    double roundness_threshold_ = 0.6;
    float min_radius_ = 10;
    float max_radius_ = 100;

    /************************************************************************************
         (C) Aufg:	Die Eingangsmatrix "image" soll nun auf Konturen hin untersucht werden,
                    um den Ball bzw. Kreis im Bild zu erkennen.

                    1.  Konvertieren Sie die Matrix "image" von RGB zu Graustufen. Durchsuchen
                        Sie anschließend das konvertierte Bild nach Konturen. Gefundene Konturen
                        sollen im Vektor "contours" (siehe oben) gespeichert werden.

                    2.  Ermitteln Sie den kleinsten Kreis für die gefundenen Konturen. Die
                        Variable "i" gibt hierbei die jeweils aktuelle Kontur im Vektor "contours"
                        an. Der Mittelpunkt und Radius des Kreises soll in den von Ihnen zu deklarierenden
                        Variablen "center" und "radius" gespeichert werden. Entfernen Sie außerdem die
                        Kommentierung ab der gekennzeichneten Zeile.

         Hinweise:	http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html

    **************************************************************************************/

    /// Aufgabenteil 1
    /// --->

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

            /// <---
            /* Kommentierung entfernen
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
            }*/
        }

        ball_center.x = circle.x;
        ball_center.y = circle.y;

        std::stringstream ss;
        ss << "Press Enter / ESC to quit. Detected Circle: x(" << ball_center.x << ") y(" << ball_center.y << ") r(" << circle.r << ")";
        write(ss.str());
    }
    return circle;
}

//############## DRAW CIRCLES ####################################################################################
cv::Mat exercise02_library::drawCircle(cv::Mat image, oberstufenlabor_mechatronik_vision::Circle circle)
{

    /************************************************************************************
         (D) Aufg:	In die Eingangsmatrix "image" soll der gefundene Kreis eingezeichnet werden.
                    Die Informationen über Mittelpunkt und Radius befinden sich in der
                    Variablen "circle". Verwenden Sie eine beliebige Farbe, Liniendicke und -typ
                    für den Kreis.

         Hinweise:	http://docs.opencv.org/modules/core/doc/drawing_functions.html

    **************************************************************************************/
    ///--->

    ///<---

    return image;
}

//############## WRITE #################################################################################################
void exercise02_library::write(std::string text)
{
    int baseline;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 1, 1, &baseline);
    double scale = std::min(0.8 * header_height_ / text_size.height, 0.9 * C.cols / text_size.width);

    cv::Mat textbox = cv::Mat::zeros(header_height_,C.cols,CV_8UC3);
    cv::putText(textbox, text, cv::Point(10, header_height_ - baseline),
                cv::FONT_HERSHEY_DUPLEX, scale, cv::Scalar(255, 255, 255), 1, CV_AA);
    cv::imshow("Textbox",textbox);
}

//########## LOAD FILTER ###############################################################################################
void exercise02_library::load_filter(std::string filename)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filename);
        if(node.IsNull())
        {
            ROS_ERROR("config.yaml is empty. Writing default filter values...");
            int cache[3][6];
            for(int n = 0; n < 3; n++)
            {
                for(int idx = 0; idx < 6; idx++)
                {
                    cache[n][idx] = 0;
                }
            }
            save_filter(filename, cache);
            try
            {
                node = YAML::LoadFile(filename);
            }
            catch(YAML::Exception e)
            {
                std::cout << e.what() << std::endl;
            }
        }

        ROS_INFO("Reading filter values...");
        int color;
        for (YAML::const_iterator it=node.begin(); it!=node.end(); ++it)
        {
            if(it->first.as<std::string>() == windowNames[0])
                color = 0;
            else if(it->first.as<std::string>() == windowNames[1])
                color = 1;
            else if(it->first.as<std::string>() == windowNames[2])
                color = 2;
            YAML::Node params = it->second;

            for(int idx = 0; idx < 6; idx++)
            {
                slider_params_[color][idx] = params[trackbarNames[idx]].as<int>();
            }
        }
    }
    catch(YAML::Exception e)
    {
        std::cout << e.what() << std::endl;
    }
}

//########## LOAD MORPH ###############################################################################################
void exercise02_library::load_morph(std::string filename)
{
    try
    {
        YAML::Node node_morph = YAML::LoadFile(filename);
        if(node_morph.IsNull())
        {
            ROS_ERROR("morph.yaml is empty. Writing default morph values...");
            save_morph(filename);
            try
            {
                node_morph = YAML::LoadFile(filename);
            }
            catch(YAML::Exception e)
            {
                std::cout << e.what() << std::endl;
            }
        }

        ROS_INFO("Reading morph values...");
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

//########## SAVE FILTER ###############################################################################################
void exercise02_library::save_filter(std::string filename, int data[3][6])
{
    ROS_INFO("Writing new filter parameter...");
    std::ofstream fout(filename.c_str());

    YAML::Node node;

    // save the parameters to a config file
    for (int color_cnt = 0; color_cnt < 3; color_cnt++)
    {
        for (int param_cnt = 0; param_cnt < 6; param_cnt++)
        {
            node[windowNames[color_cnt]][trackbarNames[param_cnt]] = (int)data[color_cnt][param_cnt];
        }
    }

    fout << node << std::endl;
}

//########## SAVE MORPH ################################################################################################
void exercise02_library::save_morph(std::string filename)
{
    ROS_INFO("Writing new morph parameter...");
    std::ofstream fout(filename.c_str());

    YAML::Node node;
    node["Erosion"]["kernel_size"] = (int)erosion_size_;
    node["Dilation"]["kernel_size"] = (int)dilation_size_;

    fout << node << std::endl;
}

//############## COLOR MAP CALLBACK ####################################################################################
void exercise02_library::colorCallback(const sensor_msgs::ImageConstPtr &msg)
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
