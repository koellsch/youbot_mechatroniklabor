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
 * @file   main.cpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  Mono camera calibration programm
 */
#include <QtGui>
#include <QApplication>
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration_window.h>
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration_qtros.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief   Main program
 * @return  run status
 */
int main(int argc, char **argv) {
    QApplication app(argc, argv);
    qRegisterMetaType< cv::Mat >("cv::Mat");

    // create the ROS Node
    CameraCalibration_QtROS qtRos(argc, argv, "cameraCalibration");

    // create a GUI
    CameraCalibration_Window gui(qtRos.getImageSize(), qtRos.getCameraTopicName());
    gui.show();
    gui.resize(gui.width(), gui.height() - 276);

    // signals and slots
    app.connect(&app,   SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    app.connect(&qtRos, SIGNAL(rosShutdown()), &app, SLOT(quit()));
    app.connect(&qtRos, SIGNAL(signalNewFrame(cv::Mat)), &gui, SLOT(slotNewFrame(cv::Mat)));

    int result = app.exec();

    return result;
}
