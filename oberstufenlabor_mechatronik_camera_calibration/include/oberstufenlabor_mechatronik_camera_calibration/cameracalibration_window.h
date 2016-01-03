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
 * @file   cameracalibration_window.hpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the CameraCalibration_Window class providing gui functionality
 */
#ifndef CAMERACALIBRATION_WINDOW_H
#define CAMERACALIBRATION_WINDOW_H

#include <QString>
#include <QtGui/QMainWindow>
#include <QtGui>
#include <QtGui/QFileDialog>
#include <QTimer>

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "ui_cameracalibration_window.h"
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration.h>

/**
 * @class CameraCalibration_Window
 * @brief This class provides gui functionality for mono camera calibration
 */
class CameraCalibration_Window: public QMainWindow {
    Q_OBJECT

signals:

private slots:
    void on_btnStartStopCamera_clicked(bool check);
    void on_btnPreviewImage_clicked(bool check);
    void on_btnDiscardImage_clicked(bool check);
    void on_btnAcceptImage_clicked(bool check);
    void on_btnClearTrace_clicked(bool check);
    void on_btnPrevImage_clicked(bool check);
    void on_btnNextImage_clicked(bool check);
    void on_btnActivateImage_clicked(bool check);
    void on_btnAutomaticCapture_clicked(bool check);
    void on_cbShowCorners_clicked(bool check);


    /**
     * @brief This SLOT is called when the user selected calibration/load of the manu bar - a previous calibration is loaded afterwards
     */
    void on_actionCalibrationLoad_triggered();


    /**
     * @brief This SLOT is called when the user selected calibration/save of the menu bar - the method saves the calibration in yaml format
     */
    void on_actionCalibrationSave_triggered();


    /**
     * @brief This SLOT is called when the user selected calibration/settings of the menu bar - the method displays the calibration settings
     */
    void on_actionCalibrationSettings_triggered();


    /**
     * @brief This SLOT is called when the calibrate button is clicked
     */
    void on_btn_calibrate_clicked(bool check);


    /**
     * @brief This SLOT is called when the user selected file/quit of the menu bar - this ends the program
     */
    void on_actionQuit_triggered();


    void on_actionExportParameter_triggered();


    /**
     * @brief This SLOT is called when the user wants to export the calibration as ROS launchfile
     */
    void on_actionExportLaunchfile_triggered();


    /**
     * @brief This SLOT is called when the user wants to load previous calibration images
     */
    void on_actionImagesLoad_triggered();


    /**
     * @brief This SLOT is called when the user wants to save the calibration images
     */
    void on_actionImagesSaveAsPNG_triggered();


    /**
     * @brief This SLOT is called when the user wants to save the calibration images
     */
    void on_actionImagesSaveAsJPG_triggered();


    /**
     * @brief This SLOT is called when the user wants to save the calibration images
     */
    void on_actionImagesSaveAsBMP_triggered();


    /**
     * @brief This SLOT is called when the user wants to discard the calibration images to start from scratch
     */
    void on_actionImagesDiscard_triggered();


    /**
     * @brief TimerSLOT for automatic image capture
     */
    void timer_tick();

    /**
     * @brief slot for processing the new frame
     * @param frame new image
     */
    void slotNewFrame(cv::Mat frame);

public:
    CameraCalibration_Window(cv::Size _frameSize, std::string _cameraTopicName, QWidget *parent = 0);
    ~CameraCalibration_Window();

    void displayCalibrationSetting(bool _status);

private:
    static const int TIMER_INIT_VALUE = 3000;   ///< pause between each image capture

    Ui::CameraCalibration_WindowClass ui;
    CameraCalibration* cameraCalib;
    cv::Mat currentImage;
    bool currentImageActive;
    cv::Mat camImage;
    cv::Size imageSize;
    int currentIndex;
    int countDownValue;

    QSize initialWindowSize;

    bool captureFromCameraActive;
    bool currentSnapshotTaken;
    bool showUndistortion;
    QTimer *timer;                              ///< timer for automatic capture of images

    cv::Size frameSize;                         ///< saves the size of the received camera frames
    std::string cameraTopicName;                ///< saves the name of the received camera topic

    void trace(QString str);

    /**
     * @brief This method print a red error message into the log label
     * @param _str string to print
     */
    void traceERR(QString _str);


    /**
     * @brief This method print a green status message into the log label
     * @param _str string to print
     */
    void traceOK(QString _str);


    void getCurrentImage();
    void displayCurrentImage();
    void setCalibrationParameters();
    void drawCorners();
    void modifyGUI();


    /**
     * @brief When this method is called, the calibration is executed
     */
    void doCalibration();


    /**
     * @brief This Method saves the calibration images
     * @param _fileType filetype
     */
    void saveImagesAs(std::string _fileType);

protected:
    void resizeEvent(QResizeEvent *event);
};

#endif // STEREOCALIBRATION_WINDOW_H

