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
 * @file   cameracalibration.hpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the CameraCalibration class providing mono camera calibration functionality
 */
#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * @class CameraCalibration
 * @brief This class provides methods for mono camera calibration
 */
class CameraCalibration {
private:
    enum PatternType {CHESSBOARD,CIRCLES_GRID,ASYMMETRIC_CIRCLES_GRID};


    int cornersX,cornersY,cornersN;             ///< chessboard board corners X,Y, N = X*Y ,  number of corners = (number of cells - 1)
    int imageCount;
    int imageActivatedCount;
    std::vector<bool> imageActivated;

    //image source stereo camera system
    cv::Size imageSize;
    std::vector< cv::Mat > images;

    //general calibration parameters
    PatternType patternType;
    double patternSize;
    std::string autosaveFilename;
    int calibError;
    double rmsErrorCameraCalibration;
    double averageErrorCameraCalibration;
    double alphaOverlayImage;
    int calibrationFlags;
    bool sparseCalibration;
    bool calibrationSuccessful;                 ///< is set to true in case of successful calibration
    int calibZeroDisparity;
    cv::Scalar colorOverlayImage;
    cv::Mat calibratedImageArea;

    //camera parameters with calibrated rectification
    cv::Mat cameraMatrix;                       ///< camera matrix
    cv::Mat newCameraMatrix;                    ///< new camera matrix
    cv::Mat distCoeffs;                         ///< distortion coefficients
    std::vector<cv::Mat> rvecs,tvecs;


    cv::Mat *undistortionMap;                   ///< undistortion map

    //corners aquired during calibration
    std::vector<cv::Point2f> pointsBufTemp;
    std::vector<cv::Point2f> pointsBuf;
    std::vector<cv::Point2f> pointsBuf_undistorted;
    std::vector<std::vector<cv::Point2f> > allPoints;
    std::vector<std::vector<cv::Point2f> > allPoints_undistorted;
    std::vector<std::vector<cv::Point3f> > allObjectPoints;


    /* private calibration methods */
    void calculateGeometricalCorners();
    void pushBackValidCorners(int index);
    void calculateUndistortionMaps();
    void calculateUndistortionFromSparseCalibration();
    double getCalibratedImageArea();
    void overlayImages(cv::Mat img, cv::Mat overlayImg, double alpha, cv::Mat dstImg);
    void computeReprojectionErrors();
    void computeCalibration();

public:
    /**
     * @brief CameraCalibration class constructor
     */
    CameraCalibration();


    /**
     * @brief CameraCalibration class destructor
     */
    ~CameraCalibration();


    void setCalibrationParameters(int patternType,
                                  double patternSize,
                                  int cornersX, int cornersY,
                                  double alphaOverlay,
                                  bool fixK1,
                                  bool fixK2,
                                  bool fixK3,
                                  bool fixK4,
                                  bool fixK5,
                                  bool fixK6,
                                  bool zeroTangentDist,
                                  bool fixAspectRatio,
                                  bool rationalModel,
                                  bool sameFocalLength,
                                  bool calibZeroDisparity);

    void resetCalibrationParameters();

    cv::Mat getImage(int index);

    void addImage(cv::Mat image);

    void deleteImage(int index);

    void activateImage(int index, bool activated);

    bool imageIsActivated(int index);

    int getImageCount();

    bool cornersFound(cv::Mat image, cv::Mat& imageWithCorners);

    double calibrate();

    bool saveCalibration(std::string filename);

    bool loadCalibration(std::string filename);

    std::string getLastAutosaveFilename();


    /**
     * @brief This Method deletes als saves images
     */
    void deleteAllImages();


    /**
     * @brief This method returns the calibration data
     * @param &_cameraMatrix camera matrix
     * @param &_distCoeffs
     * @return true in case of valid/existing calibration data
     */
    bool getCalibrationData(cv::Mat &_cameraMatrix, cv::Mat &_distCoeffs);


    /**
     * @brief This method returns the undistored image
     * @param &_raw raw image matrix
     * @param &_undistored image
     * @return true in case of valid/existing calibration
     */
    bool getCalibratedImage(cv::Mat &_raw, cv::Mat &_undistored);


    /**
     * @brief This method returns the size of the calibrated images
     * @param &_size size of the image
     * @return true in case of existing images in the image vector
     */
    bool getImageSize(cv::Size &_size);
};

#endif // CAMERACALIBRATION_H

