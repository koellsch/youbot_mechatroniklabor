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
 * @file   cameracalibration.cpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the CameraCalibration class providing mono camera calibration functionality
 */
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration.h>

using namespace std;
using namespace cv;


CameraCalibration::CameraCalibration() {
    this->patternType = CHESSBOARD;
    this->calibrationSuccessful = false;
    this->patternSize = 15.0;
    this->autosaveFilename = "camera_parameters_autosave.yaml";
    this->imageCount = 0;
    this->imageActivatedCount = 0;
    this->alphaOverlayImage = 0.3;
    this->colorOverlayImage = cv::Scalar(0,0,0);

    this->undistortionMap = new cv::Mat[2];
}

CameraCalibration::~CameraCalibration()
{
}

double CameraCalibration::getCalibratedImageArea() //in percent
{
    vector<cv::Point> p;
    cv::Rect rec;
    int pixels;

    rec = cv::boundingRect(pointsBufTemp);
    p.push_back(cv::Point(rec.x,rec.y));
    p.push_back(cv::Point(rec.x+rec.width,rec.y));
    p.push_back(cv::Point(rec.x+rec.width,rec.y+rec.height));
    p.push_back(cv::Point(rec.x,rec.y+rec.height));

    cv::fillConvexPoly(calibratedImageArea,&(p[0]),4,cv::Scalar(255));

    pixels = calibratedImageArea.rows * calibratedImageArea.cols;
    return (double)cv::countNonZero(calibratedImageArea) / pixels;
}

void CameraCalibration::overlayImages(cv::Mat img, cv::Mat overlayImg, double alpha, cv::Mat dstImg)
{
    cv::addWeighted(img,1-alpha,overlayImg,alpha,0.0,dstImg);
}

void CameraCalibration::resetCalibrationParameters()
{
    calibratedImageArea = cv::Mat(this->imageSize.height,this->imageSize.width,CV_8U,cv::Scalar(0));

    pointsBufTemp.clear();
    pointsBuf.clear();
    allPoints_undistorted.clear();
    pointsBuf_undistorted.clear();
    allPoints.clear();
    allObjectPoints.clear();
}

double CameraCalibration::calibrate()
{
    if( this->imageActivatedCount > 0)
    {
        //reset calibration conditions
        resetCalibrationParameters();

        //start calibration
        vector<cv::Mat> activeImages;
        cv::Mat tempImage;

        if (this->imageCount)
            this->imageSize = this->images.at(0).size();

        for (int i = 0; i < this->imageCount;i++)
        {
            if (imageActivated.at(i))
            {
                if (cornersFound(this->images.at(i),tempImage))
                {
                    activeImages.push_back(this->images.at(i));
                }
                else
                {
                    activateImage(i, false);
                }
            }
        }

        for (int i = 0; i < this->imageActivatedCount;i++)
        {
            if (cornersFound(activeImages.at(i),tempImage))
            {
                pushBackValidCorners(i);
            }
        }

        computeCalibration();
        computeReprojectionErrors();
        this->calibrationSuccessful = true;
        saveCalibration(autosaveFilename);
        return rmsErrorCameraCalibration;
//        return averageErrorCameraCalibration;
    }
    return NULL;
}

void CameraCalibration::setCalibrationParameters(int patternType,
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
                                                 bool calibZeroDisparity)
{
	this->patternType = (PatternType)patternType;
	this->patternSize = patternSize;
	this->cornersX = cornersX;
	this->cornersY = cornersY;
	this->cornersN = cornersX*cornersY;
        pointsBufTemp.resize(cornersN);
	this->alphaOverlayImage = alphaOverlay;

        this->calibrationFlags = 0;
        if (fixK1) this->calibrationFlags |= cv::CALIB_FIX_K1;
        if (fixK2) this->calibrationFlags |= cv::CALIB_FIX_K2;
        if (fixK3) this->calibrationFlags |= cv::CALIB_FIX_K3;
        if (fixK4) this->calibrationFlags |= cv::CALIB_FIX_K4;
        if (fixK5) this->calibrationFlags |= cv::CALIB_FIX_K5;
        if (fixK6) this->calibrationFlags |= cv::CALIB_FIX_K6;
        if (zeroTangentDist) this->calibrationFlags |= cv::CALIB_ZERO_TANGENT_DIST;
        if (fixAspectRatio) this->calibrationFlags |= cv::CALIB_FIX_ASPECT_RATIO;
        if (rationalModel) this->calibrationFlags |= cv::CALIB_RATIONAL_MODEL;
        if (sameFocalLength) this->calibrationFlags |= cv::CALIB_SAME_FOCAL_LENGTH;

        this->calibZeroDisparity = 0;
        if (calibZeroDisparity)
            this->calibZeroDisparity |= cv::CALIB_ZERO_DISPARITY;
}


bool CameraCalibration::imageIsActivated(int index)
{
    if (index >= 0 && index < this->imageCount)
    {
        return imageActivated.at(index);
    }
    else
        return false;
}

cv::Mat CameraCalibration::getImage(int index)
{
    cv::Mat tempImg;

    if (index >= 0 && index < this->imageCount)
    {
        images.at(index).copyTo(tempImg);
    }
    else if (index < 0)
    {
        images.at(0).copyTo(tempImg);
    }
    else
    {
        images.at(this->imageCount-1).copyTo(tempImg);
    }
    return tempImg;
}

void CameraCalibration::addImage(cv::Mat image)
{   
    //save current imagesize
    if (this->imageCount == 0)
        this->imageSize = image.size();
    else if (this->imageSize != image.size())
        return;

    cv::Mat temp;
    image.copyTo(temp);
    images.push_back(temp);

    this->imageCount++;
    this->imageActivatedCount++;
    this->imageActivated.push_back(true);
}

void CameraCalibration::deleteImage(int index)
{
    if (index >= 0 && index < this->imageCount)
    {
        images.erase(images.begin()+index);
        imageCount--;
        imageActivatedCount--;
    }
}


void CameraCalibration::activateImage(int index, bool activated)
{
    if (index >= 0 && index < this->imageCount)
    {
        if (imageActivated.at(index))
        {
            if (!activated)
            {
                imageActivated.at(index) = false;
                this->imageActivatedCount--;
            }
        }
        else
        {
            if (activated)
            {
                imageActivated.at(index) = true;
                this->imageActivatedCount++;
            }
        }
    }
}


void CameraCalibration::deleteAllImages()
{
    images.clear();
    imageCount = 0;
    imageActivatedCount = 0;
}


bool CameraCalibration::cornersFound(cv::Mat image, cv::Mat& imageWithCorners)
{
    cv::Mat imageGray;
    image.copyTo(imageWithCorners);

    bool found = false;

    //convert to gray
    if (image.channels() == 3)
        cv::cvtColor(image,imageGray,CV_BGR2GRAY);
    else
        image.copyTo(imageGray);

    switch (patternType)
    {
        case CHESSBOARD:
            //FIND CHESSBOARDS AND CORNERS THEREIN:
            found = cv::findChessboardCorners(imageGray, cv::Size(cornersX, cornersY),pointsBufTemp,
                                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
            break;
        case CIRCLES_GRID:
            found = cv::findCirclesGrid(imageGray,cv::Size(cornersX, cornersY),
                            pointsBufTemp);
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(imageGray,cv::Size(cornersX, cornersY), pointsBufTemp,  cv::CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            break;
    }

    if (found)
    {
        if(patternType == CHESSBOARD)
        { //optional
            //Calibration will suffer without subpixel interpolation
            cv::cornerSubPix(imageGray, pointsBufTemp,
                             cv::Size(11, 11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30, 0.1)
            );
        }

        cv::drawChessboardCorners(imageWithCorners,cv::Size(cornersX,cornersY),pointsBufTemp,true);
        return true;
    }
    else
    {
        return false;
    }
}


void CameraCalibration::pushBackValidCorners(int index)
{
    pointsBuf.resize((index+1)*cornersN);
    copy(pointsBufTemp.begin(), pointsBufTemp.end(),  pointsBuf.begin() + index*cornersN);
    allPoints.push_back(pointsBufTemp);
}

int CameraCalibration::getImageCount()
{
    return this->imageCount;
}


void CameraCalibration::computeCalibration()
{
    // camera calibration
    cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    distCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);

    //define geometrical dimensions
    calculateGeometricalCorners();

    //CALIBRATE THE STEREO CAMERAS
    rmsErrorCameraCalibration = cv::calibrateCamera(allObjectPoints, allPoints,this->imageSize,cameraMatrix,
                                                    distCoeffs,rvecs,tvecs,
                                                    this->calibrationFlags);

    cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0).copyTo(newCameraMatrix);

    computeReprojectionErrors();
    calculateUndistortionMaps();
}

void CameraCalibration::computeReprojectionErrors()
{

}

void CameraCalibration::calculateUndistortionMaps()
{
    //compute undistortion maps with calibrated parameter
    cv::initUndistortRectifyMap(cameraMatrix,distCoeffs,cv::Mat(),newCameraMatrix,this->imageSize,CV_16SC2,undistortionMap[0],undistortionMap[1]);
}

void CameraCalibration::calculateGeometricalCorners()
{
    // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    allObjectPoints.resize(imageActivatedCount);

    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for(int k=0;k<imageActivatedCount;k++)
        {
            for(int i = 0; i < cornersY; i++ )
                for(int j = 0; j < cornersX; j++ )
                {
                    allObjectPoints[k].push_back(cv::Point3f(float(j*patternSize),float(i*patternSize), 0.0f));
                }
        }
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for(int k=0;k<imageActivatedCount;k++)
        {
            for(int i = 0; i < cornersY; i++ )
                for(int j = 0; j < cornersX; j++ )
                {
                    allObjectPoints[k].push_back(cv::Point3f(float((2*j + i % 2)*patternSize),float(i*patternSize), 0.0f));
                }

        }
        break;

      default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
        break;
    }
}

bool CameraCalibration::saveCalibration(string filename)
{
    if(!this->calibrationSuccessful)
        return false;

    //save camera parameter
    cv::FileStorage fs(filename,cv::FileStorage::WRITE);
    //camera and calibration settings
    fs << "cameraWidth" << this->imageSize.width;
    fs << "cameraHeight" <<this->imageSize.height;
    fs << "patternType" << (int)patternType;
    fs << "patternSize" << patternSize;
    fs << "cornersX" << cornersX;
    fs << "cornersY" << cornersY;
    //calculated camera parameter
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "newCameraMatrix" << newCameraMatrix;

    fs << "rmsErrorCameraCalibration" << rmsErrorCameraCalibration;

    fs.release();

    return true;
}

bool CameraCalibration::loadCalibration(string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    //camera and calibration settings
    fs["cameraWidth"] >> this->imageSize.width;
    fs["cameraHeight"] >> this->imageSize.height;
    int pattern;
    fs["patternType"] >> pattern;
    patternType = (PatternType)pattern;
    fs["patternSize"] >> patternSize;
    fs["cornersX"] >> cornersX;
    fs["cornersY"] >> cornersY;
    //calculated camera parameter
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs["newCameraMatrix"] >> newCameraMatrix;

    fs["rmsErrorCameraCalibration"] >> rmsErrorCameraCalibration;

    fs.release();

    calculateUndistortionMaps();

    sparseCalibration = false;

    return true;
}


string CameraCalibration::getLastAutosaveFilename() {
    return autosaveFilename;
}


bool CameraCalibration::getCalibrationData(cv::Mat &_cameraMatrix,
                                           cv::Mat &_distCoeffs) {
    if (this->calibrationSuccessful) {
        this->cameraMatrix.copyTo(_cameraMatrix);
        this->distCoeffs.copyTo(_distCoeffs);

        return true;
    }
    else {
        _cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
        _distCoeffs   = cv::Mat::zeros(8, 1, CV_64FC1);

        return false;
    }
}


bool CameraCalibration::getCalibratedImage(cv::Mat &_raw, cv::Mat &_undistored) {
    if (this->calibrationSuccessful) {
        undistort(_raw, _undistored, this->cameraMatrix, this->distCoeffs, this->cameraMatrix);
    }
    else {
        _raw.copyTo(_undistored);
    }
}


bool CameraCalibration::getImageSize(cv::Size &_size) {
    if (this->imageCount) {
        _size = this->imageSize;
        return true;
    }
    else {
        _size = cv::Size(0,0);
        return false;
    }
}
