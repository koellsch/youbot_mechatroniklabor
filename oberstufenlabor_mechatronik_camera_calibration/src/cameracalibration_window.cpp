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
 * @file   cameracalibration_window.cpp
 * @author Andreas Schoob (andreas.schoob@imes.uni-hannover.de)
 * @author Benjamin Munske (benjamin.munske@imes.uni-hannover.de)
 * @date   5. September 2012
 *
 * @brief  This file contains the CameraCalibration_Window class providing gui functionality
 */
#include <oberstufenlabor_mechatronik_camera_calibration/cameracalibration_window.h>

using namespace std;
using namespace Qt;


CameraCalibration_Window::CameraCalibration_Window(cv::Size _frameSize, std::string _cameraTopicName, QWidget *parent)
    : QMainWindow(parent)
{
    this->ui.setupUi(this);

    this->frameSize = _frameSize;
    this->cameraTopicName = _cameraTopicName;
    this->cameraCalib = new CameraCalibration();

    this->ui.pbarCountDownTimer->setMaximum(TIMER_INIT_VALUE);

    captureFromCameraActive = false;
    currentSnapshotTaken = false;
    showUndistortion = false;
    currentIndex = 0;

    this->setCalibrationParameters();

    this->timer = new QTimer();
    connect(this->timer, SIGNAL(timeout()), this, SLOT(timer_tick()));
    this->timer->setInterval(100);
    this->timer->stop();

    this->initialWindowSize.setWidth(this->width());
    this->initialWindowSize.setHeight(this->height());

    this->displayCalibrationSetting(false);
}


void CameraCalibration_Window::resizeEvent(QResizeEvent *event)
{
    displayCurrentImage();
}


void CameraCalibration_Window::slotNewFrame(cv::Mat frame) {
    if (captureFromCameraActive) {
        frame.copyTo(this->camImage);

        if (this->ui.rbtn_calibrated->isChecked())
            this->cameraCalib->getCalibratedImage(frame, this->currentImage);
        else
            frame.copyTo(this->currentImage);

        currentImageActive = true;
        displayCurrentImage();
    }
}


void CameraCalibration_Window::modifyGUI()
{
    if (!showUndistortion)
    {
        if (captureFromCameraActive || currentSnapshotTaken) {
            ui.gbCalibrationParameters->setEnabled(false);
        }
        else {
            ui.gbCameraCapture->setEnabled(true);
            ui.gbCalibrationParameters->setEnabled(true);
        }

        if (currentSnapshotTaken)
        {
            ui.btnPreviewImage->setEnabled(false);
            ui.btnStartStopCamera->setEnabled(false);
            ui.btnDiscardImage->setEnabled(true);
            ui.btnAcceptImage->setEnabled(true);
        }
        else
        {
            ui.btnPreviewImage->setEnabled(true);
            ui.btnStartStopCamera->setEnabled(true);
            ui.btnDiscardImage->setEnabled(false);
            ui.btnAcceptImage->setEnabled(false);
        }
    }
    else {
        ui.gbCalibrationParameters->setEnabled(false);
        ui.gbCameraCapture->setEnabled(false);
    }

    this->ui.btnAutomaticCapture->setEnabled(captureFromCameraActive);
}


void CameraCalibration_Window::timer_tick() {
    if (this->countDownValue > 0) {
        this->ui.pbarCountDownTimer->setValue(this->countDownValue);
        this->countDownValue -= 100;

        if (this->countDownValue == this->TIMER_INIT_VALUE/2.)
            this->ui.lblImage->setStyleSheet("QLabel { background-color : transparent; border: 1px solid black; border-radius: 0px;}");
    }
    else {
        if (captureFromCameraActive) {
            cv::Mat image;

            if (cameraCalib->cornersFound(currentImage, image)) {
                this->cameraCalib->addImage(currentImage);
                this->ui.lblImage->setStyleSheet("QLabel { background-color : lightgreen; border: 1px solid black; border-radius: 0px;}");
            }
            else {
                this->ui.lblImage->setStyleSheet("QLabel { background-color : red; border: 1px solid black; border-radius: 0px;}");
            }
        }

        this->countDownValue = TIMER_INIT_VALUE;
    }
}

void CameraCalibration_Window::on_cbShowCorners_clicked(bool check)
{
    drawCorners();
    displayCurrentImage();
}

void CameraCalibration_Window::drawCorners()
{
    if (ui.cbShowCorners->isChecked())
    {
        cv::Mat image;

        if (cameraCalib->cornersFound(currentImage,image))
        {
            image.copyTo(currentImage);
        }
    }
    else //load new
    {
        cv::Mat image;

        image = cameraCalib->getImage(currentIndex);

        image.copyTo(currentImage);
    }
}


void CameraCalibration_Window::getCurrentImage()
{
    if (currentIndex >= 0 && currentIndex < cameraCalib->getImageCount() && cameraCalib->getImageCount() > 0) {
        cv::Mat image;

        image = cameraCalib->getImage(currentIndex);
        currentImageActive = cameraCalib->imageIsActivated(currentIndex);

        image.copyTo(currentImage);

        drawCorners();
    }
    else {
        currentImage = cv::Mat::zeros(1,1,CV_8UC1);
    }
    displayCurrentImage();
}

void CameraCalibration_Window::on_btnPrevImage_clicked(bool check)
{
    if (this->currentIndex > 0)
        currentIndex--;

    getCurrentImage();
}

void CameraCalibration_Window::on_btnNextImage_clicked(bool check)
{
    if (this->currentIndex < cameraCalib->getImageCount()-1)
        currentIndex++;

    getCurrentImage();
}


void CameraCalibration_Window::on_btnAutomaticCapture_clicked(bool check) {
    if (check) {
        this->ui.btnAutomaticCapture->setText("Stop automatic capture");

        this->countDownValue = TIMER_INIT_VALUE;
        this->ui.pbarCountDownTimer->setValue(this->countDownValue);
        this->ui.pbarCountDownTimer->setEnabled(true);

        this->timer->start();
    }
    else {
        this->ui.btnAutomaticCapture->setText("Start automatic capture");
        this->ui.pbarCountDownTimer->setEnabled(false);
        this->timer->stop();
    }
}


void CameraCalibration_Window::on_btnActivateImage_clicked(bool check)
{
    if (cameraCalib->getImageCount() > 0)
    {
        cameraCalib->activateImage(currentIndex, !currentImageActive);

        getCurrentImage();
    }
}


void CameraCalibration_Window::on_actionCalibrationLoad_triggered() {
    QString filename = QFileDialog::getOpenFileName(this,tr("Open camera calibration "), "",tr("*.yaml Files (*.yaml)"));

    if (filename.isEmpty())
        return;

    if (cameraCalib->loadCalibration(filename.toStdString()) == false) {
        traceERR("Loading calibration failed");
    }
    else {
        traceOK("Loaded calibration from file: " + filename);

        //read file
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;

        while (!file.atEnd()) {
             QByteArray line = file.readLine();
             if (!line.isEmpty()) trace(line);
        }
    }
}


void CameraCalibration_Window::on_actionCalibrationSave_triggered() {
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save camera calibration "),
                                                    "",
                                                    tr("*.yaml Files (*.yaml)"));

    if (filename.isEmpty())
        return;

    if (cameraCalib->saveCalibration(filename.toStdString()) != 0)
        traceERR("Saving calibration failed");
    else
        traceOK("Saved calibration to file: " + filename);
}


void CameraCalibration_Window::on_actionCalibrationSettings_triggered() {
    this->displayCalibrationSetting(true);
}


void CameraCalibration_Window::on_btn_calibrate_clicked(bool check) {
    this->doCalibration();
}


void CameraCalibration_Window::on_actionQuit_triggered() {
    this->close();
}


void CameraCalibration_Window::on_actionExportParameter_triggered() {
    // show save file dialog
    QString exportFileName = QFileDialog::getSaveFileName(this, tr("Export ROS parameters"),
                               QDir::homePath(),
                               tr("yaml (*.yaml)"));

    // check whether the use has selected a file
    if (exportFileName.isEmpty())
        return;

    // get calibration parameter
    cv::Mat  cameraMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat  distCoeffs_   = cv::Mat::zeros(8, 1, CV_64FC1);
    cv::Size imageSize_;

    double D[8], K[9];

    if (this->cameraCalib->getCalibrationData(cameraMatrix_, distCoeffs_) && this->cameraCalib->getImageSize(imageSize_)) {
        // copy calibration parameter into simple arrays (more readable sourcecode)
        for (size_t idx = 0; idx < distCoeffs_.rows; idx++)
            D[idx] = distCoeffs_.at<double>(idx, 0);

        for (size_t m = 0; m < 3; m++)
            for (size_t n = 0; n < 3; n++)
                K[m*3 + n] = cameraMatrix_.at<double>(m, n);

        // open file
        QFile exportFile(exportFileName);
        exportFile.open(QIODevice::ReadWrite);

        // create stream for writing to the file
        QTextStream intoFile(&exportFile);
        intoFile.setRealNumberNotation(QTextStream::FixedNotation);
        intoFile.setRealNumberPrecision(8);

        intoFile << "D: [" << D[0];

        for (size_t idx = 1; idx < distCoeffs_.rows; idx++)
            intoFile << ", " << D[idx];

        intoFile << "]" << endl;
        intoFile << "K: [" << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3] << ", " << K[4] << ", " << K[5] << ", " << K[6] << ", " << K[7] << ", " << K[8] << "]" << endl;
        intoFile << "R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]" << endl;
        intoFile << "P: [" << K[0] << ", " << K[1] << ", " << K[2] << ", 0.0, " << K[3] << ", " << K[4] << ", " << K[5] << ", 0.0, " << K[6] << ", " << K[7] << ", " << K[8] << ", 0.0]" << endl << endl;

        exportFile.close();
    }
}


void CameraCalibration_Window::doCalibration() {
    setCalibrationParameters();
    double rms = cameraCalib->calibrate();

    if (rms != NULL)
        traceOK("Calibration succeeded with rms = " + QString::number(rms));
    else
        traceERR("Calibration failed!");
}


void CameraCalibration_Window::on_actionExportLaunchfile_triggered() {
    // show save file dialog
    QString exportFileName = QFileDialog::getSaveFileName(this, tr("Export ROS Launchfile"),
                               QDir::homePath(),
                               tr("launch (*.launch)"));

    // check whether the use has selected a file
    if (exportFileName.isEmpty())
        return;

    // get calibration parameter
    cv::Mat  cameraMatrix_ = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat  distCoeffs_   = cv::Mat::zeros(8, 1, CV_64FC1);
    cv::Size imageSize_;

    double D[8], K[9];

    if (this->cameraCalib->getCalibrationData(cameraMatrix_, distCoeffs_) && this->cameraCalib->getImageSize(imageSize_)) {

        // copy calibration parameter into simple arrays (more readable sourcecode)
        for (size_t idx = 0; idx < distCoeffs_.rows; idx++)
            D[idx] = distCoeffs_.at<double>(idx, 0);

        for (size_t m = 0; m < 3; m++)
            for (size_t n = 0; n < 3; n++)
                K[m*3 + n] = cameraMatrix_.at<double>(m, n);

        // open file
        QFile exportFile(exportFileName);
        exportFile.open(QIODevice::ReadWrite);

        // create stream for writing to the file
        QTextStream intoFile(&exportFile);
        intoFile.setRealNumberNotation(QTextStream::FixedNotation);
        intoFile.setRealNumberPrecision(8);

        // create contents of the launchfile
        intoFile << "<?xml version=\"1.0\"?>" << endl;
        intoFile << "<launch>" << endl;
        intoFile << "\t<!-- start the webcam Node -->" << endl;
        intoFile << "\t<node name=\"" << this->cameraTopicName.c_str() << "\" pkg=\"usb_cam\" type=\"usb_cam_node\" respawn=\"false\" output=\"screen\">" << endl;
        intoFile << "\t\t<rosparam param=\"D\">[" << D[0];

        for (size_t idx = 1; idx < distCoeffs_.rows; idx++)
            intoFile << ", " << D[idx];

        intoFile << "]</rosparam>" << endl;
        intoFile << "\t\t<rosparam param=\"K\">[" << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3] << ", " << K[4] << ", " << K[5] << ", " << K[6] << ", " << K[7] << ", " << K[8] << "]</rosparam>" << endl;
        intoFile << "\t\t<rosparam param=\"R\">[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]</rosparam>" << endl;
        intoFile << "\t\t<rosparam param=\"P\">[" << K[0] << ", " << K[1] << ", " << K[2] << ", 0.0, " << K[3] << ", " << K[4] << ", " << K[5] << ", 0.0, " << K[6] << ", " << K[7] << ", " << K[8] << ", 0.0]</rosparam>" << endl << endl;

        intoFile << "\t\t<param name=\"camera_frame_id\" value=\"" << this->cameraTopicName.c_str() << "\" type=\"string\" />" << endl;
        intoFile << "\t\t<param name=\"video_device\" type=\"string\" value=\"/dev/video0\" />" << endl;
        intoFile << "\t\t<param name=\"io_method\" type=\"string\" value=\"mmap\" />		<!-- possible values: mmap, read, userptr -->" << endl;
        intoFile << "\t\t<param name=\"image_width\" type=\"int\" value=\"" << imageSize_.width << "\" />" << endl;
        intoFile << "\t\t<param name=\"image_height\" type=\"int\" value=\"" << imageSize_.height << "\" />" << endl;
        intoFile << "\t\t<param name=\"pixel_format\" type=\"string\" value=\"yuyv\" />	<!-- possible values: yuyv, uyvy, mjpeg -->" << endl;
        intoFile << "\t\t<param name=\"/camera/compressed_image_transport_jpeg_quality\" value=\"40\" />" << endl;
        intoFile << "\t</node>" << endl;
        intoFile << "</launch>";

        exportFile.close();
    }
    else {
        QMessageBox::critical(this, tr("Export failed"), tr("Please do a calibration first before you try to export a ROS Launchfile."), QMessageBox::Ok);
    }
}


void CameraCalibration_Window::on_actionImagesLoad_triggered() {
    // images
    QStringList filenamesImages = QFileDialog::getOpenFileNames(this,tr("Load images "), QDir::homePath() ,tr("(*.jpg *.bmp *.png)"));

    if (filenamesImages.isEmpty() || filenamesImages.length() <= 0 )
        return;

    cv::Mat image;

    cameraCalib->deleteAllImages();

    for (size_t i = 0; i<filenamesImages.length();i++) {
        traceOK(filenamesImages.at(i));
        image = cv::imread(filenamesImages.at(i).toStdString());

        this->cameraCalib->addImage(image);
    }

    currentIndex = 0;
    setCalibrationParameters();
    getCurrentImage();
    traceOK("Images loaded successfully");
}


void CameraCalibration_Window::on_actionImagesSaveAsPNG_triggered() {
    this->saveImagesAs("png");
}


void CameraCalibration_Window::on_actionImagesSaveAsJPG_triggered() {
    this->saveImagesAs("jpg");
}


void CameraCalibration_Window::on_actionImagesSaveAsBMP_triggered() {
    this->saveImagesAs("bmp");
}


void CameraCalibration_Window::saveImagesAs(std::string _fileType) {
    if (cameraCalib->getImageCount() > 0 && (_fileType == "jpg" || _fileType == "png" || _fileType == "bmp")) {
        // directory
        QString saveDirectory = QFileDialog::getExistingDirectory(this,tr("Save images to directory "), QDir::homePath());

        for (int i = 0; i < cameraCalib->getImageCount();i++) {
            cv::Mat image;

            image = cameraCalib->getImage(i);

            string fileName = saveDirectory.toStdString() + "/Calib_" + QString::number(i+1).toStdString() + "." + _fileType;
            cv::imwrite(fileName,image);
        }
        traceOK("Images saved successfully to " + saveDirectory);
    }
}


void CameraCalibration_Window::on_actionImagesDiscard_triggered() {
    this->cameraCalib->deleteAllImages();

    this->currentSnapshotTaken = false;
    this->modifyGUI();

    this->setCalibrationParameters();
    currentIndex = 0;
    captureFromCameraActive = false;
    traceOK("All images discarded");
    getCurrentImage();
}


void CameraCalibration_Window::trace(QString str) {
    this->ui.pteTrace->append(str);
}


void CameraCalibration_Window::traceERR(QString _str) {
    this->ui.pteTrace->append("<font color=red>" + _str + "</font>");
}


void CameraCalibration_Window::traceOK(QString _str) {
    this->ui.pteTrace->append("<font color=green>" + _str + "</font>");
}


void CameraCalibration_Window::setCalibrationParameters()
{
    //read parameter from user interface
    //pattern type
    int patternType=0;
    if (ui.rbCircleGrid->isChecked()) patternType = 1;
    if (ui.rbAsymCircleGrid->isChecked()) patternType = 2;

    //pattern size
    double patternSize = ui.sbPatternSize->value();
    //corners
    int cornersX = ui.sbCornersX->value();
    int cornersY = ui.sbCornersY->value();

    //alpha overlay for live image during calibration process
    double alphaOverlay = ui.sbAlphaOverlay->value();

    cameraCalib->resetCalibrationParameters();
    cameraCalib->setCalibrationParameters(patternType,
                                          patternSize,
                                          cornersX,
                                          cornersY,
                                          alphaOverlay,
                                          ui.cbk1->isChecked(),
                                          ui.cbk2->isChecked(),
                                          ui.cbk3->isChecked(),
                                          ui.cbk4->isChecked(),
                                          ui.cbk5->isChecked(),
                                          ui.cbk6->isChecked(),
                                          ui.cbp1p2->isChecked(),
                                          ui.cbFixAspectRatio->isChecked(),
                                          ui.cbRationalModel->isChecked(),
                                          ui.cbSameFocalLength->isChecked(),
                                          ui.cbCalibZeroDisparity->isChecked());

}


void CameraCalibration_Window::on_btnPreviewImage_clicked(bool check)
{
    if (captureFromCameraActive)
    {
        captureFromCameraActive = false;

        cv::Mat image;

        if (cameraCalib->cornersFound(currentImage, image))
        {
            image.copyTo(currentImage);
            currentSnapshotTaken = true;
        }
        else
            captureFromCameraActive = true;

        displayCurrentImage();
        modifyGUI();
    }
}

void CameraCalibration_Window::on_btnDiscardImage_clicked(bool check)
{
    currentSnapshotTaken = false;
    captureFromCameraActive = true;
    modifyGUI();
}

void CameraCalibration_Window::on_btnAcceptImage_clicked(bool check) {
    if (!captureFromCameraActive)
    {
        this->cameraCalib->addImage(camImage);
        currentSnapshotTaken = false;
        captureFromCameraActive = true;
        modifyGUI();
    }
}

void CameraCalibration_Window::on_btnStartStopCamera_clicked(bool check) {
    if (!captureFromCameraActive)
        setCalibrationParameters();

    this->captureFromCameraActive = !this->captureFromCameraActive;

    if (!captureFromCameraActive) {
        currentIndex = cameraCalib->getImageCount()-1;
        getCurrentImage();
    }
    modifyGUI();
}


void CameraCalibration_Window::on_btnClearTrace_clicked(bool check) {
	ui.pteTrace->clear();
}


void CameraCalibration_Window::displayCurrentImage() {
    if (currentImage.empty())
        return;

    cv::Mat resizedImage;
    QImage qImage;
    cv::Size newSize;
    if ((double)ui.lblImage->height()*(double)currentImage.cols/(double)currentImage.rows > (double)ui.lblImage->width())
        newSize = cv::Size(ui.lblImage->width(),(int)((double)ui.lblImage->width()*(double)currentImage.rows/(double)currentImage.cols));
    else
        newSize = cv::Size((int)((double)ui.lblImage->height()*(double)currentImage.cols/(double)currentImage.rows),ui.lblImage->height());;

    cv::resize(currentImage,resizedImage,newSize);

    if (!currentImageActive)
        cv::addWeighted(resizedImage,1-0.5,cv::Scalar(0,0,255),0.5,0.0,resizedImage);

    if (resizedImage.channels() == 3)
        cv::cvtColor(resizedImage,resizedImage,CV_BGR2RGB);
    else
        cv::cvtColor(resizedImage,resizedImage,CV_GRAY2RGB);

    qImage = QImage((uchar*)(resizedImage.data),resizedImage.cols,resizedImage.rows,resizedImage.step,QImage::Format_RGB888);

    ui.lblImage->setPixmap(QPixmap::fromImage(qImage, Qt::ColorOnly));
    ui.lblImage->resize(ui.lblImage->pixmap()->size());

    ui.lblImageCount->setText(QString::number(currentIndex+1) + "/" + QString::number(cameraCalib->getImageCount()));
}


void CameraCalibration_Window::displayCalibrationSetting(bool _status) {
    this->ui.gbCalibrationParameters->setVisible(_status);

    if (_status)
        this->resize(this->initialWindowSize);
    else {
        this->resize(this->initialWindowSize.width(), this->initialWindowSize.height() - 276);
        this->updateGeometry();
    }
}


CameraCalibration_Window::~CameraCalibration_Window(){
    // NOP
}
