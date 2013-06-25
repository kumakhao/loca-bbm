/*
 * cameraParam.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: josef
 */

#include "cameraParam.h"
#include <opencv.hpp>
#include <highgui/highgui_c.h>
#include <calib3d/calib3d.hpp>

cameraParam::cameraParam() {
	intrMat = cv::Mat(3, 3, CV_64FC1);
//	intrMat.at<double>(0, 0) = 2076.92;	intrMat.at<double>(0, 1) = 0.0;		intrMat.at<double>(0, 2) = 1920.0 / 2;
//	intrMat.at<double>(1, 0) = 0.0;		intrMat.at<double>(1, 1) = 2076.92;	intrMat.at<double>(1, 2) = 1080.0 / 2;
//	intrMat.at<double>(2, 0) = 0.0;		intrMat.at<double>(2, 1) = 0.0;		intrMat.at<double>(2, 2) = 1.0;
	intrMat.at<double>(0, 0) = 1476.92;	intrMat.at<double>(0, 1) = 0.0;		intrMat.at<double>(0, 2) = 1366.0 / 2;
	intrMat.at<double>(1, 0) = 0.0;		intrMat.at<double>(1, 1) = 1476.92;	intrMat.at<double>(1, 2) = 768.0 / 2;
	intrMat.at<double>(2, 0) = 0.0;		intrMat.at<double>(2, 1) = 0.0;		intrMat.at<double>(2, 2) = 1.0;
	rotMat = cv::Mat(3, 3, CV_64FC1);
	//rotv = cv::Mat(3, 1, CV_64FC1);
	rotMat4x4 = cv::Mat(4, 4, CV_64FC1);
	rotMat_World2Robot = cv::Mat(4, 4, CV_64FC1);
	rotMat_Robot2CameraCV = cv::Mat(4, 4, CV_64FC1);
			rotMat_Robot2CameraCV.at<double>(0,0) = 0;	rotMat_Robot2CameraCV.at<double>(0,1) = -1;rotMat_Robot2CameraCV.at<double>(0,2) = 0;	rotMat_Robot2CameraCV.at<double>(0,3) = -0.093889;
			rotMat_Robot2CameraCV.at<double>(1,0) = 0;	rotMat_Robot2CameraCV.at<double>(1,1) = 0;	rotMat_Robot2CameraCV.at<double>(1,2) = -1;rotMat_Robot2CameraCV.at<double>(1,3) = 0.260804;
			rotMat_Robot2CameraCV.at<double>(2,0) = 1; rotMat_Robot2CameraCV.at<double>(2,1) = 0;	rotMat_Robot2CameraCV.at<double>(2,2) = 0;	rotMat_Robot2CameraCV.at<double>(2,3) = 2.83599;
			rotMat_Robot2CameraCV.at<double>(3,0) = 0;	rotMat_Robot2CameraCV.at<double>(3,1) = 0;	rotMat_Robot2CameraCV.at<double>(3,2) = 0;	rotMat_Robot2CameraCV.at<double>(3,3) = 1;
	transMat = cv::Mat(3, 1, CV_64FC1);
	distCoeffsVec = cv::Mat(4, 1, CV_64FC1);
	distCoeffsVec.at<double>(0) = 0.0; //k1
	distCoeffsVec.at<double>(1) = 0.0; //k2
	distCoeffsVec.at<double>(2) = 0.0; //p1
	distCoeffsVec.at<double>(3) = 0.0; //p2
}

void cameraParam::setFx(double fx) {
	intrMat.at<double>(0, 0) = fx;
}

void cameraParam::setFy(double fy) {
	intrMat.at<double>(1, 1) = fy;
}

void cameraParam::setCx(double cx) {
	intrMat.at<double>(0, 2) = cx;
}

void cameraParam::setCy(double cy) {
	intrMat.at<double>(1, 2) = cy;
}

void cameraParam::setExtr(double psi, double x, double y, double z) {

	cv::Mat testTranslMatr = cv::Mat(4, 4, CV_64FC1);
	testTranslMatr.at<double>(0,0) = 1; testTranslMatr.at<double>(0,1) = 0; testTranslMatr.at<double>(0,2) = 0; testTranslMatr.at<double>(0,3) = -x;//-x-2.83599;//cos(psi)*x+sin(psi)*y;
	testTranslMatr.at<double>(1,0) = 0; testTranslMatr.at<double>(1,1) = 1; testTranslMatr.at<double>(1,2) = 0; testTranslMatr.at<double>(1,3) = -y;//-y-0.093889;//sin(psi)*x+cos(psi)*y;
	testTranslMatr.at<double>(2,0) = 0; testTranslMatr.at<double>(2,1) = 0; testTranslMatr.at<double>(2,2) = 1; testTranslMatr.at<double>(2,3) = 0;
	testTranslMatr.at<double>(3,0) = 0; testTranslMatr.at<double>(3,1) = 0; testTranslMatr.at<double>(3,2) = 0; testTranslMatr.at<double>(3,3) = 1;

	rotMat_World2Robot.at<double>(0,0) = cos(-psi);	rotMat_World2Robot.at<double>(0,1) = -sin(-psi);rotMat_World2Robot.at<double>(0,2) = 0;rotMat_World2Robot.at<double>(0,3) = 0;//cos(psi)*x+sin(psi)*y;
	rotMat_World2Robot.at<double>(1,0) = sin(-psi);	rotMat_World2Robot.at<double>(1,1) =  cos(-psi);rotMat_World2Robot.at<double>(1,2) = 0;rotMat_World2Robot.at<double>(1,3) = 0;//sin(psi)*x+cos(psi)*y;
	rotMat_World2Robot.at<double>(2,0) = 0; 		rotMat_World2Robot.at<double>(2,1) =  0; 		rotMat_World2Robot.at<double>(2,2) = 1; rotMat_World2Robot.at<double>(2,3) = 0;
	rotMat_World2Robot.at<double>(3,0) = 0;		rotMat_World2Robot.at<double>(3,1) =  0; 		rotMat_World2Robot.at<double>(3,2) = 0; rotMat_World2Robot.at<double>(3,3) = 1;

	rotMat4x4 = rotMat_Robot2CameraCV*rotMat_World2Robot*testTranslMatr;
	transMat.at<double>(0,0) = rotMat4x4.at<double>(0,3);
	transMat.at<double>(1,0) = rotMat4x4.at<double>(1,3);
	transMat.at<double>(2,0) = rotMat4x4.at<double>(2,3);

	rotMat.at<double>(0,0) = rotMat4x4.at<double>(0,0);
	rotMat.at<double>(1,0) = rotMat4x4.at<double>(1,0);
	rotMat.at<double>(2,0) = rotMat4x4.at<double>(2,0);
	rotMat.at<double>(0,1) = rotMat4x4.at<double>(0,1);
	rotMat.at<double>(1,1) = rotMat4x4.at<double>(1,1);
	rotMat.at<double>(2,1) = rotMat4x4.at<double>(2,1);
	rotMat.at<double>(0,2) = rotMat4x4.at<double>(0,2);
	rotMat.at<double>(1,2) = rotMat4x4.at<double>(1,2);
	rotMat.at<double>(2,2) = rotMat4x4.at<double>(2,2);

}

void cameraParam::setExtr(double psi, double x, double y) {
	setExtr(psi, x, y, 0.0);
}

cv::Mat cameraParam::rotX(double angle) {
	cv::Mat rotX(3,3,CV_64FC1);
		rotX.at<double>(0,0) = 1.0; 	rotX.at<double>(0,1) = 0.0;			rotX.at<double>(0,2) = 0.0;
		rotX.at<double>(1,0) = 0.0;	rotX.at<double>(1,1) = cos(angle);		rotX.at<double>(1,2) = -sin(angle);
		rotX.at<double>(2,0) = 0.0;	rotX.at<double>(2,1) = sin(angle);		rotX.at<double>(2,2) = cos(angle);
	return rotX;
}

cv::Mat cameraParam::rotY(double angle) {
	cv::Mat rotY(3,3,CV_64FC1);
		rotY.at<double>(0,0) = cos(angle); 	rotY.at<double>(0,1) = 0.0;	rotY.at<double>(0,2) = sin(angle);
		rotY.at<double>(1,0) = 0.0;			rotY.at<double>(1,1) = 1.0;	rotY.at<double>(1,2) = 0.0;
		rotY.at<double>(2,0) = -sin(angle);	rotY.at<double>(2,1) = 0.0;	rotY.at<double>(2,2) = cos(angle);
	return rotY;
}

void cameraParam::projectTo2D(cv::Mat* objectPoints,
		std::vector<cv::Point2d>* imagePoints) {
	cv::projectPoints(*objectPoints, rotMat, transMat, intrMat, distCoeffsVec, *imagePoints);
}

void cameraParam::setIntrMat(osg::Matrix mat) {
	/**
	 * Setting the intrinsic Matrix for the openCV projection.
	 * mat: is the product of projectionMatrix and windowMatrix.
	 * mat = projectionMatrix*windowMatrix;
	 * windowMatrix = viewer.getCamera()->getViewport()->computeWindowMatrix();
	 * projectionMatrix = viewer.getCamera()->getProjectionMatrix();
	 */
	setFx(mat(0,0));
	setFy(mat(1,1));
	setCx(-mat(2,0));
	setCy(-mat(2,1));
}

cv::Mat cameraParam::rotZ(double angle) {
	cv::Mat rotZ(3,3,CV_64FC1);
		rotZ.at<double>(0,0) = cos(angle); 	rotZ.at<double>(0,1) = -sin(angle);	rotZ.at<double>(0,2) = 0.0;
		rotZ.at<double>(1,0) = sin(angle);		rotZ.at<double>(1,1) = cos(angle);		rotZ.at<double>(1,2) = 0.0;
		rotZ.at<double>(2,0) = 0.0;			rotZ.at<double>(2,1) = 0.0;			rotZ.at<double>(2,2) = 1.0;
	return rotZ;
}

