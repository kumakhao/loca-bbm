/*
 * cameraParam.h
 *
 *  Created on: Feb 21, 2013
 *      Author: josef
 */

#ifndef CAMERAPARAM_H_
#define CAMERAPARAM_H_

#include <opencv.hpp>
#include <osg/Node>
#include <osgViewer/Viewer>

class cameraParam{
public:
	cv::Mat intrMat; //intrinsic parameter matrix
	cv::Mat rotMat; //rotation matrix
	//cv::Mat rotv; //rotation as Rodrigues Vector
	cv::Mat rotMat4x4;
	cv::Mat rotMat_World2Robot; //rotation matrix
	cv::Mat rotMat_Robot2CameraCV;
	cv::Mat transMat; //translation vector
	cv::Mat transMat4;
	cv::Mat distCoeffsVec; //distortion vector
	cameraParam();
	void projectTo2D(cv::Mat *objectPoints, std::vector<cv::Point2d> *imagePoints);
	void setFx(double fx);
	void setFy(double fy);
	void setCx(double cx);
	void setCy(double cy);
	void setIntrMat(osg::Matrix mat);
	void setExtr(double psi, double x, double y, double z);
	void setExtr(double psi, double x, double y);

	cv::Mat getDistCoeffsVec() const {
		return distCoeffsVec;
	}

	cv::Mat getIntrMat() const {
		return intrMat;
	}

	cv::Mat getRotMat() const {
		return rotMat;
	}

	cv::Mat getTransMat() const {
		return transMat;
	}
protected:
	double test;
	cv::Mat rotX(double angle);
	cv::Mat rotY(double angle);
	cv::Mat rotZ(double angle);
};


#endif /* CAMERA_H_ */
