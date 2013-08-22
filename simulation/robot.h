/*
 * robot.h
 *
 *  Created on: Jan 9, 2013
 *      Author: josef
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgSim/DOFTransform>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgGA/CameraManipulator>
#include <osg/ref_ptr>
#include "util/msTimer.h"

class RobotData : public osg::Referenced
{
public:
	struct RobotParameter{
		double kSpeed; // m/s
		double kPsiSpeed; // ?/s
		int kImpulesProMeter; //impulse / m
		double kDistanceWheels; //m
		double kSigmaIncrement;
	};
	explicit RobotData(osg::Node*n, osgGA::CameraManipulator* cam, RobotData::RobotParameter robotParameter);
	void UpdatePosition();
	void UpdateOrientation();
	void UpdateCamTransformation();
	void AddSpeed();
	void RemoveSpeed();
	void AddPsiSpeed();
	void RemovePsiSpeed();
	void UpdateIncrements();
	double RandomUniform();
	double RandomGaussian();
	RobotParameter parameter_;
	double incremente_left_;
	double incremente_right_;
	double speed_, psi_speed_;
	double x_pos_, y_pos_, psi_;
protected:
	osg::PositionAttitudeTransform* robotXform_;
private:
	msTimer timer_;
	osgGA::CameraManipulator* cam_on_robot_;
};

class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
		KeyboardEventHandler(RobotData* robdata);
    	virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
protected:
        RobotData* robotdata_;
        bool w_,a_,s_,d_;
};


osg::Group* SetupRobot(osgGA::CameraManipulator* cam, RobotData::RobotParameter robotParameter);


#endif /* ROBOT_H_ */
