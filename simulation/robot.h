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
#include <osg/ref_ptr>

class RobotData : public osg::Referenced
{
public:
	explicit RobotData(osg::Node*n);
	void UpdatePosition();
	void UpdateOrientation();
	void AddSpeed(double value);
	void AddPsiSpeed(double value);
	void UpdateIncrements();
	double RandomUniform();
	double RandomGaussian();
	double incremente_left_;
	double incremente_right_;
	double speed_, psi_speed_;
	double x_pos_, y_pos_, psi_;
protected:
	osg::PositionAttitudeTransform* robotXform_;
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


osg::Group* SetupRobot();


#endif /* ROBOT_H_ */
