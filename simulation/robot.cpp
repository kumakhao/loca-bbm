/*
 * robot.cpp
 *
 *  Created on: Jan 9, 2013
 *      Author: josef
 */
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgSim/DOFTransform>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <assert.h>
#include "robot.h"



//parameter
const double kSpeed = 2; // m/s
const double kPsiSpeed = 0.5; // ?/s
const int kImpulesProMeter = 57694; //impulse / m
const double kDistanceWheels = 0.07; //m
const double kSigmaIncrement = 0.0;

RobotData::RobotData(osg::Node* n)
{
	incremente_left_ = 0;
	incremente_right_ = 0;
	x_pos_ = 0;
	y_pos_ = 0;
	psi_ = 0.0;
	speed_ = 0;
	psi_speed_ = 0;
	timer_.Start();
	robotXform_ =
	 dynamic_cast <osg::PositionAttitudeTransform*> (n);
}

void RobotData::UpdatePosition()
{
	x_pos_ += cos(psi_)*speed_*timer_.last_step_time_*0.000001;
	y_pos_ += sin(psi_)*speed_*timer_.last_step_time_*0.000001;
	robotXform_->setPosition(osg::Vec3(x_pos_,y_pos_,0));
}

void RobotData::UpdateOrientation()
{
	psi_ += psi_speed_*timer_.last_step_time_*0.000001;
	robotXform_->setAttitude(osg::Quat(psi_, osg::Vec3d(0.0, 0.0, 1.0)));
}

void RobotData::AddSpeed(double value)
{
	speed_ += value;
}

void RobotData::AddPsiSpeed(double value)
{
	psi_speed_ += value;
}

void RobotData::UpdateIncrements()
{
	double deltaIncL = speed_*timer_.last_step_time_*0.000001*kImpulesProMeter -psi_speed_*timer_.last_step_time_*0.000001*kDistanceWheels*kImpulesProMeter/2.0;
	double deltaIncR = speed_*timer_.last_step_time_*0.000001*kImpulesProMeter +psi_speed_*timer_.last_step_time_*0.000001*kDistanceWheels*kImpulesProMeter/2.0;
	double errIncL = kSigmaIncrement*sqrt(abs(deltaIncL)+abs(deltaIncR))*RandomGaussian();
	double errIncR = kSigmaIncrement*sqrt(abs(deltaIncL)+abs(deltaIncR))*RandomGaussian();
	incremente_left_ += deltaIncL + errIncL;
	incremente_right_ += deltaIncR + errIncR;
}

double RobotData::RandomUniform()
{
	return ((double) rand())/RAND_MAX;
}

void RobotData::UpdateTimer() {
	timer_.getTimeStep();
}

double RobotData::RandomGaussian()
{
	double u = RandomUniform(), v = RandomUniform();
	return sqrt(-2*log(u))*cos(2*M_PI*v);
}

bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
{
	switch(ea.getEventType())
	{
		case(osgGA::GUIEventAdapter::KEYDOWN):
		{
			switch(ea.getKey())
			{
			case(osgGA::GUIEventAdapter::KEY_W):
				{
					if(!w_){robotdata_->AddSpeed(kSpeed);w_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_S):
				{
					if(!s_){robotdata_->AddSpeed(-kSpeed);s_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_A):
				{
					if(!a_){robotdata_->AddPsiSpeed(kPsiSpeed);a_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_D):
				{
					if(!d_){robotdata_->AddPsiSpeed(-kPsiSpeed);d_=true;}
					return true;
				}
			default:
				return false;
			}
			return false;
		}
		case(osgGA::GUIEventAdapter::KEYUP):
		{
			switch(ea.getKey())
			{
			case(osgGA::GUIEventAdapter::KEY_W):
				{
					robotdata_->AddSpeed(-kSpeed);
					w_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_S):
				{
					robotdata_->AddSpeed(kSpeed);
					s_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_A):
				{
					robotdata_->AddPsiSpeed(-kPsiSpeed);
					a_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_D):
				{
					robotdata_->AddPsiSpeed(kPsiSpeed);
					d_=false;
					return true;
				}
			default:
				return false;
			}
			return false;
		}
		default:
			return false;
	}
}


KeyboardEventHandler::KeyboardEventHandler(RobotData* robdata)
{
	robotdata_ = robdata;
	w_ = false;
	a_ = false;
	s_ = false;
	d_ = false;
}

class RobotNodeCallback : public osg::NodeCallback
{
public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osg::ref_ptr<RobotData> robotdata =
				dynamic_cast<RobotData*> (node->getUserData());
		if(robotdata)
		{
			robotdata->UpdateTimer();
			robotdata->UpdatePosition();
			robotdata->UpdateOrientation();
			robotdata->UpdateIncrements();
		}
		traverse(node, nv);
	}
};

osg::Group* SetupRobot()
{
	//load the robot Model
	osg::Node* robot_node = NULL;
	robot_node = osgDB::readNodeFile("/home/josef/NPS_Tutorials_src/NPS_Data/Models/t72-tank/t72-tank_des.flt");
	osg::PositionAttitudeTransform* robot_model_turn = new osg::PositionAttitudeTransform();
	robot_model_turn->addChild(robot_node);
	robot_model_turn->setAttitude(osg::Quat(-M_PI/2, osg::Vec3d(0.0, 0.0, 1.0)));

	//creat a transformation to manipulate the robot position and orientation
	osg::PositionAttitudeTransform* robotXform = new osg::PositionAttitudeTransform();

	//add the roboter to the tranformation
	robotXform->addChild(robot_model_turn);
	//scale the model down
	robotXform->setScale(osg::Vec3d(0.2,0.2,0.2));

	// Declare and initialize a Vec3 instance to change the
	// position of the robot model in the scene
	osg::Vec3 tank_position(0,0,0);
	robotXform->setPosition( tank_position );
	robotXform->setAttitude(osg::Quat(0, osg::Vec3d(0.0, 0.0, 1.0)));

	//set userData for the transformation used by a Callback
	RobotData* robotdata = new RobotData(robotXform);
	robotXform->setUserData(robotdata);
	//create and add a Callback for the robot movement
	robotXform->setUpdateCallback(new RobotNodeCallback);

	return (osg::Group*)robotXform;
}





