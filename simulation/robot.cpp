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


RobotData::RobotData(osg::Node* n)
{
	parameter_.kSpeed = 0.1;
	parameter_.kPsiSpeed = 0.05;
	parameter_.kImpulesProMeter = 57694;
	parameter_.kDistanceWheels = 0.07;
	parameter_.kSigmaIncrement = 0.2;
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
//	double bogenStrecke = speed_*timer_.last_step_time_*0.000001;
//	double dPsi = psi_speed_*timer_.last_step_time_*0.000001;
//	double sehnenStrecke = 0;
//	if(dPsi != 0)
//		sehnenStrecke = 2.0*(bogenStrecke/dPsi)*sin(dPsi/2);
//	else
//		sehnenStrecke = bogenStrecke;
//	x_pos_ += sehnenStrecke*cos(psi_+dPsi);
//	y_pos_ += sehnenStrecke*sin(psi_+dPsi);
	x_pos_ += cos(psi_)*speed_;//*timer_.last_step_time_*0.000001;
	y_pos_ += sin(psi_)*speed_;//*timer_.last_step_time_*0.000001;
	robotXform_->setPosition(osg::Vec3(x_pos_,y_pos_,0));
}

void RobotData::UpdateOrientation()
{
	psi_ += psi_speed_;//*timer_.last_step_time_*0.000001;
	robotXform_->setAttitude(osg::Quat(psi_, osg::Vec3d(0.0, 0.0, 1.0)));
}

void RobotData::AddSpeed()
{
	speed_ += parameter_.kSpeed;
}

void RobotData::RemoveSpeed()
{
	speed_ -= parameter_.kSpeed;
}

void RobotData::AddPsiSpeed()
{
	psi_speed_ += parameter_.kPsiSpeed;
}

void RobotData::RemovePsiSpeed()
{
	psi_speed_ -= parameter_.kPsiSpeed;
}
void RobotData::UpdateIncrements()
{
	//TODO: The angle of each wheel (true "physical" state)
	//should be computed and then saved befor conversion to increments.
	double deltaIncL = speed_//*timer_.last_step_time_*0.000001	// speed -> distance
						*parameter_.kImpulesProMeter 			// distance -> impluses
						-
						psi_speed_//*timer_.last_step_time_*0.000001	// rotSpeed -> rotAngle
						*parameter_.kDistanceWheels					// rotAngle -> arcLength
						*parameter_.kImpulesProMeter				// arcLength -> impluses
						/2.0;										// total impulses -> impulses for one wheel
	double deltaIncR = speed_//*timer_.last_step_time_*0.000001
						*parameter_.kImpulesProMeter
						+
						psi_speed_//*timer_.last_step_time_*0.000001
						*parameter_.kDistanceWheels
						*parameter_.kImpulesProMeter
						/2.0;
	double errIncL = 	parameter_.kSigmaIncrement
						*sqrt(abs(deltaIncL)+abs(deltaIncR))
						*RandomGaussian();
	double errIncR = 	parameter_.kSigmaIncrement
						*sqrt(abs(deltaIncL)+abs(deltaIncR))
						*RandomGaussian();
	incremente_left_ += round(deltaIncL + errIncL);
	incremente_right_ += round(deltaIncR + errIncR);
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
					if(!w_){robotdata_->AddSpeed();w_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_S):
				{
					if(!s_){robotdata_->RemoveSpeed();s_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_A):
				{
					if(!a_){robotdata_->AddPsiSpeed();a_=true;}
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_D):
				{
					if(!d_){robotdata_->RemovePsiSpeed();d_=true;}
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
					robotdata_->RemoveSpeed();
					w_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_S):
				{
					robotdata_->AddSpeed();
					s_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_A):
				{
					robotdata_->RemovePsiSpeed();
					a_=false;
					return true;
				}
			case(osgGA::GUIEventAdapter::KEY_D):
				{
					robotdata_->AddPsiSpeed();
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
	osg::Vec3 robot_position(0,0,0);
	robotXform->setPosition( robot_position );
	robotXform->setAttitude(osg::Quat(0, osg::Vec3d(0.0, 0.0, 1.0)));

	//set userData for the transformation used by a Callback
	RobotData* robotdata = new RobotData(robotXform);
	robotXform->setUserData(robotdata);
	//create and add a Callback for the robot movement
	robotXform->setUpdateCallback(new RobotNodeCallback);

	return (osg::Group*)robotXform;
}





