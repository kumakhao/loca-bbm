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


RobotData::RobotData(osg::Node* n, osgGA::CameraManipulator* cam, RobotData::RobotParameter robotParameter)
{
	parameter_ = robotParameter;
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
	cam_on_robot_ = cam;
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

void RobotData::UpdateCamTransformation() {
	osg::Matrix transMat = cam_on_robot_->getMatrix();
	double x1 = -M_PI*(110.0/180.0);
	double c1 = cos(x1);
	double c2 = cos(-psi_+(M_PI/2));
	double c3 = 1;//cos(x3);
	double s1 = sin(x1);
	double s2 = sin(-psi_+(M_PI/2));
	double s3 = 0;//sin(x3);
	transMat(0,0) =  c2; 				transMat(0,1) = -c3*s2; 			transMat(0,2) = s2*s3; 				transMat(0,3) =  0;
	transMat(1,0) =  c1*s2; 			transMat(1,1) = c1*c2*c3-s1*s3;		transMat(1,2) = -c3*s1-c1*c2*s3;	transMat(1,3) =  0;
	transMat(2,0) =  s1*s2;				transMat(2,1) = c1*s3+c2*c3*s1;		transMat(2,2) = c1*c3-c2*s1*s3;		transMat(2,3) =  0;
	transMat(3,0) =  x_pos_;			transMat(3,1) = y_pos_;				transMat(3,2) = 0.5;				transMat(3,3) =  1;
	cam_on_robot_->setByMatrix(transMat);
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
			robotdata->UpdatePosition();
			robotdata->UpdateOrientation();
			robotdata->UpdateCamTransformation();
			robotdata->UpdateIncrements();
		}
		traverse(node, nv);
	}
};

osg::Group* MakeRobot()
{
	osg::Geode* robot_model = new osg::Geode();
	osg::Geometry* robot_geo = new osg::Geometry();
	robot_model->addDrawable(robot_geo);
	double left=-0.2, right=0.2;
	double bottom=0.0, top=0.4;
	double front=-0.2, back=0.2;

	// Specify the vertices:
	osg::Vec3Array* robot_vertices = new osg::Vec3Array;
	robot_vertices->push_back( osg::Vec3(left, front, bottom) ); // front left bottom
	robot_vertices->push_back( osg::Vec3(right, front, bottom) ); // front right bottom
	robot_vertices->push_back( osg::Vec3(right, back, bottom) ); // back right bottom
	robot_vertices->push_back( osg::Vec3(left, back, bottom) ); // back left bottom
	robot_vertices->push_back( osg::Vec3(left, back, top) ); // back left top
	robot_vertices->push_back( osg::Vec3(right, back, top) ); // back right top
	robot_vertices->push_back( osg::Vec3(right, front, top) ); // front right top
	robot_vertices->push_back( osg::Vec3(left, front, top) ); // front left top

	robot_vertices->push_back( osg::Vec3(left, front, bottom) ); // front left bottom
	robot_vertices->push_back( osg::Vec3(right, front, bottom) ); // front right bottom
	robot_vertices->push_back( osg::Vec3(left, back, bottom) ); // back left bottom
	robot_vertices->push_back( osg::Vec3(left, back, top) ); // back left top
	robot_vertices->push_back( osg::Vec3(right, back, bottom) ); // back right bottom
	robot_vertices->push_back( osg::Vec3(right, back, top) ); // back right top

//	particle_vertices->push_back( osg::Vec3(0.1, 0.1, 0.0) ); // back right bottom
//	particle_vertices->push_back( osg::Vec3(0.0, 0.1, 0.0) ); // back left bottom
//	particle_vertices->push_back( osg::Vec3(0.0, 0.0, 0.0) ); // front left bottom
//	particle_vertices->push_back( osg::Vec3(0.1, 0.0, 0.0) ); // front right bottom
//	particle_vertices->push_back( osg::Vec3(0.1, 0.1, 0.1) ); // back right top
//	particle_vertices->push_back( osg::Vec3(0.0, 0.1, 0.1) ); // back left top
//	particle_vertices->push_back( osg::Vec3(0.0, 0.0, 0.1) ); // front left top
//	particle_vertices->push_back( osg::Vec3(0.1, 0.0, 0.1) ); // front right top

	robot_geo->setVertexArray( robot_vertices );

	osg::DrawElementsUInt* quadrat_ground =
			new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_ground->push_back(3); // front right bottom
		quadrat_ground->push_back(2); // front left bottom
		quadrat_ground->push_back(1); // back left bottom
		quadrat_ground->push_back(0); // back right bottom
		robot_geo->addPrimitiveSet(quadrat_ground);

	osg::DrawElementsUInt* quadrat_back =
				new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_back->push_back(5); // back right top
		quadrat_back->push_back(4); // back left top
		quadrat_back->push_back(3); // back left bottom
		quadrat_back->push_back(2); // back right bottom
		robot_geo->addPrimitiveSet(quadrat_back);

	osg::DrawElementsUInt* quadrat_top =
				new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_top->push_back(4); // back right top
		quadrat_top->push_back(5); // back left top
		quadrat_top->push_back(6); // front left top
		quadrat_top->push_back(7); // front right top
		robot_geo->addPrimitiveSet(quadrat_top);

	osg::DrawElementsUInt* quadrat_right =
				new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_right->push_back(6); // back right bottom
		quadrat_right->push_back(9); // front right bottom
		quadrat_right->push_back(12); // front right top
		quadrat_right->push_back(13); // back right top
		robot_geo->addPrimitiveSet(quadrat_right);

	osg::DrawElementsUInt* quadrat_left =
				new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_left->push_back(7); // back right bottom
		quadrat_left->push_back(8); // front right bottom
		quadrat_left->push_back(10); // front right top
		quadrat_left->push_back(11); // back right top
		robot_geo->addPrimitiveSet(quadrat_left);

	osg::DrawElementsUInt* quadrat_front =
				new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
		quadrat_front->push_back(6); // back right bottom
		quadrat_front->push_back(7); // front right bottom
		quadrat_front->push_back(8); // front right top
		quadrat_front->push_back(9); // back right top
		robot_geo->addPrimitiveSet(quadrat_front);

	osg::Vec2Array* texcoords = new osg::Vec2Array(14);
		(*texcoords)[0].set(0.4f,0.0f); // tex coord for vertex 0
		(*texcoords)[1].set(0.6f,0.0f); // tex coord for vertex 1
		(*texcoords)[2].set(0.6f,0.2f); // ""
		(*texcoords)[3].set(0.4f,0.2f); // ""
		(*texcoords)[4].set(0.4f,0.4f);
		(*texcoords)[5].set(0.6f,0.4f);
		(*texcoords)[6].set(0.6f,0.6f);
		(*texcoords)[7].set(0.4f,0.6f);
		(*texcoords)[8].set(0.4f,0.8f); // ""
		(*texcoords)[9].set(0.6f,0.8f); // ""
		(*texcoords)[10].set(0.2f,0.8f);
		(*texcoords)[11].set(0.2f,0.6f);
		(*texcoords)[12].set(0.8f,0.8f);
		(*texcoords)[13].set(0.8f,0.6f);
		robot_geo->setTexCoordArray(0,texcoords);

	osg::Vec4Array* colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
	colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	robot_geo->setColorArray(colors);
	robot_geo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	return (osg::Group*) robot_model;
}

osg::Group* SetupRobot(osgGA::CameraManipulator* cam, RobotData::RobotParameter robotParameter)
{
	//load the robot Model
	osg::Node* robot_node = MakeRobot();
	//robot_node = osgDB::readNodeFile("models/t72-tank/t72-tank_des.flt");
	osg::PositionAttitudeTransform* robot_model_turn = new osg::PositionAttitudeTransform();
	robot_model_turn->addChild(robot_node);
	robot_model_turn->setAttitude(osg::Quat(-M_PI/2, osg::Vec3d(0.0, 0.0, 1.0)));

	//creat a transformation to manipulate the robot position and orientation
	osg::PositionAttitudeTransform* robotXform = new osg::PositionAttitudeTransform();

	//add the roboter to the tranformation
	robotXform->addChild(robot_model_turn);
	//scale the model down
	//robotXform->setScale(osg::Vec3d(4,4,4));

	// Declare and initialize a Vec3 instance to change the
	// position of the robot model in the scene
	osg::Vec3 robot_position(0,0,0);
	robotXform->setPosition( robot_position );
	robotXform->setAttitude(osg::Quat(0, osg::Vec3d(0.0, 0.0, 1.0)));

	//set userData for the transformation used by a Callback
	RobotData* robotdata = new RobotData(robotXform, cam, robotParameter);
	robotXform->setUserData(robotdata);
	//create and add a Callback for the robot movement
	robotXform->setUpdateCallback(new RobotNodeCallback);

	return (osg::Group*)robotXform;
}





