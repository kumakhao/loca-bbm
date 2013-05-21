/*
 * camera.cpp
 *
 *  Created on: Jan 9, 2013
 *      Author: josef
 */

#include <osg/Group>
#include <osg/View>
#include <osgGA/CameraManipulator>
#include "camera.h"

TransformAccumulator::TransformAccumulator()
{
	parent = NULL;
	node = new osg::Node;
	mpcb = new UpdateAccumlatedMatrix();
	node->setUpdateCallback(mpcb);
}

osg::Matrix TransformAccumulator::getMatrix()
{
	return mpcb->matrix;
}

bool TransformAccumulator::attachToGroup(osg::Group* g)
{
	bool success = false;
	if (parent != NULL)
	{
		int n = parent->getNumChildren();
		for(int i = 0; i < n; i++)
		{
			if (node == parent->getChild(i))
			{
				parent->removeChild(i,1);
				success = true;
			}
		}
		if(!success)
		{
			return success;
		}
	}
	g->addChild(node);
	return true;
}

FollowNodeCameraManipulator::FollowNodeCameraManipulator(TransformAccumulator* ta)
{
	world_coordinates_of_node_ = ta; the_matrix_ = osg::Matrixd::identity();
}

void FollowNodeCameraManipulator::updateTheCamera()
{
	the_matrix_ = world_coordinates_of_node_->getMatrix();
}

osg::Matrixd FollowNodeCameraManipulator::getMatrix() const
{
	return the_matrix_;
}

osg::Matrixd FollowNodeCameraManipulator::getInverseMatrix() const
{
	osg::Matrixd m;
	m = the_matrix_ * osg::Matrix::rotate(-M_PI/2.0, osg::Vec3(1,0,0));
	return m;
}

void FollowNodeCameraManipulator::setByMatrix(const osg::Matrixd& mat)
{
	the_matrix_ = mat;
}

void FollowNodeCameraManipulator::setByInverseMatrix(const osg::Matrixd& mat)
{
	the_matrix_ = mat.inverse(mat);
}

bool FollowNodeCameraManipulator::handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa)
{
	switch(ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::FRAME):
		{
			updateTheCamera();
			return false;
		}
	default:
		return false;
	}
	return false;
}
