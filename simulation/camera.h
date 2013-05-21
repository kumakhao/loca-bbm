/*
 * camera.h
 *
 *  Created on: Jan 13, 2013
 *      Author: josef
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <osg/Node>
#include <osgViewer/Viewer>

struct UpdateAccumlatedMatrix : public osg::NodeCallback
{
	osg::Matrix matrix;
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
		{
		matrix = osg::computeWorldToLocal(nv->getNodePath());
		traverse(node,nv);
		}
};

struct TransformAccumulator
{
public:
	TransformAccumulator();
	bool attachToGroup(osg::Group* g);
	osg::Matrix getMatrix();
protected:
	osg::ref_ptr<osg::Group> parent;
	osg::Node* node;
	UpdateAccumlatedMatrix* mpcb;
};

class FollowNodeCameraManipulator : public osgGA::CameraManipulator
{
public:
	explicit FollowNodeCameraManipulator( TransformAccumulator* ta);
	bool handle (const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa);
	void updateTheCamera();
	virtual void setByMatrix(const osg::Matrixd& mat);
	virtual void setByInverseMatrix(const osg::Matrixd&mat);
	virtual osg::Matrixd getInverseMatrix() const;
	virtual osg::Matrixd getMatrix() const;
protected:
	~FollowNodeCameraManipulator(){}
	TransformAccumulator* world_coordinates_of_node_;
	osg::Matrixd the_matrix_;
};


#endif /* CAMERA_H_ */
