/*
 * hud.h
 *
 *  Created on: 25.06.2013
 *      Author: josef
 */

#ifndef HUD_H_
#define HUD_H_

#include <osgText/Text>
#include <osg/Geode>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osg/Transform>

class HUD{
public:
	osg::MatrixTransform * matrixTransform;
	osg::Projection * projectionMatrix;
	osg::Geode * textGeode;
	osgText::Text * text;

	HUD();
	~HUD(){}

	osg::Group* getGroup();
	void setText(std::string text);
	void setFont(std::string font);
	void setPosition(osg::Vec3d position);
	void setTextSize(unsigned int size);
};


#endif /* HUD_H_ */
