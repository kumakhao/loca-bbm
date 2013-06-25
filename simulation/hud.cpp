/*
 * hud.cpp
 *
 *  Created on: 25.06.2013
 *      Author: josef
 */


#include "hud.h"



HUD::HUD() {
	matrixTransform = new osg::MatrixTransform();
	projectionMatrix = new osg::Projection();
	textGeode = new osg::Geode();
	text = new osgText::Text();

	matrixTransform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	//Add the projection matrix to this transform
	matrixTransform->addChild(projectionMatrix);

	// Projection Matrix - We use a projection matrix because we need to
	// create a new "layer" upon which to draw the text. We create a
	// 2D orthographic projection.
	// Set our projection matrix to an orthographic 2D matrix with the
	// dimensions of (x_min, x_max, y_min, y_max) resolution.

	// osg::Matrix::ortho2D() is a static function that returns
	// a matrix with the properties described above.
	projectionMatrix->setMatrix(osg::Matrix::ortho2D(0,640,0,480));

	// Now add the text geometry to this new projection matrix surface
	projectionMatrix->addChild(textGeode);

	// Geode - Since osgText::Text is a derived class from drawable, we
	// must add it to an osg::Geode before we can add it to our ScenGraph.
	textGeode->addDrawable(text);

	//Set the screen alignment - always face the screen
	text->setAxisAlignment(osgText::Text::SCREEN);

	//Set the text to our default text string
	text->setText("Default Text \n tdfsadsa");
	text->setPosition(osg::Vec3d(0, 50, 0));
	text->setCharacterSize(12);
}

osg::Group* HUD::getGroup() {
	return matrixTransform;
}

void HUD::setText(std::string text) {
	this->text->setText(text);
}

void HUD::setFont(std::string font) {
	this->text->setFont(font);
}

void HUD::setPosition(osg::Vec3d position) {
	this->text->setPosition(position);
}

void HUD::setTextSize(unsigned int size) {
	this->text->setCharacterSize(size);
}
