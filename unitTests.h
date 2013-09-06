/*
 * unitTests.h
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */

#ifndef UNITTESTS_H_
#define UNITTESTS_H_

#include <string>
#include "./localisation/cameraParam.h"
#include <osg/Node>
#include <osgViewer/Viewer>

class unitTests{
public:
	static bool landmarkTest();
	static double cameraCalibrationTest(double x, double y, double psi, std::string imgPath);
	static bool ratingEval(double x, double y, double psi, std::string imgPath);
	static bool picTest();
	static bool cvProjectTest();
	static bool compareProjection(cameraParam* camp, osg::Matrix MVPW);
	//
};

typedef double Transform[4][4];


#endif /* UNITTESTS_H_ */
