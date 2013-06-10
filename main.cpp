/*
 * main.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: josef
 */


#include <osgGA/NodeTrackerManipulator>
#include <opencv.hpp>

#include "locaUtil.h"
#include "unitTests.h"

#include "simulation/camera.h"
#include "simulation/dataToFileWriter.h"
#include "simulation/particles.h"
#include "simulation/robot.h"
#include "simulation/screenShotCallback.h"
#include "simulation/simulation.h"

#include "localisation/cameraParam.h"
#include "localisation/localisation.h"
#include "framework_expobot/expoBot_test.h"

int main(){
	Simulation mainSim;
	localisation *mainLoca = new localisation;



	// 0=picTest  1=sim  2=framework

	switch(1) {
	case 0:
		//unitTests::landmarkTest();
		//unitTests::cvProjectTest();
		unitTests::picTest();

//		cv::Mat img1 = locaUtil::makeWall(locaUtil::getPatternCode93(),0);
//		cv::Mat img2 = locaUtil::makeWall(locaUtil::getPatternCode93(),1);
//		cv::Mat img3 = locaUtil::makeWall(locaUtil::getPatternCode93(),2);
//		cv::imwrite("wall1.jpg",img1);
//		cv::imwrite("wall2.jpg",img2);
//		cv::imwrite("wall3.jpg",img3);
		break;
	case 1:
		mainLoca->landmarks.addLandmark(1,-5.0,-5.0);
		mainLoca->landmarks.addLandmark(2,5.0,-5.0);
		mainLoca->landmarks.addLandmark(3,-5.0,5.0);
		mainLoca->landmarks.addLandmark(4,5.0,5.0);

		mainSim.setLocalisation(mainLoca);
		//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
		mainSim.setObserveMode(Simulation::Landmarks);
		mainSim.enablePadControl();
		mainSim.Initialize();
		mainSim.Realize();
		while(!mainSim.done()){
			mainSim.Step();
		}
		break;
	case 2:
		expoBot_test();
		;
		break;
	default:
		;
		break;
	}
	return 0;
}
