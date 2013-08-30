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

int main(int argc, char** argv){
	osg::setNotifyLevel(osg::WARN);
	std::string mode_argument = "NULL";
	int mode = 1;
	if(argc > 1)
		mode_argument = argv[1];
	if(mode_argument == "0"){
		std::cout<<"OneParticleMode chosen."<<std::endl;
		mode = 0;
	}
	if(mode_argument == "1"){
		std::cout<<"Standard Simulation chosen."<<std::endl;
		mode = 1;
	}
	if(mode_argument == "2"){
		std::cout<<"framework Mode chosen."<<std::endl;
		mode = 2;
	}
	if(mode_argument == "3"){
		std::cout<<"pic frequency Simulation chosen."<<std::endl;
		mode = 3;
	}
	if(mode_argument == "4"){
		std::cout<<"croud Simulation chosen."<<std::endl;
		mode = 4;
	}
	std::cout<<""<<std::endl;

	Simulation mainSim;
	localisation *mainLoca = new localisation;


	std::vector<double> orientaion, position;


	// 0 = picTest
	// 1 = sim
	// 2 = framework
	// 3 = pic frequency Simulation
	// 4 = croud Simulation

	cv::Mat img1;
	cv::Mat img2;
	cv::Mat img3;

	switch(mode) {
	case 0:

		mainSim.settings_.robParameter_.kSigmaIncrement = 0;
		mainSim.settings_.robParameter_.kRadiusWheelLeft = 0.08;
		mainSim.settings_.robParameter_.kRadiusWheelRight = 0.08;
		mainLoca->param.sigmaAngle = 0;
		mainLoca->param.sigmaDistance = 0;
		mainLoca->initilisation_done_ = true;
		mainSim.setLocalisation(mainLoca);
		mainSim.setObserveMode(Simulation::OneParticle);
		mainSim.enablePadControl();
		mainSim.Initialize();
		mainSim.Realize();

		while(!mainSim.done()){
			mainSim.Step();
		}
		//mainSim.WriteRobotTrajectory();
		mainSim.CleanUp();
		break;
	case 1:
		mainLoca->landmarks.addLandmark(1,-5.0,-5.0);
		mainLoca->landmarks.addLandmark(2,5.0,-5.0);
		mainLoca->landmarks.addLandmark(3,-5.0,5.0);
		mainLoca->landmarks.addLandmark(4,5.0,5.0);

		mainLoca->initilisation_done_ = false;
		mainSim.setLocalisation(mainLoca);
		//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
		mainSim.setObserveMode(Simulation::Pictures);
		mainSim.enablePadControl();
		mainSim.settings_.takepicture_intervall_ = 2000;
		mainSim.settings_.robParameter_.kSpeed = 0.05;
		mainSim.Initialize();
		mainSim.Realize();
		//mainSim.ReadRobotTrajectory("/home/josef/workspace/Loca-Projekt/trajectorys/2013-08-30_0757");
		while(!mainSim.done()){
			mainSim.Step();
			//usleep(10000);
//			orientaion = mainLoca->getOrientation();
//			position = mainLoca->getPosition();
//			std::cout<<"Angle: "<<orientaion.at(1)<<" | VAR: "<<orientaion.at(0)<<std::endl;
//			std::cout<<"X: "<<position.at(0)<<" | VAR: "<<position.at(1)<<std::endl;
//			std::cout<<"Y: "<<position.at(2)<<" | VAR: "<<position.at(3)<<std::endl;

		}
		mainSim.WriteRobotTrajectory();
		mainSim.CleanUp();
		break;
	case 2:
		expoBot_test();
		;
		break;
	case 3:
		for(int pic_time = 500; pic_time <= 1000; pic_time += 500){
			Simulation mainSim;
			localisation *mainLoca = new localisation;
			mainSim.setLocalisation(mainLoca);
			//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
			mainSim.setObserveMode(Simulation::Pictures);
			mainSim.enablePadControl();
			std::stringstream datafilename;
			datafilename << "/home/josef/Desktop/hallowelt_"<<pic_time<<".txt";
			mainSim.settings_.datafile_name_ = datafilename.str();
			mainSim.settings_.picture_path_ = "/home/josef/Desktop/";
			mainSim.settings_.takepicture_intervall_ = pic_time;
			std::stringstream plotfile;
			plotfile << "/home/josef/Desktop/plotfile_"<<pic_time;
			mainSim.settings_.plotfile_ = plotfile.str();
			mainSim.Initialize();
			mainSim.Realize();
			mainSim.ReadRobotTrajectory("/home/josef/workspace/Loca-Projekt/trajectorys/picsEvery500ms_2013-08-22_1112");
			while(!mainSim.done()){
				mainSim.Step();
			}
			std::stringstream trajectoryfile;
			trajectoryfile << "/home/josef/Desktop/trajectory_"<<pic_time;
			mainSim.WriteRobotTrajectory(trajectoryfile.str());
			mainSim.CleanUp();
		}
		break;
	case 4:
		for(int croud_size = 0; croud_size <= 23; croud_size++){
			Simulation mainSim;
			localisation *mainLoca = new localisation;
			mainSim.setLocalisation(mainLoca);
			//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
			mainSim.setObserveMode(Simulation::Pictures);
			mainSim.enablePadControl();
			std::stringstream datafilename;
			datafilename << "/home/josef/workspace/Loca-Projekt/plots/croudEffect/LocaData_"<<croud_size<<".txt";
			mainSim.settings_.crowd_size_ = croud_size;
			mainSim.settings_.takepicture_intervall_ = 1000;
			mainSim.settings_.datafile_name_ = datafilename.str();
			mainSim.settings_.picture_path_ = "/home/josef/workspace/Loca-Projekt/pictures/croudEffect/";
			std::stringstream plotfile;
			plotfile << "/home/josef/workspace/Loca-Projekt/plots/croudEffect/plotfile_"<<croud_size;
			mainSim.settings_.plotfile_ = plotfile.str();
			mainSim.Initialize();
			mainSim.Realize();
			mainSim.ReadRobotTrajectory("/home/josef/workspace/Loca-Projekt/trajectorys/picsEvery500ms_2013-08-22_1112");
			while(!mainSim.done()){
				mainSim.Step();
			}
			std::stringstream trajectoryfile;
			trajectoryfile << "/home/josef/Desktop/trajectory_"<<croud_size;
			mainSim.WriteRobotTrajectory(trajectoryfile.str());
			mainSim.CleanUp();
		}
		break;
	default:
		;
		break;
	}
	return 0;
}
