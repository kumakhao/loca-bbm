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
	srand(time(NULL));

	std::string mode_argument = "-help";
	int mode = -1;
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
		std::cout<<"Pic Test Mode chosen."<<std::endl;
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
	if(mode_argument == "-help"){
		std::cout<<"Avalible modes are:"<<std::endl;
		std::cout<<"0 - One Particle Mode"<<std::endl;
		std::cout<<"1 - Standard Simulation"<<std::endl;
		std::cout<<"2 - Pic Test Mode"<<std::endl;
		std::cout<<"3 - Pic Frequency Simulation"<<std::endl;
		std::cout<<"4 - Croud Simulation"<<std::endl;
		return 0;
	}
	std::cout<<""<<std::endl;

	Simulation mainSim;
	localisation *mainLoca = new localisation;


	std::vector<double> orientaion, position;

	cv::Mat img1;
	cv::Mat img2;
	cv::Mat img3;

	switch(mode) {
	case 0:

		mainSim.settings_.robParameter_.kSigmaIncrement = 0;
		mainSim.settings_.robParameter_.kRadiusWheelLeft = 0.08;
		mainSim.settings_.robParameter_.kRadiusWheelRight = 0.08;
		mainSim.settings_.robParameter_.kLeftWheelWidth = 0.05;
		mainSim.settings_.robParameter_.kRightWheelWidth = 0.05;
		mainSim.settings_.sys_error_on_ = false;
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

		mainLoca->initilisation_done_ = true;
		mainSim.setLocalisation(mainLoca);
		//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
		mainSim.setObserveMode(Simulation::Pictures);
		mainSim.enablePadControl();
		mainSim.settings_.takepicture_intervall_ = 2000;
		mainSim.settings_.robParameter_.kSpeed = 0.05;
		mainSim.settings_.crowd_size_ = 25;
		mainSim.settings_.particle_visibility_ratio_ = 10;
		mainSim.settings_.sys_error_on_ = false;
		mainLoca->param.nrOfParticles = 2500;
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
		unitTests::picTest();
		;
		break;
	case 3:
	{
		std::vector<std::string> trajectorys;
		trajectorys.push_back("oval");
		trajectorys.push_back("serpentinen");
		trajectorys.push_back("clockwise");
		trajectorys.push_back("antiClockwise");
		for(unsigned int name = 0; name < trajectorys.size(); name++){
			std::stringstream filename;
			filename << "/home/josef/workspace/Loca-Projekt/plots/picFrequency/" << trajectorys.at(name) << "_data";
			std::ofstream testFile (filename.str().c_str());
			testFile << "freq " << trajectorys.at(name) << std::endl;
			for(int pic_time = 500; pic_time <= 1500; pic_time += 500){
				testFile << pic_time << " ";
				double counter = 0;
				double square_distance_sum = 0;
				for(int x=0; x<1;x++){
					Simulation mainSim;
					localisation *mainLoca_frequency = new localisation;
					mainLoca_frequency->initilisation_done_ = true;
					mainSim.settings_.particle_visibility_ratio_ = 10;
					mainSim.setLocalisation(mainLoca_frequency);
					//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
					mainSim.setObserveMode(Simulation::Pictures);
					mainSim.enablePadControl();

					mainSim.settings_.takepicture_intervall_ = pic_time;

					mainSim.Initialize();
					mainSim.Realize();
					std::stringstream trajPath;
					trajPath << "/home/josef/workspace/Loca-Projekt/trajectorys/"<<trajectorys.at(name);
					mainSim.ReadRobotTrajectory(trajPath.str());
					while(!mainSim.done()){
						mainSim.Step();
						localisation::EstimatedRobotPose* es = mainLoca_frequency->getEstimatedRobotPose();
						if(es->x < INFINITY){
						counter++;
						square_distance_sum += sqrt(pow(es->x-mainSim.getRobX(),2) + pow(es->y-mainSim.getRobY(),2));
						}
					}
//					std::stringstream trajectoryfile;
//					trajectoryfile << "/home/josef/Desktop/trajectory_"<<pic_time;
//					mainSim.WriteRobotTrajectory(trajectoryfile.str());
					mainSim.CleanUp();
				}
				square_distance_sum = square_distance_sum/counter;
				//meanErrors.push_back(square_distance_sum);
				testFile << square_distance_sum << std::endl;
			}
			testFile.close();
		}
		break;
	}
	case 4:
	{
		std::vector<std::string> trajectorys;
		trajectorys.push_back("oval");
		//trajectorys.push_back("serpentinen");
		//trajectorys.push_back("clockwise");
		//trajectorys.push_back("antiClockwise");
		for(unsigned int name = 0; name < trajectorys.size(); name++){
			std::stringstream filename;
			filename << "/home/josef/workspace/Loca-Projekt/plots/croudEffect/" << trajectorys.at(name) << "_data";
			std::ofstream testFile (filename.str().c_str());
			testFile << "size " << trajectorys.at(name) << std::endl;
			for(int crowdSize = 3; crowdSize <= 3; crowdSize += 6){
				testFile << crowdSize << " ";
				double counter = 0;
				double square_distance_sum = 0;
				for(int x=0; x<10;x++){
					Simulation mainSim;
					localisation *mainLoca_frequency = new localisation;
					mainLoca_frequency->initilisation_done_ = true;
					mainSim.settings_.particle_visibility_ratio_ = 10;
					mainSim.setLocalisation(mainLoca_frequency);
					//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
					mainSim.setObserveMode(Simulation::Pictures);
					mainSim.enablePadControl();

					mainSim.settings_.crowd_size_ = crowdSize;
					mainSim.settings_.takepicture_intervall_ = 1500;

					mainSim.Initialize();
					mainSim.Realize();
					std::stringstream trajPath;
					trajPath << "/home/josef/workspace/Loca-Projekt/trajectorys/"<<trajectorys.at(name);
					mainSim.ReadRobotTrajectory(trajPath.str());
					while(!mainSim.done()){
						mainSim.Step();
						localisation::EstimatedRobotPose* es = mainLoca_frequency->getEstimatedRobotPose();
						if(es->x < INFINITY){
						counter++;
						square_distance_sum += sqrt(pow(es->x-mainSim.getRobX(),2) + pow(es->y-mainSim.getRobY(),2));
						}
					}
					mainSim.CleanUp();
				}
				square_distance_sum = square_distance_sum/counter;
				testFile << square_distance_sum << std::endl;
			}
			testFile.close();
		}
		break;
	}
	default:
		;
		break;
	}
	return 0;
}
