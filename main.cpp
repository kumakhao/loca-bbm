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
#include "util/msTimer.h"

int main(int argc, char** argv){
	osg::setNotifyLevel(osg::WARN);
	srand(time(NULL));
	msTimer simTime;
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

		mainLoca->initilisation_done_ = false;
		mainSim.setLocalisation(mainLoca);
		//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
		mainSim.setObserveMode(Simulation::Pictures);
		mainSim.enablePadControl();
		mainSim.settings_.takepicture_intervall_ = 2000;
		mainSim.settings_.robParameter_.kSpeed = 0.05;
		mainSim.settings_.crowd_size_ = 3;
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
		//trajectorys.push_back("oval");
		//trajectorys.push_back("serpentinen");
		//trajectorys.push_back("clockwise");
		trajectorys.push_back("antiClockwise");
		for(unsigned int name = 0; name < trajectorys.size(); name++){
			std::stringstream filename;
			filename << "/home/josef/workspace/Loca-Projekt/plots/picFrequency/" << trajectorys.at(name) << "_data";
			std::ofstream testFile (filename.str().c_str());
			testFile << "freq " << trajectorys.at(name) << std::endl;

			for(int pic_time = 500; pic_time <= 4000; pic_time += pic_time){
				int repeats = 10;
				double square_distance[repeats];
				testFile << pic_time << " ";
				double square_distance_sum = 0;
				double square_distance_var = 0;
				for(int x=0; x<repeats;x++){
					double counter = 0;
					Simulation mainSim;
					localisation *mainLoca_frequency = new localisation;
					mainLoca_frequency->initilisation_done_ = true;
					mainSim.settings_.particle_visibility_ratio_ = 10;
					mainSim.setLocalisation(mainLoca_frequency);
					//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
					mainSim.setObserveMode(Simulation::Pictures);
					//mainSim.enablePadControl();

					mainSim.settings_.takepicture_intervall_ = pic_time;
					mainSim.settings_.blind_mode_on_ = false;

					mainSim.Initialize();
					mainSim.Realize();
					std::stringstream trajPath;
					trajPath << "/home/josef/workspace/Loca-Projekt/trajectorys/"<<trajectorys.at(name);
					mainSim.ReadRobotTrajectory(trajPath.str());
					simTime.Start();
					while(!mainSim.done()){
						mainSim.Step();
						localisation::EstimatedRobotPose* es = mainLoca_frequency->getEstimatedRobotPose();
						if(es->x < INFINITY){
						counter++;
						double tmp = sqrt(pow(es->x-mainSim.getRobX(),2) + pow(es->y-mainSim.getRobY(),2));
						if(tmp > 20)
							std::cout<<"droped someting"<<std::endl;
						else
							square_distance[x] += tmp;
						}
					}
					double tmp = square_distance[x]/counter;
					square_distance[x] = tmp;
					square_distance_sum += tmp;
					std::cout<<"Time: "<<(double)simTime.getTimeFromStart()*0.000001<<"    "<<square_distance[x]<<std::endl;
//					std::stringstream trajectoryfile;
//					trajectoryfile << "/home/josef/Desktop/trajectory_"<<pic_time;
//					mainSim.WriteRobotTrajectory(trajectoryfile.str());
					mainSim.CleanUp();
				}
				square_distance_sum = square_distance_sum/repeats;
				for(int i=0;i<repeats;i++)
					square_distance_var = pow((square_distance[i]-square_distance_sum),2);
				square_distance_var = square_distance_var/repeats;
				testFile << square_distance_sum << " " << square_distance_var << std::endl;
			}
			testFile.close();
			std::cout<<std::endl;
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

				for(int crowdSize = 3; crowdSize <= 21; crowdSize += 6){
					int repeats = 10;
					double square_distance[repeats];
					testFile << crowdSize << " ";
					double square_distance_sum = 0;
					double square_distance_var = 0;
					for(int x=0; x<repeats;x++){
						double counter = 0;
						Simulation mainSim;
						localisation *mainLoca_frequency = new localisation;
						mainLoca_frequency->initilisation_done_ = true;
						mainSim.settings_.particle_visibility_ratio_ = 10;
						mainSim.setLocalisation(mainLoca_frequency);
						//GPS mode is buggy. Partikel verschwinden und führen zu out of range exception für den Vektor der sie hält.
						mainSim.setObserveMode(Simulation::Pictures);
						//mainSim.enablePadControl();

						mainSim.settings_.crowd_size_ = crowdSize;
						mainSim.settings_.takepicture_intervall_ = 1500;
						mainSim.settings_.blind_mode_on_ = false;

						mainSim.Initialize();
						mainSim.Realize();
						std::stringstream trajPath;
						trajPath << "/home/josef/workspace/Loca-Projekt/trajectorys/"<<trajectorys.at(name);
						mainSim.ReadRobotTrajectory(trajPath.str());
						simTime.Start();
						while(!mainSim.done()){
							mainSim.Step();
							localisation::EstimatedRobotPose* es = mainLoca_frequency->getEstimatedRobotPose();
							if(es->x < INFINITY){
							counter++;
							double tmp = sqrt(pow(es->x-mainSim.getRobX(),2) + pow(es->y-mainSim.getRobY(),2));
							if(tmp > 20)
								std::cout<<"droped someting"<<std::endl;
							else
								square_distance[x] += tmp;
							}
						}
						double tmp = square_distance[x]/counter;
						square_distance[x] = tmp;
						square_distance_sum += tmp;
						std::cout<<"Time: "<<(double)simTime.getTimeFromStart()*0.000001<<"    "<<square_distance[x]<<std::endl;
	//					std::stringstream trajectoryfile;
	//					trajectoryfile << "/home/josef/Desktop/trajectory_"<<pic_time;
	//					mainSim.WriteRobotTrajectory(trajectoryfile.str());
						mainSim.CleanUp();
						testFile<< x << " " << square_distance[x]<<std::endl;
					}
					square_distance_sum = square_distance_sum/repeats;
					for(int i=0;i<repeats;i++)
						square_distance_var = pow((square_distance[i]-square_distance_sum),2);
					square_distance_var = square_distance_var/repeats;
					testFile << square_distance_sum << " " << square_distance_var << std::endl;
				}
				testFile.close();
				std::cout<<std::endl;
			}
			break;
		}
	default:
		;
		break;
	}
	return 0;
}
