/*
 * localisation.h
 *
 *  Created on: Jan 20, 2013
 *      Author: josef
 */

#ifndef LOCALISATION_H_
#define LOCALISATION_H_

#include <opencv.hpp>

#include "landmarkSet.h"
#include "cameraParam.h"


class localisation
{
public:
	class Parameters
	{
	public:
		int nrOfInitialOrientations;
		int nrOfParticlesPerLenth;
		int nrOfParticles;
		double fieldY;
		double fieldX;
		double impulesProMeter; //impulse / m
		double distanceWheels; //m
		double sigmaGPS;
		double sigmaLandmarke;
		double sigmaIncrement;
		double sigmaDistance;
		double sigmaAngle;
		Parameters();
	};

	class Particle
	{
	public:
		localisation* loca;
		double weight;
		enum State{UNDEFINED, POSITIONDEFINED, FULLDEFINED};
		State state;
		double xPos;
		double yPos;
		double psi;
		double time;
		double timeOfLastObservation;

		Particle(localisation* loca, double weight, double xPos, double yPos, double psi)
			:loca(loca),
			 weight(weight),
			 state(UNDEFINED),
			 xPos(xPos),
			 yPos(yPos),
			 psi(psi),
			 time(0),
			 timeOfLastObservation(0)
		{}

		void observeImg (cv::Mat* img);
		void observeLandmark(int ID, double angle);
		void observeGPS(double x, double y, double psi);
		void dynamic (double dDistance, double dPsi);
	};

	Parameters param;
	std::vector<Particle> particles;
	landmarkSet landmarks;
	cv::Mat Points_3D_;
	//cameraParam camera_model_;
	cameraParam camera_model_;
	double highscore;
	int highscore_count;
	int good_rating_count;

	localisation();
	void observeImg(cv::Mat* img);
	void observeLandmark(int ID, double angle);
	void observeGPS(double x, double y, double psi);
	//TODO changed from int -> double 27.06
	// the truncing resulted in an error with the particle orientation
	// over time. For testing this is not a good behavior.
	// I need to decide if i want to keep this.
	void dynamic(double incLeft, double incRight);
	void createSamples(int nrOfParticles);
	void createOneParticle();
	void resample(int nrOfParticles);
	void resample() {resample(particles.size());}

	std::vector<localisation::Particle> getParticles();
	std::vector<double> getPosition();
	std::vector<double> getOrientation();
	class EstimatedRobotPose{
	public:
		double x;
		double y;
		double psi;

		double sigmaPsi;

		double sigmaXYLarge;
		double sigmaXYSmall;
		double sigmaXYAngle;
	};
	localisation::EstimatedRobotPose getEstimatedRobotPose();
};


#endif /* LOCALISATION_H_ */
