/*
 * localisation.h
 *
 *  Created on: Jan 20, 2013
 *      Author: josef
 */

#ifndef LOCALISATION_H_
#define LOCALISATION_H_


#include <vector>
#include <math.h>
#include <stdlib.h>
#include <opencv.hpp>
#include <highgui/highgui_c.h>
#include <iostream>
#include "landmarkSet.h"

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

	Parameters param;

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

	std::vector<Particle> particles;

	landmarkSet landmarks;

	localisation();

	void observeImg(cv::Mat* img);

	void observeLandmark(int ID, double angle);

	void observeGPS(double x, double y, double psi);

	void dynamic(int incLeft, int incRight);

	void createSamples(int nrOfParticles);

	void resample(int nrOfParticles);

	void resample() {resample(particles.size());}

	std::vector<localisation::Particle> getParticles();

	std::vector<double> getPosition();

	std::vector<double> getOrientation();

};


#endif /* LOCALISATION_H_ */
