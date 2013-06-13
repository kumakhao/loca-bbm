#include "localisation.h"
#include "../locaUtil.h"
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>

localisation::Parameters::Parameters() :
		nrOfInitialOrientations(1),
		nrOfParticlesPerLenth(20),
		nrOfParticles(nrOfInitialOrientations * nrOfParticlesPerLenth * nrOfParticlesPerLenth),
		fieldY(12),
		fieldX(12),
		impulesProMeter(57694),
		distanceWheels(0.07),
		sigmaGPS(0.05),
		sigmaLandmarke(0.1),
		sigmaIncrement(0.1),
		sigmaDistance(0.1),
		sigmaAngle(0.1) {
}

void localisation::Particle::observeImg(cv::Mat* img) {
	//TODO
}

void localisation::Particle::observeLandmark(int ID, double angle) {
	double errPsi = locaUtil::angleNormalisation(
			loca->landmarks.getAngleToLandmark(ID, xPos, yPos, psi) - angle);
	double errSum2Psi = errPsi*errPsi;
	this->weight *= exp(-errSum2Psi / (2*loca->param.sigmaAngle*loca->param.sigmaAngle)	);
}

void localisation::Particle::observeGPS(double x, double y, double psi) {
	double errX = xPos - x;
	double errY = yPos - y;
	double errPsi = locaUtil::angleNormalisation(
			locaUtil::angleNormalisation(this->psi)
					- locaUtil::angleNormalisation(psi));
	double errSum2Pos = errX * errX + errY * errY;
	double errSum2Psi = errPsi * errPsi;
	//winkel normalisieren
	//std::cout<<"weight vorher: "<<this->weight<<" nachher: ";
	this->weight *= exp(
			-errSum2Pos / (2 * loca->param.sigmaGPS * loca->param.sigmaGPS));
	//std::cout<<this->weight<<std::endl;
	this->weight *= exp(-errSum2Psi / (2 * 1 * 1));
}

void localisation::Particle::dynamic(double dDistance, double dPsi) {
	//TODO: randomGaussian
	double errDistance = dDistance * locaUtil::randomGaussian()
			* loca->param.sigmaDistance;
	// auch bei geradeausfahrt winkelfehler möglich
	double errPsi = dPsi * locaUtil::randomGaussian() * loca->param.sigmaAngle;
	xPos = xPos + cos(psi + (dPsi + errPsi) / 2) * (dDistance + errDistance);
	yPos = yPos + sin(psi + (dPsi + errPsi) / 2) * (dDistance + errDistance);
	psi = psi + (dPsi + errPsi);
}

localisation::localisation() {

}

void localisation::dynamic(int incLeft, int incRight) {
	double dPsi = (incRight - incLeft) / param.impulesProMeter
			/ param.distanceWheels;
	double dDistance = (incLeft + incRight) / 2 / param.impulesProMeter;
	//std::cout<<"dPsi: "<<dPsi<<"    dDistance: "<<dDistance<<endl;
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles.at(i).dynamic(dDistance, dPsi);
	}
}

void localisation::observeImg(cv::Mat* img) {
	//TODO
}

void localisation::observeLandmark(int ID, double angle) {
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles.at(i).observeLandmark(ID, angle);
	}
}

void localisation::observeGPS(double x, double y, double psi) {
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles.at(i).observeGPS(x, y, psi);
	}
}

void localisation::createSamples(int nrOfParticles) {
	particles.clear();
	particles.reserve(nrOfParticles);
	double weight = 1.0 / nrOfParticles;
	double x = 2.0, y = 2.0, psi = M_PI * 2 / param.nrOfInitialOrientations;
	Particle tempPart(this, weight, x, y, psi);
	for (int i = 1; i <= param.nrOfParticlesPerLenth; i++) {
		for (int u = 1; u <= param.nrOfParticlesPerLenth; u++) {
			//the Origion is in the middle of the ground plate. to let particle koordiantes and GUIParticles koordiantes match its
			//transformed here.
			tempPart.xPos = this->param.fieldX
					* ((double) i / param.nrOfParticlesPerLenth) - 6;
			tempPart.yPos = this->param.fieldY
					* ((double) u / param.nrOfParticlesPerLenth) - 6;
			for (int p = 0; p < param.nrOfInitialOrientations; p++) {
				tempPart.psi = psi * p;
				particles.push_back(tempPart);
			}
		}

	}
}

void localisation::resample(int nrOfParticles) {
	double totalWeight = 0;
	for (unsigned int i = 0; i < particles.size(); i++)
		totalWeight += particles.at(i).weight;
	std::vector<Particle> pNew;
	double weightUpToJ = 0;
	int j = -1;
	double weightChosen = locaUtil::randomUniform() * totalWeight
			/ nrOfParticles;
	if (weightChosen == totalWeight / nrOfParticles)
		weightChosen = locaUtil::randomUniform() * totalWeight / nrOfParticles;
	for (int i = 0; i < nrOfParticles; i++) {
		while (weightChosen >= weightUpToJ) {
			j++;
			weightUpToJ += particles.at(j).weight;
		}
		pNew.push_back(particles.at(j));
		pNew.back().weight = 1.0 / nrOfParticles;
		weightChosen += totalWeight / nrOfParticles;
	}
	particles = pNew;
}

std::vector<localisation::Particle> localisation::getParticles() {
	return particles;
}

/**
 * computes the mean orientation over all particles.
 * @return the first value is the variance from 0..1, the 2nd is the angle.
 */
std::vector<double> localisation::getOrientation() {
	std::vector<double> ori;
	double sumSinPsi, sumCosPsi, variancePsi, meanPsi;
	for (unsigned int i = 0; i < particles.size(); i++) {
		sumSinPsi += sin(particles.at(i).psi);
		sumCosPsi += cos(particles.at(i).psi);
	}
	sumSinPsi = sumSinPsi/particles.size();
	sumCosPsi = sumCosPsi/particles.size();
	variancePsi = sqrt(sumSinPsi*sumSinPsi+sumCosPsi*sumCosPsi);
	if( (sumSinPsi == 0) && (sumCosPsi == 0) )
		meanPsi = 0;
	else
		meanPsi = atan2(sumSinPsi,sumCosPsi);
	ori.push_back(variancePsi);
	ori.push_back(meanPsi);
	return ori;
}

/**
 * computes the mean position over all particles.
 * @return (0):meanX, (1):varX, (2):meanY, (3):varY
 */
std::vector<double> localisation::getPosition() {
	std::vector<double> pos;
	double meanXPos = 0, meanYPos = 0;
	double varXPos = 0, varYPos = 0;
	for (unsigned int i = 0; i < particles.size(); i++) {
		meanXPos += particles.at(i).xPos;
		meanYPos += particles.at(i).yPos;
	}
	meanXPos = meanXPos/particles.size();
	meanYPos = meanYPos/particles.size();
	for (unsigned int i = 0; i < particles.size(); i++) {
			varXPos += pow(particles.at(i).xPos-meanXPos,2);
			varYPos += pow(particles.at(i).yPos-meanYPos,2);
	}
	pos.push_back(meanXPos);
	pos.push_back(varXPos);
	pos.push_back(meanXPos);
	pos.push_back(varYPos);
	return pos;
}
