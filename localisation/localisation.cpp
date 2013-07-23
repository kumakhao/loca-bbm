#include "localisation.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <highgui/highgui_c.h>
#include <iostream>
#include "picRating.h"
#include "../locaUtil.h"

localisation::Parameters::Parameters() :
		nrOfInitialOrientations(1),
		nrOfParticlesPerLenth(50),
		nrOfParticles(nrOfInitialOrientations * nrOfParticlesPerLenth * nrOfParticlesPerLenth),
		fieldY(12),
		fieldX(12),
		impulesProMeter(57694),
		distanceWheels(0.07),
		sigmaGPS(0.05),
		sigmaLandmarke(0.1),
		sigmaIncrement(0.1),
		sigmaDistance(0.2),
		sigmaAngle(0.2) {
}

void localisation::Particle::observeImg(cv::Mat* img) {
	//TODO work in progress
	std::vector<cv::Point2d> imagePoints, imagePointsCliped;
	std::vector<patternPoint> clipedImagePoints;
	int grid = 1;
	this->loca->camera_model_.setExtr(psi,xPos,yPos);
	loca->camera_model_.projectTo2D(&(loca->Points_3D_),&imagePoints);
	clipedImagePoints = picRating::clipANDmark(imagePoints,locaUtil::getPatternCode93(), img->cols, img->rows);
	double p = picRating::rateImage(*img, clipedImagePoints, grid);
	//std::cout<<"Particle::observeImg: "<<p<<std::endl;
	if(loca->highscore < p){
		loca->highscore = p;
		loca->highscore_count = 1;
	}
	if(loca->highscore == p)
		loca->highscore_count++;
	if(p>0.001){
		this->weight *= p;
		loca->good_rating_count++;
	}
	else
		this->weight *= 0.001;
//	for(unsigned int i=0; i<imagePoints.size(); i++)
//		//cv::circle(img,cv::Point(imagePoints.at(i).x,imagePoints.at(i).y),grid,cv::Scalar(0,0,255),1,8);
//		cv::rectangle(*img,cv::Point(imagePoints.at(i).x-grid,imagePoints.at(i).y-grid),cv::Point(imagePoints.at(i).x+grid,imagePoints.at(i).y+grid),cv::Scalar(0,0,255),1,8,0);
//	std::ostringstream s;
//	s<<"Gewichtung: "<<p;
//	cv::putText(*img, s.str(), cvPoint(30,500),
//		    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);
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
	//TODO hier wird 2x weight berechnet!
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
	//TODO auch bei geradeausfahrt winkelfehler möglich
	double errPsi = dPsi * locaUtil::randomGaussian() * loca->param.sigmaAngle;
	xPos = xPos + cos(psi + errPsi) * (dDistance + errDistance);
	yPos = yPos + sin(psi + errPsi) * (dDistance + errDistance);
//	xPos = xPos + cos(psi + (dPsi + errPsi) / 2) * (dDistance + errDistance);
//	yPos = yPos + sin(psi + (dPsi + errPsi) / 2) * (dDistance + errDistance);
	psi = psi + (dPsi + errPsi);
}

localisation::localisation() {
	Points_3D_ = locaUtil::getBit3Dlocations_all();

}

void localisation::dynamic(double incLeft, double incRight) {
	double dPsi = (incRight - incLeft) / param.impulesProMeter	/ param.distanceWheels;
	double dDistance = (incLeft + incRight) / 2.0 / param.impulesProMeter;
	//std::cout<<"dPsi: "<<dPsi<<"    dDistance: "<<dDistance<<endl;
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles.at(i).dynamic(dDistance, dPsi);
	}
}

void localisation::observeImg(cv::Mat* img) {
	//TODO changed for testing, needs reverting
//	if(particles.size() > 0)
//		particles.at(0).observeImg(img);
	highscore = 0.0;
	good_rating_count = 0;
	for (unsigned int i = 0; i < particles.size(); i++) {
			particles.at(i).observeImg(img);
	}
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

void localisation::createOneParticle() {
	particles.clear();
	//Particle oneParticle(this, 1.0, -2.83599, -0.093889, 0.0);
	Particle oneParticle(this, 1.0, 0.0, 0.0, 0.0);
	particles.push_back(oneParticle);
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
	double varXPos = 0, varYPos = 0, varXYPos = 0;
	for (unsigned int i = 0; i < particles.size(); i++) {
		meanXPos += particles.at(i).xPos;
		meanYPos += particles.at(i).yPos;
	}
	meanXPos = meanXPos/particles.size();
	meanYPos = meanYPos/particles.size();
	for (unsigned int i = 0; i < particles.size(); i++) {
			varXPos += pow(particles.at(i).xPos-meanXPos,2);
			varYPos += pow(particles.at(i).yPos-meanYPos,2);
			varXYPos += (particles.at(i).xPos-meanXPos)*(particles.at(i).yPos-meanYPos);
	}
	varXPos = varXPos/particles.size();
	varYPos = varYPos/particles.size();
	varXYPos = varXYPos/particles.size();
	pos.push_back(meanXPos);
	pos.push_back(sqrt(varXPos));
	pos.push_back(meanYPos);
	pos.push_back(sqrt(varYPos));
	pos.push_back(sqrt(varXYPos));
	return pos;
}

localisation::EstimatedRobotPose localisation::getEstimatedRobotPose() {
	//TODO: untested!
	EstimatedRobotPose es = new EstimatedRobotPose();

	double sumSinPsi, sumCosPsi;
	for (unsigned int i = 0; i < particles.size(); i++) {
			es.x += particles.at(i).xPos;
			es.y += particles.at(i).yPos;
			sumSinPsi += sin(particles.at(i).psi);
			sumCosPsi += cos(particles.at(i).psi);
	}
	es.x = es.x/particles.size();
	es.y = es.y/particles.size();
	sumSinPsi = sumSinPsi/particles.size();
	sumCosPsi = sumCosPsi/particles.size();
	es.sigmaPsi = sqrt(sumSinPsi*sumSinPsi+sumCosPsi*sumCosPsi);
	if( (sumSinPsi == 0) && (sumCosPsi == 0) )
		es.psi = 0;
	else
		es.psi = atan2(sumSinPsi,sumCosPsi);


	double varXPos, varYPos, varXYPos;
	for (unsigned int i = 0; i < particles.size(); i++) {
			varXPos += pow(particles.at(i).xPos-es.x,2);
			varYPos += pow(particles.at(i).yPos-es.y,2);
			varXYPos += (particles.at(i).xPos-es.x)*(particles.at(i).yPos-es.y);
	}
	varXPos = varXPos/particles.size();
	varYPos = varYPos/particles.size();
	varXYPos = varXYPos/particles.size();

	double eigen0, eigen1;
	es.sigmaXYAngle = atan2(-2*varXYPos, varYPos-varXPos)/2;
	if(isnan(es.sigmaXYAngle)) es.sigmaXYAngle = 0;
	double c = cos(es.sigmaXYAngle);
	double s = sin(es.sigmaXYAngle);
	double c2 = c*c, s2=s*s, cs = c*s;

	eigen0 = c2*varXPos+2*cs*varXYPos+s2*varYPos;
	eigen1 = s2*varXPos+2*cs*varXYPos+c2*varYPos;
	if(eigen0<eigen1){
		if(es.sigmaXYAngle>0) es.sigmaXYAngle-=M_PI/2;
		else es.sigmaXYAngle+=M_PI/2;
		es.sigmaXYLarge = sqrt(eigen1);
		es.sigmaXYSmall = sqrt(eigen0);
	}
	else{
		es.sigmaXYLarge = sqrt(eigen0);
		es.sigmaXYSmall = sqrt(eigen1);
	}
	return es;
}

