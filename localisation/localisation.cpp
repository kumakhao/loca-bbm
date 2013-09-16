#include "localisation.h"
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <highgui/highgui_c.h>
#include <iostream>
//TODO: included for fileoutput.
#include <fstream>
#include <iomanip>

#include "picRating.h"
#include "../locaUtil.h"

localisation::Parameters::Parameters() :
		nrOfInitialOrientations(1),
		nrOfParticlesPerLenth(50),
		nrOfParticles(nrOfInitialOrientations * nrOfParticlesPerLenth * nrOfParticlesPerLenth),
		fieldY(12),
		fieldX(12),
		impulesProMeter(57693.66687),
		distanceWheels(0.7),
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
	double errDistance = dDistance * locaUtil::randomGaussian()
									* loca->param.sigmaDistance;
	//TODO auch bei geradeausfahrt winkelfehler möglich
	double errPsi = dPsi * locaUtil::randomGaussian() * loca->param.sigmaAngle;
	xPos = xPos + cos(psi + (errPsi + dPsi)/2) * (dDistance + errDistance);
	yPos = yPos + sin(psi + (errPsi + dPsi)/2) * (dDistance + errDistance);
	psi = psi + (dPsi + errPsi);
}

localisation::localisation():
	initilisation_done_(false){
	Points_3D_ = locaUtil::getBit3Dlocations_all();
	highscore = 0;
	highscore_count = 0;
	old_increment_left_ = 0;
	old_increment_right_ = 0;

}

void localisation::dynamic(double incLeft, double incRight) {
	double diffLeft = incLeft-old_increment_left_;
	double diffRight = incRight-old_increment_right_;
	old_increment_left_ = incLeft;
	old_increment_right_ = incRight;
	double dPsi = (diffRight - diffLeft) / param.impulesProMeter	/ param.distanceWheels;
	double dDistance = (diffLeft + diffRight) / 2.0 / param.impulesProMeter;
	//std::cout<<"dPsi: "<<dPsi<<"    dDistance: "<<dDistance<<std::endl;
	for (unsigned int i = 0; i < particles.size(); i++) {
		particles.at(i).dynamic(dDistance, dPsi);
	}
}

void localisation::observeImg(cv::Mat* img) {
	cv::Mat gray_img;
	cvtColor(*img,gray_img,CV_RGB2GRAY);
	if(!initilisation_done_)
	{
		initilisation_done_ = findInitialLocatio(&gray_img);
	}
	else{
		//TODO changed for testing, needs reverting
//		if(particles.size() > 0)
//			particles.at(0).observeImg(img);
		highscore = 0.0;
		highscore_count = 0;
		good_rating_count = 0;
		for (unsigned int i = 0; i < particles.size(); i++) {
				particles.at(i).observeImg(&gray_img);
		}
	}
}

void localisation::observeImgOneParticle(cv::Mat* img) {
	cv::Mat gray_img;
	cvtColor(*img,gray_img,CV_RGB2GRAY);
	localisation::Particle oneParticle = particles.at(0);

	std::vector<cv::Point2d> imagePoints, imagePointsCliped;
	std::vector<patternPoint> clipedImagePoints;
	int grid = 1;
	camera_model_.setExtr(oneParticle.psi,oneParticle.xPos,oneParticle.yPos);
	std::cout<<"pos: "<<oneParticle.xPos<<" "<<oneParticle.yPos<<std::endl;
	camera_model_.projectTo2D(&Points_3D_,&imagePoints);
	clipedImagePoints = picRating::clipANDmark(imagePoints,locaUtil::getPatternCode93(), gray_img.cols, gray_img.rows);
	double p = picRating::rateImage(gray_img, clipedImagePoints, grid);
	std::cout<<"Particle::observeImg: "<<p<<std::endl;
	if(p>0.001){
		oneParticle.weight *= p;
	}
	else
		oneParticle.weight *= 0.001;
	for(unsigned int i=0; i<imagePoints.size(); i++)
		cv::rectangle(*img,cv::Point(imagePoints.at(i).x-grid,imagePoints.at(i).y-grid),cv::Point(imagePoints.at(i).x+grid,imagePoints.at(i).y+grid),cv::Scalar(0,0,255),1,8,0);
	std::ostringstream s;
	s<<"Gewichtung: "<<p;
	cv::putText(*img, s.str(), cvPoint(30,500),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);
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

			//Seed equidistant over the hole ground plate, facing in "equidistant" angle steps

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

localisation::EstimatedRobotPose* localisation::getEstimatedRobotPose() {
	EstimatedRobotPose es;
	if(!initilisation_done_){
		es.psi = INFINITY;
		es.x = INFINITY;
		es.y = INFINITY;
		es.sigmaPsi = INFINITY;
		es.sigmaXYAngle = INFINITY;
		es.sigmaXYLarge = INFINITY;
		es.sigmaXYSmall = INFINITY;
		return &es;
	}

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
	/*
	 * Multiplied by 3 to get 3 sigma
	 * and multiplied by 2 to get diameter
	 */
	if(eigen0<eigen1){
		if(es.sigmaXYAngle>0) es.sigmaXYAngle-=M_PI/2;
		else es.sigmaXYAngle+=M_PI/2;
		es.sigmaXYLarge = sqrt(eigen1)*3*2;
		es.sigmaXYSmall = sqrt(eigen0)*3*2;
	}
	else{
		es.sigmaXYLarge = sqrt(eigen0)*3*2;
		es.sigmaXYSmall = sqrt(eigen1)*3*2;
	}
	return &es;
}

bool localisation::findInitialLocatio(cv::Mat* img) {
	std::vector<cv::Point2d> imagePoints;
	std::vector<patternPoint> clipedImagePoints;
	std::vector<Particle> pNew;
	Particle pTemp(this,0,0,0,0);
	int grid = 0, hits = -1, cols = img->cols, rows = img->rows;
	double sumX = 0, sumY = 0, sumPsi = 0;
	double p, inImagePoints = 0;
	unsigned char* pattern = locaUtil::getPatternCode93();
	//std::ofstream datafile;
	//datafile.open ("test.txt",std::ios_base::app);



	std::cout<<"Starting Initilisation Step 1 ..."<<std::endl;
	for(double xPos = -6; xPos <= 6; xPos+=0.1){
		std::cout<<"#";
		std::flush(std::cout);
		for(double yPos = -6; yPos <= 6; yPos+=0.1){
			inImagePoints = 0;
			for(double psi= -0.1415; psi <= 0.1415; psi+=0.01){
				camera_model_.setExtr(psi,xPos,yPos);
				camera_model_.projectTo2D(&(Points_3D_),&imagePoints);
				clipedImagePoints = picRating::clipANDmark(imagePoints, pattern, cols, rows);
				inImagePoints += clipedImagePoints.size();
				p = picRating::rateImage(*img, clipedImagePoints, grid);
				if(p>0.5){
					pTemp.psi = psi;
					pTemp.xPos = xPos;
					pTemp.yPos = yPos;
					pTemp.weight = p;
					pNew.push_back(pTemp);
					sumX += xPos;
					sumY += yPos;
					sumPsi += psi;
					hits++;
					{
						std::stringstream imgName;
						int grid = 1;
						imgName << "/home/josef/workspace/Loca-Projekt/initPics/initImage_";
						imgName << hits;
						imgName <<".jpg";
						cv::Mat tmpImg = img->clone();
						//std::cout<<"Path:  "<<imgName.str()<<std::endl;
						for(unsigned int i=0; i<imagePoints.size(); i++)
							//cv::circle(img,cv::Point(imagePoints.at(i).x,imagePoints.at(i).y),grid,cv::Scalar(0,0,255),1,8);
							cv::rectangle(tmpImg,cv::Point(imagePoints.at(i).x-grid,imagePoints.at(i).y-grid),cv::Point(imagePoints.at(i).x+grid,imagePoints.at(i).y+grid),cv::Scalar(255,255,255),1,8,0);
						std::ostringstream s;
						s<<"Gewichtung: "<<p<<std::endl<<"Pos: ("<<xPos<<" "<<yPos<<")   "<<psi;
						cv::putText(tmpImg, s.str(), cvPoint(30,500),
								cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);
						cv::imwrite(imgName.str(), tmpImg);
					}
					//std::cout<<"found Position: "<<p<<"   X: "<<xPos<<" Y: "<<yPos<<std::endl;
				}
			}
//			std::ostringstream text;
//			text << std::setw(11) << std::setfill(' ') << xPos << "; ";
//			text << std::setw(11) << std::setfill(' ') << yPos << "; ";
//			text << std::setw(11) << std::setfill(' ') << inImagePoints << "; ";
//			text << "\n";
//			datafile << text.str();
		}
	}
	std::cout<<std::endl;
	if(hits < 5)
	{
		std::cout<<"Step1 was not sufficient ... starting Step2"<<std::endl;
		for(double xPos = -6+0.05; xPos <= 6; xPos+=0.1){
			std::cout<<"#";
			std::flush(std::cout);
			for(double yPos = -6+0.05; yPos <= 6; yPos+=0.1){
				inImagePoints = 0;
				for(double psi= -0.1415; psi <= 0.1415; psi+=0.01){
					camera_model_.setExtr(psi,xPos,yPos);
					camera_model_.projectTo2D(&(Points_3D_),&imagePoints);
					clipedImagePoints = picRating::clipANDmark(imagePoints, pattern, cols, rows);
					inImagePoints += clipedImagePoints.size();
					p = picRating::rateImage(*img, clipedImagePoints, grid);
					if(p>0.5){
						pTemp.psi = psi;
						pTemp.xPos = xPos;
						pTemp.yPos = yPos;
						pTemp.weight = p;
						pNew.push_back(pTemp);
						sumX += xPos;
						sumY += yPos;
						sumPsi += psi;
						hits++;
						{
							std::stringstream imgName;
							int grid = 1;
							imgName << "/home/josef/workspace/Loca-Projekt/initPics/initImage_";
							imgName << hits;
							imgName <<".jpg";
							cv::Mat tmpImg = img->clone();
							//std::cout<<"Path:  "<<imgName.str()<<std::endl;
							for(unsigned int i=0; i<imagePoints.size(); i++)
								//cv::circle(img,cv::Point(imagePoints.at(i).x,imagePoints.at(i).y),grid,cv::Scalar(0,0,255),1,8);
								cv::rectangle(tmpImg,cv::Point(imagePoints.at(i).x-grid,imagePoints.at(i).y-grid),cv::Point(imagePoints.at(i).x+grid,imagePoints.at(i).y+grid),cv::Scalar(255,255,255),1,8,0);
							std::ostringstream s;
							s<<"Gewichtung: "<<p<<std::endl<<"Pos: ("<<xPos<<" "<<yPos<<")   "<<psi;
							cv::putText(tmpImg, s.str(), cvPoint(30,500),
									cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);
							cv::imwrite(imgName.str(), tmpImg);
						}
						//std::cout<<"found Position: "<<p<<"   X: "<<xPos<<" Y: "<<yPos<<std::endl;
					}
				}
	//			std::ostringstream text;
	//			text << std::setw(11) << std::setfill(' ') << xPos << "; ";
	//			text << std::setw(11) << std::setfill(' ') << yPos << "; ";
	//			text << std::setw(11) << std::setfill(' ') << inImagePoints << "; ";
	//			text << "\n";
	//			datafile << text.str();
			}
		}
		std::cout<<std::endl;


	}
	//datafile.close();
	std::cout<<"Ending Initilisation";
	if(hits < 0){
		std::cout<<" NO MATCH!"<<std::endl;
		return false;
	}
	else{
		int counter = 0;
		while(pNew.size()<particles.size()){
			pNew.push_back(pNew.at(counter));
			counter++;
			if(counter > hits)
				counter = 0;
		}
		particles = pNew;
		sumX = sumX / hits;
		sumY = sumY / hits;
		sumPsi = sumPsi / hits;
		//std::cout<<", Position: ("<<sumX<<" "<<sumY<<") Angle: "<<sumPsi<<std::endl;
		//std::cout<<"Nr of hits: "<<hits<<std::endl;
		return true;
	}
}


