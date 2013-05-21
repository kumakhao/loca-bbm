/*
 * landmarkSet.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */

#include "landmarkSet.h"
#include <math.h>
#include "../locaUtil.h"

landmarkSet::landmarkSet() {
}

void landmarkSet::addLandmark(int ID, double x, double y) {

	if(1>landmarks.count(ID)){
		std::vector<double> pos;
		pos.push_back(x);
		pos.push_back(y);
		landmarks.insert(std::make_pair(ID,pos));
	}
}

void landmarkSet::removeLandmark(int ID) {
	landmarks.erase(ID);
}

double landmarkSet::getAngleToLandmark(int ID, double x, double y, double psi)
/**
 * computes the angle between the position given (x,y) with orientaton (psi)
 * and the landmark selected with (ID)
 */
{
	double deltaPsi = 0;
	double lmX = landmarks.at(ID).at(0);
	double lmY = landmarks.at(ID).at(1);
	deltaPsi = atan2(lmY-y,lmX-x)-locaUtil::angleNormalisation(psi);
	return locaUtil::angleNormalisation(deltaPsi);
}

int landmarkSet::getSize() {
	return landmarks.size();
}

std::vector<int> landmarkSet::getIDVector() {
	std::vector<int> IDs;
	for(std::map<int,std::vector<double> >::iterator it = landmarks.begin(); it != landmarks.end();it++)
		IDs.push_back(it->first);
	return IDs;
}

double landmarkSet::getXofID(int ID) {
	return landmarks.at(ID).at(0);
}

double landmarkSet::getYofID(int ID) {
	return landmarks.at(ID).at(1);
}







