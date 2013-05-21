/*
 * landmarkSet.h
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */

#ifndef LANDMARKSET_H_
#define LANDMARKSET_H_

#include <map>
#include <vector>

class landmarkSet{
public:
	landmarkSet();
	void addLandmark(int ID, double x, double y);
	void removeLandmark(int ID);
	double getAngleToLandmark(int ID, double x, double y, double psi);
	int getSize();
	std::vector<int> getIDVector();
	double getXofID(int ID);
	double getYofID(int ID);
protected:
	std::map<int,std::vector<double> > landmarks;
};


#endif /* LANDMARKSET_H_ */
