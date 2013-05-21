/*
 * locaUtil.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: josef
 */

#include <osgViewer/Viewer>
#include "locaUtil.h"
#include <stdlib.h>
#include <math.h>

/**
 * Projectes an angle in rad into and interval from ]-PI to PI]
 * @param in The angle in rad
 * @return The angle in new interval from -Pi to Pi
 */
double locaUtil::angleNormalisation(double in)
{
	double out = in;
	while(out>M_PI)
		out -= 2*M_PI;
	while(out<=-M_PI)
		out += 2*M_PI;
	return out;
}

/**
 *
 * @return Random number between 0 and 1 inclusive.
 */
double locaUtil::randomUniform() {
	return ((double) rand())/RAND_MAX;
}

/**
 * Will generate a normally distributed random number.
 * Using the Box–Muller transform the random number is generated using
 * two samples from a uniform distribution.
 * @see randomUniform()
 * @return A normally distributed random number.
 */
double locaUtil::randomGaussian() {
	double u = randomUniform(), v = randomUniform();
	return sqrt(-2*log(u))*cos(2*M_PI*v);
}

/**
 * Calculates the 3D position of squares in top row of 1st light wall.
 * The center of the coordiante system used here is in the middle of the
 * stage. Corners are (6,6) (-6,6) (-6,-6) (6,-6)
 * @return A matrix containing the 64 3D Points of 1st light wall
 */
cv::Mat locaUtil::getBit3Dlocations_1() {
	double x = 6.0;
	double y = 6.0 - squareSize/2;
	cv::Mat bitLocation_1 (64,3,CV_64FC1);
	for(int i=0;i<64;i++){
		bitLocation_1.at<double>(i,0) = x;
		bitLocation_1.at<double>(i,1) = y;
		bitLocation_1.at<double>(i,2) = squareHeight;
		y -= squareSize;
	}
	return bitLocation_1;
}

/**
 * Calculates the 3D position of squares in top row of 2nd light wall.
 * The center of the coordiante system used here is in the middle of the
 * stage. Corners are (6,6) (-6,6) (-6,-6) (6,-6)
 * @return A matrix containing the 64 3D Points of 2nd light wall
 */
cv::Mat locaUtil::getBit3Dlocations_2() {
	double x = -6.0 + squareSize/2;
	double y = 6.0;
	cv::Mat bitLocation_2 (64,3,CV_64FC1);
	for(int i=0;i<64;i++){
		bitLocation_2.at<double>(i,0) = x;
		bitLocation_2.at<double>(i,1) = y;
		bitLocation_2.at<double>(i,2) = squareHeight;
		x += squareSize;
	}
	return bitLocation_2;
}

/**
 * Calculates the 3D position of squares in top row of 3rd light wall.
 * The center of the coordiante system used here is in the middle of the
 * stage. Corners are (6,6) (-6,6) (-6,-6) (6,-6)
 * @return A matrix containing the 64 3D Points of 3rd light wall
 */
cv::Mat locaUtil::getBit3Dlocations_3() {
	double x = 6.0 - squareSize/2;
	double y = -6.0;
	cv::Mat bitLocation_3 (64,3,CV_64FC1);
	for(int i=0;i<64;i++){
		bitLocation_3.at<double>(i,0) = x;
		bitLocation_3.at<double>(i,1) = y;
		bitLocation_3.at<double>(i,2) = squareHeight;
		x -= squareSize;
	}
	return bitLocation_3;
}

/**
 * Calculates the 3D position of squares in top row of all 3 light walls.
 * The center of the coordiante system used here is in the middle of the
 * stage. Corners are (6,6) (-6,6) (-6,-6) (6,-6)
 * @return A matrix containing the 192 3D Points all 3 light walls. 1st:0-63 2nd:64-127 3rd:128-191
 */
cv::Mat locaUtil::getBit3Dlocations_all() {
	cv::Mat bitLoc (192,3,CV_64FC1);
	cv::Mat bitLoc_1 = getBit3Dlocations_1();
	cv::Mat bitLoc_2 = getBit3Dlocations_2();
	cv::Mat bitLoc_3 = getBit3Dlocations_3();
	for(int i=0; i<64;i++){
		bitLoc.at<double>(i, 0) = bitLoc_1.at<double>(i, 0);
		bitLoc.at<double>(i, 1) = bitLoc_1.at<double>(i, 1);
		bitLoc.at<double>(i, 2) = bitLoc_1.at<double>(i, 2);
		bitLoc.at<double>(i+64, 0) = bitLoc_2.at<double>(i, 0);
		bitLoc.at<double>(i+64, 1) = bitLoc_2.at<double>(i, 1);
		bitLoc.at<double>(i+64, 2) = bitLoc_2.at<double>(i, 2);
		bitLoc.at<double>(i+128, 0) = bitLoc_3.at<double>(i, 0);
		bitLoc.at<double>(i+128, 1) = bitLoc_3.at<double>(i, 1);
		bitLoc.at<double>(i+128, 2) = bitLoc_3.at<double>(i, 2);
	}
	return bitLoc;
}

/**
 * TODO
 * @param node
 * @return
 */
osg::Matrixd* locaUtil::getWorldCoords(osg::Node* node) {
	getWorldCoordOfNodeVisitor* ncv = new getWorldCoordOfNodeVisitor();
	   if (node && ncv)
	   {
	      node->accept(*ncv);
	      return ncv->giveUpDaMat();
	   }
	   else
	   {
	      return NULL;
	   }
}
/**
 * Creates an array with an alternating pattern starting with white(255).
 * @return pointer to unsigned char array [192]
 */
unsigned char* locaUtil::getPatternAlternate() {
	unsigned char *ret = new unsigned char[192];
	for(int i = 0; i<192;i++)
		ret[i] = 255*((i+1)%2);
	return ret;
}

/**
 * Creates an array with a pattern using Code93.
 * @return pointer to unsigned char array [192]
 */
unsigned char* locaUtil::getPatternCode93() {
	unsigned char *ret = new unsigned char[192];
	unsigned char *start = ret;
	ret = code93(ret,100);
	ret = code93(ret,10);
	ret = code93(ret,11);
	ret = code93(ret,12);
	ret = code93(ret,13);
	ret = code93(ret,14);
	ret = code93(ret,100);
	ret[0] = 0;
	ret++;
	ret = code93(ret,100);
	ret = code93(ret,20);
	ret = code93(ret,21);
	ret = code93(ret,22);
	ret = code93(ret,23);
	ret = code93(ret,24);
	ret = code93(ret,100);
	ret[0] = 0;
	ret++;
	ret = code93(ret,100);
	ret = code93(ret,30);
	ret = code93(ret,31);
	ret = code93(ret,32);
	ret = code93(ret,33);
	ret = code93(ret,34);
	ret = code93(ret,100);
	ret[0] = 0;
	return start;
}

/**
 * Creates a pyramid as an OSG-object.
 * @return pointer to the pyramid object.
 */
osg::Geode* locaUtil::pyramid() {
	osg::Geode* pyramidGeode = new osg::Geode();
	osg::Geometry* pyramidGeometry = new osg::Geometry();

	pyramidGeode->addDrawable(pyramidGeometry);

	osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
	pyramidVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
	pyramidVertices->push_back( osg::Vec3(10, 0, 0) ); // front right
	pyramidVertices->push_back( osg::Vec3(10,10, 0) ); // back right
	pyramidVertices->push_back( osg::Vec3( 0,10, 0) ); // back left
	pyramidVertices->push_back( osg::Vec3( 5, 5,10) ); // peak

	pyramidGeometry->setVertexArray( pyramidVertices );

	osg::DrawElementsUInt* pyramidBase =
	  new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
	pyramidBase->push_back(3);
	pyramidBase->push_back(2);
	pyramidBase->push_back(1);
	pyramidBase->push_back(0);
	pyramidGeometry->addPrimitiveSet(pyramidBase);

	osg::DrawElementsUInt* pyramidFaceOne =
	  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	pyramidFaceOne->push_back(0);
	pyramidFaceOne->push_back(1);
	pyramidFaceOne->push_back(4);
	pyramidGeometry->addPrimitiveSet(pyramidFaceOne);

	osg::DrawElementsUInt* pyramidFaceTwo =
	  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	pyramidFaceTwo->push_back(1);
	pyramidFaceTwo->push_back(2);
	pyramidFaceTwo->push_back(4);
	pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);

	osg::DrawElementsUInt* pyramidFaceThree =
	  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	pyramidFaceThree->push_back(2);
	pyramidFaceThree->push_back(3);
	pyramidFaceThree->push_back(4);
	pyramidGeometry->addPrimitiveSet(pyramidFaceThree);

	osg::DrawElementsUInt* pyramidFaceFour =
	  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	pyramidFaceFour->push_back(3);
	pyramidFaceFour->push_back(0);
	pyramidFaceFour->push_back(4);
	pyramidGeometry->addPrimitiveSet(pyramidFaceFour);

	return pyramidGeode;
}

/**
 * Creates an white image with the pattern in top row.
 * @param pattern The pattern used in the image.
 * @param wallNr 0, 1 or 2
 * @return An openCV image (4096,8192,CV_8UC1)
 * @see getPatternAlternate()
 * @see getPatternCode93()
 */
cv::Mat locaUtil::makeWall(unsigned char* pattern, int wallNr) {
	cv::Mat M = cv::Mat::zeros(4096,8192,CV_8UC1);
	M = M+255;
	if(wallNr>2 || wallNr<0)
		return M;
	unsigned char* p = M.data;
	unsigned char* fill;
	for(int i=0;i<64;i++){
		for(int y=0;y<128;y++)
		{
			fill = p+8192*y;
			for(int x=0;x<128;x++)
			{
				fill[0] = pattern[i+wallNr*64];
				fill++;
			}
		}
		p = p+128;
	}
	return M;
}
/**
 * Inputs a number encoded in code93 into the input array @param arr.
 * @param arr The encoded number will be written here.
 * @param a0 The number to encode (TODO: finish encoding for all numbers)
 * @return A pointer to the imput array moved to the next free entry.
 */
unsigned char* locaUtil::code93(unsigned char* arr, int a0) {
	switch (a0){
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	case 9:
		break;
	case 10:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 0;
		arr[6] = 255;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 11:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 12:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 255;
		arr[6] = 255;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 13:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 14:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 255;
		arr[6] = 255;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 15:
		break;
	case 16:
		break;
	case 17:
		break;
	case 18:
		break;
	case 19:
		break;
	case 20:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 255;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 0;
		arr[6] = 255;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 21:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 0;
		arr[6] = 255;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 22:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 255;
		arr[4] = 255;
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 23:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 255;
		arr[4] = 255;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 24:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 255;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 25:
		break;
	case 26:
		break;
	case 27:
		break;
	case 28:
		break;
	case 29:
		break;
	case 30:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 31:
		arr[0] = 0;
		arr[1] = 0;
		arr[2] = 255;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 0;
		arr[6] = 255;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 32:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 255;
		arr[8] = 255;
		arr += 9;
		break;
	case 33:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 0;
		arr[4] = 255;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 34:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 255;
		arr[3] = 0;
		arr[4] = 0;
		arr[5] = 255;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	case 35:
		break;
	case 36:
		break;
	case 37:
		break;
	case 38:
		break;
	case 39:
		break;
	case 40:
		break;
	case 41:
		break;
	case 42:
		break;
	case 43:
		break;
	case 44:
		break;
	case 45:
		break;
	case 46:
		break;
	default:
		arr[0] = 0;
		arr[1] = 255;
		arr[2] = 0;
		arr[3] = 255;
		arr[4] = 0;
		arr[5] = 0;
		arr[6] = 0;
		arr[7] = 0;
		arr[8] = 255;
		arr += 9;
		break;
	}
	return arr;
}




