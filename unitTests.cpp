/*
 * unitTests.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: josef
 */

#include "unitTests.h"
#include <iostream>
#include <fstream>
#include "localisation/landmarkSet.h"
#include <math.h>
#include <opencv.hpp>
#include <highgui/highgui_c.h>
#include <calib3d/calib3d.hpp>
#include "locaUtil.h"
#include "./localisation/cameraParam.h"
#include "./localisation/picRating.h"
#include "./simulation/dataToFileWriter.h"

// For manual testing of correctness of the landmark set class.
bool unitTests::landmarkTest() {
	landmarkSet testSet;
	double roboX = 7.0;
	double roboY = 7.1;
	double roboPsi = 0.0;
	testSet.addLandmark(1337, 7.0, 1.0);
	testSet.addLandmark(1338, 2.0, -3.0);
	testSet.addLandmark(1339, 1.0, 7.0);
	std::cout<<"Angle LM 1337: "<<testSet.getAngleToLandmark(1337, roboX, roboY, roboPsi)*180/M_PI<<std::endl;
	std::cout<<"Angle LM 1338: "<<testSet.getAngleToLandmark(1338, roboX, roboY, roboPsi)*180/M_PI<<std::endl;
	std::cout<<"Angle LM 1339: "<<testSet.getAngleToLandmark(1339, roboX, roboY, roboPsi)*180/M_PI<<std::endl;
	return true;
}

// To test the projection on a set of input data. Will display the image with projected bits.
// +rating of pattern under projected 2D points.
double unitTests::cameraCalibrationTest(double x, double y, double psi, std::string imgPath) {

	std::vector<cv::Point2d> imagePoints, imagePointsCliped;
	std::vector<patternPoint> clipedImagePoints;
	cv::Mat objectPoints = locaUtil::getBit3Dlocations_all();
	cv::Mat color_img = cv::imread( imgPath, 1 );
	cv::Mat img;
	cvtColor(color_img,img,CV_RGB2GRAY);
	cameraParam cam;
	int grid = 1;

	cam.setExtr(psi,x,y);
	cam.projectTo2D(&objectPoints,&imagePoints);

	clipedImagePoints = picRating::clipANDmark(imagePoints,locaUtil::getPatternCode93(), img.cols, img.rows);
	double p = picRating::rateImage(img, clipedImagePoints,grid);
//	std::cout<<"  p = "<<p<<std::endl;
//	for(int i=0; i<imagePoints.size(); i++)
//		//cv::circle(img,cv::Point(imagePoints.at(i).x,imagePoints.at(i).y),grid,cv::Scalar(0,0,255),1,8);
//		cv::rectangle(img,cv::Point(imagePoints.at(i).x-grid,imagePoints.at(i).y-grid),cv::Point(imagePoints.at(i).x+grid,imagePoints.at(i).y+grid),cv::Scalar(0,0,255),1,8,0);
//	std::ostringstream s;
//	s<<"Gewichtung: "<<p;
//	cv::putText(img, s.str(), cvPoint(30,700),
//		    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);
//
//	cv::namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
//	cv::imshow( "Display Image", img );
//	cv::waitKey(0);
	return p;
}

//
bool unitTests::picTest() {
	std::string line;
	std::vector<std::string> imgPaths;
	std::vector<double> pose;
	std::ifstream myfile ("/home/josef/workspace/Loca-Projekt/locaDatafile.txt");
	if (myfile.is_open())
	{
		while ( myfile.good() )
		{
			getline (myfile,line);
			int index = line.find_first_of("/");
			if(index != line.npos){ // if a / is found, there is a picture path in that line
				imgPaths.push_back(line.substr(index,line.size()));
				int index_follow = 0;
				index = line.find_first_of(";",0);
				while(index != line.npos){ // read the saved variables from the log and push them into a vector.
					pose.push_back(std::atof(line.substr(index_follow,index-index_follow).data()));
					index_follow = index+2;
					index = line.find_first_of(";",index+1);
				}
			}
			index = 0;
		}
		myfile.close();
	}
	else {
		std::cout << "Unable to open file";
		return false;
	}
	std::ofstream testFile ("/home/josef/Desktop/testfile.txt");
	std::vector<cv::Point2d> imagePoints, imagePointsCliped;
	std::vector<patternPoint> clipedImagePoints;
	cv::Mat objectPoints = locaUtil::getBit3Dlocations_all();
	cv::Mat color_img = cv::imread( imgPaths.at(0), 1 );
	cv::Mat img;
	unsigned char* pattern = locaUtil::getPatternCode93();
	cvtColor(color_img,img,CV_RGB2GRAY);
	cameraParam cam;
	std::cout<<"from -0.5 to 0.5 in 0.005 steps"<<std::endl;
	for(double x = -0.5; x <= 0.5; x += 0.005){
		std::cout<<x<<std::endl;
		for(double y = -0.5; y <= 0.5; y += 0.005){
			double psum = 0;
			for(double psi = -0.5; psi <= 0.5; psi += 0.005){
				double p = 0;
				cam.setExtr(psi,x,y);
				cam.projectTo2D(&objectPoints,&imagePoints);
				clipedImagePoints = picRating::clipANDmark(imagePoints,pattern, img.cols, img.rows);
				p = picRating::rateImage(img, clipedImagePoints,1);
				//if(p > 0.9)
					psum += p;
				//psum += cameraCalibrationTest(x, y, psi, imgPaths.at(0));
			}
			testFile << x <<" "<< y << " "<< psum <<std::endl;
		}
	}
	testFile.close();
//	double x, y, psi, incRight, incLeft;
//	int ansPics =  imgPaths.size();
//	int ansValues = pose.size()/ansPics;
//	assert(pose.size()%ansPics == 0);
//	for(int i=0;i<ansPics;i++){
//		psi = pose.at(i*ansValues+8);
//		y = pose.at(i*ansValues+5);
//		x = pose.at(i*ansValues+2);
//		incRight = pose.at(i*ansValues+1);
//		incLeft = pose.at(i*ansValues);
//		std::cout<<"pic: "<<imgPaths.at(i);
//		cameraCalibrationTest(x, y, psi, imgPaths.at(i));
//		//ratingEval(x, y, psi, imgPaths.at(i));
//	}
return true;
}

bool unitTests::cvProjectTest() {
		std::vector<cv::Point2d> imagePoints_1;
		cv::Mat objectPoints_1 (1,3,CV_64FC1);
		objectPoints_1.at<double>(0,0) = 6.0;
		objectPoints_1.at<double>(0,1) = 0.0;
		objectPoints_1.at<double>(0,2) = 2.9375;

		cv::Mat K(3,3,CV_64FC1); //intrinsic parameter matrix
			K.at<double>(0,0) = 2101.54;	K.at<double>(0,1) = 0.0; 		K.at<double>(0,2) = 1366.0/2;
			K.at<double>(1,0) = 0.0; 		K.at<double>(1,1) = 1476.92; 	K.at<double>(1,2) = 768.0/2;
			K.at<double>(2,0) = 0.0; 		K.at<double>(2,1) = 0.0;		K.at<double>(2,2) = 1.0;

		cv::Mat noRot(3,3,CV_64FC1);
			noRot.at<double>(0,0) = 1.0; 	noRot.at<double>(0,1) = 0.0;	noRot.at<double>(0,2) = 0.0;
			noRot.at<double>(1,0) = 0.0;	noRot.at<double>(1,1) = 1.0;	noRot.at<double>(1,2) = 0.0;
			noRot.at<double>(2,0) = 0.0;	noRot.at<double>(2,1) = 0.0;	noRot.at<double>(2,2) = 1.0;

		double angle = M_PI/2;
		cv::Mat rotX(3,3,CV_64FC1);
			rotX.at<double>(0,0) = 1.0; 	rotX.at<double>(0,1) = 0.0;	rotX.at<double>(0,2) = 0.0;
			rotX.at<double>(1,0) = 0.0;	rotX.at<double>(1,1) = cos(angle);	rotX.at<double>(1,2) = -sin(angle);
			rotX.at<double>(2,0) = 0.0;	rotX.at<double>(2,1) = sin(angle);	rotX.at<double>(2,2) = cos(angle);

			double tmp = 0;
		cv::Mat rotY(3,3,CV_64FC1);
			rotY.at<double>(0,0) = cos(-M_PI/2-tmp); 	rotY.at<double>(0,1) = 0.0;	rotY.at<double>(0,2) = sin(-M_PI/2-tmp);
			rotY.at<double>(1,0) = 0.0;	rotY.at<double>(1,1) = 1.0;	rotY.at<double>(1,2) = 0.0;
			rotY.at<double>(2,0) = -sin(-M_PI/2-tmp);	rotY.at<double>(2,1) = 0.0;	rotY.at<double>(2,2) = cos(-M_PI/2-tmp);



		cv::Mat rotT(3,3,CV_64FC1);
		rotT = rotY*rotX;
		cv::Mat rvec(3,1,CV_64FC1); //rotation matrix
		cv::Rodrigues(noRot, rvec);

		cv::Mat testVec(4,1,CV_64FC1);
		testVec.at<double>(0,0) = 0;
		testVec.at<double>(0,1) = 0;
		testVec.at<double>(0,2) = 0;
		testVec.at<double>(0,3) = 1;

		std::cout<<"WorldVec :("<<testVec.at<double>(0,0)<<", "<<testVec.at<double>(0,1)<<", "<<testVec.at<double>(0,2)<<", "<<testVec.at<double>(0,3)<<")"<<std::endl;

		cv::Mat testRobMat(4,4,CV_64FC1);

//		cv::Mat testMat(4,4,CV_64FC1);
//		testMat.at<double>(0,0) = 0;	testMat.at<double>(0,1) = -1;	testMat.at<double>(0,2) = 0;	testMat.at<double>(0,3) = -0.093889;
//		testMat.at<double>(1,0) = 0;	testMat.at<double>(1,1) = 0;	testMat.at<double>(1,2) = -1;	testMat.at<double>(1,3) = 0.260804;
//		testMat.at<double>(2,0) = 1;	testMat.at<double>(2,1) = 0;	testMat.at<double>(2,2) = 0;	testMat.at<double>(2,3) = 2.83599;
//		testMat.at<double>(3,0) = 0;	testMat.at<double>(3,1) = 0;	testMat.at<double>(3,2) = 0;	testMat.at<double>(3,3) = 1;

		cv::Mat testMat(4,4,CV_64FC1);
		testMat.at<double>(0,0) = 0;	testMat.at<double>(0,1) = -1;	testMat.at<double>(0,2) = 0;	testMat.at<double>(0,3) = 0;
		testMat.at<double>(1,0) = 0;	testMat.at<double>(1,1) = 0;	testMat.at<double>(1,2) = -1;	testMat.at<double>(1,3) = 0;
		testMat.at<double>(2,0) = 1;	testMat.at<double>(2,1) = 0;	testMat.at<double>(2,2) = 0;	testMat.at<double>(2,3) = 0;
		testMat.at<double>(3,0) = 0;	testMat.at<double>(3,1) = 0;	testMat.at<double>(3,2) = 0;	testMat.at<double>(3,3) = 1;

		cv::Mat testMat2(4,4,CV_64FC1);
		testMat2.at<double>(0,0) = 1;	testMat2.at<double>(0,1) = 0;	testMat2.at<double>(0,2) = 0;	testMat2.at<double>(0,3) = -2.83599;
		testMat2.at<double>(1,0) = 0;	testMat2.at<double>(1,1) = 1;	testMat2.at<double>(1,2) = 0;	testMat2.at<double>(1,3) = -0.093889;
		testMat2.at<double>(2,0) = 0;	testMat2.at<double>(2,1) = 0;	testMat2.at<double>(2,2) = 1;	testMat2.at<double>(2,3) = 0.260804;
		testMat2.at<double>(3,0) = 0;	testMat2.at<double>(3,1) = 0;	testMat2.at<double>(3,2) = 0;	testMat2.at<double>(3,3) = 1;

		testVec = testMat*testMat2*testVec;

		std::cout<<"NewVec :("<<testVec.at<double>(0,0)<<", "<<testVec.at<double>(0,1)<<", "<<testVec.at<double>(0,2)<<", "<<testVec.at<double>(0,3)<<")"<<std::endl;

		cv::Mat T(3,1,CV_64FC1); //translation vector
		T.at<double>(0,0) = 0;//(Y)
		T.at<double>(1,0) = 0;//(Z)
		T.at<double>(2,0) = 6;//(-X)

		//Create zero distortion
		cv::Mat distCoeffs(4,1,CV_64FC1); //distortion vector
		distCoeffs.at<double>(0,0) = 0;
		distCoeffs.at<double>(1,0) = 0;
		distCoeffs.at<double>(2,0) = 0;
		distCoeffs.at<double>(3,0) = 0;

//		cv::Mat img = cv::imread( "/home/josef/workspace/Loca-Projekt/pictures/0000.jpg", 1 );
//
//		cv::projectPoints(objectPoints_1, rotT, T, K, distCoeffs, imagePoints_1);
//
//		std::cout<<"3d Point: ("
//			<<objectPoints_1.at<double>(0,0)<<","
//			<<objectPoints_1.at<double>(0,1)<<","
//			<<objectPoints_1.at<double>(0,2)<<")   ";
//		std::cout<<"2d Point: ("
//			<<imagePoints_1.at(0).x<<", "
//			<<imagePoints_1.at(0).y<<")"<<std::endl;
//		cv::circle(img,cv::Point(imagePoints_1.at(0).x,imagePoints_1.at(0).y),5.0,cv::Scalar(255,0,0),1,8);
//
//		cv::namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
//		cv::imshow( "Display Image", img );
//
//		cv::waitKey(0);

		return true;

}

bool unitTests::ratingEval(double x, double y, double psi, std::string imgPath) {
	std::vector<cv::Point2d> imagePoints, imagePointsCliped;
	std::vector<patternPoint> clipedImagePoints;
	cv::Mat objectPoints = locaUtil::getBit3Dlocations_all();
	cv::Mat img = cv::imread( imgPath, 1 );
	cameraParam cam;
	std::ostringstream path;
	path<<"/home/josef/workspace/Loca-Projekt/ratingEvalData_"<<imgPath.substr(44,4)<<".txt";
	if(true){
		DataWriter evalRatingFile;
		evalRatingFile.path_ = "/home/josef/workspace/Loca-Projekt/test/";
		evalRatingFile.datafile_path_ = path.str();
		for(double j=-0.5;j<=+0.5;j+=0.01){
			for(double i=-0.5;i<=+0.5;i+=0.01){
				std::vector<patternPoint> clipedImagePointsEval;
				std::vector<cv::Point2d> imagePointsEval;
				cam.setExtr(psi,x+i,y+j);
				cam.projectTo2D(&objectPoints,&imagePointsEval);
				clipedImagePointsEval = picRating::clipANDmark(imagePointsEval,locaUtil::getPatternCode93(), img.cols, img.rows);
				double p = picRating::rateImage(img, clipedImagePointsEval,2);
				evalRatingFile.WriteData(i,j,p,0,0);

			}
		}
		evalRatingFile.WriteData(0,0,0,0,0);
	}

	cam.setExtr(psi,x,y);

	cam.projectTo2D(&objectPoints,&imagePoints);

	clipedImagePoints = picRating::clipANDmark(imagePoints,locaUtil::getPatternCode93(), img.cols, img.rows);
	double p = picRating::rateImage_3x3(img, clipedImagePoints);

	for(int i=0; i<imagePoints.size(); i++)
		cv::circle(img,cv::Point(imagePoints.at(i).x,imagePoints.at(i).y),5.0,cv::Scalar(0,0,255),1,8);
	std::ostringstream s;
	s<<"Gewichtung: "<<p;
	cv::putText(img, s.str(), cvPoint(30,700),
		    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,250,250), 1, CV_AA);

	cv::namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
	cv::imshow( "Display Image", img );
	cv::waitKey(0);
	return true;
}

bool unitTests::compareProjection(cameraParam* camp, osg::Matrix MVPW) {
	/**
	 * Erzeugt zufällige 3D Punkte im Intervall [-12.0 .. 12.0] und vergleich
	 * die openCV Projektion mit der MVPW-Matrix aus OpenSceneGraph.
	 */
	double x, y, z;
	x = locaUtil::randomUniform()*12.0-locaUtil::randomUniform()*12.0;
	y = locaUtil::randomUniform()*12.0-locaUtil::randomUniform()*12.0;
	z = locaUtil::randomUniform()*12.0-locaUtil::randomUniform()*12.0;

	// Projektion mittels der MVPW aus OpenSceneGraph
	osg::Vec3d osgVector(x,y,z);//6.0,-0.093889,0.260804
	osg::Vec3d osgImagePoints;
	osgImagePoints = osgVector*MVPW;

	// Projektion mit der openCV Camera funktion
	cv::Mat cvVector (1,3,CV_64FC1);
	cvVector.at<double>(0,0) = x;
	cvVector.at<double>(0,1) = y;
	cvVector.at<double>(0,2) = z;
	std::vector<cv::Point2d> cvImagePoints;
	camp->projectTo2D(&cvVector,&cvImagePoints);

	if(abs(cvImagePoints.at(0).x-osgImagePoints._v[0])>0.000001 || abs(cvImagePoints.at(0).y-768+osgImagePoints._v[1])>0.000001){
		std::cout<<std::endl;
		std::cout<<"--------UnitTest 2D Projection---------"<<std::endl;
		std::cout<<"3D_Point: ("<<x<<", "<<y<<", "<<z<<")"<<std::endl;
		std::cout<<"CV_Projection:  ("<<cvImagePoints.at(0).x<<", "<<cvImagePoints.at(0).y<<")"<<std::endl;
		std::cout<<"OSG_Projection: ("<<osgImagePoints._v[0]<<", "<<768-osgImagePoints._v[1]<<")"<<std::endl;
		std::cout<<"Difference:     ("<<cvImagePoints.at(0).x-osgImagePoints._v[0]<<", "
									  <<cvImagePoints.at(0).y-768+osgImagePoints._v[1]<<")"<<std::endl;
		std::cout<<std::endl;
		assert(false);
	}
	return true;
}






