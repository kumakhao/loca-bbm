/*
 * picRating.h
 *
 *  Created on: Mar 7, 2013
 *      Author: josef
 */

#ifndef PICRATING_H_
#define PICRATING_H_
#include <opencv.hpp>

struct patternPoint{
	int x;
	int y;
	int patternValue;
	int pixelValue;
};

class picRating {
public:
	static std::vector<patternPoint> clipANDmark(std::vector<cv::Point2d> imagePoints, unsigned char pattern[], int width, int height);
	static double rateImage_1x1(cv::Mat img, std::vector<patternPoint> pattern);
	static double rateImage_3x3(cv::Mat img, std::vector<patternPoint> pattern);
	static double rateImage(cv::Mat img, std::vector<patternPoint> pattern, int grid);
	static int pixel(cv::Mat img, int x, int y);
	static int bw_pixel(cv::Mat img, int x, int y);
};


#endif /* PICRATING_H_ */
