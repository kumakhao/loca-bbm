/*
 * picRating.cpp
 *
 *  Created on: Mar 7, 2013
 *      Author: josef
 */


#include "picRating.h"
/**
 *	Test doku
 *
 * @param imagePoints
 * @param pattern
 * @param width
 * @param height
 * @return
 */
std::vector<patternPoint> picRating::clipANDmark(
		std::vector<cv::Point2d> imagePoints, unsigned char pattern[],
		int width, int height) {
	std::vector<patternPoint> patternPoints;
	patternPoint tmp;
	int index = 0;
	for(std::vector<cv::Point2d>::iterator i = imagePoints.begin() ; i != imagePoints.end();i++){
		if((i->x < width) && (i->x >= 0)){
			if((i->y < height) && (i->y >= 0)){
				tmp.x = i->x;
				tmp.y = i->y;
				tmp.patternValue = pattern[index];
				patternPoints.push_back(tmp);
			}
		}
		index++;
	}
	return patternPoints;
}

double picRating::rateImage_1x1(cv::Mat img, std::vector<patternPoint> pattern) {
	/**
	 * Errechnet auf Grundlage der gegebenen Bildpunkt und Muster in pattern
	 * einen Gewichtungsfaktor: 1 = gute Übereinstimmung
	 * 							0 = keine Übereinstimmung
	 */
	int whiteCount = 0;
	double avgWhite = 0.0;
	int blackCount = 0;
	double avgBlack = 0.0;
	double kontrast = 0.0;
	double pSum = 0.0;

	// Berechnet mittlere Scharz- und Weißwerte auf
	// Grundlage des Patterns.
	for(unsigned int i=0;i<pattern.size();i++){
		if(pattern.at(i).patternValue == 255){
			whiteCount++;
			avgWhite += pixel(img, pattern.at(i).x, pattern.at(i).y);
			pattern.at(i).pixelValue = pixel(img, pattern.at(i).x, pattern.at(i).y);
		}
		if(pattern.at(i).patternValue == 0){
			blackCount++;
			avgBlack += pixel(img, pattern.at(i).x, pattern.at(i).y);
			pattern.at(i).pixelValue = pixel(img, pattern.at(i).x, pattern.at(i).y);
		}
	}
	// Wenn es Punkte im Bild gibt, werden die Mittelwerte gebildet.
	if(pattern.size()>0){
		avgBlack = avgBlack/blackCount;
		avgWhite = avgWhite/whiteCount;
	}

	// Wenn der mittlere Schwarzwert der dunkeln Pixel größer ist als der
	// mittlere Weißwert der weißen Pixel, dann besteht keine übereinstimmung.
	if(avgWhite-avgBlack<=0)
		return 0.0;

	// Der Kontrast ist der quadratische Abstand der mittleren Schwarz- und Weißwerte
	kontrast = avgWhite-avgBlack;
	kontrast *= kontrast;

	// Hier wird der quadratische Abstand der Bildpixelwerte zu den mittleren
	// Schwarz- bzw Weißwerten gebildet und durch den Kontrast geteilt.
	for(unsigned int i=0;i<pattern.size();i++){
		double p = 0;
		if(pattern.at(i).patternValue == 0)
			p = pattern.at(i).pixelValue - avgBlack;
		if(pattern.at(i).patternValue == 255)
			p = pattern.at(i).pixelValue - avgWhite;
		p *= p;
		pSum += p/kontrast;
	}
	pSum = pSum/pattern.size();
//	std::cout<<std::endl;
//	std::cout<<"avgWhite: "<<avgWhite<<"  avgBlack: "<<avgBlack<<"  pSum: "<<pSum<<"  exp(-pSum): "<<exp(-pSum)<<std::endl;
//	std::cout<<std::endl;

	return exp(-pSum);
}

double picRating::rateImage_3x3(cv::Mat img,
		std::vector<patternPoint> pattern) {
	/**
	 * Errechnet auf Grundlage der gegebenen Bildpunkt und Muster in pattern
	 * einen Gewichtungsfaktor: 1 = gute Übereinstimmung
	 * 							0 = keine Übereinstimmung
	 */
	int whiteCount = 0;
	double avgWhite = 0.0;
	int blackCount = 0;
	double avgBlack = 0.0;
	double kontrast2 = 0.0;
	double minKontrast = 20;
	double pSum = 0.0;

	// Berechnet mittlere Scharz- und Weißwerte auf
	// Grundlage des Patterns.
	for(unsigned int i=0;i<pattern.size();i++){
		int x = pattern.at(i).x;
		int y = pattern.at(i).y;
		double value = 0;
		value += pixel(img, x-1, y-1);
		value += pixel(img, x-1, y);
		value += pixel(img, x-1, y+1);
		value += pixel(img, x, y-1);
		value += pixel(img, x, y);
		value += pixel(img, x, y+1);
		value += pixel(img, x+1, y-1);
		value += pixel(img, x+1, y);
		value += pixel(img, x+1, y+1);
		value = value / 9;
		if(pattern.at(i).patternValue == 255){
			whiteCount++;
			avgWhite += value;
			pattern.at(i).pixelValue = value;
		}
		if(pattern.at(i).patternValue == 0){
			blackCount++;
			avgBlack += value;
			pattern.at(i).pixelValue = value;
		}
	}
	// Wenn es Punkte im Bild gibt, werden die Mittelwerte gebildet.
	if(pattern.size()>0){
		avgBlack = avgBlack/blackCount;
		avgWhite = avgWhite/whiteCount;
	}

	// Wenn der mittlere Schwarzwert der dunkeln Pixel größer ist als der
	// mittlere Weißwert der weißen Pixel, dann besteht keine übereinstimmung.
	if(avgWhite-avgBlack<=0)
		return 0.0;

	// Der Kontrast ist der quadratische Abstand der mittleren Schwarz- und Weißwerte
	kontrast2 = avgWhite-avgBlack;
	kontrast2 *= kontrast2;

	// Hier wird der quadratische Abstand der Bildpixelwerte zu den mittleren
	// Schwarz- bzw Weißwerten gebildet und durch den Kontrast geteilt.
	for(unsigned int i=0;i<pattern.size();i++){
		double p = 0;
		if(pattern.at(i).patternValue == 0)
			p = pattern.at(i).pixelValue - avgBlack;
		if(pattern.at(i).patternValue == 255)
			p = pattern.at(i).pixelValue - avgWhite;
		p *= p;
		pSum += (p+minKontrast*minKontrast)/(kontrast2/4+minKontrast*minKontrast);
	}
	//pSum = pSum;
//	std::cout<<std::endl;
//	std::cout<<"avgWhite: "<<avgWhite<<"  avgBlack: "<<avgBlack<<"  pSum: "<<pSum<<"  exp(-pSum): "<<exp(-pSum)<<std::endl;
//	std::cout<<std::endl;

	return exp(-0.5*pSum);
}

double picRating::rateImage(cv::Mat img, std::vector<patternPoint> pattern,
		int grid) {
	/**
	 * Errechnet auf Grundlage der gegebenen Bildpunkt und Muster in pattern
	 * einen Gewichtungsfaktor: 1 = gute Übereinstimmung
	 * 							0 = keine Übereinstimmung
	 */
	//TODO: Need to improve the rating function.
	int whiteCount = 0;
	double avgWhite = 0.0;
	int blackCount = 0;
	double avgBlack = 0.0;
	double kontrast = 0.0;
	double pSum = 0.0;

	if(pattern.size()<10)
		return 0.0;

	// Berechnet mittlere Scharz- und Weißwerte auf
	// Grundlage des Patterns.
	for(unsigned int i=0;i<pattern.size();i++){
		int x = pattern.at(i).x;
		int y = pattern.at(i).y;
		double value = 0;
		for(int j=-grid;j<=grid;j++){
			for(int u=-grid;u<=grid;u++)
				value += bw_pixel(img, x+j, y+u);
		}
		value = value / ((grid+grid+1)*(grid+grid+1));
		if(pattern.at(i).patternValue == 255){
			whiteCount++;
			avgWhite += value;
			pattern.at(i).pixelValue = value;
		}
		if(pattern.at(i).patternValue == 0){
			blackCount++;
			avgBlack += value;
			pattern.at(i).pixelValue = value;
		}
	}
	// Wenn es Punkte im Bild gibt, werden die Mittelwerte gebildet.
	if(pattern.size()>0){
		avgBlack = avgBlack/(blackCount);
		avgWhite = avgWhite/(whiteCount);
	}

	// Wenn der mittlere Schwarzwert der dunkeln Pixel größer ist als der
	// mittlere Weißwert der weißen Pixel, dann besteht keine übereinstimmung.
	if(avgWhite-avgBlack<=0)
		return 0.0;

	// Der Kontrast ist der quadratische Abstand der mittleren Schwarz- und Weißwerte
	kontrast = avgWhite-avgBlack;
	kontrast *= kontrast;

	// Hier wird der quadratische Abstand der Bildpixelwerte zu den mittleren
	// Schwarz- bzw Weißwerten gebildet und durch den Kontrast geteilt.
	for(unsigned int i=0;i<pattern.size();i++){
		double p = 0;
		if(pattern.at(i).patternValue == 0)
			p = pattern.at(i).pixelValue - avgBlack;
		if(pattern.at(i).patternValue == 255)
			p = pattern.at(i).pixelValue - avgWhite;
		p *= p;
		pSum += p/kontrast;
	}
	pSum = pSum/pattern.size();//*((log(133))/(log(kontrast/2+1)));
//	std::cout<<std::endl;
//	std::cout<<"avgWhite: "<<avgWhite<<"  avgBlack: "<<avgBlack<<"  pSum: "<<pSum<<"  exp(-pSum): "<<exp(-pSum)<<std::endl;
//	std::cout<<std::endl;

	return exp(-pSum);
	//return kontrast;

}

/**
 * get pixel at x/y coordinates. If out of bound, it will be set to min or max value
 * @param img
 * @param x
 * @param y
 * @return
 */
int picRating::pixel(cv::Mat img, int x, int y) {
	if(x < 0) x = 0;
	if(x >= img.cols) x = img.cols-1;
	if(y < 0) y = 0;
	if(y >= img.rows) y = img.rows-1;
	int ret = 0;
	unsigned char *adr = img.data+img.step[0]*y+img.step[1]*x;
			ret += *adr;
			adr++;
			ret += *adr;
			adr++;
			ret += *adr;
	return ret/3;
}

int picRating::bw_pixel(cv::Mat img, int x, int y){
	if(x < 0) x = 0;
	if(x >= img.cols) x = img.cols-1;
	if(y < 0) y = 0;
	if(y >= img.rows) y = img.rows-1;
	unsigned char *adr = img.ptr<unsigned char>(y)+x;
	int ret = *adr;
	return ret;
}
