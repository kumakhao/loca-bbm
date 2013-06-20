/*
 * dataToFileWriter.h
 *
 *  Created on: Feb 4, 2013
 *      Author: josef
 */

#ifndef DATATOFILEWRITER_H_
#define DATATOFILEWRITER_H_

#include <opencv.hpp>
#include <highgui/highgui_c.h>
#include <iomanip>

class DataWriter
{
public:
	DataWriter():
		img_counter_(0),
		width_numeric_entry_(11),
		path_("/home/josef/workspace/Loca-Projekt/pictures/"),
		datafile_path_("/home/josef/workspace/Loca-Projekt/locaDatafile.txt"),
		delimiter_("; "),
		blank_entry_("BLANK")
	{
		WriteData(Header());
	}
	DataWriter(std::string picPath, std::string filePath);

	void WriteData(double increment_left, double increment_right);
	void WriteData(double increment_left, double increment_right, 	double robX, double robY, double robPsi);
	void WriteData(double increment_left, double increment_right, 	double robX, double camX,
																	double robY, double camY,
																	double robPsi, double camPsi);
	void WriteData(double increment_left, double increment_right, 	double robX, double camX, double partX,
																	double robY, double camY, double partY,
																	double robPsi, double camPsi, double partPsi);
	void WriteData(double increment_left, double increment_right, cv::Mat img);
	void WriteData(double increment_left, double increment_right, 	double robX, double robY, double robPsi, cv::Mat img);
	void WriteData(double increment_left, double increment_right, 	double robX, double camX, double partX,
																	double robY, double camY, double partY,
																	double robPsi, double camPsi, double partPsi, cv::Mat img);
	void WriteData(double increment_left, double increment_right, 	double robX, double camX,
																	double robY, double camY,
																	double robPsi, double camPsi, cv::Mat img);


protected:
	int img_counter_;
	int width_numeric_entry_;
	std::string path_;
	std::string datafile_path_;
	std::string delimiter_;
	std::string blank_entry_;
	std::string WriteImg(cv::Mat img);
	void WriteData(std::string imgName);
	std::string Header();
};


#endif /* DATATOFILEWRITER_H_ */
