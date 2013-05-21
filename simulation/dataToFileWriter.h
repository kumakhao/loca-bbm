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
		std::ostringstream header;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "incLeft" << delimiter_;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "incRight" << delimiter_;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPosX" << delimiter_;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPosY" << delimiter_;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPsi" << delimiter_;
		header << std::setw(width_numeric_entry_) << std::setfill(' ') << "imageFile" << delimiter_;
		WriteData(header.str());
	}
	DataWriter(std::string picPath, std::string filePath);

	void WriteData(double incrementLeft, double incrementRight);
	void WriteData(double incrementLeft, double incrementRight, cv::Mat img);
	void WriteData(double incrementLeft, double incrementRight, double x, double y, double psi, cv::Mat img);
	void WriteData(double incrementLeft, double incrementRight, double x, double y, double psi);
protected:
	int img_counter_;
	int width_numeric_entry_;
	std::string path_;
	std::string datafile_path_;
	std::string delimiter_;
	std::string blank_entry_;
	std::string WriteImg(cv::Mat img);
	void WriteData(std::string imgName);
};


#endif /* DATATOFILEWRITER_H_ */
