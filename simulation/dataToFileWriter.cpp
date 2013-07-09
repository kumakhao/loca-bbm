/*
 * dataToFileWriter.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: josef
 */
#include <opencv.hpp>
#include <highgui/highgui_c.h>
#include <iostream>
#include <fstream>
#include "dataToFileWriter.h"



void DataWriter::WriteData(double increment_left, double increment_right) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << blank_entry_;
	WriteData(text.str());
}

std::string DataWriter::WriteImg(cv::Mat img) {
	std::ostringstream img_name;
	img_name << path_;
	img_name << std::setw(4) << std::setfill('0') << img_counter_;
	img_name << ".jpg";
	//cv::imwrite( img_name.str(), img );
	imgBuffer.push_back(img);
	imgNameBuffer.push_back(img_name.str());
	img_counter_++;
	return img_name.str();
}

void DataWriter::WriteData(double increment_left, double increment_right,	cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << WriteImg(img);
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double robY, double robPsi, cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << WriteImg(img);
	WriteData(text.str());
}

void DataWriter::WriteData(std::string text) {
	text += "\n";
	std::ofstream datafile;
	datafile.open (datafile_path_.c_str(),std::ios_base::app);
	datafile << text;
	datafile.close();
}

DataWriter::DataWriter(std::string pic_path, std::string file_path):
				img_counter_(0),
				width_numeric_entry_(11),
				plotfile_path_("/home/josef/workspace/Loca-Projekt/plotData.txt"),
				delimiter_("; "),
				blank_entry_("BLANK")
{
	datafile_path_ = file_path;
	path_ = pic_path;
	Header();
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double robY, double robPsi) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << blank_entry_;
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double camX, double robY, double camY, double robPsi,
		double camPsi) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << blank_entry_;
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double camX, double partX, double robY, double camY,
		double partY, double robPsi, double camPsi, double partPsi) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partX << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partY << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partPsi << delimiter_;

	text << blank_entry_;
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double camX, double partX, double robY, double camY,
		double partY, double robPsi, double camPsi, double partPsi,
		cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partX << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partY << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << partPsi << delimiter_;

	text << WriteImg(img);
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double robX, double camX, double robY, double camY, double robPsi,
		double camPsi, cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << robPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << camPsi << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << blank_entry_ << delimiter_;

	text << WriteImg(img);
	WriteData(text.str());
}

void DataWriter::SaveImages() {
	for(unsigned int i=0;i<imgBuffer.size();i++)
	{
		cv::imwrite( imgNameBuffer.at(i), imgBuffer.at(i) );
	}
}

void DataWriter::WritePlotData(double trueRobX, double trueRobY,
		double trueRobPsi, double meanX, double meanY, double varX,
		double varY, bool observe) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << trueRobX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << trueRobY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << trueRobPsi << delimiter_;

	text << std::setw(width_numeric_entry_) << std::setfill(' ') << meanX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << meanY << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << varX << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << varY << delimiter_;
	if(observe)
		text << std::setw(width_numeric_entry_) << std::setfill(' ') << trueRobX << delimiter_;
	else
		text << std::setw(width_numeric_entry_) << std::setfill(' ') << "noimage" << delimiter_;

	text << "\n";
	std::ofstream datafile;
	datafile.open (plotfile_path_.c_str(),std::ios_base::app);
	datafile << text.str();
	datafile.close();
}

void DataWriter::Header() {
	std::ostringstream dataHeader;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "incLeft" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "incRight" << delimiter_;

	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPosX" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "camPosX" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "partPosX" << delimiter_;

	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPosY" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "camPosY" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "partPosY" << delimiter_;

	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "robPsi" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "camPsi" << delimiter_;
	dataHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "partPsi" << delimiter_;

	dataHeader << "imageFile";

	dataHeader << "\n";
	std::ofstream datafile;
	datafile.open (datafile_path_.c_str(),std::ios_base::out);
	datafile << dataHeader.str();
	datafile.close();

	std::ostringstream plotHeader;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "trueRobX" << delimiter_;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "trueRobY" << delimiter_;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "trueRobPsi" << delimiter_;

	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "meanX" << delimiter_;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "meanY" << delimiter_;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "varX" << delimiter_;
	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "varY" << delimiter_;

	plotHeader << std::setw(width_numeric_entry_) << std::setfill(' ') << "Observe" << delimiter_;

	plotHeader << "\n";
	std::ofstream plotfile;
	plotfile.open (plotfile_path_.c_str(),std::ios_base::out);
	plotfile << plotHeader.str();
	plotfile.close();
}


