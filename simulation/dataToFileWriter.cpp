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
	text << blank_entry_ << delimiter_;
	text << blank_entry_ << delimiter_;
	text << blank_entry_ << delimiter_;
	text << blank_entry_;
	WriteData(text.str());
}

std::string DataWriter::WriteImg(cv::Mat img) {
	std::ostringstream img_name;
	img_name << path_;
	img_name << std::setw(4) << std::setfill('0') << img_counter_;
	img_name << ".jpg";
	cv::imwrite( img_name.str(), img );
	img_counter_++;
	return img_name.str();
}

void DataWriter::WriteData(double increment_left, double increment_right,	cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;
	text << blank_entry_ << delimiter_;
	text << blank_entry_ << delimiter_;
	text << blank_entry_ << delimiter_;
	text << WriteImg(img);
	WriteData(text.str());
}

void DataWriter::WriteData(double increment_left, double increment_right,
		double x, double y, double psi, cv::Mat img) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << x << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << y << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << psi << delimiter_;
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
				path_(pic_path),
				datafile_path_(file_path),
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

void DataWriter::WriteData(double increment_left, double increment_right,
		double x, double y, double psi) {
	std::ostringstream text;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_left << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << increment_right << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << x << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << y << delimiter_;
	text << std::setw(width_numeric_entry_) << std::setfill(' ') << psi << delimiter_;
	text << blank_entry_;
	WriteData(text.str());
}

