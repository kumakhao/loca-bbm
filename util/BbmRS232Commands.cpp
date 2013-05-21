/*
 * BbmRS232Commands.cpp
 *
 *  Created on: Apr 25, 2013
 *      Author: josef
 */

#include "BbmRS232Commands.h"
#include <sstream>
#include <iostream>


/**
 * Set runmode.
 *
 * @param mode : Stop(0), Start(1), Remote(2)
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SM(int mode) {
	std::ostringstream command;
	command << "SM,"<<mode<<"\n";
	return command.str();
}

/**
 * Request current runmode.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGM() {
	return "SGM\n";
}

/**
 * Set Speed of the robot.
 * Values may be between -1023(max backwards speed) and 1023(max forward speed)
 * @param left : Speed for left wheel.
 * @param right : Speed for right wheel.
 * @return The command to be transmitted to the controller
 */
std::string BbmRS232Commands::SS(int left, int right) {
	if(left > 1023) left = 1023;
	if(left < -1023) left = -1023;
	if(right > 1023) right = 1023;
	if(right < -1023) right = -1023;

	std::ostringstream command;
	command << "SS,";
	command << left;
	command << ",";
	command << right;
	command << "\n";
	return command.str();
}

/**
 * Set Speed to zero.
 * @return The command to be transmitted to the controller
 */
std::string BbmRS232Commands::SS0() {
	return SS(0,0);
}

/**
 * Request the current speed of the robot.
 * These values are not the same as the ones set with @see SS()
 * @return The command to be transmitted to the controller
 */
std::string BbmRS232Commands::SGS() {
	return "SGS\n";
}

/**
 * Request the current raw data from the rotary encoder.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGEG() {
	return "SGEG\n";
}

/**
 * Request reset of rotary encoder increments.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SC() {
	return "SC\n";
}

/**
 * Request position since last reset in mm.
 * For odometry purpose use @see SGEG()
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGEA() {
	return "SGEA\n";
}

/**
 * Request position since the last time this command was used in mm.
 * For odometry purpose use @see SGEG()
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGER() {
	return "SGER\n";
}

/**
 * Set the rotary encoder increments to these values.
 * @param left
 * @param right
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SE(int left, int right) {
	std::ostringstream command;
	command<<"SE,"<<left<<","<<right<<"\n";
	return command.str();
}

/**
 * Request compass bearing in degree.
 * A value of -1 means not compass or defect.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGB() {
	return "SGB\n";
}

/**
 * Move number of steps with a defined speed.
 * @param steps_left
 * @param steps_right
 * @param speed_left
 * @param speed_right
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SP(int steps_left, int steps_right,
		int speed_left, int speed_right) {
	std::ostringstream command;
	command<<"SP,"<<steps_left<<","<<steps_right<<","<<speed_left<<","<<speed_right<<"\n";
	return command.str();
}

/**
 * Request status of @see SP() command.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGP() {
	return "SGP\n";
}

/**
 * Set selected EEProm parameter.
 * !! VALUES HAVE NO EFFECT !! (25.04.13)
 * @param param The parameter to set
 * @param value The new value for the parameter
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SN(int param, int value) {
	std::ostringstream command;
	command<<"SN,"<<param<<","<<value<<"\n";
	return command.str();
}

/**
 * Request value of selected parameter from EEProm.
 * !! VALUES HAVE NO EFFECT !! (25.04.13)
 * @param param The parameter to read
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGN(int param) {
	std::ostringstream command;
	command<<"SGN,"<<","<<param<<"\n";
	return command.str();
}

/**
 * DEBUG command: Sets a selected register of the controller.
 * Be careful with this command, you can potentially write to
 * EVERY register of the controller.
 *
 * @param address The address to write to
 * @param value The value to write
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SX(int address, int value) {
	std::ostringstream command;
	command<<"SX,"<<address<<","<<value<<"\n";
	return command.str();
}

/**
 * DEBUG command: Request value of selected controller address.
 * @param address The address to read from.
 * @return The command to be transmitted to the controller.
 */
std::string BbmRS232Commands::SGX(int address) {
	std::ostringstream command;
	command<<"SGX,"<<address<<"\n";
	return command.str();
}


