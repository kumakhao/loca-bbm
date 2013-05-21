/*
 * BbmRS232Commands.h
 *
 *  Created on: Apr 25, 2013
 *      Author: josef
 */

#ifndef BBMRS232COMMANDS_H_
#define BBMRS232COMMANDS_H_

#include <string>

class BbmRS232Commands{
private:
protected:
public:

	std::string SM(int mode);
	std::string SGM();
	std::string SS(int left, int right);
	std::string SS0();
	std::string SGS();
	std::string SGEG();
	std::string SC();
	std::string SGEA();
	std::string SGER();
	std::string SE(int left, int right);
	std::string SGB();
	std::string SP(int steps_left, int steps_right, int speed_left, int speed_right);
	std::string SGP();
	std::string SN(int param, int value);
	std::string SGN(int param);
	std::string SX(int address, int value);
	std::string SGX(int address);
};


#endif /* BBMRS232COMMANDS_H_ */
