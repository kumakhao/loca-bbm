/*
 * IncrementParser.h
 *
 *  Created on: Apr 12, 2013
 *      Author: josef
 */

#ifndef INCREMENTPARSER_H_
#define INCREMENTPARSER_H_

#include <stdio.h>
#include <stdlib.h>
#include <ftdi.h>
#include <iostream>
#include <sstream>
#include <queue>
#include <pthread.h>
#include "../util/msTimer.h"

class IncrementParser{
private:
	template<class T> std::string toString(const T& t);
	template<class T> T fromString(const std::string& s);
	void ReadFromRS232();
	void WriteToRS232();
	int InitRS232();
	void FindIncrements(unsigned char buf[],int f);

	msTimer write_timer_;
	struct ftdi_context *ftdi_;
	pthread_t thread_;
	bool init_done_;
	bool active_;

	int baudrate_;
	int write_sleep_us_;

	int increments_left_;
	int increments_right_;
	std::queue<std::string> command_buffer_;
	unsigned int max_buffer_size_;
	pthread_mutex_t command_buffer_mutex_;
protected:
public:
	IncrementParser();
	~IncrementParser();
	static void* loop(void* obj);
	void Start();
	void Stop();
	void Init();
	bool Send(std::string msg);
	int getIncrementsLeft();
	int getIncrementsRight();

};

template<class T>
inline std::string IncrementParser::toString(const T& t) {
    std::ostringstream stream;
    stream << t;
    return stream.str();
}

template<class T>
inline T IncrementParser::fromString(const std::string& s) {
    std::istringstream stream (s);
    T t;
    stream >> t;
    return t;
}



#endif /* INCREMENTPARSER_H_ */
