/*
 * msTimer.h
 *
 *  Created on: May 14, 2013
 *      Author: josef
 */

#ifndef MSTIMER_H_
#define MSTIMER_H_

#include <sys/time.h>

class msTimer{
public:
	long long start_time_;
	long long last_step_time_;
	long long last_absolut_time_step_;
	struct timeval stTimeVal;
	void Start();
	long long getTimeFromStart();
	long long getTimeStep();
};


#endif /* MSTIMER_H_ */
