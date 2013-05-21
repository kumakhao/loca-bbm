/*
 * msTimer.cpp
 *
 *  Created on: May 14, 2013
 *      Author: josef
 */



#include "util/msTimer.h"
#include <stdlib.h>

/**
 * Resets the start and step time marks.
 */
void msTimer::Start() {
	gettimeofday(&stTimeVal, NULL);
	start_time_ =  stTimeVal.tv_sec * 1000000ll + stTimeVal.tv_usec;
	step_time_ = start_time_;
}

/**
 * Returns the time passed since @see Start() was called in µs.
 */
long long msTimer::getTimeFromStart() {
	gettimeofday(&stTimeVal, NULL);
	return (stTimeVal.tv_sec * 1000000ll + stTimeVal.tv_usec) - start_time_;
}

/**
 * Returns time passed since last call of @see getTimeStep() in µs.
 */
long long msTimer::getTimeStep() {
	gettimeofday(&stTimeVal, NULL);
	long long time_holder = (stTimeVal.tv_sec * 1000000ll + stTimeVal.tv_usec) - step_time_;
	step_time_ = stTimeVal.tv_sec * 1000000ll + stTimeVal.tv_usec;
	return time_holder;
}
