/*
 * IncrementParser.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: josef
 */


#include "IncrementParser.h"



IncrementParser::IncrementParser() {
	active_ = false;
	init_done_ = false;
	increments_left_ = -1;
	increments_right_ = -1;
	//do_write_ = false;
	max_buffer_size_ = 10;
	baudrate_ = 38400;
	write_sleep_us_ = 0;  //looks like a write delay is not needed after all
	write_timer_.Start();
	pthread_mutex_init(&command_buffer_mutex_, NULL);
}

IncrementParser::~IncrementParser() {
	Stop();
	pthread_mutex_destroy(&command_buffer_mutex_);
}
/**
 * The loop used by the thread.
 * Each cycle is one read and one write attempt.
 * @param obj
 */
void* IncrementParser::loop(void* obj) {
	while(reinterpret_cast<IncrementParser *>(obj)->active_){
		reinterpret_cast<IncrementParser *>(obj)->ReadFromRS232();
		reinterpret_cast<IncrementParser *>(obj)->WriteToRS232();
	}
}

/**
 * Creates a new thread to handle the communication with the µController.
 */
void IncrementParser::Start() {
	if(!init_done_)
		InitRS232();
	if(!active_){
		std::cout<<"Starting Thread."<<std::endl;
		active_ = true;
		pthread_create(&thread_, 0, &IncrementParser::loop, this);
	}
}

/**
 * Finds and reads the msg with the Odometry data.
 * The data will be written into increments_left and increments_right.
 * @param buf
 * @param f
 */
void IncrementParser::FindIncrements(unsigned char buf[], int f) {
	std::string wert_links = "";
	std::string wert_rechts = "";
	for(int i=0;i<f;i++)
	{
		if(!(i+5<f))
			break;
		if( (buf[i]==115) & (buf[i+1]==103) & (buf[i+2]==101) & (buf[i+3]==103)){//s
			i+=5;
			while(buf[i]!=44){
				wert_links += buf[i];
				//std::cout<<buf[i];
				i++;
			}
			i++;
			while(buf[i]!=10){
				wert_rechts += buf[i];
				//std::cout<<buf[i];
				i++;
			}
			increments_left_ = fromString<double>(wert_links);
			increments_right_ = fromString<double>(wert_rechts);
		}
	}
}

/**
 * Stops the thread loop.
 * It waits till the command_buffer_ is empty.
 * It also closes the ftdi_usb object.
 */
void IncrementParser::Stop() {
	while(!command_buffer_.empty())
		;
	if(active_){
		active_ = false;
		pthread_join(thread_, 0);
	}
	if(init_done_){
		init_done_ = false;
		ftdi_usb_close(ftdi_);
	}
}

/**
 * attempts to read from ftdi object.
 * If a string is read, it calls FindIncrements().
 */
void IncrementParser::ReadFromRS232() {
	int f;
	unsigned char buf[1024];
	f = ftdi_read_data(ftdi_, buf, sizeof(buf));
		if(f<0)
			fprintf(stderr,"size < 0\n");
		else if(f>0)
			FindIncrements(buf, f);
}

/**
 * Will initialize the ftdi connection.
 * @return 0 on success.
 */
int IncrementParser::InitRS232() {
	int ret  = 100;

	if ((ftdi_ = ftdi_new()) == 0){
		fprintf(stderr, "ftdi_new failed\n");
		return EXIT_FAILURE;
	}

	if ((ret = ftdi_usb_open(ftdi_, 0x0403, 0x6001)) < 0){
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi_));
		ftdi_free(ftdi_);
		return EXIT_FAILURE;
	}

	// Read out FTDIChip-ID of R type chips
	if (ftdi_->type == TYPE_R){
		unsigned int chipid;
		printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(ftdi_, &chipid));
		printf("FTDI chipid: %X\n", chipid);
	}
	ftdi_set_baudrate(ftdi_, baudrate_);
	printf("Baudrate set to: %d\n", baudrate_);
	init_done_ = true;
	return EXIT_SUCCESS;
}

int IncrementParser::getIncrementsLeft() {
	return increments_left_;
}

/**
 * Will try and initialize the ftdi connection.
 * If already running or initialized, it will stop the thread and release the ftdi
 * before it attempts to re initialize.
 */
void IncrementParser::Init() {
	if(active_ || init_done_){
		Stop();
		sleep(1);
	}
	InitRS232();
}

/**
 * Tries to put a command in the command buffer.
 * @param msg Command to be send.
 * @return true if successful / false if buffer full.
 */
bool IncrementParser::Send(std::string msg) {
	bool result;
	pthread_mutex_lock(&command_buffer_mutex_);
	if(command_buffer_.size() <= max_buffer_size_){
		command_buffer_.push(msg);
		result = true;
	}else
		result = false;
	pthread_mutex_unlock(&command_buffer_mutex_);
	if(!active_ && init_done_)
		WriteToRS232();
	return result;
}

/**
 * Attempts to write the current command to the ftdi object.
 * If successful the command is removed from the buffer, else it will remain
 * in the buffer.
 */
void IncrementParser::WriteToRS232() {
	pthread_mutex_lock(&command_buffer_mutex_);
	if(!command_buffer_.empty()){
		int ret = ftdi_write_data(ftdi_,(unsigned char*)command_buffer_.front().c_str(),command_buffer_.front().size());
		if(ret > 0)//no error; value is number of bytes written.
			command_buffer_.pop();
	}
	pthread_mutex_unlock(&command_buffer_mutex_);
}

int IncrementParser::getIncrementsRight() {
	return increments_right_;
}

