//============================================================================
// Name        : expoBot_test.cpp
// Author      : Josef
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "IncrementParser.h"
#include <sys/time.h>
#include "util/msTimer.h"
#include "util/BbmRS232Commands.h"
#include "util/SIXAXIS_Interface.h"
#include "util/cjoystick.h"



int expoBot_test() {
	IncrementParser testParser;
	BbmRS232Commands bbmc;
	cJoystick sixaxes;

	testParser.Start();
	//testParser.Init();
	testParser.Send(bbmc.SX(105,32)); // set periodic messages to raw data only
	testParser.Send(bbmc.SX(104,10)); // set interval of messages to 100ms
	testParser.Send(bbmc.SM(1));

	while(!sixaxes.buttonPressed(BUTTON_CIRCLE)){
		if(sixaxes.buttonPressed(BUTTON_L1) && sixaxes.buttonPressed(BUTTON_R1))
			testParser.Send(bbmc.SS(sixaxes.axis(AXIS_RX)*100,sixaxes.axis(AXIS_LX)*100));
		else
			testParser.Send(bbmc.SS0());
		std::cout<<"Left: "<<testParser.getIncrementsLeft()<<"  Right: "<<testParser.getIncrementsRight()<<std::endl;
		usleep(35000);
	}

	while(!testParser.Send(bbmc.SM(0)))
		std::cout<<"######_NoStop_######"<<std::endl;

	testParser.Stop();
	return 0;
}
