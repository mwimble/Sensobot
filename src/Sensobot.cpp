//============================================================================
// Name        : Sensobot.cpp
// Author      : wimble
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "Sensors.h"
using namespace std;

int main() {
	cout << "!!!Hello Wimble!!!" << endl; // prints !!!Hello Wimble!!!
	PositionSensors::Sensors sensors = PositionSensors::Sensors();
	printf("---- getAccelerometerBlock ----\n");
	sensors.getAccelerometerBlock();
	printf("---- dumpAccelerometerSensorBlock ----\n");
	sensors.dumpAccelerometerSensorBlock();
	//usleep(67000);
	for (int compassTries = 0; compassTries < 400; compassTries++) {
		printf("---- getCompassBlock ----\n");
		sensors.getCompassBlock();
		printf("---- dumpCompassSensorBlock ----\n");
		sensors.dumpCompassSensorBlock();
//		printf("\nheading: %f\n", sensors.getCompassHeading());
		sleep(1);
	}

	return 0;
}


