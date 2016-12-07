// SerialBot.h
// by Andrew Kramer
// 11/28/2016

// issues commands to and receives sensor and odometry information from
// a robot controller using UART communication

#ifndef SerialBot_h
#define SerialBot_h

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <string.h>
#include <pthread.h>

using namespace std;

const char DEL = ',';
const char SOP = '<';
const char EOP = '>';

class SerialBot
{
public:
	SerialBot();
	~SerialBot();
	void setSpeed(int translational, double angular); 
	void getDistances(int* distances); // copies values in distances_ to distances
	void getPose(int* x, int* y, double* theta); // copies values in x_, y_, and theta_ to x, y, and theta
	void commThreadFunction();
private:
	int x_, y_; // robot's x and y coordinates
	double theta_; // robot's heading in radians
	int16_t translational_; // commanded translational speed in cm/s
	double angular_; // commanded angular velocity in rad/s
	int16_t* distances_; // array of distance readings from sonar sensors
	pthread_t commThread_;
	int serialFd_; // file descriptor for serial connection
	int readPeriod_; // delay between updates in microseconds
	int inPacketSize_; // default size for sensor update packet
	int numSensors_; // number of values in the sensor packets
	int commandPacketSize_;
	
	void openSerial(); // opens serial connection with robot controller
	int transmit(char* commandPacket); // transmits command packet to robot 
	                                   //controller
	int receive(char* inPacket); // receives sensor update packet
												// from robot controller
	void makeCommandPacket(char* commandPacket); // builds a command packet from the commanded speeds
	int parseSensorPacket(char* sensorPacket); // parses a packet of sensor
																// updates from the robot																											 
};


#endif
