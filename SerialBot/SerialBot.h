// SerialBot.h
// by Andrew Kramer
// 11/28/2016

// Class handles serial communication with differential drive robot (Colin)
// Allows for control if the robot is running colin_controller.ino

// COMMAND PACKETS
// Sends command packets to Colin's ATmega328 via serial
// Command packets contain two 16 bit ints representing the 
// commanded translational speed and angular velocity
// The two ints should be broken into bytes, least significant byte (LSB) first
// To avoid problems with float representations, angular velocity is 
// expected to be multiplied by 1000 and casted to an int before sending
// Command packet format is as follows:
//         byte 0         |        byte 1       |       byte 2         |       byte 3
//    translational (LSB) | translational (MSB) | angular * 1000 (LSB) | angular * 1000 (MSB)

// SENSOR PACKETS
// After sending a command, controller expects Colin to respond with 
// a packet containing its sonar sensor readings and pose in response.
// Sensor packets will contain numSonar + numPoseVariables 16 bit ints 
// broken into bytes, least significant byte (LSB) first
// Indices 0 through numSonar * 2 - 1 of the packet will contain sonar 
// distance readings
// Indices numSonar * 2 through numSonar * 2 + 5 will contain the robot's 
// x position in cm, y position in cm, and heading in radians
// To avoid problems with float representation, heading is multiplied by 
// 1000 and casted to an int before sending
// Sensor packet format is as follows:
//     byte 0       |     byte 1      |    byte 2      | ... |  byte NUM_SONAR * 2   | byte NUM_SONAR * 2 + 1 | byte NUM_SONAR * 2 + 2 | byte NUM_SONAR * 2 + 3 | byte NUM_SONAR * 2 + 4 | byte NUM_SONAR * 2 + 5
//   sonar 0 (LSB)  |  sonar 0 (MSB)  |  sonar 1 (LSB) | ... |   x position (LSB)    |    x position (MSB)    |    y position (LSB)    |    y position (MSB)    |  heading * 1000 (LSB)  |  heading * 1000 (MSB)  

// commThreadFunction should be run in a separate thread
// It will send a command and receive an update 4 times per second

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
#include <wiringPi.h>

using namespace std;

const int commandPacketSize = 4;
const int numPoseVariables = 3;

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
	int serialFd_; // file descriptor for serial connection
	int readPeriod_; // delay between updates in microseconds
	int sensorPacketSize_; // default size for sensor update packet
	int numSonar_; // number of sonar sensors
	int commandPacketSize_;
	
	void openSerial(); // opens serial connection with robot controller
	void resetController(); // resets the robot controller using gpio
	int transmit(char* commandPacket); // transmits command packet to robot 
	                                   //controller
	int receive(char* sensorPacket); // receives sensor update packet
												// from robot controller
	void makeCommandPacket(char* commandPacket); // builds a command packet from the commanded speeds
	int parseSensorPacket(char* sensorPacket); // parses a packet of sensor
																// updates from the robot																											 
};


#endif
