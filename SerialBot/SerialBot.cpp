// SerialBot.cpp
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

#include "SerialBot.h"

SerialBot::SerialBot()
{
	// initialize member variables
	x_ = 0;
	y_ = 0;
	theta_ = 0.0;
	translational_ = 0;
	angular_ = 0.0;
	serialFd_ = -1;
	sensorPacketSize_ = (numSonar_ + numPoseVariables) * 2;
	readPeriod_ = 250000;
	numSonar_ = 8;
	distances_ = new int16_t[numSonar_];
	
	openSerial();
	resetController();
}

SerialBot::~SerialBot()
{
	delete[] distances_;
}

void SerialBot::getDistances(int *distances)
{
	for (int i = 0; i < numSonar_; i++)
		distances[i] = distances_[i];
}

void SerialBot::getPose(int *x, int *y, double *theta)
{
	*x = x_;
	*y = y_;
	*theta = theta_;
}

void SerialBot::setSpeed(int translational, double angular)
{
	translational_ = translational;
	angular_ = angular;
}

// opens serial connection with robot controller
void SerialBot::openSerial()
{
	serialFd_ = open("/dev/serial0", O_RDWR); // left out O_NOCTTY and O_NDELAY
																						// to allow blocking read
	if (serialFd_ == -1)
	{
		cerr << "Error - unable to open uart" << endl;
		exit(-1);
	}	
	
	struct termios options;
	tcgetattr(serialFd_, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(serialFd_, TCIOFLUSH);
	tcsetattr(serialFd_, TCSANOW, &options);
}

void SerialBot::resetController()
{
	pinMode(4, OUTPUT);
	digitalWrite(4, LOW);
	delay(50);
	digitalWrite(4, HIGH);
	delay(5000);
}

// transmits command packet to the robot controller
int SerialBot::transmit(char* commandPacket)
{
	int result = -1;
	if (serialFd_ != -1) 
	{
		result = write(serialFd_, commandPacket, commandPacketSize);
	}
	return result;
}

// receives sensor update packet from the robot controller
int SerialBot::receive(char* sensorPacket)
{
	memset(sensorPacket, '\0', sensorPacketSize_);
	int rxBytes;
	if (serialFd_ != -1)
	{
		// set up blocking read with timeout at .25 seconds
		fd_set set;
		FD_ZERO(&set); // clear the file descriptor set
		FD_SET(serialFd_, &set); // add serial file descriptor to the set
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 25000;
		
		// wait for serial to become available
		int selectResult = select(serialFd_ + 1, &set, NULL, NULL, &timeout);
		if (selectResult < 0)
		{
			cerr << "blocking read failed" << endl;
			return -1;
		}
		else if (selectResult == 0)
		{
			cerr << "read failed: timeout occurred" << endl;
			return 0;
		}
		else
		{
			rxBytes = read(serialFd_, sensorPacket, numSonar_ + numPoseVariables);
		}
	}
	return rxBytes;
}

// builds a command packet from the commanded speeds
void SerialBot::makeCommandPacket(char* commandPacket)
{
	int16_t intAngular = (int)(angular_ * 1000.0);
	commandPacket[0] = (char)(translational_ & 0xFF);
	commandPacket[1] = (char)((translational_ >> 8) & 0xFF);
	commandPacket[2] = (char)(intAngular & 0xFF);
	commandPacket[3] = (char)((intAngular >> 8) & 0xFF);
}

// parses a packet of sensor updates from the robot's controller
// updates distance array and pose
int SerialBot::parseSensorPacket(char* sensorPacket)
{
	int16_t firstByte;
	int16_t secondByte;
	int16_t inValues[numSonar_ + numPoseVariables];
	for (int i = 0; i < numSonar_ + numPoseVariables; i++)
	{
		firstByte = sensorPacket[2 * i];
		secondByte = sensorPacket[(2 * i) + 1];
		inValues[i] = (secondByte << 8) | firstByte;
	}

	for (int i = 0; i < numSonar_; i++)
	{
		distances_[i] = inValues[i];
	}
	
	x_ = inValues[8];
	y_ = inValues[9];
	theta_ = ((double)inValues[10]) / 1000.0;
}

// handles communication with the robot
// needs to be run in a separate thread
void SerialBot::commThreadFunction()
{
	while (true) 
	{
		char commandPacket[commandPacketSize];
		makeCommandPacket(commandPacket);
		if (transmit(commandPacket) < 1)
			cerr << "command packet transmission failed" << endl;
		char sensorPacket[sensorPacketSize_];
		memset(sensorPacket, '\0', sensorPacketSize_);
		int receiveResult = receive(sensorPacket);
		if (receiveResult < 1)
		{
			cerr << "sensor packet not received" << endl;
		}
		else if (receiveResult < commandPacketSize)
		{
			cerr << "incomplete sensor packet received" << endl;
		}
		else
		{
			//cout << sensorPacket << endl; // for testing purposes
			/*
			for (int i = 0; i < sensorPacketSize_; i++)
			{
				printf("%d ", (int)sensorPacket[i]);
			}
			cout << endl;
			*/
			parseSensorPacket(sensorPacket);
		}
		usleep(readPeriod_);
	}
}
