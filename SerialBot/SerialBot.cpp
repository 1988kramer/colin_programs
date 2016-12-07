// SerialBot.cpp
// by Andrew Kramer
// 11/28/2016

// issues commands to and receives sensor and odometry information from
// a robot controller using UART communication

// Sends command packets with the following format:
//   <(int)translational,(int)(angular * 1000)>

// Expects sensor packets with the following format:
//   <(int)sonar0,(int)sonar1,...(int)sonar7,(int)x,(int)y,(int)(theta * 1000)>

// ints are transmitted and received as their character representations
// this is a definite area for improvement 

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
	inPacketSize_ = 22;
	readPeriod_ = 250000;
	numSensors_ = 11;
	distances_ = new int16_t[numSensors_ - 3];
	commandPacketLength_ = 4;
	
	openSerial();
}

SerialBot::~SerialBot()
{
	delete[] distances_;
}

void SerialBot::getDistances(int *distances)
{
	for (int i = 0; i < numSensors_ - 3; i++)
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
	tcflush(serialFd_, TCIFLUSH);
	tcsetattr(serialFd_, TCSANOW, &options);
}

// transmits command packet to the robot controller
int SerialBot::transmit(char* commandPacket)
{
	int result = -1;
	if (serialFd_ != -1) 
	{
		result = write(serialFd_, commandPacket, commandPacketLength_);
	}
	return result;
}

// receives sensor update packet from the robot controller
int SerialBot::receive(byte* inPacket)
{
	memset(inPacket, '\0', inPacketSize_);
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
			rxBytes = read(serialFd_, inPacket, inPacketSize_);
		}
	}
	return rxBytes;
}

// builds a command packet from the commanded speeds
void SerialBot::makeCommandPacket(byte* commandPacket)
{
	int16_t intAngular = (int)(angular * 1000.0);
	commandPacket[0] = (byte)(translational_ & 0xFF);
	commandPacket[1] = (byte)((translational_ >> 8) & 0xFF);
	commandPacket[2] = (byte)(intAngular & 0xFF);
	commandPacket[3] = (byte)((intAngular >> 8) & 0xFF);
}

// parses a packet of sensor updates from the robot's controller
int SerialBot::parseSensorPacket(byte* inPacket)
{
	byte firstByte;
	byte secondByte;
	for (int i = 0; i < numSensors_ - 3; i++)
	{
		firstByte = inPacket[2 * i];
		secondByte = inPacket[(2 * i) + 1];
		inValues[i] = (secondByte << 8) | firstByte;
	}

	firstByte = inPacket[16];
	secondByte = inPacket[17];

	x_ = (secondByte << 8) | firstByte;

	firstByte = inPacket[18];
	secondByte = inPacket[19];

	y_ = (secondByte << 8) | firstByte;

	firstByte = inPacket[20];
	secondByte = inPacket[21];

	theta_ = ((double)(secondByte << 8) | firstByte) / 1000.0;
}

// handles communication with the robot
void SerialBot::commThreadFunction()
{
	while (true) 
	{
		byte commandPacket[commandPacketLength_];
		makeCommandPacket(commandPacket);
		if (transmit(commandPacket) < 1)
			cerr << "command packet transmission failed" << endl;
		char inPacket[inPacketSize_];
		int receiveResult = receive(inPacket);
		if(receiveResult < 1)
		{
			cerr << "sensor packet not received" << endl;
		}
		else
		{
			// cout << inPacket << endl; // for testing purposes
			parseSensorPacket(inPacket);
		}
		usleep(readPeriod_);
	}
}
