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
	inPacketSize_ = 100;
	readPeriod_ = 250000;
	numSensors_ = 11;
	distances_ = new int[numSensors_ - 3];
	
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
		result = write(serialFd_, commandPacket, strlen(commandPacket));
	}
	return result;
}

// receives sensor update packet from the robot controller
int SerialBot::receive(char* inPacket)
{
	memset(inPacket, '\0', inPacketSize_);
	int rxBytes;
	if (serialFd_ != -1)
	{

		int packetIndex = 0; // current index in inPacket
		bool started = false; // start-of-packet character has been encountered
		bool ended = false; // end-of-packet character has been encountered
		
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
void SerialBot::makeCommandPacket(char* commandPacket)
{
	memset(commandPacket, '\0', 32);
	strncat(commandPacket, &SOP, 1);
	char buffer[10];
	memset(buffer, '\0', 10);
	sprintf(buffer, "%d", translational_);
	strcat(commandPacket, (const char*)buffer);
	strncat(commandPacket, &DEL, 1);
	memset(buffer, '\0', 10);
	sprintf(buffer, "%d", (int)(angular_ * 1000.0));
	strcat(commandPacket, (const char*)buffer);
	strncat(commandPacket, &EOP, 1);
}

// parses a packet of sensor updates from the robot's controller
int SerialBot::parseSensorPacket(char* inPacket)
{
	bool started = false; // start-of-packet character has been encountered
	bool ended = false; // end-of-packet character has been encountered
	int inPacketIndex = 0; // current index in inPacket
	int bufSize = 10;
	char buffer[bufSize];
	memset(buffer, '\0', bufSize);
	int bufferIndex = 0; // current index in the buffer
	int inValues[numSensors_];
	int valueIndex = 0; // current index for the inValues
	
	// advance until start-of-packet character is found
	while (!started && inPacketIndex < inPacketSize_)
	{
		if (inPacket[inPacketIndex] == SOP)
			started = true;
		inPacketIndex++;
	}
	// parse packet until end-of-packet character is found
	while (!ended && inPacketIndex < inPacketSize_
					&& valueIndex < numSensors_)
	{
		if (inPacket[inPacketIndex] == DEL)
		{
			bufferIndex = 0;
			inValues[valueIndex] = atoi(buffer);
			memset(buffer, '\0', bufSize);
			valueIndex++;
		}
		else if (inPacket[inPacketIndex] == EOP)
		{
			ended = true;
			inValues[valueIndex] = atoi(buffer);
		}
		else
		{
			buffer[bufferIndex] = inPacket[inPacketIndex];
			bufferIndex++;
		}
		inPacketIndex++;
	}
	if (started && ended)
	{
		// update sonar distances
		for (int i = 0; i < numSensors_ - 3; i++) 
			distances_[i] = inValues[i];
		
		// update pose from odometry
		x_ = inValues[numSensors_ - 3];
		y_ = inValues[numSensors_ - 2];
		theta_ = ((double)inValues[numSensors_ - 1]) / 1000.0;
		return 1;
	}
	else if (!started)
	{
		cerr << "Bad packet: start-of-packet character not found" << endl;
		return -1;
	}
	else
	{
		cerr << "Bad packet: end-of-packet character not found" << endl;
		return -1;
	}
}

// handles communication with the robot
void SerialBot::commThreadFunction()
{
	while (true) 
	{
		char commandPacket[32];
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
