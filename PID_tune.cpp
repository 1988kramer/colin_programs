// PID_tune.cpp
// by Andrew Kramer
// 12/16/2016

// Assists in manual PID tuning of Colin the Robot
// Must be used with PID_tune.ino running on Colin's ATmega328

#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

const int commandPacketLength = 12;
const char ACK = 'a';
int serialFD;

// opens serial connection 
void openSerial()
{
	serialFD = -1;

	serialFD = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (serialFD == -1)
	{
		cerr << "Error - unable to open UART" << endl;
		exit(-1);
	}

	struct termios options;
	tcgetattr(serialFD, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(serialFD, TCIFLUSH);
	tcsetattr(serialFD, TCSANOW, &options);
}

int transmit(char* commandPacket)
{
	int result = -1;
	if (serialFD != -1)
	{
		result = write(serialFD, commandPacket, commandPacketLength);
	}
	return result;
}

void makeCommandPacket(char* commandPacket, int16_t speed, int16_t time, 
																		double angular, double kP, double kI, double kD)
{
	commandPacket[0] = (char)(speed & 0xFF);
	commandPacket[1] = (char)((speed >> 8) & 0xFF);
	int16_t intAngular = (int16_t)(angular * 10000.0);
	commandPacket[2] = (char)(intAngular & 0xFF);
	commandPacket[3] = (char)((intAngular >> 8) & 0xFF);
	commandPacket[4] = (char)(time & 0xFF);
	commandPacket[5] = (char)((time >> 8) & 0xFF);
	int16_t intKP = (int16_t)(kP * 10000.0);
	commandPacket[6] = (char)(intKP & 0xFF);
	commandPacket[7] = (char)((intKP >> 8) & 0xFF);
	int16_t intKI = (int16_t)(kI * 10000.0);
	commandPacket[8] = (char)(intKI & 0xFF);
	commandPacket[9] = (char)((intKI >> 8) & 0xFF);
	int16_t intKD = (int16_t)(kD * 10000.0);
	commandPacket[10] = (char)(intKD & 0xFF);
	commandPacket[11] = (char)((intKD >> 8) & 0xFF);
}

int receive()
{
	if (serialFD != 1)
	{
		fd_set set;
		FD_ZERO(&set);
		FD_SET(serialFD, &set);
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 50000; // timeout after 50ms
		
		int selectResult = select(serialFD + 1, &set, NULL, NULL, &timeout);
		if (selectResult < 0)
		{
			return -1;
		}
		else if (selectResult == 0)
		{
			return 0;
		}
		else
		{
			char inChar;
			read(serialFD, &inChar, 1);
			if (inChar == ACK) return 1;
			else return -2;
		}
	}
}

void getUserParams(int16_t& speed, int16_t& time, double& angular,
																	double& kP, double& kI, double& kD)
{
	cout << "Enter speed in cm/s: ";
	cin >> speed;
	cout << "Enter angular velocity in rad/s: ";
	cin >> angular;
	cout << "Enter time in ms: ";
	cin >> time;
	cout << "Enter kP: ";
	cin >> kP;
	cout << "Enter kI: ";
	cin >> kI;
	cout << "Enter kD: ";
	cin >> kD;
	cout << endl;
}

int main()
{
	openSerial();
	while(true)
	{
		int16_t speed, time;
		double angular, kP, kI, kD;
		getUserParams(speed, time, angular, kP, kI, kD);
		char commandPacket[commandPacketLength];
		makeCommandPacket(commandPacket, speed, time, angular, kP, kI, kD);
		transmit(commandPacket);
		int ack = receive();
		if (ack == 1) cerr << "Command acknowledged" << endl;
		else if (ack == 0) cerr << "timeout occurred on receive" << endl;
		else if (ack == -1) cerr << "select attempt failed" << endl;
		else if (ack == -2) cerr << "incorrect ACK received" << endl;
	}
}