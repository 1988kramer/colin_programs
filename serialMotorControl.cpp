// serialMotorControl.cpp
// allows for control of Colin the robot via serial communication


#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

int serial, x, y;
double theta;
const char SOP = '<'; // start-of-packet character
const char EOP = '>'; // end-of-packet character
const char DEL = ','; // delimiter character

// opens serial connection 
void openSerial()
{
	serial = -1;

	serial = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial == -1)
	{
		cerr << "Error - unable to open UART" << endl;
		exit(-1);
	}

	struct termios options;
	tcgetattr(serial, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(serial, TCIFLUSH);
	tcsetattr(serial, TCSANOW, &options);
}

// transmits a string to the serial connection
// accepts a string, the data to be transmitted
void transmit(string tx_string)
{
	if (serial != -1)
	{
		char *tx_buffer = new char[tx_string.length() + 1];
		tx_buffer[tx_string.length()] = 0;
		strcpy(tx_buffer, tx_string.c_str());
		int tx_length = write(serial, &tx_buffer[0], tx_string.length() + 1);
		if (tx_length < 0)
		{
			cerr << "UART transmission error" << endl;
		}
		delete[] tx_buffer;
	}
}

// reads data from the serial connection if available
// returns an int, the number of bytes received
// accepts a reference to a string that will be used to contain the received data
// any data previously in the string is overwritten
int receive(char* inPacket, int packetSize)
{
	int result = -1;
	if (serial != -1) 
	{
		char inChar = '\0';
		char *c = &inChar;
		int packetIndex = 0; // current index inPacket
		bool started = false; // start-of-packet character has been encountered
		bool ended = false; // end-of-packet character has been encountered
		int rxBytes = 0;
		while (!started)
		{
			rxBytes = read(serial, (void*)c, 1);
			if (rxBytes > 0 && inChar == '<')
				started = true;
		}
		while (!ended)
		{
			rxBytes = read(serial, (void*)c, 1);
			if (rxBytes > 0)
			{
				if (inChar == '>')
				{
					ended = true;
				}
				else
				{
					inPacket[packetIndex] = inChar;
					packetIndex++;
				}
			}
		}
		if (started && ended)
			return 1;
		else if (!started)
			cerr << "Start-of-packet character not found" << endl;
		else if (!ended)
			cerr << "End-of-packet character not found" << endl;
	}
	return -1;
}

// assembles a motion command packet from user-specified values
// returns a string, the command packet
// accepts a string, the translational velocity formatted as an int
// a string, the rotational velocity formatted as a decimal
// and a string, the time the robot should execute the given motion formatted as an int
string assembleMotionPacket(string translational, string angular, string time)
{
	string packet = SOP + translational + DEL + angular + DEL + time + EOP;
	return packet;
}

void deleteBuffer(char** buffer)
{
	for (int i = 0; i < 3; i++) delete[] buffer[i];
		delete[] buffer;
}

// reads a pose packet and updates the robot's pose
// accepts a string, the pose packet
int readPosePacket(char* inPacket, int packetSize)
{
	int curValue = 0; // index of value currently being read
	int packetIndex = 0; // index in traversal of packet
	int startIndex = 0; // start index of current value in the packet
	// array of 3 char arrays for the x, y, and theta values
	char** buffer;
	buffer = new char*[3];
	for (int i = 0; i < 3; i++)
		buffer[i] = new char[packetSize]; 
	while (inPacket[packetIndex] != '\0')
	{
		if (isdigit(inPacket[packetIndex]) 
			|| inPacket[packetIndex] == '-'
			|| (curValue == 2 && inPacket[packetIndex] == '.'))
		{
			buffer[curValue][packetIndex - startIndex] 
							= inPacket[packetIndex];
		}
		else if (inPacket[packetIndex] == DEL)
		{
			curValue++;
			startIndex = packetIndex + 1;
		}
		else
		{
			cerr << "Bad packet: contains non-numeric character" << endl;
			deleteBuffer(buffer);
			return -1;
		}
		packetIndex++;
	}
	x = atoi(buffer[0]);
	y = atoi(buffer[1]);
	theta = atof(buffer[2]);
	return 1;
}

int main()
{
	x = 0;
	y = 0;
	theta = 0.0;
	openSerial();
	string translational, angular, time;
	while (true)
	{
		cout << "Enter translational velocity: ";
		cin >> translational;
		cout << "Enter angular velocity: ";
		cin >> angular;
		cout << "Enter time: ";
		cin >> time;
		string motionPacket = assembleMotionPacket(translational, angular, time);
		transmit(motionPacket);
		char posePacket[32];
		memset(posePacket, '\0', 32);
		const char* t = time.c_str();
		usleep(atoi(t) * 1000);
		receive(posePacket, 32);
		if (readPosePacket(posePacket, 32) > 0)
		{
			cout << "New pose (x, y, theta): (";
			cout << x << ", " << y << ", " << theta << ")" << endl << endl;
		}
	}
}
