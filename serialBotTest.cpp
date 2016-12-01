// serialBotTest.cpp
// by Andrew Kramer
// 11/30/2016

// testing for the SerialBot class

#include "SerialBot.h"
#include <pthread.h>

using namespace std;

SerialBot colin;
pthread_t commThread;

void* threadFunction(void* args)
{
	colin.commThreadFunction();
}

// prompts user for new speeds and sets Colin's speeds accordingly
void enterSpeed()
{
	int translational;
	double angular;
	cout << "Enter translational speed: ";
	cin >> translational;
	cout << "Enter angular velocity: ";
	cin >> angular;
	cout << endl;
	colin.setSpeed(translational, angular);
}

// prints sonar distance readings
void getDistances()
{
	int sonarReadings[8];
	colin.getDistances(sonarReadings);
	cout << "Colin's sonar readings: " << endl;
	for (int i = 0; i < 8; i++)
		cout << "Sonar " << i << ": " << sonarReadings[i] << endl;
	cout << endl;
}

// prints odometry readings
void getPose()
{
	int x, y;
	double theta;
	colin.getPose(&x, &y, &theta);
	cout << "Colin's new pose:" << endl;
	cout << "      x = " << x << endl;
	cout << "      y = " << y << endl;
	cout << "  theta = " << theta << endl;
	cout << endl;
}

int main()
{
	pthread_create(&commThread, NULL, threadFunction, NULL);
}