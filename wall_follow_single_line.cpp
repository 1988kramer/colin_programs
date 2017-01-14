// wall_follow_single_line.cpp
// by Andrew Kramer
// 12/13/2016

// Wall following program for Colin the robot
// Models walls as a single line constructed using weighted linear regression
// on obstacle locations in Colin's local coordinate system measured with 
// sonar sensors

#include "SerialBot/SerialBot.h"
#include "LineFitter/LineFitter.h"
#include "LineFitter/Point.h"
#include <pthread.h>
#include <cmath>

using namespace std;

SerialBot colin;

const double pi = 3.14159265;
const int numSonar = 8;
const int maxTrans = 200; // max translational speed
const double maxAng = 2.0; // max angular velocity
const double sensorAngles[] = {0.0, 7.0*pi/4.0, 3.0*pi/2.0, 5.0*pi/4.0, pi, 3.0*pi/4.0, pi/2.0, pi/4.0};
int distances[numSonar];
Point points[numSonar];
LineFitter line(points, numSonar);
double setPoint = 10; // initial set point for following distance
double kE = 0.2; // gain for error in following distance
double kS = 0.01; // gain for slope of wall relative to Colin's path

int translational = 0;
double angular = 0.0;

void* commFunction(void* args)
{
	colin.commThreadFunction();
}

void updatePoints()
{
	for (int i = 0; i < numSonar; i++)
		points[i].setCoordinates(distances[i], sensorAngles[i]);
}

// accepts two doubles as parameters
//  the slope of the line
//  the y intercept of the line
// returns the minimum distance between the robot's location (the origin)
// and the given line
double distanceToLine(double slope, double intercept)
{
	if (slope == 0.0) // if the line is perfectly horizontal
		return abs(intercept);
	double xIntercept = intercept / (-(1.0 / slope) - slope);
	double yIntercept = intercept / (1.0 + pow(slope, 2.0));
	double distance = sqrt(pow(xIntercept, 2.0) + pow(yIntercept, 2.0));
	return distance;
}

void* wallFollowFunction(void* args)
{
	while(true)
	{
		if (translational > 0)
		{
			colin.getDistances(distances);
			updatePoints();
			line.setPoints(points);
			line.updateLine();
			double slope = line.getM();
			double intercept = line.getB();
			double distance = distanceToLine(slope, intercept);
			double error = distance - setPoint;
			double eTerm = kE * error;
			double sTerm = kS * slope * (double)translational;
			if (intercept < 0.0) eTerm *= -1; // flip sign of eTerm if wall is on Colin's right
			angular = eTerm + sTerm;
			if (angular > maxAng)
			{
				// need to do something or other here
			}
			if (angular < maxAng * -1.0)
			{
				// also need to do something here
			}
			colin.setSpeed(translational, angular);
		}
		usleep(500000); // repeat every 0.5 seconds
	}
}

int main()
{
	pthread_t commThread;
	pthread_t lineFollowThread;
	pthread_create(&commThread, NULL, commFunction, NULL);
	pthread_create(&lineFollowThread, NULL, wallFollowFunction, NULL);
	while (true)
	{
		cout << "Enter translational speed: ";
		cin >> translational;
		cout << endl;
		if (translational > maxTrans) translational = maxTrans;
		if (translational < -1 * maxTrans) translational = -1 * maxTrans;
	}
	return 1;
}