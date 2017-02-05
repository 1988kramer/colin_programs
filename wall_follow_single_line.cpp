// wall_follow_single_line.cpp
// by Andrew Kramer
// 12/13/2016

// Wall following program for Colin the robot
// Models walls as a single line constructed using weighted linear regression
// on obstacle locations in Colin's local coordinate system measured with 
// sonar sensors

// local coordinate system is defined as follows:
//    x axis: forward-aft with forward positive
//    y axis: left-right with left positive

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
double setPoint = 30; // initial set point for following distance
double kE = 0.0; // gain for error in following distance
double kS = 0.1; // gain for slope of wall relative to Colin's path

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

// accepts two doubles as parameters:
//    the slope of the line representing the wall
//    the y intercept of the line representing the wall
// returns the error between the distance between the robot's location (the origin)
// and the given line and the set point
// positive error indicates the robot is between the set point and the wall
// negative error indicates the set point is between the robot and the wall
double getDistanceToSetPoint(double slope, double intercept)
{
	if (slope == 0.0) // if the line is perfectly horizontal
		return abs(intercept) - setPoint;
	double xIntercept = (-1.0 * intercept) / ((1.0 / slope) + slope);
	double yIntercept = (slope * xIntercept) + intercept;
	double distance = sqrt(pow(xIntercept, 2.0) + pow(yIntercept, 2.0));
	double error = distance - setPoint;
	if (intercept < 0)
		error *= -1;
	return error;
}

// accepts one double as a parameter:
//    the slope of the line representing the wall
// returns velocity of the set point in the robot's local coordinate system
// velocity is positive if the set point is moving to the left
// velocity is negative if set point is moving to the right
double getVelocityOfSetPoint(double slope) 
{
	if (slope == 0.0)
		return 0.0;
	// calculate speedToSetPoint
	double angleOfLine = atan(slope);
	double speedToSetPoint = (double)translational * sin(angleOfLine);
	// alternate method, doesn't use trig functions:
	//double speedToSetPoint = sqrt((pow((double)translational, 2) * pow(slope, 2))/(1 + pow(slope, 2)));
	if (slope < 0.0)
		speedToSetPoint *= -1.0;
	return speedToSetPoint;
}

// sets colin's speed set points
// limits the angular and translational velocities to the max angular velocity
// but preserves the commanded radius of travel
void setSpeed()
{
	if (abs(angular) > maxAng)
	{
		double radius = (double)translational / abs(angular);
		int adjustedTrans = radius * maxAng;
		double adjustedAng = (angular > 0.0)? maxAng : maxAng * -1.0;
		colin.setSpeed(adjustedTrans, adjustedAng);
	}
	else
	{
		colin.setSpeed(translational, angular);	
	}
}

void* wallFollowFunction(void* args)
{
	while(true)
	{
		if (translational != 0)
		{
			colin.getDistances(distances);
			updatePoints();
			line.setPoints(points);
			line.updateLine();
			double slope = line.getM();
			double intercept = line.getB();
			printf("y=%.2fx + %.2f\n", slope, intercept);
			double error = getDistanceToSetPoint(slope, intercept);
			double dError = getVelocityOfSetPoint(slope);
			double eTerm = kE * error;
			double sTerm = kS * dError;
			angular = eTerm + sTerm;
		}
		else
		{
			angular = 0.0;
		}
		setSpeed();
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
		int newTrans;
		cin >> newTrans;
		cout << endl;
		if (abs(newTrans) > maxTrans)
		{
			translational = (newTrans > 0)? maxTrans : maxTrans * -1;
		}
		else
		{
			translational = newTrans;
		}
	}
	return 1;
}
