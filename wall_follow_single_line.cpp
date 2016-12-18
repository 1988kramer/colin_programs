// wall_follow_single_line.cpp
// by Andrew Kramer
// 12/13/2016

// Wall following program for Colin the robot
// Models walls as a single line constructed using weighted linear regression
// on obstacle locations in Colin's local coordinate system measured with 
// sonar sensors

#include "SerialBot.h"
#include <pthread.h>
#include <math.h>

using namespace std;

SerialBot colin;

const double pi = 3.14159265;
const int numSonar = 8;
const int maxTrans = 200; // max translational speed
const double maxAng = 2.0; // max angular velocity
const double sensorAngles[] = {0.0, 7.0*pi/4.0, 3.0*pi/2.0, 5.0*pi/4.0, pi, 3.0*pi/4.0, pi/2.0, pi/4.0};
int distances[numSonar];
double setPoint = 10; // initial set point for following distance
double kE = 0.2; // gain for error in following distance
double kS = 0.01; // gain for slope of wall relative to Colin's path
int translational = 0;
double angular = 0.0;

void* commFunction(void* args)
{
	colin.commThreadFunction();
}

void* wallFollowFunction(void* args)
{
	while(true)
	{
		if (translational > 0)
		{
			colin.getDistances(distances);
			double wMatrix[numSonar];
			findWeights(wMatrix);
			double aMatrix[numSonar][2];
			makeAMatrix(aMatrix);
			double bMatrix[numSonar];
			makeBMatrix(bMatrix);
			double x[2]; 
			findCoefficients(aMatrix, bMatrix, wMatrix, x);
			double distance = distanceToLine(x[1], x[0]);
			double error = distance - setPoint;
			double eTerm = kE * error;
			double sTerm = kS * x[1] * (double)translational;
			if (x[0] < 0.0) eTerm *= -1; // flip sign of eTerm if wall is on Colin's right
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

// assumes weights matrix dimensions are numSonar * 1
// coefficients must be adjusted to provide reasonable weighting 
void findWeights(double* wMatrix)
{
	for (int i = 0; i < numSonar; i++)
	{
		wMatrix[i] = exp(-1.0 * pow((double)distances[i], 2.0) / 1.0);
	}
}

// expects a numSonar x 2 matrix as a parameter
// fills matrix with t values for linear regression
void makeAMatrix(double** aMatrix)
{
	for (int i = 0; i < numSonar; i++)
	{
		aMatrix[i][0] = 1.0;
		aMatrix[i][1] = (double)distances[i] * cos(sensorAngles[i]);
	}
}

// expects a numSonar x 1 matrix as a parameter
// fills matrix with b values for linear regression
void makeBMatrix(double* bMatrix)
{
	for (int i = 0; i < numSonar; i++)
	{
		bMatrix[i] = (double)distances[i] * sin(sensorAngles[i]);
	}
}

void findCoefficients(double** aMatrix, double* bMatrix, double* wMatrix, double* x)
{
	// A(transpose) * W
	double Atrans_W[2][numSonar];
	for (int i = 0; i < numSonar; i++)
	{
		Atrans_W[0][i] = aMatrix[i][0] * wMatrix[i];
		Atrans_W[1][i] = aMatrix[i][1] * wMatrix[i];
	}
	// find A(transpose) * W * A
	double Atrans_W_A[2][2];
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < numSonar; k++)
				Atrans_W_A[i][j] += Atrans_W[i][k] * aMatrix[k][j];
		}
	}
	// find det(A(transpose) * W * A)
	double determinant = Atrans_W_A[0][0] * Atrans_W_A[1][1];
	determinant -= Atrans_W_A[1][0] * Atrans_W_A[0][1];
	// find inverse(A(transpose) * W * A
	double inv_At_W_A[2][2];
	inv_At_W_A[0][0] = Atrans_W_A[1][1];
	inv_At_W_A[1][1] = Atrans_W_A[0][0];
	inv_At_W_A[0][1] = -Atrans_W_A[0][1];
	inv_At_W_A[1][0] = -Atrans_W_A[1][0];
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
			inv_At_W_A[i][j] /= determinant;
	}
	// find A(transpose) * W * b
	double At_W_b[2];
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < numSonar; j++)
			At_W_b[i] += Atrans_W[i][j] * bMatrix[j];
	}
	// find x where x[0] = c and x[1] = d in b = dt + c
	for (int i = 0; i < 2; i++)
	{
		x[i] = 0.0; // make sure x vector is initially empty
		for (int j = 0; j < 2; j++)
			x[i] += inv_At_W_A[i][j] * At_W_b[j];
	}
}

// accepts two doubles as parameters
// m - the slope of the line
// b - the y intercept of the line
// returns the minimum distance between the robot's location (the origin)
// and the given line
double distanceToLine(double m, double b)
{
	if (m == 0.0) // if the line is perfectly horizontal
		return abs(b);
	double xIntercept = b / (-(1.0 / m) - m);
	double yIntercept = b / (1.0 + pow(m, 2.0));
	double distance = sqrt(pow(xIntercept, 2.0) + pow(yIntercept, 2.0));
	return distance;
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