// program to test the line fitting class

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "LineFitter/LineFitter.h"

using namespace std;

const int numSensors = 8;

void readPoints(Point* points)
{
	ifstream file;
	file.open("sonar.txt");
	int range;
	double heading;
	for (int i = 0; i < numSensors; i++)
	{
		file >> range;
		file >> heading;
		points[i].setCoordinates(range, heading);
	}
	file.close();
}

int main()
{
	Point points[numSensors];
	LineFitter line(points, numSensors);
	readPoints(points);
	line.setPoints(points);
	line.updateLine();
	double slope = line.getM();
	double intercept = line.getB();
	printf("y = %.4f x + %.2f\n", slope, intercept);
	// segfaults here for some reason
	fflush(stdout);
	cout << "press any key to exit: ";
	getchar();
	return 1;
}