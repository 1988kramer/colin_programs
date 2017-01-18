// program to test the line fitting class

#include <stdlib.h>
#include "LineFitter/LineFitter.h"

using namespace std;

const int numSensors = 8;

void readPoints(Point* points)
{
	FILE* file;
	file = fopen("sonar.txt", "r");
	int range;
	float heading;
	for (int i = 0; i < numSensors; i++)
	{
		fscanf(file, "%d", &range);
		fscanf(file, "%f", &heading);
		points[i].setCoordinates(range, (double)heading);
	}
	fclose(file);
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
	printf("press any key to exit: ");
	fflush(stdout);
	printf("after fflush\n");
	getchar();
	return 1;
}