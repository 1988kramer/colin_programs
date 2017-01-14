#include<math.h>

#ifndef POINT_H
#define POINT_H

class Point
{
private:
	double x_;
	double y_;
	double range_;   // in cm
	double heading_; // in radians
public:
	Point(double range, double heading);
	Point();
	double getX();
	double getY();
	double getRange();
	double getHeading();
	void setCoordinates(double range, double heading);
};

#endif