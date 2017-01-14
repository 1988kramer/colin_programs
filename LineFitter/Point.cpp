#include"Point.h"

Point::Point(double range, double heading)
{
	setCoordinates(range, heading);
}

Point::Point()
{
	x_ = 0.0;
	y_ = 0.0;
	range_ = 0.0;
	heading_ = 0.0;
}

double Point::getX()
{
	return x_;
}

double Point::getY()
{
	return y_;
}

double Point::getRange()
{
	return range_;
}

double Point::getHeading()
{
	return heading_;
}

void Point::setCoordinates(double range, double heading)
{
	range_ = range;
	heading_ = heading;
	x_ = range_ * cos(heading_);
	y_ = range_ * sin(heading_);
}