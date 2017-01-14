#ifndef LINEFITTER_H
#define LINEFITTER_H

class LineFitter
{
public:
	LineFitter(Point* points, int numPoints);
	~LineFitter();
	void setPoints(Point* points);
	void updateLine();
	double getM();
	double getB();

private:
	double** W_;
	double** A_;
	double** B_;
	Point* points_;
	double m_;
	double b_;
	int numPoints_;
	void buildAMatrix();
	void buildBMatrix();
	void buildWMatrix();
	void findCoefficients();
	double** multiplyMatrices(double** A, int aRows, int aColumns, 
						 	  double** B, int bRows, int bColumns);
	double** find_At_W();
	double** find2by2inverse(double** matrix);
};

#endif