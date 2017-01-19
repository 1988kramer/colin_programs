// uses weighted linear least squares regression to fit a line with the
// equation y = mx + b to the given array of points

// uses exponential weighting function to weight points based on their distance to
// the origin, with closer points being weighted more heavily

#include"LineFitter.h"

void LineFitter::setPoints(Point* points)
{
	for (int i = 0; i < numPoints_; i++)
		points_[i].setCoordinates(points[i].getRange(), points[i].getHeading());
}

void LineFitter::updateLine()
{
	buildAMatrix();
	buildBMatrix();
	buildWMatrix();
	findCoefficients();
}


LineFitter::LineFitter(Point* points, int numPoints)
{
	numPoints_ = numPoints;
	W_ = new double*[numPoints_];
	for (int i = 0; i < numPoints_; i++)
		W_[i] = new double[numPoints_];
	A_ = new double*[numPoints_];
	for (int i = 0; i < numPoints_; i++)
		A_[i] = new double[2];
	B_ = new double*[numPoints_];
	for (int i = 0; i < numPoints_; i++)
		B_[i] = new double[1];
	points_ = new Point[numPoints_];
	setPoints(points);
}

LineFitter::~LineFitter()
{
	for (int i = 0; i < numPoints_; i++)
	{
		delete[] W_[i];
		delete[] A_[i];
		delete[] B_[i];
	}
	delete[] W_;
	delete[] A_;
	delete[] B_;
	delete[] points_;
}

void LineFitter::findCoefficients()
{
	double** At_W = find_At_W();

	//printf("\nAt_W built \n");
	//printMatrix(At_W, 2, numPoints_);

	double** At_W_A = multiplyMatrices(At_W, 2, numPoints_, A_, numPoints_, 2);

	//printf("\nAt_W_A built \n");
	//printMatrix(At_W_A, 2, 2);

	double** inv_At_W_A = find2by2inverse(At_W_A);

	//printf("\ninv_At_W_A built \n");
	//printMatrix(inv_At_W_A, 2, 2);

	double** At_W_B = multiplyMatrices(At_W, 2, numPoints_, B_, numPoints_, 1);

	//printf("\nAt_W_B built \n");
	//printMatrix(At_W_B, 2, 1);
	
	double** x = multiplyMatrices(inv_At_W_A, 2, 2, At_W_B, 2, 1);

	//printf("\nx matrix built \n");
	//printMatrix(x, 2, 1);

	m_ = x[1][0];
	b_ = x[0][0];

	// delete matrices
	for (int i = 0; i < 2; i++)
	{
		delete[] At_W[i];
		delete[] inv_At_W_A[i];
		delete[] x[i];
		delete[] At_W_B[i];
	}
	delete[] At_W;
	delete[] inv_At_W_A;
	delete[] x;
	delete[] At_W_B;
}

double LineFitter::getM()
{
	return m_;
}

double LineFitter::getB()
{
	return b_;
}


void LineFitter::buildAMatrix()
{
	for (int i = 0; i < numPoints_; i ++) 
	{
		A_[i][0] = 1.0;
		A_[i][1] = points_[i].getX();
	}
	//printf("A matrix built \n");
	//printMatrix(A_, numPoints_, 2);
	//printf("\n");
}

void LineFitter::buildBMatrix()
{
	for (int i = 0; i < numPoints_; i++)
		B_[i][0] = points_[i].getY();
	//printf("B matrix built \n");
	//printMatrix(B_, numPoints_, 1);
	//printf("\n");
}

// weight decays to 0 by range = 200
void LineFitter::buildWMatrix()
{
	for (int i = 0; i < numPoints_; i++)
		W_[i][i] = exp(-1.0 * pow(points_[i].getRange(), 2.0) / 7500.0);
	//printf("W matrix built \n");
	//printMatrix(W_, numPoints_, numPoints_);
	//printf("\n");
}

/*
void LineFitter::printMatrix(double** matrix, int rows, int columns)
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			printf("%.2f ", matrix[i][j]);
		}
		printf("\n");
	}
}
*/

double** LineFitter::find2by2inverse(double** matrix)
{
	// calculate the determinant of the matrix
	double determinant = matrix[0][0] * matrix[1][1];
	determinant -= matrix[1][0] * matrix[0][1];
	// calculate inverse of the matrix
	double** matrixInv = new double*[2];
	for (int i = 0; i < 2; i++)
		matrixInv[i] = new double[2];
	matrixInv[0][0] = matrix[1][1];
	matrixInv[1][1] = matrix[0][0];
	matrixInv[0][1] = -matrix[0][1];
	matrixInv[1][0] = -matrix[1][0];
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
			matrixInv[i][j] /= determinant;
	}
	return matrixInv;
}

double** LineFitter::multiplyMatrices(double** A, int aRows, int aColumns, 
									  double** B, int bRows, int bColumns)
{
	double** result = new double*[aRows];
	for (int i = 0; i < aRows; i++)
		result[i] = new double[bColumns];
	for (int i = 0; i < aRows; i++)
	{
		for (int j = 0; j < bColumns; j++)
		{
			for (int k = 0; k < aColumns; k++)
				result[i][j] += A[i][k] * B[k][j];
		}
	}
	return result;
}

double** LineFitter::find_At_W()
{
	double** Atranspose = new double*[2];
	for (int i = 0; i < 2; i++)
		Atranspose[i] = new double[numPoints_];
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j < 2; j++)
			Atranspose[j][i] = A_[i][j];
	}
	return multiplyMatrices(Atranspose, 2, numPoints_, W_, numPoints_, numPoints_);
}
