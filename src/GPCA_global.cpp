// **************************************************************************
// Global.cpp
//
//   Implementation of some global helper functions defined in Global.h
//   
//   For more detailed function descriptions, including inputs/outpus,
//    please see Global.h
//
//   Questions?
//     John Wright -- jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#include "GPCA_Global.h"

// **************************************************************************
//  Is_Valid
//   
//    returns true if x is not infinity or NaN
// **************************************************************************
bool Is_Valid( double x )
{
	return (!cvIsInf(x)) && (!cvIsNaN(x));
}



// **************************************************************************
//  Color_Tag
//
//    Assigns a color to each group label
// **************************************************************************
CvScalar Color_Tag( int i )
{
	CvScalar color;

	switch( i )
	{
	case 0:
		color = RED;
		break;
	case 1:
		color = GREEN;
		break;
	case 2: 
		color = BLUE;
		break;
	case 3:
		color = YELLOW;
		break;
	case 4:
		color = MAGENTA;
		break;
	case 5:
		color = CYAN;
		break;
	case 6:
		color = ORANGE;
		break;
	case 7:
		color = WHITE;
		break;
	default:
		color = GRAY;
	}

	return color;
}



// **************************************************************************
// Draw_Line
//
//   Draws a line defined by its coimage, L.
// **************************************************************************   
void Draw_Line( IplImage *dest, CvMat *L, CvScalar color, 
				int width, double xCenter, double yCenter)
{
	// grab line components, adjust for center offset
	double l1 = cvGetReal2D(L,0,0);
	double l2 = cvGetReal2D(L,1,0);
	double l3 = cvGetReal2D(L,2,0) - l1 * xCenter - l2 * yCenter;

	// this gives equation 
	//   l1*x + l2*y + l3 = 0 for all points on the line

	// determine line's intersection with the four sides of the image
		double xl, yl, xr, yr, xt, yt, xb, yb;

		xl = 0;
		yl = -l3 / l2;

		xr = dest->width - 1;
		yr = -( l1*xr + l3 ) / l2;

		yb = 0;
		xb = -l3 / l1;

		yt = dest->height - 1;
		xt = -( l2*yt + l3 ) / l1;
	//

	// pick the two of these which are on the screen
		CvPoint p1, p2;
		bool haveP1 = false;
		bool haveP2 = false;

		if( Is_Valid( yl ) && yl >= 0 && yl < dest->height )
		{
			p1 = cvPoint(xl,yl);
			haveP1 = true;
		}
		if( Is_Valid( yr ) && yr >= 0 && yr < dest->height )
		{
			if( !haveP1 )
			{
				p1 = cvPoint( xr, yr );
				haveP1 = true;
			}
			else 
			{
				p2 = cvPoint( xr, yr );
				haveP2 = true;
			}
		}
		if( Is_Valid( xb ) && xb >= 0 && xb < dest->width )
		{
			if( !haveP1 )
			{
				p1 = cvPoint( xb, yb );
				haveP1 = true;
			}
			else if( !haveP2 )
			{
				p2 = cvPoint( xb, yb );
				haveP2 = true;
			}
		}
		if( Is_Valid( xt ) &&  xt >= 0 && xt < dest->width )
		{
			if( !haveP1 )
			{
				p1 = cvPoint( xt, yt );
				haveP1 = true;
			}
			else if( !haveP2 )
			{
				p2 = cvPoint( xt, yt );
				haveP2 = true;
			}
		}
	//

	// draw the line
	if( haveP1 && haveP2 )
	{
		cvLine( dest, p1, p2, color, width, 8 );
	}
}



// **************************************************************************
// Get_Matrix_Block
//
//   sets destHeader to point to M( i0:i1, j0:j1 )
// **************************************************************************
void Get_Matrix_Block( CvMat *M, CvMat *destHeader, int i0, int i1, int j0, int j1 )
{
	cvGetSubRect( M, destHeader, cvRect( j0, i0, j1 - j0 + 1, i1 - i0 + 1) );
}



// **************************************************************************
// Print_Out
//  
//   Prints a matrix (CvMat) to the screen
// **************************************************************************
void Print_Out(CvMat *M)
{
	int i, j;
	char dst[20];
	double d;

	// print out dimensions
	cout << endl << M->rows << "x" << M->cols << " matrix:" << endl;

	// print out values
	for( i = 0; i < M->rows; ++i )
	{
		for( j = 0; j < M->cols; ++j )
		{
			d = cvGetReal2D(M,i,j);
			sprintf(dst,"%4.4f",d);

			cout << dst << "  ";
		}
		cout << endl;
	}

	cout << endl;
}



// **************************************************************************
// Homogeneous_Least_Squares
//
//   Computes the unit norm vector x minimizing:   || A x ||
// 
//   i.e. the least squares solution to:  Ax = 0
//                                        subj |x| = 1
// **************************************************************************
CvMat* Homogeneous_Least_Squares( CvMat *A )
{
	int n = A->cols;
	
	// the return value (least squares solution)
	CvMat *X = cvCreateMat(n,1,CV_64FC1);

	// temporaries for taking the SVD
	CvMat *S = cvCreateMat(n,1,CV_64FC1);
	CvMat *V_T = cvCreateMat(n,n,CV_64FC1);

	cvSVD( A, S, NULL, V_T, CV_SVD_V_T );

	// the smallest singular vector of A
	//  is now the last row of V_T (since SVD in previous line returns
	//  V transposed)
	memcpy( X->data.db, 
			(double*) cvPtr2D( V_T, n - 1, 0 ), 
			n*sizeof(double));

	cvReleaseMat( &S );
	cvReleaseMat( &V_T );

	return X;
}



// **************************************************************************
// Choose 
//                 n!
//    returns  ----------
//              m!(n-m)!
// **************************************************************************
double Choose( int n, int m )
{
	int i;
	double n_choose_m = 1.0;

	for( i = n; i > n - m; --i )
		n_choose_m *= i;
	for( i = 1; i <= m; ++i )
		n_choose_m /= (double) i;

	return n_choose_m;
}
