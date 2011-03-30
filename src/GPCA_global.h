// **************************************************************************
// Global.h
//
//   Function definitions for helper functions, a few useful #defines and
//   #includes that are used throughout the rest of the code
//
//   Functions: Is_Valid, Color_Tag, Draw_Line, Get_Matrix_Block
//              Print_Out, Homogeneous_Least_Squares, Choose
//
//   Questions?
//     John Wright - jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#ifndef _GPCA_GLOBAL_H__
#define _GPCA_GLOBAL_H__

// is this a Microsoft compiler?
#ifdef _MSC_VER
// disable a couple of annoying warnings
#pragma warning(disable: 4786)
#pragma warning(disable: 4244)
#endif  // _MSC_VER

// includes used all over the place:

// OpenCV
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>

#include <math.h>

// contains definition of FLT_MAX
#include <float.h>

// C++ I/O
#include <iostream>
#include <fstream>

// C++ STL
#include <vector>
#include <list>

using namespace std;

#ifndef PI
#define PI  3.1415926
#endif

// color definitions
#define RED      CV_RGB(255,0,0)
#define GREEN    CV_RGB(0,255,0)
#define BLUE     CV_RGB(0,0,255)
#define YELLOW   CV_RGB(255,255,0)
#define CYAN     CV_RGB(0,255,255)
#define MAGENTA  CV_RGB(255,0,255)
#define WHITE    CV_RGB(255,255,255)
#define BLACK    CV_RGB(0,0,0)
#define GRAY	 CV_RGB(128,128,128)
#define ORANGE   CV_RGB(255,152,0)


// **************************************************************************
//  Is_Valid
//   
//    returns true if x is not infinity or NaN
// **************************************************************************
bool Is_Valid( double x );



// **************************************************************************
// Color_Tag
//
//   Returns a color associated with the input integer. 
//    Values 0-7 have their own unique tags. Default is GRAY
//
//   Useful for indicating group membership by color
//   See Global.cpp for actual color assignments
//
//   Input:
//     i - integer tag (e.g. group index)
//
//   Output:
//     color - CvScalar (bgr) color representation used in 
//              OpenCV drawing functions
// **************************************************************************
CvScalar Color_Tag( int i );



// **************************************************************************
// Draw_Line
//
//   Draws a line which is represented by its coimage, L, ( the vector such
//    that for all (homogeneous) points X on the line,  X' * L = 0 )
//
//   Inputs:
//     dest - place to draw, should be 8 bit 3 channel
//     L - the coimage, 3x1 matrix, all depths supported
//     color - can be generated using CV_RGB macro
//     width - width of line, in pixels
//     xCenter, yCenter - location of the origin, in pixels    
//
// **************************************************************************
void Draw_Line( IplImage *dest, CvMat *L, CvScalar color = WHITE, 
				int width = 1, double xCenter = 0, double yCenter = 0);



// **************************************************************************
// Get_Matrix_Block
//
//   sets destHeader to point to M( i0:i1, j0:j1 )
//
//   I (John) prefer this to cvGetSubRect (although this function is 
//    really just a wrapper for cvGetSubRect)
//
//   Inputs:
//     M - the matrix we are indexing into
//     destHeader - header that will point to a subarray of M
//     i0 - first row of block 
//     i1 - last row of block
//     j0 - first column of block
//     j1 - last column of block
//   The i's and j's are all 0-indexed 
//    ( 0 -- m-1 for rows and 0 -- n-1 for columns )
//
//   Outputs:
//     none, although destHeader is overwritten
//
//   Note:
//     This function is designed to suggest Matlab notation. 
//     The decision to keep 0-indexing was made to avoid utter 
//      confusion when using it together with other C++ / OpenCV code.
// **************************************************************************
void Get_Matrix_Block( CvMat *M, CvMat *destHeader, 
					   int i0, int i1, int j0, int j1 );



// **************************************************************************
// Print_Out
//  
//   Prints a matrix (CvMat) to the screen
//
//   Inputs:
//     M - single channel matrix (all bit-depths accepted)
//
// **************************************************************************
void Print_Out( CvMat *M );



// **************************************************************************
// Homogeneous_Least_Squares
//
//   Computes the unit norm vector x minimizing:   || A x ||
// 
//   i.e. the least squares solution to:  Ax = 0
//                                        subj |x| = 1
//
//   Inputs:
//     A -- m by n matrix, type CV_64FC1
//
//   Outputs:
//     X -- n x 1 matrx, the singular vector of A corresponding to the 
//           smallest singular value
//
// **************************************************************************
CvMat *Homogeneous_Least_Squares( CvMat *A );



// **************************************************************************
// Choose 
//
//   Returns the value of n choose m:
//   
//         n!
//     ---------
//      m!(n-m)!
//
//   Note:
//     returned as a double to avoid overflow
// **************************************************************************
double Choose( int n, int m );


#endif // _GPCA_GLOBAL_H__
