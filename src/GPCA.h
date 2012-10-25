// **************************************************************************
// GPCA.h
//
//   GENERALIZED PRINCIPAL COMPONENT ANALYSIS
//
//   Function definitions for hyperplane GPCA. These methods segment data
//    lying on a known number of hyperplanes.
//
//   The only function which needs to be called externally is 
//    Hyperplane_GPCA_Known_Group_Count.
// 
//   More advanced versions, which can handle unknown numbers of groups,
//    subspaces of varying dimension, higher noise levels, and outliers 
//    are available at:
//       http://perception.csl.uiuc.edu/gpca/sample_code/index.html
//
//   An introduction to the GPCA family of algorithms can be found at
//       http://perception.csl.uiuc.edu/gpca
//
//   Defines: Hyperplane_GPCA_Known_Group_Count, 
//             Compute_Derivative, 
//			   Hyperplane_GPCA_Result,
//			   Assign_Points_To_Hyperplanes
//
//   Questions?
//     John Wright -- jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#ifndef __GPCA_H__
#define __GPCA_H__

#include "GPCA_Global.h"
#include "Veronese.h"

// **************************************************************************
// Hyperplane_GPCA_Result
//  
//   structure for returning the results of hyperplane segmentation
// **************************************************************************
struct Hyperplane_GPCA_Result
{
	// default constructor
	Hyperplane_GPCA_Result() : assignments(NULL), normals(NULL) {}

	// constructor
	//  
	//   allocates the return structure for a problem with
	//		N points in D dimensions, with n groups.
	//
	//   Inputs:
	//     N - the number of points
	//     n - the number of subspaces
	//     D - the dimension of the data points
	Hyperplane_GPCA_Result(int N, int n, int D) : assignments( new int[N] ),
												  numGroups(n),
												  normals( new CvMat*[n] )
	{
		// allocate n 1xD row vectors to store the normals
		for( int i = 0; i < n; ++i )
			normals[i] = cvCreateMat(1,D,CV_64FC1);
	}

	// destructor, frees arrays which are not null
	~Hyperplane_GPCA_Result() 
	{ 
		if(assignments != NULL) 
			delete [] assignments;
		if( normals != NULL )
		{
			for( int i = 0; i < numGroups; ++i )
				cvReleaseMat( normals + i );
			delete [] normals;
		}
	}

	int *assignments;  // integer array, the i-th value is in 
					   //  0 ... numGroups-1,
					   //  indicating which of the subspaces the 
					   //  i-th point is in.

	int numGroups;     // the number of subspaces

	CvMat **normals;   // array of CvMat's, length is numGroups.
					   //  j-th element is a 1 by D matrix, giving the
					   //  normal to the j-th subspace
};



// **************************************************************************
// Hyperplane_GPCA_Known_Group_Count
//
//   Segments data lying on a known number of hyperplanes. Determines the
//    normal to each hyperplane, and assigns each point to the closest of the
//    n hyperplanes.
//
//   The implementation roughly follows:
//    Generalized Princpal Component Analysis (GPCA)
//     Rene Vidal, Yi Ma and Shankar Sastry
//     IEEE PAMI 2005
//
//   Inputs:
//     data -- the data matrix, N by D, where N is the number of points and
//              D is the dimension of the ambient space. Its rows are the 
//				data vectors
//
//     n -- the number of hyperplanes
//
//     veroneseMap (optional) -- external callers should leave this NULL.
//								 This is used internally by some model
//								  selection functions, which already compute
//								  the veronese embedding
//
//   Output:
//     result -- structure containing the assignments of the data points and
//                the normals to the hyperplanes (see Hyperplane_GPCA_Result
//                above for more detail)
//
// **************************************************************************
Hyperplane_GPCA_Result* 
	Hyperplane_GPCA_Known_Group_Count( CvMat *data, 
									   int n,
									   CvMat* paramVeroneseEmbedding = NULL );




// ***************************************************************************
// Compute_Derivative
//
//    Calculates the gradient of the fitting polynomial, evaluated
//     at each of the data points
//
//    Inputs:
//      Data - matrix of original data points (each row is a point)
//      n - the order of the veronese map (number of groups)
//      symbolicVeronese - symbolic representation of the veronese map of 
//                         order n (see Compute_Symbolic_Veronese_Map)
//      fittingPolynomial - coefficients of the best fit polynomial 
//							(see Get_Fitting_Polynomial)
//
//    Outputs:
//      Normals - matrix whose i_th row is the gradient of fittingPolynomial, 
//                 evaluated at the i_th data point (i_th row of data)
//                 called Normals because (if the data points lie on the 
//                 zeroset of the polynomial) these gradient vectors will
//                 be normal to that zeroset
// ***************************************************************************
void Compute_Derivative( CvMat *Data, 
						 int n, 
						 CvMat *symbolicVeronese,
						 CvMat *fittingPolynomial, 
						 CvMat *Normals );


// **************************************************************************
// Assign_Points_To_Hyperplanes
//
//   Takes a set of n hyperplanes (defined by their normals) and assigns 
//    each data vector to the nearest hyperplane.
//
//   Inputs:
//     data -- the data matrix, rows are the data vectors
//
//     n -- the number of hyperplanes
//
//     normals -- the normals to the n hyperplans. Each entry should be a 
//                 1 by D matrix (where D is the dimension of the data
//                 vectors).
//
//     assignments -- integer array, length should be equal to the number of
//                     rows of data. The assignments are written here.
//
//   Outputs:
//     none, but writes to the assignments array. The groups are numbered
//      0 ... n-1
//
// **************************************************************************
void Assign_Points_To_Hyperplanes( CvMat *Data, 
								   int n, 
								   CvMat **normals, 
								   int *assignments );

#endif // __GPCA_H__
