// **************************************************************************
// GPCA.cpp
//
//   Implementation of functions for segmenting data lying on a known number
//    of hyperplanes.
//
//   For more detailed function headers, please see GPCA.h
//
//   Questions?
//     John Wright -- jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#include "GPCA.h"

#define DELTA			.01


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
//   The basic idea is:
//
//   Fit a polynomial, P(x).
//   To get the j-th hyperplane, find a 'good' point, x*,
//
//		   arg min   sqrt(  P(Xi)^2 /  |dP(x)|^2 ) + DELTA
//			    Xi   -------------------------------------
//				       |B_1'*Xi|*...*|B_{j-1}'*Xi| + DELTA
//
//		where dP is the derivative (gradient) of the polynomial
//		and B_1 ... B_j-1 are the normals to the hyperplanes found so far
//
//   Evaluate the derivative of the polynomial at x* to get the normal to the
//     j-th hyperplane
//
//	 Assign each point to the closest hyperplane ( the one 
//		minimizing |B_j' * X_i| )
//
// **************************************************************************
Hyperplane_GPCA_Result* 
	Hyperplane_GPCA_Known_Group_Count( CvMat *Data,
									   int n, 
									   CvMat *paramVeroneseEmbedding )
{
	int i, j;

	int N = Data->rows;
	int D = Data->cols;

	// allocate return structure
	Hyperplane_GPCA_Result *result = new Hyperplane_GPCA_Result(N,n,D);

	// embed the data in a higher dimensional space,
	//  so that our union of hyperplanes becomes a single linear subspace
	CvMat *veroneseEmbedding;
	
	if( paramVeroneseEmbedding == NULL )
		veroneseEmbedding = Compute_Veronese_Embedding(Data,n);
	else
		veroneseEmbedding = paramVeroneseEmbedding;

	// compute the best fitting coefficient vector 
	CvMat *C = Homogeneous_Least_Squares(veroneseEmbedding);

	// will hold the values of the fitting polynomial at each data point
	CvMat *P = cvCreateMat(N,1,CV_64FC1);

	// evaluate the polynomial at each point
	cvMatMul( veroneseEmbedding, C, P );

	// will hold the derivative of the polynomial, evaluated at each data point
	CvMat *dP = cvCreateMat( N, D, CV_64FC1 );

	// get a symbolic representation of the Veronese map, 
	//  used in the next step for computing derivatives
	CvMat *symbolicVeroneseMap = Compute_Symbolic_Veronese_Map(D,n);

	// evaluate the derivative of the polynomial at each point
	Compute_Derivative(Data, n, symbolicVeroneseMap, C, dP );

	// If we know one 'good' point on a hyperplane, the derivative of the 
	//  fitting polynomial at that point gives us the normal to the hyperplane
	//
	// We choose the j-th good point according to the criterion:
	//
	//    arg min   sqrt(  P(Xi)^2 /  |dP(x)|^2 ) + DELTA
	//         Xi   -------------------------------------
	//               |B_1'*Xi|*...*|B_{j-1}'*Xi| + DELTA
	//
	// The product in the denominator is the product of distances from Xi to
	//  the j-1 subspaces chosen so far. If Xi lies on one of the subspaces,
	//  this product will be very small, making the criterion value large.
	//
	// The numerator is small when the fitting polynomial is small, and its
	//  derivative is large -- meaning that the point is very close to a 
	//  subspace and the normal is well-defined.
	//
	// So, this criterion is small for a point lying very close to a subspace
	//  we have not selected so far.

	double *criterionNumerator = new double[N];

	// holds the product of B_j'*Xi on the bottom of the criterion expression
	double *productOfProjections = new double[N];

	// compute the numerator of the criterion at each of the points
	for( i = 0; i < N; ++i )
	{
		// set dP_Xi to point to the derivative at the i-th point
		CvMat dP_Xi;
		Get_Matrix_Block( dP, &dP_Xi, i, i, 0, D-1 );

		// P_Xi is the value of the fitting polynomial at the i-th point
		double P_Xi = cvmGet(P,i,0);

		criterionNumerator[i] = 
			sqrt( P_Xi*P_Xi / cvDotProduct(&dP_Xi,&dP_Xi) ) + DELTA;

		productOfProjections[i] = 1;
	}

	// now find the normals to our n hyperplanes
	for( j = 0; j < n; ++j )
	{
		// select the best point:
		double bestCriterionValue = FLT_MAX;
		double curCriterionValue;
		int bestPointIndex = 0;

		// iterate through all the points,
		//  compute the value of the criterion, and compare to the best value
		//  found so far
		for( i = 0; i < N; ++i )
		{
			// special case for j = 0, because we initialized 
			//  productOfProjections[i] to be 1 instead of zero
			if( i == 0 )
				curCriterionValue = criterionNumerator[i] / DELTA;
			else
			{
				curCriterionValue = criterionNumerator[i] / 
				( productOfProjections[i] + DELTA );
			}

			if( curCriterionValue < bestCriterionValue )
			{
				bestPointIndex = i;
				bestCriterionValue = curCriterionValue;
			}
		}

		// ok, we have the index of the best point. 
		//  now get its normal and save it in the return structure
		CvMat dP_best;

		// make dP_best point to the derivative evaluated at the best point
		Get_Matrix_Block(dP,&dP_best,bestPointIndex,bestPointIndex,0,D-1);

		CvMat *B_j = result->normals[j];

		// rescale the deriviative so that it is a unit vector and
		//   store the result in result->normals[j]
		cvScale(&dP_best, B_j, 1.0/cvNorm(&dP_best) );

		// ok, all that's left to do is to multiply |B_j'*Xi| into 
		//  the product of projections term in the denominator of the
		//  criterion:
		CvMat X_i;

		// only compute this if we're going to use it (if we haven't found
		//  the n-th normal yet) ... j is compared to n-1 instead of n
		//  because this is 0-indexed C++ code.
		if( j < n - 1 )
		{
			for( i = 0; i < N; ++i )
			{
				// set X_i to point to the i-th data vector
				Get_Matrix_Block( Data, &X_i, i, i, 0, D-1 );

				// compute |B_j' * X_i|, and multiply it into our product of
				//  projections. note that this is the distance from
				//  X_i to the j-th hyperplane.
				productOfProjections[i] *= fabs(cvDotProduct(B_j,&X_i));
			}
		}
	}

	// compute the final segmentation
	Assign_Points_To_Hyperplanes(Data,n,result->normals,result->assignments);

	// clean up the trash
		cvReleaseMat( &P );
		cvReleaseMat( &dP );
		cvReleaseMat( &C );

		// if the passed embedding is NULL, then the embedding was computed
		//  locally, so we need to clean it up here
		if( paramVeroneseEmbedding == NULL )
			cvReleaseMat( &veroneseEmbedding );

		cvReleaseMat( &symbolicVeroneseMap );

		delete [] criterionNumerator;
		delete [] productOfProjections;
	//

	return result;
}



// ***************************************************************************
// Compute_Derivative
//
//    Calculates the gradient of the fitting polynomial, evaluated
//     at each of the data points.
//
//    Straightforward implementation -- at each point, iterate through 
//     variables. For each varible, Xj, compute dP/dXj by summing
//     contributions of all monomials containing Xj.
//
// ***************************************************************************
void Compute_Derivative( CvMat *data, 
						   int n, 
						   CvMat *symbolicVeronese, 
						   CvMat *fittingPolynomial,
						   CvMat *derivative )
{
	// dimension of ambient space
	int D = derivative->cols;

	// set C to point to the actual double array in the CvMat structure
	double *C = fittingPolynomial->data.db;

	int veroneseDimension = Choose( n + D - 1, D - 1 );

	// for each point
	for( int i = 0; i < derivative->rows; ++i )
	{
		// the i-th data vector
		double *x_i = (double*) cvPtr2D( data, i, 0 );

		// the derivative of the polynomial, evaluated at the
		//  i-th data vector, 
		//    dP(x_i) = [ dP/dX1 dP/dX2 ... dP/dXD ](x_i)
		double *dP_i = (double*) cvPtr2D( derivative, i, 0 );

		// for each variable
		for( int j = 0; j < D; ++j )
		{
			// compute dP / Xj (x_i)  (where Xj is the j-th variable)
			dP_i[j] = 0;

			// for each monomial
			for( int k = 0; k < veroneseDimension; ++k )
			{
				// compute the derivative of the k-th monomial
				//  with respect to the j-th variable

				// pointer to symbolic representation of k-th monomial
				int *curTableRow = (int*) cvPtr2D( symbolicVeronese, k, 0 );

				int power_Xj = curTableRow[j];
				
				// does this monomial depend on Xj?
				if( power_Xj > 0 )
				{
					// compute the contribution of k-th monomial to dP/dXj( x_i )
					double dVk_dXj = 1.0;
					
					// multiply by all terms in the monomial except Xj
					for( int m = 0; m < D; ++m )
					{
						if( m != j )
							dVk_dXj *= pow( x_i[ m ], curTableRow[ m ] );
					}

					// derivative of Xj^power_Xj
					dVk_dXj *= power_Xj * pow( x_i[ j ], power_Xj - 1 );

					// multiply by the coefficient of the k-th monomial in
					//  the original polynomial, giving the final value of the
					//  derivative
					dP_i[j] += C[k] * dVk_dXj;
				}

			} // done iterating through monomials

		} // done iterating through variables

	} // done iterating through point

	// the return values were written to the input variable, Derivative
}


// **************************************************************************
// Assign_Points_To_Hyperplanes
//
//   Takes a set of n hyperplanes (defined by their normals) and assigns 
//    each data vector to the nearest hyperplane.
// **************************************************************************
void Assign_Points_To_Hyperplanes( CvMat *data, 
								   int n, 
								   CvMat **normals, 
								   int *assignments )
{
	double curDistance, bestDistance;
	int bestSubspace;

	// the i-th data vector
	CvMat X_i;

	int N = data->rows;
	int D = data->cols;

	// loop through data points
	for( int i = 0; i < N; ++i )
	{
		// set X_i to point to i-th data vector
		Get_Matrix_Block(data,&X_i,i,i,0,D-1);

		bestSubspace = 0;
		bestDistance = FLT_MAX;

		// compute the distance from the i-th data point to each
		//  subspace, find the smallest
		for( int j = 0; j < n; ++j )
		{
			// distance from X_i to the j-th subspace
			curDistance = fabs( cvDotProduct( &X_i, normals[j] ) );

			if( curDistance < bestDistance )
			{
				bestDistance = curDistance;
				bestSubspace = j;
			}
		}

		assignments[i] = bestSubspace;
	}
}

