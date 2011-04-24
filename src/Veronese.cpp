// ***************************************************************************
// Veronese.cpp
//
//   Implementation of functions for computing veronese embeddings and 
//    symbolic representations of the veronese map.
//
//   For more detailed function headers, including input/output descriptions
//    please see Veronese.h
//
//   Questions?
//     John Wright -- jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#include "Veronese.h"

// ***************************************************************************
// Compute_Symbolic_Veronese_Map
//
//    Computes a representation of the veronese map
//  
//    V_3( [x y] ) = [ x^3 x^2*y x*y^2 y^2 ]
//
//    a symbolic representation of this map is:
//
//    [  3  0  ]
//    [  2  1  ]
//    [  1  2  ]
//    [  0  3  ]
//
// **************************************************************************
CvMat *Compute_Symbolic_Veronese_Map( int D, int n )
{
	int i;

	//  M_n(D) = 
	//        < n + D - 1 >
	//        <   D - 1   > 
	int veroneseDimension = Choose( n + D - 1, D - 1 );

	// destination
	int *V = new int[ veroneseDimension * D ];

	// temporary used in recursion
	int *acc = new int[ D ];

	for( i = 0; i < D; ++i )
		acc[i] = 0;

	// call reccursive helper
	Recursive_Symbolic_Veronese( 0, D, n, V, acc, D );

	delete [] acc;

	CvMat *symVer = cvCreateMatHeader( veroneseDimension, D, CV_32SC1 );
	
	cvSetData( symVer, V, D * sizeof(int) );

	return symVer;
}



// ***************************************************************************
// Recursive_Symbolic_Veronese
//    
//   Recursive helper function for Compute_Symbolic_Veronese_Map
//
//   The recursion here is pretty much the same as that used in numerically
//    computing the Veronese embedding. See the function description for
//    Recursive_Veronese( ) for an explaination and example
//
// **************************************************************************
int Recursive_Symbolic_Veronese( int cur, int D, int n, int *dest, 
					             int *acc, int destStep )
{
	// base case -- all variables have been considered
	if( D == 0 )
		return 0;

	int num_Monomials_Computed = 1;

	// handle the first monomial (just the first variable to the n-th power)
	//  times the contributions of variables previously eliminated 
	//  (as represented by the acc array)
	for( int k = 0; k < destStep; ++k )
		dest[k] = acc[k];
	dest[cur] += n;

	// recurse on all other monomials (all powers j < n of the first 
	//  variable

	// for each power j  
	for( int j = n - 1; j >= 0; --j )
	{
		// compute terms containing  x_cur^j 

		//  include x_cur^j in the acculator
		acc[cur] += j;

		// compute the contributions of x_k : k = cur+1, cur+2, ... destStep
		//  (the rest of the variables)
		num_Monomials_Computed += 
			Recursive_Symbolic_Veronese( cur + 1, D - 1, n - j, 
			                   dest + num_Monomials_Computed * destStep, 
							   acc, destStep );

		// reset accumulator to original state
		acc[cur] -= j;
	}

	return num_Monomials_Computed;
}



// ***************************************************************************
// Compute_Veronese_Embedding
//
//    Computes the veronese map of order n of the data matrix
//
// ***************************************************************************
CvMat *Compute_Veronese_Embedding( CvMat *Data, int n )
{
	int D = Data->cols;

	//  M_n(D) = 
	//        < n + D - 1 >
	//        <   D - 1   > 
	int veroneseDimension = Choose( n + D - 1, D - 1 );

	CvMat *V = cvCreateMat( Data->rows, veroneseDimension, CV_64FC1 );

	// for each data vector
	for( int i = 0; i < Data->rows; ++i )
	{
		double *v = (double*) cvPtr2D( V, i, 0 );
		double *x = (double*) cvPtr2D( Data, i, 0 );

		// compute veronese embedding of this data vector
		Recursive_Veronese(x,D,n,v,1.0);
	}

	return V;
}



// ***************************************************************************
// Recursive_Veronese
//  
//    Recursive helper function for Compute_Veronese_Embedding
//     Should not be called by outside functions 
//     (Call Compute_Veronese_Embedding instead)
//
//    The idea is to compute the veronese
//     map by eliminating the first variable and then computing
//     a bunch of smaller veronese maps in D-1 variables
//     The contribution of the first variable (and any previously
//     eliminated variables) is passed through the input variable
//     mul.
//
// ***************************************************************************
int Recursive_Veronese( double *X, int D, int n, double *dest, double mul )
{
	// base case -- no variables left to consider
	if( D == 0 )
		return 0;

	int num_Monomials_Computed = 1;
	
	//  first monomial --  X[0]^n times contributions of all
	//   variables previously eliminated
	dest[0] = mul * pow( X[0], n );

	//  for each power of X[0]
	for( int j = n - 1; j >= 0; --j )
	{
		//  include the contribution of X[0] ( X[0]^j )
		//  and then recurse on the other variables X[1] ... X[D]
		num_Monomials_Computed += Recursive_Veronese( X + 1, D - 1, n - j, 
										dest + num_Monomials_Computed, 
										mul * pow(X[0], j) );
	}

	return num_Monomials_Computed;
}
