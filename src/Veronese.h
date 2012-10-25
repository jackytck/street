// **************************************************************************
// Veronese.h
//
//   Functions for computing, representing and manipulating the Veronese
//    embedding. 
//   
//   Defines:
//     Compute_Symbolic_Veronese_Map, Recursive_Symbolic_Veronese, 
//     Compute_Veronese_Embedding, Recursive_Veronese
//
//   Questions?
//     John Wright -- jnwright@uiuc.edu
//
//   Copyright 2005 University of Illinois
//
// **************************************************************************

#ifndef __VERONESE_H__
#define __VERONESE_H__

#include "GPCA_global.h"


// ***************************************************************************
// Compute_Symbolic_Veronese_Map
//
//    Computes a representation of the veronese map:
//    
//    V_n : R^D -> R^M_n(D)  where M_n(D) = (n + D - 1) choose (D - 1)
//
//    V_n maps the input vector (D variables) to a vector that contains
//         all the monomials of degree n in those D variables, in 
//         degree-lexicographic order
//
//  Inputs: 
//    D - the number of variables (dimension of data vectors)
//    n - the degree of the map (number of groups)
//
//  Outputs
//    an integer matrix of size    M_n(D) x D
//      the ij-th entry of this matrix is the exponent of the j-th
//      variable in the i-th monomial
//
//  Example:
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
//   Note: 
//    Right now (7/1/05), the numerical method Compute_Veronese_Embedding
//    is used for the actual embedding, and the symbolic map is only 
//    used to compute derivatives.
// ***************************************************************************
CvMat *Compute_Symbolic_Veronese_Map( int D, 
									  int n );




// ***************************************************************************
// Recursive_Symbolic_Veronese
//    
//   Recursive helper function for Compute_Symbolic_Veronese_Map
//
//   The recursion here is pretty much the same as that used in numerically
//    computing the Veronese embedding. See the function description for
//    Recursive_Veronese for an explanation and example
//   
//   External functions should call Compute_Symbolic_Veronese_Map instead
//
//   Inputs:
//     cur - indicates the lowest variable still active. Zero indexed
//            (e.g. for 3 variables [x y z], if cur = 1, the 
//            recursive computation should only be performed with y and z
//     D - the number of variables remaining
//     n - the order of the Veronese map (degree of each monomial)
//     dest - place where results should be written
//     acc - the representation computed so far, from variables previously
//            eliminated in the recursion
//     destStep - total number of variables (length of a row in the 
//                 symbolic representation) (probably redundant)
//
//   Outputs:
//     i - the number of rows of the symbolic veronese map that this call 
//          (and its recursive children) have filled in
// ***************************************************************************
int Recursive_Symbolic_Veronese( int cur, 
								 int D, 
								 int n, 
								 int *dest, 
								 int *acc, 
								 int destStep );




// ***************************************************************************
// Compute_Veronese_Embedding
//
//    Computes the veronese embedding of order n of the data matrix
//
//    Inputs:
//      Data - p by q matrix, whose rows are the data vectors
//      n - the order of the Veronese map (i.e. the number of groups)
//
//    Outputs:
//      Veronese - a p by M_n(q) matrix whose rows are the veronese
//					embedding of the rows of Data
// ***************************************************************************
CvMat *Compute_Veronese_Embedding( CvMat *Data, 
								   int n );




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
//    Inputs:
//      X - pointer to the part of the data vector that hasn't been
//           used yet
//      D - number of variables
//      n - degree of the map
//      dest - pointer to place to write results
//      mul - constant multiplier for each new element computed
//             (contributions by variables already considered)
//
//    Outputs:
//      i - the number of monomials computed by this call and its
//           children
//
//    Example:
//      Suppose the data has 3 variables [x y z]
//      The veronese map of order 3 is: 
//        [ x^3   x^2*y x^2*z   x*y^2 xyz x*z^2  y^3 y^2*z y*z^2 z^3 ]
//      = 
//        [ x^3 * V_0([y z])   x^2 * V_1([y z])   x * V_2([y z])   V_3([y z]) ]
//
//      Where V_n([y z]) is the veronese map of order n in the variables y and z
//       and V_0([y z]) is defined to be the scalar 1
// ***************************************************************************
int Recursive_Veronese( double *X, 
					    int D, 
						int n, 
						double *dest, 
						double mul );

#endif // __VERONESE_H__
