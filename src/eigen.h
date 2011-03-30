#ifndef __EIGEN_H
#define __EIGEN_H

#include <iostream>
#include <math.h>

#define SIGN(a,b) ((b)<0 ? -fabs(a) : fabs(a))

void tred2(int, float **, float *, float *);
void tqli(int, float, float, float);
void eig_sys(int, float **, float **, float *);
#endif


