#include "ConvexHull2D.h"
#include <stdlib.h>
#include <stdio.h>

ConvexHull2D::ConvexHull2D(): _size(0), _points(NULL), _P(NULL)
{
}

ConvexHull2D::~ConvexHull2D()
{
	for(int i=0; i<_size; i++)
		delete[] _points[i];

	delete[] _points;
	delete[] _P;
}

int ConvexHull2D::load_points(std::vector <osg::Vec3> input)
{
	std::vector <osg::Vec2> converted;
	for(unsigned int i=0; i<input.size(); i++)
		converted.push_back(osg::Vec2(input[i].x(), input[i].z()));

	return load_points(converted);
}

int ConvexHull2D::load_points(std::vector <osg::Vec2> input)
{
	if(input.empty())
		return 0;

	_dict.clear();
	_size = input.size();

	_points = new double *[_size];
	for(int i=0; i<_size; i++)
		_points[i] = new double[2];

	_P = new double *[_size+1];

	for(int i=0; i<_size; i++)
	{
		_points[i][0] = input[i].x();
		_points[i][1] = input[i].y();
		_P[i] = _points[i];
		_dict[_P[i]] = i;
	}

	//debug
	/*
	for(int i=0; i<_size; i++)
		printf("%f %f\n", _points[i][0], _points[i][1]);
	*/

	return _size;
}

std::vector <int> ConvexHull2D::get_hull(double **P, int m)
{
	std::vector <int> ret;

	int i;
	for (i=0; i<m; i++) 
		ret.push_back(_dict[P[i]]);

	return ret;
}

void ConvexHull2D::print_hull(double **P, int m)
{
	int i;
	for (i=0; i<m; i++) 
		printf("%d ", _dict[P[i]]);
	printf("\n");
}

int ConvexHull2D::ccw(double **P, int i, int j, int k)
{
	double	a = P[i][0] - P[j][0],
			b = P[i][1] - P[j][1],
			c = P[k][0] - P[j][0],
			d = P[k][1] - P[j][1];
	return a*d - b*c <= 0;	   /* true if points i, j, k counterclockwise */
}

int cmpl(const void *a, const void *b)
{
	double v; 
	v = (*(double**)a)[0] - (*(double**)b)[0];
	if (v>0) return 1;
	if (v<0) return -1;

	v = (*(double**)b)[1] - (*(double**)a)[1];
	if (v>0) return 1;
	if (v<0) return -1;

	return 0;
}

int cmph(const void *a, const void *b)
{
	return cmpl(b,a);
}

int ConvexHull2D::make_chain(double** V, int n, int (*cmp)(const void*, const void*))
{
	int i, j, s = 1;
	double* t;

	qsort(V, n, sizeof(double*), cmp);
	for (i=2; i<n; i++) {
		for (j=s; j>=1 && ccw(V, i, j, j-1); j--){}
		s = j+1;
		t = V[s]; V[s] = V[i]; V[i] = t;
	}
	return s;
}

int ConvexHull2D::ch2d(double **P, int n)
{
	int u = make_chain(P, n, cmpl);		/* make lower hull */
	if (!n) return 0;
	P[n] = P[0];
	return u+make_chain(P+u, n-u+1, cmph);	/* make upper hull */
}

std::vector <int> ConvexHull2D::work()
{
	//print_hull(_P, ch2d(_P, _size));
	return get_hull(_P, ch2d(_P, _size));
}
