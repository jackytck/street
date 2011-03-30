#ifndef __CONVEXHULL2D_H_
#define __CONVEXHULL2D_H_

#include <vector>
#include <map>
#include <osg/Group>

/* Usage:
 * 1. load_points()
 * 2. work()
 * 3. get_hull()
 */

class ConvexHull2D
{
	public:
		ConvexHull2D();
		~ConvexHull2D();
		int load_points(std::vector <osg::Vec3> pts);
		int load_points(std::vector <osg::Vec2> pts);
		std::vector <int> work();
		std::vector <int> get_hull(double **P, int m);
		void print_hull(double **P, int m);

	protected:
		inline int ccw(double **P, int i, int j, int k);
		inline int make_chain(double** V, int n, int (*cmp)(const void*, const void*));
		inline int ch2d(double **P, int n);

	private:
		int _size;
		double **_points, **_P;
		std::map <double *, int> _dict;
};

#endif
