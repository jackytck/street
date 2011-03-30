#ifndef __TRIANGLE_H
#define __TRIANGLE_H

#include <osg/Vec3>

/* To represent a simple triangle for the icosahedron
 */
class Triangle
{
	public:
		Triangle(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c);
		Triangle(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz);

        osg::Vec3 _a, _b, _c;
};

#endif
