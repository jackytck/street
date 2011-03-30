#include "Triangle.h"

Triangle::Triangle(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c): _a(a), _b(b), _c(c)
{
}

Triangle::Triangle(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz): _a(ax, ay, az), _b(bx, by, bz), _c(cx, cy, cz)
{
}
