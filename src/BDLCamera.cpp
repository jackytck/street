#include "BDLCamera.h"

BDLCamera::BDLCamera(int id, float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l)
{
    _id = id;
    _projection[0][0] = a;
    _projection[0][1] = b;
    _projection[0][2] = c;
    _projection[0][3] = d;
    _projection[1][0] = e;
    _projection[1][1] = f;
    _projection[1][2] = g;
    _projection[1][3] = h;
    _projection[2][0] = i;
    _projection[2][1] = j;
    _projection[2][2] = k;
    _projection[2][3] = l;
}
