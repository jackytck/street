#include "LaserCamera.h"
#include "LaserCameraInfo.h"
#include <cstdlib>
#include <iostream>

LaserCamera::LaserCamera()
{
}

void LaserCamera::set_id_type(int id, int type)
{
    _id = id;
    _type = type;
}

void LaserCamera::set_projection(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l)
{
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

void LaserCamera::set_projection(const BDLCamera& cam)
{
    float a = cam._projection[0][0];
    float b = cam._projection[0][1];
    float c = cam._projection[0][2];
    float d = cam._projection[0][3];
    float e = cam._projection[1][0];
    float f = cam._projection[1][1];
    float g = cam._projection[1][2];
    float h = cam._projection[1][3];
    float i = cam._projection[2][0];
    float j = cam._projection[2][1];
    float k = cam._projection[2][2];
    float l = cam._projection[2][3];

    set_projection(a, b, c, d, e, f, g, h, i, j, k, l);
}

void LaserCamera::set_parameters(float fx, float fy, float cx, float cy, float k1, float k2, float fov_max, float k3, float p1, float p2)
{
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _k1 = k1;
    _k2 = k2;
    _fov_max = fov_max;

    if(_type == 0)
    {
        _k3 = k3;
        _p1 = p1;
        _p2 = p2;
    }
}

void LaserCamera::set_parameters(int info)
{
    if(info >= 0 && info <= 8)
    {
        float fx, fy, cx, cy, k1, k2, fov_max, k3, p1, p2;
        LaserCameraInfo::parameters(info, fx, fy, cx, cy, k1, k2, fov_max, k3, p1, p2);
        set_parameters(fx, fy, cx, cy, k1, k2, fov_max, k3, p1, p2);
    }
}

void LaserCamera::set_img_size(int w, int h)
{
    _w = w;
    _h = h;
}

bool LaserCamera::inverse3x3(float **m)
{
	float det = m[0][0]*(m[1][1]*m[2][2]-m[2][1]*m[1][2]) - m[1][0]*(m[0][1]*m[2][2]-m[2][1]*m[0][2]) + m[2][0]*(m[0][1]*m[1][2]-m[1][1]*m[0][2]);
	if(det == 0)
        return false;

	float **mi = (float**) calloc (3, sizeof(float*));
	for(int i=0; i<3; i++)
		mi[i] = (float *) calloc (3, sizeof(float));
	mi[0][0] = m[1][1]*m[2][2]-m[2][1]*m[1][2];
	mi[0][1] = m[0][2]*m[2][1]-m[2][2]*m[0][1];
	mi[0][2] = m[0][1]*m[1][2]-m[1][1]*m[0][2];
	mi[1][0] = m[1][2]*m[2][0]-m[2][2]*m[1][0];
	mi[1][1] = m[0][0]*m[2][2]-m[2][0]*m[0][2];
	mi[1][2] = m[0][2]*m[1][0]-m[1][2]*m[0][0];
	mi[2][0] = m[1][0]*m[2][1]-m[2][0]*m[1][1];
	mi[2][1] = m[0][1]*m[2][0]-m[2][1]*m[0][0];
	mi[2][2] = m[0][0]*m[1][1]-m[1][0]*m[0][1];
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			m[i][j] = mi[i][j]/det;
	for(int i=0; i<3; i++)
		free(mi[i]);
	free(mi);
    return true;
}

osg::Vec2 LaserCamera::distort(float x_dir, float y_dir)
{
    osg::Vec2 ret(-23418715.0f, -23418715.0f);

    //perspective
    if(_type == 0)
    {
        float r_max = tan(_fov_max / 360.0 * M_PI);
        float r2_max = r_max * r_max;

        float x2 = x_dir * x_dir;
        float y2 = y_dir * y_dir;
        float r2 = x2 + y2;

        if(r2 > r2_max) //will not project
            return ret;

        float xy = x_dir * y_dir;

        float dist_scale = 1 + _k1 * r2 + _k2 * r2 * r2 + _k3 * r2 * r2 *r2;
        float x_distorted = dist_scale * x_dir + 2 * _p1 * xy + _p2 * (r2 + 2 * x2);
        float y_distorted = dist_scale * y_dir + _p1 * ( r2 + 2 * y2 ) + 2 * _p2 * xy;

        float x_raw = _fx * x_distorted + _cx;
        float y_raw = _fy * y_distorted + _cy;

        ret.x() = x_raw;
        ret.y() = y_raw;
    }
    //fisheye
    else if(_type == 1)
    {
        //float r_max = tan(_fov_max / 360.0 * M_PI);
        //float r2_max = r_max * r_max;

        float r2 = x_dir * x_dir + y_dir * y_dir;
        float r = sqrt(r2);

        float theta_tilde = 1.0f;
        if(r > 1e-8)
        {
            float theta = atan(r);
            theta_tilde = theta * (1+ _k1*theta*theta + _k2*theta*theta*theta*theta) / r;
        }

        float x_distorted = theta_tilde * x_dir;
        float y_distorted = theta_tilde * y_dir;

        float x_raw = _fx * x_distorted + _cx;
        float y_raw = _fy * y_distorted + _cy;

        ret.x() = x_raw;
        ret.y() = y_raw;
    }

    return ret;
}

osg::Vec2 LaserCamera::project(osg::Vec3 P)
{
    osg::Vec2 ret(-23418715.0f, -23418715.0f);

    float a = _projection[0][0];
    float b = _projection[0][1];
    float c = _projection[0][2];
    float d = _projection[0][3];
    float e = _projection[1][0];
    float f = _projection[1][1];
    float g = _projection[1][2];
    float h = _projection[1][3];
    float i = _projection[2][0];
    float j = _projection[2][1];
    float k = _projection[2][2];
    float l = _projection[2][3];

    //1. project
    osg::Vec3 p;
    p.x() = a * P.x() + b * P.y() + c * P.z() + d;
    p.y() = e * P.x() + f * P.y() + g * P.z() + h;
    p.z() = i * P.x() + j * P.y() + k * P.z() + l;

    osg::Vec2 uv(p.x() / p.z(), p.y() / p.z());

    //2. distort
    float **K = (float**) calloc (3, sizeof(float*));
	for(int i=0; i<3; i++)
		K[i] = (float *) calloc (3, sizeof(float));
	K[0][0] = _fx; K[0][1] =   0; K[0][2] = _cx; 
	K[1][0] =   0; K[1][1] = _fy; K[1][2] = _cy; 
	K[2][0] =   0; K[2][1] =   0; K[2][2] =   1; 

	if(inverse3x3(K)) //if non-singular
    {
        osg::Vec3 dir;
        dir.x() = K[0][0] * uv.x() + K[0][1] * uv.y() + K[0][2];
        dir.y() = K[1][0] * uv.x() + K[1][1] * uv.y() + K[1][2];
        dir.z() = K[2][0] * uv.x() + K[2][1] * uv.y() + K[2][2];

        float x_dir = dir.x() / dir.z();
        float y_dir = dir.y() / dir.z();

        ret = distort(x_dir, y_dir);
    }

	for(int i=0; i<3; i++)
		free(K[i]);
	free(K);

    return ret;
}

osg::Vec2 LaserCamera::project2tex(osg::Vec3 P)
{
    osg::Vec2 ret = project(P);

    //origin at lower-left corner
    ret.x() /= _w;
    ret.y() = _h - ret.y();
    ret.y() /= _h;
    
    //cap
    if(ret.x() > 1.0f)
        ret.x() = 1.0f;
    if(ret.y() > 1.0f)
        ret.y() = 1.0f;

    return ret;
}
