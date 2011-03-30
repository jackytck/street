#include "SimpleVolumeFloat.h"
#include <stdio.h>

SimpleVolumeFloat::SimpleVolumeFloat(float sizeX, float sizeY, float sizeZ, float stepX, float stepY, float stepZ): _sizeX(sizeX), _sizeY(sizeY), _sizeZ(sizeZ), _stepX(stepX), _stepY(stepY), _stepZ(stepZ)
{
    clear();
}

void SimpleVolumeFloat::clear()
{
    floatVector x(_sizeX/_stepX, 0);
    float2dVector y(_sizeY/_stepY, x);
    float3dVector z(_sizeZ/_stepZ, y);
    _box = z;
}

SimpleVolumeIndex SimpleVolumeFloat::add_point(float x, float y, float z)
{
    SimpleVolumeIndex ret(-1, -1, -1);

    if(check_bounds(x, y, z))
    {
        _box[int(x/_stepX)][int(y/_stepY)][int(z/_stepZ)]++;

        ret._vx = int(x/_stepX);
        ret._vy = int(y/_stepY);
        ret._vz = int(z/_stepZ);
    }

    return ret;
}

SimpleVolumeIndex SimpleVolumeFloat::add_potential(float x, float y, float z, int charge, float radius)
{
    SimpleVolumeIndex ret(-1, -1, -1);

    if(check_bounds(x, y, z))
    {
		if(radius == -1.0f)
			radius = _stepX * 10.0f;

		for(float i=-radius; i<radius; i+=_stepX)
			for(float j=-radius; j<radius; j+=_stepY)
				for(float k=-radius; k<radius; k+=_stepZ)
				{
					float u = sqrt(i*i+j*j+k*k);
					if(u == 0.0f)
						continue;

					u = charge / u;

					int update_x = int((x+i)/_stepX);
					int update_y = int((y+j)/_stepY);
					int update_z = int((z+k)/_stepZ);

					if(update_x >= 0.0f && update_x < _sizeX && update_y >= 0.0f && update_y < _sizeY && update_z >= 0.0f && update_z < _sizeZ)
						_box[int((x+i)/_stepX)][int((y+j)/_stepY)][int((z+k)/_stepZ)] += u;
				}

        ret._vx = int(x/_stepX);
        ret._vy = int(y/_stepY);
        ret._vz = int(z/_stepZ);
    }

    return ret;
}

float SimpleVolumeFloat::count(float x, float y, float z) const
{
    float ret = -1.0f;

    if(check_bounds(x, y, z))
        ret = _box[int(x/_stepX)][int(y/_stepY)][int(z/_stepZ)];

    return ret;
}

float SimpleVolumeFloat::count(SimpleVolumeIndex index) const
{
    float ret = 0.0f;

    index.bound(_sizeX / _stepX, _sizeY / _stepY, _sizeZ / _stepZ);
    ret = _box[index._vx][index._vy][index._vz];

    return ret;
}

bool SimpleVolumeFloat::is_empty(float x, float y, float z)
{
    bool ret = true;

    if(count(x, y, z) > 0)
        ret = false;

    return ret;
}

bool SimpleVolumeFloat::is_empty(SimpleVolumeIndex index)
{
    int cnt = count(index);
    return cnt > 0 ? false : true;
}

void SimpleVolumeFloat::reset(float x, float y, float z)
{
    if(check_bounds(x, y, z))
    {
        _box[int(x/_stepX)][int(y/_stepY)][int(z/_stepZ)] = 0;
    }
}

bool SimpleVolumeFloat::check_bounds(float x, float y, float z) const
{
    bool ret = false;

    if(x >= 0.0f && x < _sizeX && y >= 0.0f && y < _sizeY && z >= 0.0f && z < _sizeZ)
        ret = true;

    if(!ret)
        printf("SimpleVolumeFloat:outbound:p(%f,%f,%f) size(%f,%f,%f)\n", x, y, z, _sizeX, _sizeY, _sizeZ);

    return ret;
}
