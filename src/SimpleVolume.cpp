#include "SimpleVolume.h"
#include <stdio.h>

SimpleVolumeIndex::SimpleVolumeIndex(int x, int y, int z): _vx(x), _vy(y), _vz(z)
{
}

void SimpleVolumeIndex::bound(int maxX, int maxY, int maxZ)
{
    if(_vx < 0)
        _vx = 0;
    if(_vy < 0)
        _vy = 0;
    if(_vz < 0)
        _vz = 0;

    if(_vx >= maxX)
        _vx = maxX-1;
    if(_vy >= maxY)
        _vy = maxY-1;
    if(_vz >= maxZ)
        _vz = maxZ-1;
}

bool SimpleVolumeIndex::is_equal(SimpleVolumeIndex index)
{
    bool ret = false;
    
    if(index._vx == _vx && index._vy == _vy && index._vz == _vz)
        ret = true;

    return ret;
}

SimpleVolume::SimpleVolume(int sizeX, int sizeY, int sizeZ, int stepX, int stepY, int stepZ): _sizeX(sizeX), _sizeY(sizeY), _sizeZ(sizeZ), _stepX(stepX), _stepY(stepY), _stepZ(stepZ)
{
    clear();
}

void SimpleVolume::clear()
{
    intVector x(_sizeX/_stepX, 0);
    int2dVector y(_sizeY/_stepY, x);
    int3dVector z(_sizeZ/_stepZ, y);
    _box = z;
}

SimpleVolumeIndex SimpleVolume::add_point(float x, float y, float z)
{
    SimpleVolumeIndex ret(-1, -1, -1);

    if(check_bounds(x, y, z))
    {
        _box[int(x)/_stepX][int(y)/_stepY][int(z)/_stepZ]++;

        ret._vx = int(x)/_stepX;
        ret._vy = int(y)/_stepY;
        ret._vz = int(z)/_stepZ;
    }

    return ret;
}

int SimpleVolume::count(float x, float y, float z)
{
    int ret = -1;

    if(check_bounds(x, y, z))
        ret = _box[int(x)/_stepX][int(y)/_stepY][int(z)/_stepZ];

    return ret;
}

int SimpleVolume::count(SimpleVolumeIndex index)
{
    int ret = 0;

    index.bound(_sizeX / _stepX, _sizeY / _stepY, _sizeZ / _stepZ);
    ret = _box[index._vx][index._vy][index._vz];

    if(ret < 0)
        printf("SimpleVolume: hihi, count(%d) < 0\n", ret);

    return ret;
}

bool SimpleVolume::is_empty(float x, float y, float z)
{
    bool ret = true;

    if(count(x, y, z) > 0)
        ret = false;

    return ret;
}

bool SimpleVolume::is_empty(SimpleVolumeIndex index)
{
    int cnt = count(index);
    return cnt > 0 ? false : true;
}

void SimpleVolume::reset(float x, float y, float z)
{
    if(check_bounds(x, y, z))
    {
        _box[int(x)/_stepX][int(y)/_stepY][int(z)/_stepZ] = 0;
    }
}

bool SimpleVolume::check_bounds(float x, float y, float z)
{
    bool ret = false;

    if(x >= 0 && x < _sizeX && y >= 0 && y < _sizeY && z >= 0 && z < _sizeZ)
        ret = true;

    if(!ret)
        printf("SimpleVolume:outbound:p(%f,%f,%f) size(%d,%d,%d)\n", x, y, z, _sizeX, _sizeY, _sizeZ);

    return ret;
}
