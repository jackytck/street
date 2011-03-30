#ifndef __SIMPLEVOLUME__H
#define __SIMPLEVOLUME__H

#include <iostream>
#include <vector>
#include <math.h>
//#include <QtGui>

typedef std::vector<int> intVector;
typedef std::vector<intVector> int2dVector;
typedef std::vector<int2dVector> int3dVector;

/** for indexing the Volume
  */
class SimpleVolumeIndex
{
    public:
        SimpleVolumeIndex(int x, int y, int z);

        /* to bound the instance to avoid out of bound error
         */
        void bound(int maxX, int maxY, int maxZ);

        /* to check the equality of two given VolumeIndex
         */
        bool is_equal(SimpleVolumeIndex index);

        int _vx, _vy, _vz;
};

/** Only a stripped down version of Volume
  * for doing the most simpliest stuff
  */
class SimpleVolume
{
    public:
        SimpleVolume(int sizeX = 1000, int sizeY = 1000, int sizeZ = 1000, int stepX = 5, int stepY = 5, int stepZ = 5);

        void clear();

        /* increase the count of the appropriate voxel
         * return the newly added index
         */
        SimpleVolumeIndex add_point(float x, float y, float z);

        /* get the count in the voxel
         * return -1 if out of bounds
         * note: (x, y, z) is the actual world coords
         */
        int count(float x, float y, float z);
        int count(SimpleVolumeIndex index);

        /* for collision detection
         * return true is out of bounds
         */
        bool is_empty(float x, float y, float z);
        bool is_empty(SimpleVolumeIndex index);

        /* reset the count of this point to zero
         */
        void reset(float x, float y, float z);

        inline bool check_bounds(float x, float y, float z);

        int _sizeX, _sizeY, _sizeZ;
        int _stepX, _stepY, _stepZ;

        int3dVector _box;
};

#endif
