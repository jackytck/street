#ifndef __SIMPLEVOLUMEFLOAT__H
#define __SIMPLEVOLUMEFLOAT__H

#include <iostream>
#include <vector>
#include <math.h>
#include "SimpleVolume.h"

typedef std::vector <float> floatVector;
typedef std::vector <floatVector> float2dVector;
typedef std::vector <float2dVector> float3dVector;

/** Only a stripped down version of Volume
  * for doing the most simpliest stuff
  */
class SimpleVolumeFloat
{
    public:
        SimpleVolumeFloat(float sizeX = 1000.0f, float sizeY = 1000.0f, float sizeZ = 1000.0f, float stepX = 5.0f, float stepY = 5.0f, float stepZ = 5.0f);

        void clear();

        /* increase the count of the appropriate voxel
         * return the newly added index
         */
        SimpleVolumeIndex add_point(float x, float y, float z);

        /* add potential energy around the given point charge
		 * charge: 1 or -1
		 * radius: active region, default to 10 step sizes
         * return the index of the center point charge
         */
        SimpleVolumeIndex add_potential(float x, float y, float z, int charge, float radius = -1.0f);

        /* get the count in the voxel
         * return -1 if out of bounds
         * note: (x, y, z) is the actual world coords
         */
        float count(float x, float y, float z) const;
        float count(SimpleVolumeIndex index) const;

        /* for collision detection
         * return true is out of bounds
         */
        bool is_empty(float x, float y, float z);
        bool is_empty(SimpleVolumeIndex index);

        /* reset the count of this point to zero
         */
        void reset(float x, float y, float z);

        inline bool check_bounds(float x, float y, float z) const;

        float _sizeX, _sizeY, _sizeZ;
        float _stepX, _stepY, _stepZ;

        float3dVector _box;
};

#endif
