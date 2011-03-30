#ifndef __LASERCAMERAINFO_H
#define __LASERCAMERAINFO_H

/* A class hard-coded for providing the laser camera info
 * Usage:
 * 1. parameters()
 */
class LaserCameraInfo
{
    public:
        LaserCameraInfo();

        /* get the parameters of the n-th camera
         * return -23418715.0f if no value for a parameter
         */
        static void parameters(int n, float& fx, float& fy, float& cx, float& cy, float& k1, float& k2, float& fov_max, float& k3, float& p1, float& p2);
};

#endif
