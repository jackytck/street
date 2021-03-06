#ifndef __FASTCACHE__H
#define __FASTCACHE__H

#include <iostream>
#include <vector>
#include <math.h>
#include <osg/Vec3>

typedef std::vector<int> intVector;
typedef std::vector<intVector> int2dVector;
typedef std::vector<int2dVector> int3dVector;

/** In single tree modeling, the most-outer bound of the segmentation is used to construct a set of 3D energy points. This class FastCache constructs these points in an efficient way.
  * Usage:
  * 1. FastCache(relative_bounds, step);
  * 2. surface_points(...);
  */
class FastCache
{
    public:
        /*
         * relative_bounds: the relative bounds generated by ISPLoader
         * step: number of divisions per dimension
         */
        FastCache(std::vector <osg::Vec3> relative_bounds, int step = 20);

        /* infer the surface points
         */
        std::vector <osg::Vec3> surface_points();

        protected:
        /* infer width, heigth, depth and origin of the logical volume
         * bounds: the relative bounds from constructor
         * note: origin is defined at (minX, minY, minZ), so that everywhere is non-negative
         */
        void inferCacheBound();

        /* check if this instance is properly setup
         * return true is this instance is properly setup
         * note: voxel size is not checked
         */
        bool check_instance();

        /* map Vec3 to an int key for _voxel_dict
         * return -1 if error occurs
         * note: valid only if check_instance() returns true
         */
        inline int key(osg::Vec3 p);

        /* inverse of key()
         * return the center of the containing voxel
         */
        inline osg::Vec3 unkey(int k);

        /* find the center mass of a set of points
         */
        osg::Vec3 center_mass(std::vector <osg::Vec3> bounds);

        /* get the bound_metrics by the same implementation in ISPLoader
         */
        void bound_metrics(std::vector <osg::Vec3> rbound, osg::Vec3 cg, float& height, float& width_right, float& width_left);

    private:
        int _step;
        float _more_margin;//put more margins when inferring the boudns
        osg::Vec3 _origin;//r.w.t. the inputted root (r_x, r_y, 0)
        float _w, _h, _d;//width, height and depth of the virtual bounding volume
        osg::Vec3 _voxel;//store the dimensions of each voxel of the sub-divided volumes
        int _step_p1, _step_p2;//for perfomance only
        osg::Vec3 _cg;//cg of the relative bounds
        float _b_h, _w_r, _w_l;//bound_height, width-right, width-left
        std::vector <osg::Vec3> _rbounds;//relative bounds
};

#endif
