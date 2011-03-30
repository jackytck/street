#ifndef __ADVANCEDVOLUME__H
#define __ADVANCEDVOLUME__H

#include <osg/Vec3>
#include <vector>
#include <map>

/** To complement the SimpleVolume (which is integer based) by 
  * using a dictionary and arbitrary step sizes.
  * This is primary used for constructing Billboard leaves
  * Usage:
  * 1. AdvancedVolume()
  * 2. cg()
  * 3. get_non_empty_voxels()
  */
class AdvancedVolume
{
    public:
        /* setup the volume by 'pts' and number of 'step' on each side
         * of the bounding box of the points
         */
        AdvancedVolume(int step, std::vector <osg::Vec3> pts);

        /* clear this instance
         */
        void clear();

        /* get the cg of the raw points
         * return (-23418.715f, -23418.715f, -23418.715f) if _points is empty
         */
        osg::Vec3 cg();

        /* get back the world coords of all non-empty voxels
         */
        std::vector <osg::Vec3> get_non_empty_voxels();

        /* used to rank the 3D points in dict
         */
        int key(osg::Vec3 p);

        /* given a key, return the 3D position
         * return (-23418.715f, -23418.715f, -23418.715f) if key is invalid
         */
        osg::Vec3 unkey(int k);

        int _step;//input step, number of step on each side
        std::vector <osg::Vec3> _points;//input points
        osg::Vec3 _origin;//left-out-bottom corner
        osg::Vec3 _voxel;//step size of each voxel
        float _right, _left, _top, _bottom, _in, _out;//bounding metrics
        int _upper;//the largest key in value
        std::map <int, bool> _voxel_dict_visited;//tell which voxel is visited
};

#endif
