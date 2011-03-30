#ifndef __VOLUMESURFACE__H
#define __VOLUMESURFACE__H

#include <vector>
#include <string>
#include <osg/Vec3>

typedef std::vector<int> intVector;
typedef std::vector<intVector> int2dVector;
typedef std::vector<int2dVector> int3dVector;

/** A thin class for loading and exporting the surface points.
  * A typical scenario is to first get all the surface points from a segmentation, then store it, and reload it for later usage.
  * note: the surface points depend on the segmentation + the root point, so an isp0 is used to load things
  */
class VolumeSurface
{
    public:
        //methods
        VolumeSurface();

        /* load either a segmentation or a text file
         * if path is *.isp0, then load as isp0,
         * else load as text file
         * mb_lenght_d: the main branch length of the default skeleton
         */
        float load(std::string path, float mb_lenght_d = -1.0f);

        /* store back the computed surface_pts from segmentation and root point to file
         * the scheme is as follow:
         * first line is the number of points n
         * the next n lines contain (%f %f %f) on each line
         */
        void save(std::string path = "");

        /* clear all the data members of this instance
         */
        void clear();

        /* does this instance loaded?
         */
        bool is_loaded();

        //members
        std::vector <osg::Vec3> _surface_pts;
        float _k_d;
        std::vector <osg::Vec3> _relative_pts;

    protected:
        std::string _save_name;
};

#endif
