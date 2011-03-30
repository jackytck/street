#ifndef __ISPLOADER__H
#define __ISPLOADER__H

#include <vector>
#include <string>
#include <osg/Vec3>
#include <osg/Vec2>

/* A thin loader class to load an .isp0 file
 * and also conveniently construct the surface points
 * and a min_texture
 * Usage:
 * 1. ISPLoader()
 * 2. load()
 * 3. surface_points() (optional)
 * 3. construct_min_texture() (optional)
 */
class ISPLoader
{
    public:
        //methods
        ISPLoader();

        /* load one image path and one segmentation path and a root point
         */
        void load(std::string path);

        /* get the surface points implied by the segmentation and the rootpoint
         * mb_length_d: the lenght of the branch from the root to the closest point in cloud in the skeleton space
         * surface_points calls relative_contour_pts calls absolute_contour_pts
         */
        std::vector <osg::Vec3> surface_points(float mb_length_d, int v_step = -1, int v_size = 1000);

        /* from the segmentation, find the smallest bounding box,
         * and use it as a mask to get the minimum texture
         * path: the path where the new texture is saved
         * return:
         * cg: the (absolute) center of the image bound in image coords
         * bottomleft: the bottomleft corner of the bound
         * width: the bound width
         * height: the bound height
         * note: got to use qt since opencv does not support alpha channel
         */
        void construct_min_texture(std::string path, osg::Vec3& cg, osg::Vec2& bottomleft, int& width, int& height);

        //members
        std::string _img_path;
        std::string _seg_path;
        int _rootX, _rootY;
        std::vector <osg::Vec3> _surface_pts;
        std::vector <osg::Vec3> _relative_pts;
        float _k_d; //valid after calling surface_points()

    protected:
        inline std::vector <osg::Vec3> absolute_contour_pts();
        inline std::vector <osg::Vec3> relative_contour_pts(float mb_length_d);
        inline osg::Vec3 center_mass(std::vector <osg::Vec3> bounds);
        inline void bound_metrics(std::vector <osg::Vec3> rbound, osg::Vec3 cg, float& height, float& width_right, float& width_left);
};

#endif
