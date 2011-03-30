#ifndef __LEAFGROWER__H
#define __LEAFGROWER__H

#include "BDLPointGraph.h"
#include <osg/Vec3>
#include <osg/Vec2>
#include "SimplePruner.h"

/* A thin class for growing leafs, i.e. positions, texture coordinates,
 * normals of a plane leaves, for a given BDLSkeleton graph
 * Usage:
 * 1. LeafGrower()
 * 2. setup()
 * 3. grow_planes() //optional
 * 4. grow()
 * 5. billboard_data() //optional
 */
class LeafGrower
{
    public:
        LeafGrower();
        
        /* set the BDLSkeleton graph, in which where the leaves grow
         * root: the root node
         * isp0: the path to an image-segmentation pair
         * s: the scale (i.e. k_d) between the skeleton space and image space
		 * path: the clipped texture path
         * same as in SimplePruner
         */
        void setup(BDLSkeletonNode *root, std::string isp0, float s, std::string path = std::string("/tmp/texture.png"));

        /* setting the scale, for re-using this LeafGrower object
         */
        void set_scale(float s);

        /* ask if this instance is loaded, for caching
         */
        bool is_loaded();

        /* grow textured leaves
         * all_v: all the leaf vertices, four per leaf
         * return the corresponding texture coords for each vertex
         * note: len(ret) == len(all_v)
         */
        std::vector <osg::Vec2> grow(const std::vector <osg::Vec3>& all_v);

        /* grow the oriented leaf plane (all_v) along the skeleton
         * origin: the origin of the tree
         * height: the height of the tree
         * angle: rotated around the z-axis
         * maya: in maya coordinates
         * note: only suitable for generating a lot of leaves
         */
        //std::vector <osg::Vec3> grow_planes(osg::Vec3 origin, double height, int angle, bool maya);
        std::vector <osg::Vec3> grow_planes();

        /* return the points for the BillboardTree class
         * note: 
         */
        void billboard_data(std::vector <osg::Vec3>& all_pos, std::vector <osg::Vec3>& all_v, std::vector <osg::Vec2>& all_tex);

        //all the leaf vertices, four vertices per leaf
        std::vector <osg::Vec3> _all_pos; //set by grow_planes()
        std::vector <osg::Vec3> _all_v; //set by grow()
        std::vector <osg::Vec2> _all_tex; //set by grow()
        //cg of the cloud
        osg::Vec3 _cg;

    protected:
        /* skeleton to image space
         */
        inline osg::Vec2 obj_to_img_space(osg::Vec3 v);
        
        /* return the region where the normal is pointing to
         * return values are 0, 1, 2, 3, 4
         * 0: front 120'
         * 1: right 120'
         * 2: left 120'
         * 3: top
         * 4: bottom
         * -1: error
         */
        inline int which_quadrant(osg::Vec3 normal);

        inline std::vector <osg::Vec2> texture_coords_from_vertex(const std::vector <osg::Vec3>& all_v);

    private:
        //data members
        BDLSkeletonNode *_root;
        int _rootX, _rootY;
        float _scale;
        osg::Vec2 _bound_bottomLeft;
        int _bound_width, _bound_height;
        SimplePruner _pruner;
};

#endif
