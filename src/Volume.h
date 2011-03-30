#ifndef __VOLUME__H
#define __VOLUME__H

#include <iostream>
#include <vector>
#include <math.h>
#include "BDLPointGraph.h"
#include <QtGui>
#include "ImageContour.h"

typedef std::vector<int> intVector;
typedef std::vector<intVector> int2dVector;
typedef std::vector<int2dVector> int3dVector;

/** for indexing the Volume
  */
class VolumeIndex
{
    public:
        VolumeIndex(int x, int y, int z);

        /* to bound the instance to avoid out of bound error
         */
        void bound(int maxX, int maxY, int maxZ);

        /* to check the equality of two given VolumeIndex
         */
        bool is_equal(VolumeIndex index);

        /* compute the Euclidean distance of two index
         * return -1.0f if error occurs
         */
        static float dist(VolumeIndex a, VolumeIndex b);

        int _vx, _vy, _vz;
};

/** 3D volume quantized by _stepX * _stepY * _stepZ
  * this is used for growing a BDLSkeletonNode,
  * for detecting self-collision and 
  * for growing more uniformly.
  * It also owns an ImageContour which helps finding the seeds inside the bound for lowest_density_3().
  */
class Volume
{
    public:
        Volume(int sizeX = 1000, int sizeY = 1000, int sizeZ = 1000, int stepX = 10, int stepY = 10, int stepZ = 10);

        void clear();

        /* store a reference of tree and vote it
         */
        void initialize_tree(BDLSkeletonNode *root);

        /* update the instance or count,
         * because the skeleton may have grown
         */
        void update_tree();

        /* increase the count of the appropriate voxel
         */
        void add_point(float x, float y, float z);

        /* overloaded for BDLSkeletonNode *
         * also a translation to the middle is added
         * assume the root of the tree is at the origin
         */
        void add_point(BDLSkeletonNode *node);

        /* overloaded for VolumeIndex
         * note: this is for raw index, i.e. positive
         */
        void add_point(VolumeIndex index);

        /* get the count in the voxel
         * return -1 if out of bounds
         * note: (x, y, z) is the actual world coords
         */
        int count(float x, float y, float z);

        /* overloaded for VolumeIndex
         */
        int count(VolumeIndex index);

        /* for collision detection
         * return true is out of bounds
         */
        bool is_empty(float x, float y, float z);

        /* overloaded for VolumeIndex
         */
        bool is_empty(VolumeIndex index);

        /* overloaded for BDLSkeletonNode *
         * return false if node == NULL
         * the translation is taken into consideration
         */
        bool is_empty(BDLSkeletonNode *node);

        /* vote the entire tree to _box
         * if a node has more than one children, then vote by n, n is the number of children of that node
         */
        void vote_tree(BDLSkeletonNode *root);

        /* compute the density of a t*t*t cube around the given voxel
         * (x, y, z) is the 3d coordinates
         * t is the number of voxel on one side
         * return -1.0f if error occurs
         */
        float density_at(float x, float y, float z, int t = 2);

        /* overloaed for BDLSkeletonNode *
         * the translation is taken into consideration
         */
        float density_at(BDLSkeletonNode *node, int t = 2);

        bool is_main_branch(BDLSkeletonNode *node);

        /* t-neighbor of the given voxel, bounded appropriately
         */
        std::vector <VolumeIndex> t_neighbor(VolumeIndex center, int t);

        /* find the 3 lowest density node, sorted in ascending order
         */
        std::vector <BDLSkeletonNode *> lowest_density_3();

        /* check if the potential new sub-tree would cause a collision with existing tree
         * this will NOT copy the element, coz supposed to be new by 'transform' or manually
         * return the non-intersecting sub-tree,
         * element: the transformed element of an element in the library
         */
        BDLSkeletonNode *intersection_cut(BDLSkeletonNode *element);

        /* return the voxel's index of a given 3d point,
         * this is used for constructing the outermost surface of a tree
         * return (-1, -1, -1) if it is out of bound
         * all returned index are raw, i.e. positive
         */
        VolumeIndex voxel_index(float x, float y, float z);

        /* visualize the given VolumeIndex as a point graph in osgviewer
         * note: all input index are raw index, i.e. all are positive, translation will be done inside this method
         */
        osg::ref_ptr <osg::Group> visualize_volume_index(std::vector <VolumeIndex> index, bool maya = false);

        /* visualize a Perlin displaced VolumeIndex, very similar to the above
         */
        static osg::ref_ptr <osg::Group> visualize_volume_index_perlin(std::vector <osg::Vec3> index, bool maya = false);

        /* overloaded for illustration purpose in paper only
         */
        osg::ref_ptr <osg::Group> visualize_volume_index_color(std::vector <osg::Vec3> index, std::vector <int> segments, bool maya = false);

        inline bool check_bounds(float x, float y, float z);

        int _sizeX, _sizeY, _sizeZ;
        int _stepX, _stepY, _stepZ;

        int3dVector _box;

        BDLSkeletonNode *_root;

        //for returning the in-bound seeds in lowest_density_3
        ImageContour _img_bound;

        //for updating the statusbar of mainwindow
        QStatusBar *_status_bar;
};

#endif
