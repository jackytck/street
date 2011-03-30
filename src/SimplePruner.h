#ifndef __SIMPLEPRUNER_H
#define __SIMPLEPRUNER_H

#include <string>
#include <cv.h>
#include "BDLPointGraph.h"

/* A simplified version of the ImageContour. It is used to prune a tree
 * after each branch replacement by a binary image segmentaion.
 * Usage:
 * 1. SimplePruner()
 * 2. setup() // for setting segmentation, root and sclae
 * 3. set_scale() // optional, if another skeleton is used, scale can be re-set
 * 4. is_inside()
 */
class SimplePruner
{
    public:
        SimplePruner();
        ~SimplePruner();

        /* clear this instance
         */
        void clear();

        /* set the segmentation, root and scale
         */
        void setup(std::string path, int x, int y, float s);
        
        /* set isp0 and scale
         */
        void setup(std::string isp0, float s);

        /* load a segmentation for pruning,
         * this file should be the same for constructing the surface
         * return true if successful
         */
        bool set_segmentation(std::string path);

        /* the root in image coordinates
         */
        void set_root(int x, int y);

        /* set the conversion scale from query space to original image space
         * note: scale is the float k_d = mb_length_d / mb_length; 
         * and must be positive
         */
        void set_scale(float s);

        /* ask if this instance is loaded
         */
        bool is_loaded();

        /* query
         */
        bool is_inside(BDLSkeletonNode *node, bool lenient = false);
        bool is_inside(osg::Vec3 node, bool lenient = false);

        /* use orthogonal projection
         */
        bool is_inside_ortho(osg::Vec3 node, bool lenient = false);

    protected:
        inline int width();
        inline int height();

        /* x: col-th
         * y: row-th
         * origin(0,0) at the top-left corner
         */
        inline uchar read(int x, int y);

        /* 2d version
         */
        inline bool is_inside(float x, float y, bool lenient = false);

    private:
        IplImage *_segmentation;
        int _rootX, _rootY;
        float _scale;
};

#endif
