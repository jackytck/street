#ifndef __IMAGECONTOUR_H
#define __IMAGECONTOUR_H

#include <QtGui>
#include <vector>
#include <string>
#include <osg/Group>
#include "BDLPointGraph.h"
#include "SimpleVolume.h"

/* To represent a 2d segmentation of a tree or anything.
 * This 2d region is already enough for bounding a 3d shape by thinking of it as revolving.
 * Assume one image has one tree with one root
 * Usage: 
 * 1. create ImageContour, 
 * 2. set_segmentation()
 * 3. set_root() 
 * 4. set_scale()
 * 5. set_surface()
 * 6. call validated()
 * 7. query by isInside(osg::Vec3)
 */
class ImageContour
{
    public:
        ImageContour();
        ~ImageContour();

        /* clear the bounds
         */
        void clear();

        /* the path to the segmentation file
         */
        void set_segmentation(QString path);

        /* the image coordinates of the root
         */
        void set_root(int x, int y);

        /* set the conversion scale from query space to original image space
         * note: scale is the float k_d = mb_length_d / mb_length; in ImageGrower::construct_relative_bound
         * and must be positive
         */
        void set_scale(float s);

        /* set the voxel surface of the volume used for pruning
         */
        void set_surface(std::vector <osg::Vec3> surface, osg::Vec3 cg);

        /* the 3d version, query point is transformed to the canonical plane
         * query point uses the osg convention
         */
        bool isInside(osg::Vec3 q);

        /* the same as the osg::Vec3 version
         */
        bool isInside(BDLSkeletonNode *node);

        /* test if is inside by orthogonal projection
         */
        bool isInsideOrtho(osg::Vec3 q);

        /* test by a volume's surface
         * by constructing a line from cg to q, then find the points with the smallest distance,
         * and test if that point comes before or after q
         * note: this method requires the set_surface() to be called beforehand
         * it is only used by SkeletonGrower to prune tree globally, since it is too restrictive
         */
        bool isInsideVolume(osg::Vec3 q);

        /* set this instance as valid
         */
        void validated();

        /* ask if this instance is valid
         */
        bool isValid();

        /* transform a 3d point in viewer's space to the original image space
         * note: the point needs to be within the image bound
         */
        QPoint obj_to_img_space(osg::Vec3 v);

        /* find the perpendicular distance from a point to a line in 3D
         * a line is formed by a and b, c is the query point
         * return -1.0f if a == b
         */
        static inline float perp_dist(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c);

    protected:
        /* inside outside test by simply probing if the point under test is zero or not
         * (x, y): image coordinates of _segmentation
         * note: a 2d helper function for the 3d version
         * note: non-zero is inside
         */
        bool isInside(float x, float y);

    private:
        QPoint _root;
        float _scale;
        QImage _segmentation;

        //indicate if this instance is valid or not, by default false
        bool _valid;

        std::vector <osg::Vec3> _surface;//for pruning by volume
        osg::Vec3 _cg;//the cg of the surface cloud, no use now
        SimpleVolume _volume;//as a 3D look up table 
};

#endif
