#ifndef __SINGLEIMAGEPALM__H
#define __SINGLEIMAGEPALM__H

#include <vector>
#include <osg/Vec3>
#include <osg/Vec2>
#include <string>
#include <QImage>
#include <QPainter>

/* A helper class for doing bfs in the segmentation.
 */
class ImageNode
{
    public:
        ImageNode(int x, int y): _pos(x, y), _prev(-1, -1), _dist(0.0f)
        {
        }
        ImageNode(int x, int y, int px, int py, float d): _pos(x, y), _prev(px, py), _dist(d)
        {
        }
        osg::Vec2 _pos;
        osg::Vec2 _prev;
        float _dist;
};

/* A class to model a palm tree from an image-segmentation-pari.
 * Flow: refer to grow()
 *
 * Usage:
 * 1. SingleImagePalm()
 * 2. grow()
 */
class SingleImagePalm
{
    public:
        /*
         * isp0: the image and segmentation pair
         */
        SingleImagePalm(std::string isp0);
        ~SingleImagePalm();

        /*
         * set if to print debug msg and other related info
         */
        void setVerbose(bool debug);

        /*
         * 1. Find the root of the palm from segmentation
         * 2. bfs the whole segmentation
         */
        void grow();

    protected:
        /*
         * for debugging and illustration purpose only
         * x, y: coordinates, upper left is origin
         * w, h: width, height of ellipse
         * color: color of ellipse
         */
        void airbrush(int x, int y, int w = 50, int h = 50, QColor color = Qt::red);

        /*
         * for debugging and illustration purpose only
         */
        void drawLine(int x1, int y1, int x2, int y2);

        /*
         * test if a given point is inside the segmentation
         */
        inline bool isInside(int x, int y);

        /*
         * map a scalar value to a hue
         * value: [0,1]
         * return the mapped color
         */
        inline QColor mapColor(float v);

        /*
         * find the root of palm from its segmentation
         * return true if the root can be found
         */
        bool findRoot();

        /*
         * bfs from root and construct a bfs-tree
         * if in verbose mode, debug image will be drawn when bfs() is running at the 2nd time
         */
        void bfs();

    private:
        bool _verbose;
        bool _data_valid;
        QImage _img;
        QImage _seg;
        osg::Vec2 _root;
        QImage _debug_img;//for debugging purpose only
        int _w;
        int _h;
        float _max_dist;//max dist from root, for visualization purpose only
};

#endif
