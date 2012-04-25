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
        ImageNode(int x, int y): _pos(x, y), _prev(-1, -1), _dist(0.0f), _valid(false), _considered(false)
        {
        }
        ImageNode(int x, int y, int px, int py, float d): _pos(x, y), _prev(px, py), _dist(d), _valid(false), _considered(false)
        {
        }
        osg::Vec2 _pos;
        osg::Vec2 _prev;
        float _dist;
        bool _valid;//true if is inside segmentation
        bool _considered;//for kingdom selection
        int _bin;
        int _kingdom;
        std::vector <ImageNode *> _children;//pointers static references
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

        /* setup children of each node
         */
        void setupChildren();

        /*
         * bfs from root and construct a bfs-tree
         */
        void bfs();

        /*
         * after getting the bfs distance maps, segment it into different bins
         * by packing ranges of dists in bins
         * divide: larger than zero, length of each bin is equal to _max_dist / division
         * note: _max_bin will be set
         */
        void assignBin(int divide = 30);

        /*
         * label all the nodes from subtree rooted at (x,y) to label
         */
        void labelSubtreeAt(int x, int y, int label);

        /*
         * infer kingdom by finding the connected component(s) in each bin
         */
        void inferKingdom();

        /*
         * visualize bfs distance as scalar field
         */
        void visualize_bfs();

        /*
         * visualize bins
         */
        void visualize_bin();

        /*
         * visualize kingdom
         */
        void visualize_kingdom();

        void visualize_children();

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
        int _max_bin;//max bin id
        int _max_kingdom;//max kingdom id
        std::vector <std::vector <ImageNode> > _nodes;
};

#endif
