#ifndef __SINGLEIMAGEPALM__H
#define __SINGLEIMAGEPALM__H

#include <vector>
#include <osg/Vec3>
#include <osg/Vec2>
#include <string>
#include <QImage>
#include <QPainter>
#include "BDLPointGraph.h"

/* A helper class for doing bfs in the segmentation.
 */
class ImageNode
{
    public:
        ImageNode(int x, int y): _pos(x, y), _prev(-1, -1), _dist(0.0f), _valid(false), _considered(false), _king(NULL)
        {
        }
        ImageNode(int x, int y, int px, int py, float d): _pos(x, y), _prev(px, py), _dist(d), _valid(false), _considered(false), _king(NULL)
        {
        }
        osg::Vec2 _pos;
        osg::Vec2 _prev;
        float _dist;
        bool _valid;//true if is inside segmentation
        bool _considered;//for kingdom selection
        int _king;//king id stored in SingleImagePalm
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
        void airbrush(int x, int y, int w = 10, int h = 10, QColor color = Qt::red);

        /*
         * for debugging and illustration purpose only
         */
        void drawLine(int x1, int y1, int x2, int y2, QColor color = Qt::green, int size = 5);

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
         * helper function for floodFillAt() to determine if (x,y) is a valid, un-considered and
         * laying in the same bin
         * neighbor
         */
        inline ImageNode *isGoodNeighbor(int x, int y, int bin);

        /*
         * flood fill region
         * for inferring the connected component(s) in each bin
         */
        void floodFillAt(int x, int y, int label);

        /*
         * infer kingdom by finding the connected component(s) in each bin
         */
        void inferKingdom();

        /*
         * infer averaged skeleton node(king) of each kingdom
         */
        void inferKing();

        /*
         * infer skeleton from kingdom and king
         * note: this skeleton is directly obtained from flood filling regions in each bin
         * _raw_skeleton will be set
         */
        void inferRawSkeleton();

        /*
         * extract the main branch from the raw skeleton
         * note: this is the only reliable and robust info we can get from it
         * the result is stored in _skeleton
         */
        void extractMainBranch();

        /*
         * though robust, bfs from root and construct a skeleton does not work
         * so line sweep inside segmentation to get the main branch first
         * note: it goes from root to the top without stopping
         * the locus will be stored in _main_branch_locus
         */
        void lineSweep();

        /*
         * do convolution at a given point with a hard-coded mask
         */
        long long detectBranchingConvolution(int x, int y);

        /*
         * enlarge a pixel to 2x2, 3x3, 4x4, ..., etc
         * until it reaches the segmentation boundary
         */
        long long detectBranchingBlockFilling(int x, int y);

        /*
         * pick the best terminal node from result of lineSweep()
         * by convoluting with a special mask
         * result will be set in _first_branching_node
         */
        void inferBestTerminalNode();

        /*
         * ### Debug Visualizations ###
         */

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

        /*
         * visualize kingdom
         */
        void visualize_king();

        /*
         * visualize line sweep of main branch
         */
        void visualize_linesweep();

        /*
         * visualize skeleton
         * root: root of skeleton
         * show_node: print the nodes
         * show_edge: print the edges
         * color: color of edges
         */
        void visualize_skeleton(BDLSkeletonNode *root, bool show_node = true, bool show_edge = true, QColor color = Qt::green);

        /*
         * visualize the edges by using LSD
         */
        void visualize_edge();

    private:
        bool _verbose;
        bool _data_valid;
        QImage _img;
        QImage _seg;
        osg::Vec2 _root;
        osg::Vec2 _first_branching_node;
        QImage _debug_img;//for debugging purpose only
        int _w;
        int _h;
        float _max_dist;//max dist from root, for visualization purpose only
        int _max_bin;//max bin id
        int _max_kingdom;//max kingdom id
        std::vector <std::vector <ImageNode> > _nodes;
        std::vector <osg::Vec2> _kings;//map the i-th kingdom to the position of king
        BDLSkeletonNode *_raw_skeleton;
        BDLSkeletonNode *_skeleton;
        std::vector <osg::Vec2> _main_branch_locus;//set by lineSweep()
        std::vector <long long> _convolute_score;
        long long _max_convolute_score;
};

#endif
