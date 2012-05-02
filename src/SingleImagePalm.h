#ifndef __SINGLEIMAGEPALM__H
#define __SINGLEIMAGEPALM__H

#include <vector>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>
#include <string>
#include <QImage>
#include <QPainter>
#include "BDLPointGraph.h"

/* A helper class for doing bfs in the segmentation.
 */
class ImageNode
{
    public:
        ImageNode(int x, int y): _pos(x, y), _prev(-1, -1), _dist(0.0f), _valid(false), _considered(false), _king(NULL), _kingdom(-1)
        {
        }
        ImageNode(int x, int y, int px, int py, float d): _pos(x, y), _prev(px, py), _dist(d), _valid(false), _considered(false), _king(NULL), _kingdom(-1)
        {
        }
        const bool operator < (const ImageNode&) const;
        osg::Vec2 _pos;
        osg::Vec2 _prev;
        float _dist;
        bool _valid;//true if is inside segmentation
        bool _considered;//for kingdom selection
        int _king;//king id stored in SingleImagePalm
        int _kingdom;
        int _bin;
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
         * img: the image to draw
         */
        void airbrush(int x, int y, int w = 10, int h = 10, QColor color = Qt::red, QImage *img = NULL);

        /*
         * for debugging and illustration purpose only
         */
        void drawLine(int x1, int y1, int x2, int y2, QColor color = Qt::green, int size = 5, QImage *img = NULL);

        /*
         * test if a given point is inside the segmentation
         */
        inline bool isInside(int x, int y);

        /*
         * test if a given point is an edge point
         */
        inline bool isEdge(int x, int y);

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
         * dijkstra from the first branching node and construct a dijkstra-tree
         */
        void dijkstra();

        /*
         * after getting the dijkstra distance maps, segment it into different bins
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
         * strong: true if strong connection is desired
         * return the populaton of this kingdom
         */
        long long floodFillAt(int x, int y, int label, bool strong = false);

        /*
         * infer kingdom by finding the connected component(s) in each bin
         */
        void inferKingdom();

        /*
         * infer averaged skeleton node(king) of each kingdom
         */
        void inferKingAvg();

        /*
         * infer king who has the highest potential
         */
        void inferKingPotential();

        /*
         * infer skeleton from kingdom and king
         * note: this skeleton is directly obtained from flood filling regions in each bin
         * _raw_skeleton will be set
         */
        void inferRawSkeleton();

        /*
         * extract the main branch from the raw skeleton
         * note: this is the only reliable and robust info we can get from raw branch
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
         * given a point, find the closest point in segmentation to this point
         * return (rx, ry)
         */
        void closestPoint(int x, int y, int& rx, int& ry);

        /*
         * vote the voting space by the given charge
         * (x,y): position of charge
         * radius: the radius of effect of the charge
         * positive: true if it is a positive charge
         */
        inline void voteSingleCharge(int x, int y, float radius, bool positive);

        /*
         * produce edges + widths by LSD for computation and visualization
         * note: must be called after dijkstra()
         */
        void produceLineEdge();

        /*
         * convolute with the edge map
         * higher return value indicates higher number of edges
         * note: outdated
         */
        long long convoluteEdgeMap(int x, int y);

        /*
         * pick the best terminal node from result of lineSweep()
         * by convoluting with a special mask
         * result will be set in _first_branching_node
         */
        void inferBestTerminalNode();

        /*
         * sample and construct the main branch from line swept points
         * this replaces the original extractMainBranch()
         * result is stored in _skeleton
         */
        void extractMainBranch2();

        /*
         * disqualify all the kingdoms that overlap with the main branch
         */
        void dqMainbranchKingdom();

        /*
         * compute the socre of this path
         * a: first control point
         * b: second control point
         * c: third control point
         * d: forth control point
         * k: number of interpolation
         */
        float computePathScore(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2 d, int k = 1000);

        /*
         * return a list of kingdoms that overlap with the dijkstra's path
         * transversed from the query point to the root
         * return the sorted kingdom id in increasing order
         */
        std::vector <int> overlappedKingdom(osg::Vec2 query);

        /*
         * get a list of nodes on the dijkstra's path
         * that are closest to the list of query distances
         * leaf: the leaf or terminal node
         * percents: fraction of distance from leaf to root
         */
        std::vector <osg::Vec2> getRetracement(osg::Vec2 leaf, std::vector <float> percents);

        /*
         * determine if the query point is properly oriented
         * with respect to the center, usually be to the first branching point
         * center: the other end-point of the line
         */
        bool isWellOriented(osg::Vec2 leaf, osg::Vec2 query, osg::Vec2 center);

        /*
         * return a circular zone of interest around the central pixel
         * center: center of the zone
         * radius: radius of the zone
         * step: separation between two points in zone
         */
        inline std::vector <osg::Vec2> circularZone(osg::Vec2 center, float radius, int step = 5);

        /*
         * extract a single sub-branch by letting the 4th control point be living in the farest kingdom
         * and exhaustively find out the other 2 control point
         * return false if no branch is extracted
         */
        bool extractSingleSubBranch();

        /*
         * ### Debug Visualizations ###
         */

        /*
         * visualize bfs distance as scalar field
         */
        void visualize_dijkstra();

        /*
         * visualize bins
         */
        void visualize_bin();

        /*
         * visualize kingdom
         * show_all: true to show all, else only show the available state according to _kingdom_states
         */
        void visualize_kingdom(bool show_all = true);

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
         * node_color: color of nodes
         * edge_color: color of edges
         */
        void visualize_skeleton(BDLSkeletonNode *root, bool show_node = true, bool show_edge = true, QColor node_color = Qt::magenta, QColor edge_color = Qt::black);

        /*
         * visualize the edges by using LSD
         * thin_edge: true if to draw a thin edge
         */
        void visualize_edge(bool thin_edge = false, QImage *img = NULL);

        /*
         * visualize search limits for branching node
         */
        void visualize_branch_search_limit();

        /*
         * visualize polarity
         */
        void visualize_polarity();

        /*
         * visualize the voting space
         */
        void visualize_voting_space();

    private:
        bool _verbose;
        bool _data_valid;
        QImage _img;
        QImage _seg;
        osg::Vec2 _root;
        osg::Vec2 _first_branching_node;
        int _first_branching_node_idx;//index to _main_branch_locus
        QImage _debug_img;//for debugging purpose only
        int _w;
        int _h;
        float _max_dist;//max dist from root, for visualization purpose only
        int _max_bin;//max bin id
        int _max_kingdom;//max kingdom id
        std::vector <std::vector <ImageNode> > _nodes;
        std::vector <osg::Vec2> _kings;//map the i-th kingdom to the position of king
        std::vector <long long> _population;//i-th kingdom has population _population[i]
        BDLSkeletonNode *_raw_skeleton;
        BDLSkeletonNode *_skeleton;
        BDLSkeletonNode *_branching;//points to the first branching node
        std::vector <osg::Vec2> _main_branch_locus;//set by lineSweep()
        std::vector <long long> _convolute_score;
        long long _max_convolute_score;
        std::vector <osg::Vec4> _edges;
        std::vector <float> _edge_widths;
        std::vector <bool> _edge_polarity;//true if (_edges[i].x(),_edges[i].y()) is positive
        QImage _edge_map;//the edge map drawn from results of LSD
        QImage _edge_field;//the vector field constructd from edge map
        int _lower_foliage_y;
        int _higher_foliage_y;
        std::vector <bool> _kingdom_states;//false if it overlaps with main-branch or if it is consumed, size = _max_kingdom + 1
        int _root_kingdom_id;//kingdom id of first branching node
        std::vector <std::vector <double> > _voting_space;//for voting the potential from edge points
        double _min_potential;//global min of _voting_space
        double _max_potential;//global max of _voting_space
        std::vector <std::vector <osg::Vec2> > _citizens;//_citizens[i] contains citizens of i-th kingdom
        int _branch_id;//for counting which branch is growing, more signficiant for lower id
};

#endif
