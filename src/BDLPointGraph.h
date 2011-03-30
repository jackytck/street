#ifndef __BDLPOINTGRAPH_H
#define __BDLPOINTGRAPH_H

#include "BDL.h"
#include <vector>
#include "TopologicalSkeleton.h"

class BDLPointGraphNode;

/**A node for representing the skeleton*/
class BDLSkeletonNode
{
    public:
        BDLSkeletonNode();
        BDLSkeletonNode(double x, double y, double z);
        double dist(BDLSkeletonNode *);

        //members
        double _sx, _sy, _sz;
        double _radius;
        BDLSkeletonNode *_prev;
        std::vector <BDLSkeletonNode *> _children;

        //the original nodes that this king represents
        std::vector <BDLPointGraphNode *> _slave_nodes;

        //for mesh generation, each skeleton node is only output once
        bool _mesh_done;

        //for rectifying the radius of the skeleton tree, the parent branch should have a size = sum(child[i]'s size) for all i
        //meaningless for leaf node, i.e. _children.size() == 0
        double _inherited_radius;

        //for implementating the topological sort of rectifying the radius
        bool _radius_rectified;

        //for approximating the radius by Biological model
        //the total length that this node needs to support
        double _supporting_length;

        //for storing the height for the implementation of skeleton_mesh
        double _height;

        //for pruning, and only meaningful for leaves, if it is pruned before, it should not grow in the next step
        bool _pruned;

        //for branch replacement purpose, points to the root of the supporting branch for this node
        BDLSkeletonNode *_prev_support;

        //store the generation, or the number of hops from root
        int _generation;

        //static methods
        //convert TopologicalSkeleton into this BDLSkeletonNode
        static BDLSkeletonNode *ts2bdlsn(TopologicalSkeleton *ts);

        //compute the supporting length
        static double compute_supporting_length(BDLSkeletonNode *node);

        //translate to origin and scale to a given height if not equal -1.0 and rotate angle degree above the root
        //then find the a)height and b)radius of each node of this skeleton
        //return the scale used to scale the model
        //maya: true if it is rectified for maya
        //the order of transformation is: local rotation, scalling, translation, exchange y and z-axis if maya is true
        static double rectify_skeleton_tree(BDLSkeletonNode *root, osg::Vec3 origin = osg::Vec3(0.0, 0.0, 0.0), double height = -1.0, int angle = 0, bool maya = false);

        //debug by printing radius of each node
        static void debug_radius(BDLSkeletonNode *root);
        static void debug_support_length(BDLSkeletonNode *root);

        //find all the occurence of A->B->C, then transformed it to A->C, by deleting B
        //now assume the whole graph as a feasible region
        //todo: specify the feasible region
        static void compress_skeleton_tree(BDLSkeletonNode *root);

        //find
        //return the remaining number of edges that can be collapsed after this function
        static int collaspe_skeleton_tree(BDLSkeletonNode *root, int iteration = 1);

        //create the mesh for this skeleton
        //origin: the origin of the object model
        //height: the height of the model
        //angle: angle in degree to rotate the model above the root
        //output_branch: include branch if true
        //output_leaf: include leaves if true
        //maya: true if it is used in maya
        //simplication: the fraction of the original that it represents, if it is smaller than 1.0, flat leaves will be used
        static osg::ref_ptr <osg::Group> skeleton_mesh(BDLSkeletonNode *root, osg::Vec3 origin = osg::Vec3(0.0, 0.0, 0.0), double height = 100.0, int angle = 0, bool output_branch = true, bool output_leaf = true, bool maya = false, double simplication = 1.0);

        //overloaded for skeleton_mesh for storing the branch and leave in two separate pointers
        //triangle_cnt: the number of triangle of the generated mesh, eventually get it right!!
        static osg::ref_ptr <osg::Group> skeleton_mesh(BDLSkeletonNode *root, osg::Group *&branch_group,  osg::Geode *&leave_group, int& triangle_cnt, osg::Vec3 origin = osg::Vec3(0.0, 0.0, 0.0), double height = 100.0, int angle = 0, bool output_branch = true, bool output_leaf = true, bool maya = false, double simplication = 1.0);

        //create an osg::Group which shows the skeleton rooted in root
        static osg::ref_ptr <osg::Group> skeleton_node(BDLSkeletonNode *root);

        //copy this tree
        //only copy position, prev, children, prev_support
        static BDLSkeletonNode *copy_tree(BDLSkeletonNode *);

        //delete this tree
        static void delete_this(BDLSkeletonNode *);

        //find the distance between two BDLSkeletonNode
        static float dist(BDLSkeletonNode *a, BDLSkeletonNode *b);

        //give a hint of the scale for leaves
        static float leaf_scale_hint(BDLSkeletonNode *root);

        //a skeleton is singular if it is only a chain
        static bool is_singular(BDLSkeletonNode *root);

        //compute the height of this subtree, i.e. the maximum distance from any node to root
        static float max_height(BDLSkeletonNode *root);

        //compute the height hops of this subtree, i.e. the maximum distance from any node to root, and get the number of 
        //hops from this node to the root
        static int max_height_hops(BDLSkeletonNode *root);

        //compute the total number of nodes contained in this sub-tree
        static int tree_size(BDLSkeletonNode *root);

        //export the given skeleton into file, this file can be loaded by class BDLSkeletonLoader
        static void save_skeleton(BDLSkeletonNode *root, const char *path = "/tmp/skeleton.bdlsg");

        //caculate the length of the main branch, i.e. the Euclidean distance from the root to the first branching node
        //return -1.0f if error occurs
        static float main_branch_length(BDLSkeletonNode *root);

        //rectify the default skeleton by the parameters in bound
        //root: the default skelton
        //mb_length: the closest distance from root to bound
        //b_h: the vertical height of the bound
        //g: the occupancy percentage
        static void rectify_default_skeleton(BDLSkeletonNode *root, float mb_length, float b_h, int g = 65);

        //remove a child if it's really a child of root
        static void remove_child(BDLSkeletonNode *root, BDLSkeletonNode *child);
};

/**A node for BDLPointGraph*/
class BDLPointGraphNode
{
    public:
        BDLPointGraphNode();
        const bool operator < (const BDLPointGraphNode&) const;


        BDLPoint _self;
        //std::vector <BDLPoint> _edges;//todo: change BDLPoint to BDLPointGraphNode *
        std::vector <BDLPointGraphNode *> _edges;//todo: change BDLPoint to BDLPointGraphNode *
        std::vector <double> _costs;

        bool _done;
        double _dist;

        //reference of the original parent object
        BDLPointGraphNode *_prev;

        //reference of the original object being copied from
        BDLPointGraphNode *_this;

        //sub-group id
        int _sub_group;
        //bin id, starting from zero
        int _bin;

        //my skeleton node's king, i.e. the skeletion node that I belong to
        BDLSkeletonNode *_skeleton_king;
        //for keeping track of which pigeon is visited during DFS
        bool _pigegon_visited;
};

/**A graph that has node of type BDLPointGraphNode*/
class BDLPointGraph
{
    public:
        BDLPointGraph();
        ~BDLPointGraph();
        BDLPointGraphNode *new_node();
        void new_nodes(int num=1);

        void find_root();
        void run_dijkstra();

        BDLSkeletonNode * build_skeleton();
        
        //class membres
        BDLPointGraphNode *_root_node;
        std::vector <BDLPointGraphNode *> _nodes;

        //the step-size of cutting the shortest path
        double _quantization_step;

        //for answering sth like: group 0 has 100 bins
        int _group_size;//starting from zero
        std::vector <int> _bin_sizes;

        //debuging purposes
        BDLPointGraphNode *_lowest_node;
        BDLPointGraphNode *_highest_node;
};

#endif
