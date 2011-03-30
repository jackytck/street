#ifndef __BDLTREEPARSER__H
#define __BDLTREEPARSER__H

#include "BDL.h"
#include "BDLPointGraph.h"

/**Parse the bdl into a set of shortest path tree
  */
class BDLTreeParser
{
    public:
        BDLTreeParser(BDL *bdl);
        ~BDLTreeParser();

        //buid the ann tree by k-nearest neighbor(s)
        void build_ann_tree(int k = 20);

        //build threshold graph within a given threshold
        void build_threshold_graph(float threshold);

        //build the skeleton tree
        BDLSkeletonNode *build_skeleton_tree();

        //compute the supporting length for each node recursively
        double compute_supporting_length(BDLSkeletonNode *node);
        //rectify the radius of the skeleton tree
        void rectify_skeleton_tree();

    private:
        BDL *_bdl;
        int _k;
        BDLPointGraph *_bdlPointGraph;
        BDLSkeletonNode *_skeleton_root;
};

#endif
