#ifndef __BDLSKELETONLOADER_H
#define __BDLSKELETONLOADER_H

#include <QtGui>
#include <vector>
#include <string>
#include <osg/Group>
#include "BDLPointGraph.h"

/* To read a BDLSkeleton Graph formed by BDLSkeletonNode from file
 */
class BDLSkeletonLoader
{
    public:
        BDLSkeletonLoader();
        ~BDLSkeletonLoader();

        /* read the bdl-skeleton-graph (.bdlsg) and initialize this instance
         * for schematic, refer to the resources/bdlskeleton/BDLSkeletonLoader.scheme
         */
        void load_file(QString filename);

        /* construct the bdl skeleton graph from the loaded instance variables
         * will not be responsible for handling memory deallocation
         * can use BDLSkeletonNode::delete_this(bdlsn); to delete this tree
         */
        BDLSkeletonNode *construct_bdl_skeleton_tree();

        /* print all the values of this instance variable
         * for debugging BranchLearner2D()
         */
        void print_self();

    protected:
        int _number_of_node;
        int _number_of_link;
        std::vector <osg::Vec3> _nodes;
        std::vector <int> _parents;
        std::vector <int> _childs;
};

#endif
