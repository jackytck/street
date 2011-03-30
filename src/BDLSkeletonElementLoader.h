#ifndef __BDLSKELETONELEMENTLOADER_H
#define __BDLSKELETONELEMENTLOADER_H

#include <QtGui>
#include <vector>
#include <string>
#include "BDLSkeletonLoader.h"
#include "SLink.h"

/* To read a BDLSkeleton Graph formed by BDLSkeletonNode from file
 */
class BDLSkeletonElementLoader: public BDLSkeletonLoader
{
    public:
        BDLSkeletonElementLoader();

        /* read the bdl-skeleton-graph-element (.bdlsge) and initialize this instance
           in this subclass, it only read 1 more line for the supporting branch link
         */
        void load_file(QString filename);

        /* override for storing the nodes' pointers
         */
        BDLSkeletonNode *construct_bdl_skeleton_tree();

        /* get the link for the support branch
         */
        SLink support_branch();

        /* clear the instance
         */
        void clear();

        /* print the original input, for debugging purpose only
         */
        void print_self();

    private:
        //secondary and primary supporting branch
        int _num_sec_sup, _num_pri_sup;
        std::vector <std::pair <int, int> > _sec_sup_branches;
        std::vector <std::pair <int, int> > _pri_sup_branches;

        int _sup_par, _sup_child; //currently only allow the last primary supporting link
        SLink _support_branch;
};

#endif
