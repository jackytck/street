#ifndef __SLINK_H
#define __SLINK_H

#include "BDLPointGraph.h"

/* Represent a branch or link in a tree
 */
class SLink
{
    public:
        SLink();
        SLink(BDLSkeletonNode *par, BDLSkeletonNode *child);
        BDLSkeletonNode *_par;
        BDLSkeletonNode *_child;
};

#endif
