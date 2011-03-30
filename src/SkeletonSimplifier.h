#ifndef __SKELETONSIMPLIFIER__H
#define __SKELETONSIMPLIFIER__H

#include "BDLPointGraph.h"
#include <osg/Vec3>
#include <string>

/** To simplify the skeleton by ray-tracing from vertices of icosahedron
  * to each skeleton nodes, it the ray is blocked by anything other than
  * itself for all the ray. It is marked as invisible.
  * If all the nodes in the subtree are invisible, the subtree will be deleted
  * note: this class takes care the deletion of the skeleton
  * Usage:
  * 1. SkeletonSimplifier()
  * 2. setup()
  * 3. simplify()
  * 4. get _root
  */
class SkeletonSimplifier
{
    public:
        SkeletonSimplifier();
        ~SkeletonSimplifier();

        /* setup the simplifier by skeleton and the leaves
         * skeleton: filepath to skeleton
         * leaves: filepath to leaves
         */
        void setup(std::string skeleton, std::string leaves);

        /* simplify the skeleton by ray-tracing from vertices of icosahedron
         * volume_step: the step size of the foliage (advanced) volume
         * depth: depth of subdivision of icosahedron
         */
        void simplify(int volume_step = 5, int depth = 2);

        BDLSkeletonNode *_root;
        std::vector <osg::Vec3> _leaves;
};

#endif
