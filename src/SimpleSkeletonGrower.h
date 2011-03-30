#ifndef __SIMPLESKELETONGROWER_H
#define __SIMPLESKELETONGROWER_H

#include <vector>
#include <string>
#include "VolumeSurface.h"
#include "Library.h"
#include "LibraryElement.h"
#include "SimplePruner.h"
#include "SimpleVolumeFloat.h"

/* A simple skeleton grower used to grow a default skeleton by default
 * elements from the library. Brute force or back tracking approach
 * is used to obtain the orientation.
 * Usage:
 * 1. SimpleSkeletonGrower()
 * 2. setup()
 * 3. set_initial_skeleton()
 * 4. grow()
 */

class SimpleSkeletonGrower
{
    public:
        SimpleSkeletonGrower();
        ~SimpleSkeletonGrower();
        
        /* setup the paths of master elements, master skeletons and isp0
         * cache: if true, then use the provided file as surface points
         */
        void setup(std::string master_element, std::string master_skeleton, std::string isp0, std::string cache = "");

        /* choose the k-th default skeleton in the library as the
         * initial skeleton for subsequent growing,
         * this will reload the surface points and set the new root
         */
        void set_initial_skeleton(int k);

        /* pick a branch from the tree to replace
		 * space_aware: if true, return the farest node from _surface, else use minimum generation
         * return the tail of the branch, its _prev_support points to its root
         */
        BDLSkeletonNode *pick_branch(bool space_aware = false);

        /* replace the branch with an library element and a certain rotation
         * branch_tail: the existing branch is defined by <branch_tail, branch_tail->_prev_support>
         * element: the library element
         * angle: the rotation angle along the branch
         * return all the sub-tree roots for pruning
         */
        std::vector <BDLSkeletonNode *> replace_branch(BDLSkeletonNode *branch_tail, LibraryElement element, int angle);

		/* for determining the optimal element and angle
		 * same as replace_branch but it will not change anything to existing _root
		 * note: need to manually delete the return values to prevent leaking
		 */
        std::vector <BDLSkeletonNode *> replace_branch_unchanged(BDLSkeletonNode *branch_tail, LibraryElement element, int angle);

        /* prune the sub_tree rooted at root
         */
        void prune(std::vector<BDLSkeletonNode *> roots);

        /* grow the tree by first pick_branch(), then replace_branch()
         * and finally prune()
		 * note: this is purely random
         */
        void grow(int no_branch);

		/* grow the tree by exhaustively finding the optimal rotation
		 */
		void grow_exhaustive(int no_branch);

		/* grow the tree by potential energy
		 */
		void grow_potential(int no_branch);

		/* backward growing
		 */
		void backward_grow();

        void test();

        // the root of the tree
        BDLSkeletonNode *_root;
        // the library
        Library _library;
        //if >= threshold, then not too close (between two line segments)
        float _close_threshold;
        float _k_d;

        VolumeSurface _surface;

    protected:
        inline std::vector <BDLSkeletonNode *> bfs();
        inline void prune(BDLSkeletonNode *root);
        inline void prune_strictly(BDLSkeletonNode *root);
        inline bool is_too_close(BDLSkeletonNode *node, BDLSkeletonNode *child);

		/* to compute the energy of the given tree with the _surface points
		 * return -1.0f if root is NULL
		 */
		inline float compute_energy(BDLSkeletonNode *root);

		/* to compute the potential energy of the given tree with the given space
		 * return -1.0f if root is NULL
		 */
		inline float compute_potential_energy(BDLSkeletonNode *root, const SimpleVolumeFloat& space);
		inline float compute_potential_energy(std::vector <BDLSkeletonNode *> points, const SimpleVolumeFloat& space);

        /* prune the sub_tree rooted at root
         */
        inline std::vector <BDLSkeletonNode *> after_pruned(BDLSkeletonNode *root);
        inline std::vector <BDLSkeletonNode *> after_pruned(std::vector <BDLSkeletonNode *> roots);

    private:
        std::string _path_isp0;
        //VolumeSurface _surface;
        SimplePruner _pruner;
		bool _approaching_done; //for signaling the end of the growing stage
};

#endif
