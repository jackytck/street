#ifndef __LIBRARYELEMENT_H
#define __LIBRARYELEMENT_H

#include "SLink.h"

/* To represent a subtree in the tree-library
 * It contains a BDLSkeletonNode * + the SLink for the 'supporting branch'
 * BDLSkeletonNode's _prev_support will 'be replaced' in later iteration
 * but this _support_branch is used 'to replace' the existing branch
 */
class LibraryElement
{
    public:
    //methods
        LibraryElement(BDLSkeletonNode *root, SLink support);
        LibraryElement(const LibraryElement& element);

        /* for std::vector only,
         * if this element is added to an existing tree,
         * it will be deleted by the Library with the tree
         * and shouldn't be called by iitself
         */
        ~LibraryElement();

        /* determine if all the pointers are not NULL
         */
        bool isValid();

        /* compute the length of the supporting branch,
         * return -1.0f if this instance is not valid
         */
        float support_branch_length();

    //data
        BDLSkeletonNode *_root;
        SLink _support_branch;
        bool _is_added;
};

#endif
