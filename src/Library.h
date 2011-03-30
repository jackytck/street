#ifndef __LIBRARY_H
#define __LIBRARY_H

#include <vector>
#include <string>
#include "LibraryElement.h"

/* To manage all the library (default+learnt) elements,
 * plus the default skeletons
 */
class Library
{
    public:
    //methods
        Library();

        /* handle deallocation for the allocations made in the BDLSkeletonElementLoader
         */
        ~Library();

        /* load a set of default library elements 
         * path: points to a master file *.bdlsgm that contains all the paths for each *.bdlsge 
         */
        void load_default_elements(std::string path);

        /* load a set of default skeletons
         * path: points to a master file *.bdlsgm that contains all the paths for each *.bdlsge
         */
        void load_default_skeleton(std::string path);

    //data
        //a set of possible candidate subtree
        std::vector <LibraryElement> _library_element;
        //a set of possible skeleton
        std::vector <BDLSkeletonNode *> _library_skeleton;
};

#endif
