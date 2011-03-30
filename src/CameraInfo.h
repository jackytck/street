#ifndef __LASERPROJECTOR_H
#define __LASERPROJECTOR_H

#include <vector>
#include <osg/Vec3>
#include <string>
#include "BDLPointGraph.h"
#include "SLink.h"
#include "LibraryElement.h"

/* A helper class for dealing with IO and use the class Camera
 * this class reads in all the 3x4 projection matrices and all the camera parameters
 * A query is made by asking:
 * Given a view and a 3D point, what is the 2D coords on the view?
 * Usage:
 * 1. LaserProjector()
 * 2. setup()
 */
class LaserProjector
{
    public:
        LaserProjector();

        /* read in the 3x4 projecion matrices with their associated id
         * and also read in all the camera parameters
         */
        void setup(std::string matrix_path, std::string cam_path);
};

#endif
