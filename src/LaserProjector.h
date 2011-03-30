#ifndef __LASERPROJECTOR_H
#define __LASERPROJECTOR_H

#include <vector>
#include <string>
#include <osg/Vec2>
#include <osg/Vec3>
#include "BDLCamera.h"

//#include <osg/Vec3>
//#include "BDLPointGraph.h"
//#include "SLink.h"
//#include "LibraryElement.h"

/* A helper class for dealing with IO and use the class Camera
 * this class reads in all the 3x4 projection matrices and all the camera parameters
 * A query is made by asking:
 * Given a view and a 3D point, what is the 2D coords on the view?
 * Usage:
 * 1. LaserProjector()
 * 2. setup()
 * 3. project(osg::Vec3 p, int view)
 */
class LaserProjector
{
    public:
        LaserProjector();

        /* read in the 3x4 projecion matrices with their associated id
         * return true if successful
         */
        bool setup(std::string matrix_path);

        /* tell if setup is successful
         */
        bool is_loaded();

        /* project a point p to a given view 'view'
         * view: the index to _view(e.g. 0,1,2...), not its view_id
         * texture: return texture coords if true, otherwise absolute coords
         * return the texture coordinates
         * return (-1.0f,-1.0f) if error occurs
         */
        osg::Vec2 project(osg::Vec3 P, int view, bool texture = true, bool fast = false);

        /* return the filename of the image of a view
         * view: the index to _view(e.g. 0,1,2...), not its view_id
         */
        std::string view_name(int view);

        std::vector <int> _view;
        std::vector <std::string> _view_img;
        std::vector <std::string> _view_seg;

    private:
        bool _loaded;
        std::vector <BDLCamera> _bdlcameras;
};

#endif
