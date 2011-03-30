#ifndef __ORIENTEDCIRCLE2__H
#define __ORIENTEDCIRCLE2__H

#include "OrientedCircle.h"
//class OrientedCircle;

/* Data structure of two 3d-oriented 2d-circles sharing one geode
 */
class OrientedCircle2
{
    public:
        OrientedCircle2(OrientedCircle up, OrientedCircle down, osg::ref_ptr <osg::Geode> geode);

        /*setter*/
        //set _up and _down with the same transformations
        void makeScale(osg::Vec3 s, bool down=true);
        void makeTranslate(osg::Vec3 d);
        void makeRotate(osg::Vec3 center, osg::Vec3 axis, double degree);

        /*getter*/
        OrientedCircle *up();
        OrientedCircle *down();
        //return the asso_geometry from the _down, should be the same as that from _up
        osg::ref_ptr <osg::Group> asso_geometry();

    private:
        OrientedCircle _up, _down;
};

#endif
