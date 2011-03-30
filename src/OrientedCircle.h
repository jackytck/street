#ifndef __ORIENTEDCIRCLE__H
#define __ORIENTEDCIRCLE__H

#include <vector>
#include <osg/Geode>
#include <osg/Matrix>

/* Data structure of an 3d-oriented 2d-circle
 * Abstraction of the transformed members
 */
class OrientedCircle
{
    public:
        OrientedCircle();
        OrientedCircle(osg::Vec3 center, osg::Vec3 Tx, osg::Vec3 Ty, osg::Vec3 Tz);

        /*setter*/
        void setRadius(int s);
        void setMatrix(osg::Matrix m);
        void preMultMatrix(osg::Matrix m);
        void postMultMatrix(osg::Matrix m);
        void set_asso_geometry(osg::ref_ptr <osg::Group> node);
        //overalled for the leaf node (Geode)
        void set_asso_geometry(osg::ref_ptr <osg::Geode> node);
        void add_asso_geometry(osg::ref_ptr <osg::Group> node);
        //for segment mode, what is the opposite OrientedCircle
        //void set_opposite_oc(OrientedCircle *oc);

        /*getter (all are transformed except matrix)*/
        bool valid();
        int radius();
        double trueRadius();//||Tx||
        osg::Vec3 center();
        osg::Vec3 normal();
        osg::Vec3 Tx();
        osg::Vec3 Ty();
        osg::Matrix matrix();
        //construct points by using osgModeler
        osg::ref_ptr <osg::Vec3Array> points();
        //3d points for center, the 2 noraml and tengent
        osg::ref_ptr <osg::Vec3Array> debug_points();
        //associated geometry
        osg::ref_ptr <osg::Group> asso_geometry();

        /*core operations*/
        //translate by a vector
        void makeTranslate(osg::Vec3 d);
        //translate the geometry node only, but the oc
        void makeTranslateGeometry(osg::Vec3 d);
        //rotate according to right-hand rule, thumb to axis
        void makeRotate(osg::Vec3 center, osg::Vec3 axis, double degree);
        void makeScale(int r);
        void makeScale(osg::Vec3 center, int r);
        void makeScale(osg::Vec3 s);
        void makeScale(osg::Vec3 center, osg::Vec3 s);

        /* side query
         * a plane is spanned by Tx, Ty and center, looking from above the plane
         * return true if p is below the plane, otherwise false
         */
        bool whichSide(osg::Vec3 p);
        /* does this oc overlapped with another oc?
         */
        bool isOverlapped(OrientedCircle *oc);

        /*debug*/
        void printAxis();
        void printMatrix();

    private:
        osg::Vec3 _c, _Tx, _Ty, _Tz;
        int _radius;
        osg::Matrix _matrix;
        //bool _has_geometry;
        osg::ref_ptr <osg::Group> _geometry;
        bool _valid;
};

#endif
