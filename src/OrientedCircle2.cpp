#include "OrientedCircle2.h"

OrientedCircle2::OrientedCircle2(OrientedCircle up, OrientedCircle down, osg::ref_ptr <osg::Geode> geode): _up(up), _down(down)
{
    _up.set_asso_geometry(geode);
    _down.set_asso_geometry(geode);
}

void OrientedCircle2::makeScale(osg::Vec3 s, bool down)
{
    if(down)
    {
        _up.makeScale(_down.center(), s);
        _down.makeScale(_down.center(), s);
    }
    else
    {
        _down.makeScale(_up.center(), s);//need to update down first, otherwise up.cneter is changed
        _up.makeScale(_up.center(), s);
    }
}

void OrientedCircle2::makeTranslate(osg::Vec3 d)
{
    _up.makeTranslate(d);
    _down.makeTranslate(d);
}

void OrientedCircle2::makeRotate(osg::Vec3 center, osg::Vec3 axis, double degree)
{
    _up.makeRotate(center, axis, degree);
    _down.makeRotate(center, axis, degree);
}

OrientedCircle *OrientedCircle2::up()
{
    return &_up;
}

OrientedCircle *OrientedCircle2::down()
{
    return &_down;
}

osg::ref_ptr <osg::Group> OrientedCircle2::asso_geometry()
{
    /*
    printf("up:\n");
    _up.printMatrix();
    printf("down:\n");
    _down.printMatrix();
    */
    return _down.asso_geometry();
}
