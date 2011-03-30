#include "OrientedCircle.h"
//(OrientedCircle depends osgModeler depends OrientedCircle2 depends OrientedCircle)
#include "osgModeler.h"//need to be placed here to break C++ hell of circular dependency

#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
//#include <osgDB/ReadFile>
//#include <osgViewer/Viewer>
#include <stdio.h>

OrientedCircle::OrientedCircle(): _valid(false)
{
}

OrientedCircle::OrientedCircle(osg::Vec3 center, osg::Vec3 Tx, osg::Vec3 Ty, osg::Vec3 Tz)
    : _c(center), _Tx(Tx), _Ty(Ty), _Tz(Tz), _radius(1), _valid(true)
{
    _matrix.makeIdentity();
    _Tx.normalize();
    _Ty.normalize();
    _Tz.normalize();
}

bool OrientedCircle::valid()
{
    return _valid;
}

void OrientedCircle::setRadius(int s)
{
    _radius = s;
}

void OrientedCircle::setMatrix(osg::Matrix m)
{
    _matrix = m;
}

void OrientedCircle::preMultMatrix(osg::Matrix m)
{
    _matrix = m * _matrix;
}

void OrientedCircle::postMultMatrix(osg::Matrix m)
{
    _matrix = _matrix * m;
}

void OrientedCircle::set_asso_geometry(osg::ref_ptr <osg::Group> node)
{
    _geometry = node;
}

void OrientedCircle::set_asso_geometry(osg::ref_ptr <osg::Geode> node)
{
    if(node.valid())
    {
        osg::ref_ptr <osg::Group> group = new osg::Group;
        group->addChild(node);
        _geometry = group;
    }
}

void OrientedCircle::add_asso_geometry(osg::ref_ptr <osg::Group> node)
{
    if(node.valid())
    {
        if(_geometry.valid())
        {
            _geometry->addChild(node);
        }
        else
            set_asso_geometry(node);
    }
}

int OrientedCircle::radius()
{
    //return _radius;
    osg::Vec3 Tx = this->Tx();
    return int(Tx.length());
}

double OrientedCircle::trueRadius()
{
    osg::Vec3 Tx = this->Tx();
    return Tx.length();
}

osg::Vec3 OrientedCircle::center()
{
    return _c * _matrix;
}

osg::Vec3 OrientedCircle::normal()
{
    //return _Tz * _radius * _matrix;
    osg::Vec3 p = _c + _Tz;
    osg::Vec3 Tp = p * _matrix;
    return (Tp-this->center()) * _radius;
}

osg::Vec3 OrientedCircle::Tx()
{
    //return _Tx * _radius * _matrix;
    osg::Vec3 p = _c + _Tx;
    osg::Vec3 Tp = p * _matrix;
    return (Tp-this->center()) * _radius;
}

osg::Vec3 OrientedCircle::Ty()
{
    //return _Ty * _radius * _matrix;
    osg::Vec3 p = _c + _Ty;
    osg::Vec3 Tp = p * _matrix;
    return (Tp-this->center()) * _radius;
}

osg::Matrix OrientedCircle::matrix()
{
    return _matrix;
}

osg::ref_ptr <osg::Vec3Array> OrientedCircle::points()
{
    osg::Vec3 c = this->center();
    osg::Vec3 Tx = this->Tx();
    osg::Vec3 Ty = this->Ty();
    osg::Vec3 Tz = this->normal();
    //printf("c(%f,%f,%f) Tx(%f,%f,%f), Ty(%f,%f,%f), Tz(%f,%f,%f)\n", _c.x(), _c.y(), _c.z(), _Tx.x()*_radius, _Tx.y()*_radius, _Tx.z()*_radius, _Ty.x()*_radius, _Ty.y()*_radius, _Ty.z()*_radius, _Tz.x()*_radius, _Tz.y()*_radius, _Tz.z()*_radius); 
    //printf("c'(%f,%f,%f) Tx'(%f,%f,%f), Ty'(%f,%f,%f), Tz'(%f,%f,%f)\n", c.x(), c.y(), c.z(), Tx.x(), Tx.y(), Tx.z(), Ty.x(), Ty.y(), Ty.z(), Tz.x(), Tz.y(), Tz.z()); 
    return osgModeler::orientedCircle(c, Tx, Ty, Tz);
}

osg::ref_ptr <osg::Vec3Array> OrientedCircle::debug_points()
{
    osg::ref_ptr<osg::Vec3Array> ret = new osg::Vec3Array;

    osg::Vec3 c = this->center();
    osg::Vec3 n1 = this->Tx();
    osg::Vec3 n2 = this->Ty();
    osg::Vec3 t = this->normal();

    int len = ceil(n1.length());

    n1.normalize();
    n2.normalize();
    t.normalize();

    ret->push_back(c);
    for(int i=0; i<len; i++)
    {
        ret->push_back(c + n1*i);
        ret->push_back(c + t*i);
        ret->push_back(c + n2*i);
    }

    return ret;
}

osg::ref_ptr <osg::Group> OrientedCircle::asso_geometry()
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;
    if(_geometry.valid())
    {
        osg::ref_ptr <osg::MatrixTransform> mt = new osg::MatrixTransform;
        mt->setMatrix(_matrix);
        mt->addChild(_geometry);
        return mt;
        //ret = _geometry;
    }
    return ret;
}

void OrientedCircle::makeTranslate(osg::Vec3 d)
{
    osg::Matrix m;
    m.makeTranslate(d);
    /*
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            printf("%f ", m.ptr()[j*4+i]);
        }
        printf("\n");
    }
    osg::Vec3 p(1, 2, 3);
    osg::Vec3 pp = p * m;
    printf("pp (%f %f %f)\n", pp.x(), pp.y(), pp.z());
    */
    //this->preMultMatrix(m);
    this->postMultMatrix(m);
    //todo: translate the associated geometry too!
}

void OrientedCircle::makeTranslateGeometry(osg::Vec3 d)
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;
    if(_geometry.valid())
    {
        osg::ref_ptr <osg::MatrixTransform> mt = new osg::MatrixTransform;
        osg::Matrix m;
        m.makeTranslate(d);
        mt->setMatrix(m);
        mt->addChild(_geometry);
        _geometry = mt;
    }
}

void OrientedCircle::makeRotate(osg::Vec3 center, osg::Vec3 axis, double degree)
{
    osg::Matrix T, T2, R;
    T.makeTranslate(center);
    T2.makeTranslate(center * -1);

    double rad = osg::PI / 180.0 * degree;
    R.makeRotate(rad, axis);
    /*
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            printf("%f ", R.ptr()[j*4+i]);
        }
        printf("\n");
    }
    */
    //this->preMultMatrix(T2 * R * T);
    this->postMultMatrix(T2 * R * T);
    //todo: rotate the associated geometry too!
}

void OrientedCircle::makeScale(int r)
{
    if(r > 0)
    {
        double ratio = r / this->trueRadius();
        this->makeScale(osg::Vec3(ratio, ratio, ratio));
    }
}

void OrientedCircle::makeScale(osg::Vec3 center, int r)
{
    if(r > 0)
    {
        double ratio = r / this->trueRadius();
        this->makeScale(center, osg::Vec3(ratio, ratio, ratio));
    }
}

void OrientedCircle::makeScale(osg::Vec3 s)
{
    this->makeScale(this->center(), s);
}

void OrientedCircle::makeScale(osg::Vec3 center, osg::Vec3 s)
{
    if(s.x() == s.y() && s.y() == s.z() && s.x() > 0)
    {
        osg::Matrix T, T2, S;

        T.makeTranslate(center);
        T2.makeTranslate(center * -1.0);
        S.makeScale(s);

        //this->preMultMatrix(T2 * S * T);
        this->postMultMatrix(T2 * S * T);
    }
}

bool OrientedCircle::whichSide(osg::Vec3 p)
{
    bool ret = true;

    osg::Vec3 Tx = this->Tx();
    osg::Vec3 Ty = this->Ty();
    osg::Vec3 center = this->center();

    if(osgModeler::orient3D(center+Tx, center+Ty, center, p) < 0)
        ret = false;

    return ret;
}

bool OrientedCircle::isOverlapped(OrientedCircle *oc)
{
    bool ret = false;

    if(oc && oc->points()->size() > 0)
    {
        osg::ref_ptr <osg::Vec3Array> other_pts = oc->points();
        bool what_side = whichSide((*other_pts)[0]);
        
        for(unsigned int i=0; i<other_pts->size(); i++)
            if(whichSide((*other_pts)[i]) != what_side)
            {
                ret = true;
                break;
            }
    }

    return ret;
}

void OrientedCircle::printAxis()
{
    printf("_c(%f %f %f) _Tx(%f %f %f) _Ty(%f %f %f) _Tz(%f %f %f) _radius(%d)\n", _c.x(), _c.y(), _c.z(), _Tx.x(), _Tx.y(), _Tx.z(), _Ty.x(), _Ty.y(), _Ty.z(), _Tz.x(), _Tz.y(), _Tz.z(), _radius);
}

void OrientedCircle::printMatrix()
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            printf("%f ", _matrix.ptr()[i+j*4]);
        }
        printf("\n");
    }
}
