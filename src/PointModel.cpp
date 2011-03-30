#include "PointModel.h"
#include <osg/Geometry>
#include <osg/StateSet>

PointModel::PointModel(): _color(osg::Vec4(1.0, 1.0, 1.0, 1.0))
{
}

PointModel::PointModel(BDL *bdl): _bdl(bdl)
{
}

void PointModel::setColor(float r, float g, float b, float a)
{
    _color = osg::Vec4(r, g, b, a);
}

void PointModel::setSize(float s)
{
    if(_root)
    {
        osg::StateSet *state = _root->getOrCreateStateSet();
        state->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);
        osg::Point *point = new osg::Point(s);
        state->setAttribute(point);
    }
}

osg::ref_ptr<osg::Node> PointModel::createScene()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    // create Geometry object to store all the vertices and points primitive.
    osg::ref_ptr<osg::Geometry> pointsGeom = new osg::Geometry();
    int pointSize = _bdl->getPointSize();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    vertices->resize(pointSize);
    for(int i=0; i<pointSize; i++)
    {
        BDLPoint p = _bdl->getPoint(i);
        (*vertices)[i].set(p.x/p.w, p.y/p.w, p.z/p.w);
    }
    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);

    // create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    /*
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    
    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    */
    colors->resize(pointSize, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    // set the normal in the same way color.
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
    
    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    _root = geode.get();

    return geode.get();
}

osg::ref_ptr<osg::Node> PointModel::createScene(osg::ref_ptr<osg::Vec3Array> vertices)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    // create Geometry object to store all the vertices and points primitive.
    osg::ref_ptr<osg::Geometry> pointsGeom = new osg::Geometry();
    /*
    int pointSize = _bdl->getPointSize();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    vertices->resize(pointSize);
    for(int i=0; i<pointSize; i++)
    {
        BDLPoint p = _bdl->getPoint(i);
        (*vertices)[i].set(p.x/p.w, p.y/p.w, p.z/p.w);
    }
    */
    // pass the created vertex array to the points geometry object.
    pointsGeom->setVertexArray(vertices);

    // create the color of the geometry, one single for the whole geometry.
    // for consistency of design even one single color must added as an element
    // in a color array.
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    // add a white color, colors take the form r,g,b,a with 0.0 off, 1.0 full on.
    /*
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    
    // pass the color array to points geometry, note the binding to tell the geometry
    // that only use one color for the whole object.
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    */
    colors->resize(vertices->size(), _color);//osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    pointsGeom->setColorArray(colors);
    pointsGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    // set the normal in the same way color.
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    pointsGeom->setNormalArray(normals);
    pointsGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // create and add a DrawArray Primitive (see include/osg/Primitive).  The first
    // parameter passed to the DrawArrays constructor is the Primitive::Mode which
    // in this case is POINTS (which has the same value GL_POINTS), the second
    // parameter is the index position into the vertex array of the first point
    // to draw, and the third parameter is the number of points to draw.
    pointsGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
    
    // add the points geometry to the geode.
    geode->addDrawable(pointsGeom);
    _root = geode.get();

    return geode.get();
}

/** point to the root of the point model scene graph*/
osg::Node *PointModel::get()
{
    return _root.get();
}
