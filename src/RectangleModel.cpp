#include "RectangleModel.h"
#include <osg/Geometry>
#include <stdio.h>

RectangleModel::RectangleModel(osg::ref_ptr <osg::Vec3Array> v): _color(osg::Vec4(0.0, 0.0, 1.0, 1.0))
{
    //push the last points again, for LINE_STRIP to work
    if(v->size() == 4)
    {
        v->push_back((*v)[0]);
        _vertices = v;
    }
}

RectangleModel::RectangleModel(std::vector <osg::Vec3> v): _color(osg::Vec4(0.0, 0.0, 1.0, 1.0))
{
    osg::ref_ptr <osg::Vec3Array> osg_v = new osg::Vec3Array;
    for(unsigned int i=0; i<v.size(); i++)
        osg_v->push_back(v[i]);

    //push the last points again, for LINE_STRIP to work
    if(v.size() == 4)
    {
        osg_v->push_back(v[0]);
        _vertices = osg_v;
    }
}

void RectangleModel::setColor(float r, float g, float b, float a)
{
    _color = osg::Vec4(r, g, b, a);
}

osg::Node *RectangleModel::get()
{
    createScene();
    return _root.get();
}

osg::ref_ptr<osg::Node> RectangleModel::createScene()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    // create Geometry object to store all the vertices and lines primitive.
    osg::ref_ptr<osg::Geometry> linesGeom = new osg::Geometry();

    //check if the vertices size is equal to 5
    if(!_vertices || _vertices->size() != 5)
    {
        if(!_vertices)
            printf("RectangleModel::createScene:_vertices(NULL) error\n");
        else
            printf("RectangleModel::createScene:_vertices->size(%d) error\n", int(_vertices->size()));
        return geode.get();
    }

    // pass the created vertex array to the points geometry object.
    linesGeom->setVertexArray(_vertices);

    // set the colors as before, plus using the above
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(_color);
    linesGeom->setColorArray(colors);
    linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    linesGeom->setNormalArray(normals);
    linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // This time we simply use primitive, and hardwire the number of coords to use 
    // since we know up front,
    linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,_vertices->size()));

    // add the points geometry to the geode.
    geode->addDrawable(linesGeom);

    _root = geode.get();

    return geode.get();
}
