#include "LineModel.h"
#include <osg/Geometry>

LineModel::LineModel(): _color(osg::Vec4(0.0, 1.0, 0.0, 1.0))
{
}

LineModel::LineModel(osg::ref_ptr<osg::Vec3Array> vertices)
{
    _vertices = vertices;
    this->setColor(0.0, 1.0, 0.0, 1.0);/**why did i get segmentation fault if i write _color = osg::Vec4(0.0, 1.0, 0.0, 1.0)?*/
}

void LineModel::setColor(float r, float g, float b, float a)
{
    _color = osg::Vec4(r, g, b, a);
}

osg::Node *LineModel::get()
{
    return _root.get();
}

osg::ref_ptr<osg::Node> LineModel::createScene(bool strip)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    // create Geometry object to store all the vertices and lines primitive.
    osg::ref_ptr<osg::Geometry> linesGeom = new osg::Geometry();

    // this time we'll preallocate the vertex array to the size we
    // need and then simple set them as array elements, 8 points
    // makes 4 line segments.
    /*
    osg::Vec3Array* vertices = new osg::Vec3Array(8);
    (*vertices)[0].set(-1.13704, -2.15188e-09, 0.40373);
    (*vertices)[1].set(-0.856897, -2.15188e-09, 0.531441);
    (*vertices)[2].set(-0.889855, -2.15188e-09, 0.444927);
    (*vertices)[3].set(-0.568518, -2.15188e-09, 0.40373);
    (*vertices)[4].set(-1.00933, -2.15188e-09, 0.370773);
    (*vertices)[5].set(-0.716827, -2.15188e-09, 0.292498);
    (*vertices)[6].set(-1.07936, 9.18133e-09, 0.317217);
    (*vertices)[7].set(-0.700348, 9.18133e-09, 0.362533);
    */

    // pass the created vertex array to the points geometry object.
    linesGeom->setVertexArray(_vertices);

    // set the colors as before, plus using the above
    osg::Vec4Array* colors = new osg::Vec4Array;
    //colors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
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
    if(strip)
        linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,_vertices->size()));
    else
        linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,_vertices->size()));


    // add the points geometry to the geode.
    geode->addDrawable(linesGeom);

    _root = geode.get();

    return geode.get();
}
