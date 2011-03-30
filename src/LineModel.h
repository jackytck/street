#ifndef __LINEMODEL__H
#define __LINEMODEL__H

#include <osg/Geode>

/** give this class a vertex array and it returns a line model*/
/** should be generalized with point model*/
class LineModel
{
    public:
        LineModel();
        LineModel(osg::ref_ptr<osg::Vec3Array>);
        /**setter*/
        void setColor(float, float, float, float);
        /**getter*/
        osg::Node *get();
        /**scenes*/
        osg::ref_ptr<osg::Node> createScene(bool strip=false);
    private:
        osg::ref_ptr<osg::Node> _root;
        osg::ref_ptr<osg::Vec3Array> _vertices;
        osg::Vec4 _color;
};

#endif
