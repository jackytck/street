#ifndef __RECTANGLEMODEL__H
#define __RECTANGLEMODEL__H

#include <osg/Geode>

/* give this class a vertex array and it returns a rectangle model,
 * the vertex array must have size of 4
 */
class RectangleModel
{
    public:
        RectangleModel(osg::ref_ptr <osg::Vec3Array>);
        RectangleModel(std::vector <osg::Vec3>);
        /**setter*/
        void setColor(float, float, float, float);
        /**getter*/
        osg::Node *get();
        /**scenes*/
        osg::ref_ptr<osg::Node> createScene();

    private:
        osg::ref_ptr<osg::Node> _root;
        osg::ref_ptr<osg::Vec3Array> _vertices;
        osg::Vec4 _color;
};

#endif
