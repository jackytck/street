#ifndef __POINTMODEL_H
#define __POINTMODEL_H

#include <osg/Geode>
#include <osg/Point>
#include "BDL.h"

class PointModel
{
    public:
        PointModel();
        PointModel(BDL *bdl);
        /**setter*/
        void setColor(float, float, float, float);
        void setSize(float);
        /**Scenes generations*/
        osg::ref_ptr<osg::Node> createScene();
        osg::ref_ptr<osg::Node> createScene(osg::ref_ptr<osg::Vec3Array>);/**pass a new osg::Vec3Array pointer to it, new memory automatically delete*/
        osg::Node *get();
    private:
        osg::Vec4 _color;
        BDL *_bdl;
        osg::ref_ptr<osg::Node> _root;
};

#endif
