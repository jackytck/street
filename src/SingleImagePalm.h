#ifndef __SINGLEIMAGEPALM__H
#define __SINGLEIMAGEPALM__H

#include <vector>
#include <osg/Vec3>
#include <osg/Vec2>
#include <string>
#include <QImage>

/* A class to model a palm tree from an image-segmentation-pari.
 * Flow: 
 * 1. Find the root of the palm
 *
 * Usage:
 * 1. SingleImagePalm()
 * 2. grow()
 */
class SingleImagePalm
{
    public:
        /*
         * isp0: the image and segmentation pair
         */
        SingleImagePalm(std::string isp0);
        ~SingleImagePalm();

        /*
         */
        void grow();

    protected:
        /*
         * find the root of palm from its segmentation
         */
        void findRoot();

    private:
        bool _data_valid;
        QImage _img;
        QImage _seg;
        osg::Vec2 _root;
};

#endif
