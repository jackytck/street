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
         * set if to print debug msg and other related info
         */
        void setVerbose(bool debug);

        /*
         */
        void grow();

    protected:
        /*
         * for debugging and illustration purpose only
         */
        void airbrush(int x, int y);

        /*
         * for debugging and illustration purpose only
         */
        void drawLine(int x1, int y1, int x2, int y2);

        /*
         * find the root of palm from its segmentation
         */
        void findRoot();

    private:
        bool _verbose;
        bool _data_valid;
        QImage _img;
        QImage _seg;
        osg::Vec2 _root;
        QImage _debug_img;//for debugging purpose only
};

#endif
