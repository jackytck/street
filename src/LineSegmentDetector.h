#ifndef __LINESEGMENTDETECTOR__H
#define __LINESEGMENTDETECTOR__H

#include <vector>
#include <osg/Vec4>
#include <QImage>

/* A wrapper class for using the LSD: Line Segment Detector
 *
 * Usage:
 * 1. LineSegmentDetector(QImage)
 * 2. run()
 */
class LineSegmentDetector
{
    public:
        /*
         * QImage: the input image
         */
        LineSegmentDetector(const QImage& img);
        ~LineSegmentDetector();

        /*
         * return list of <a, b, c, d> pair
         * for line (a, b) to (c, d)
         */
        std::vector <osg::Vec4> run();

        /*
         * official example test
         */
        void test();

    protected:
        /*
         * same algorithm as Matlab: rgb2gray()
         */
        inline int rgb2gray(int r, int g, int b);

    private:
        const QImage& _img;
        bool _valid;
        int _w;
        int _h;
};

#endif
