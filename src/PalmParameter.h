#ifndef __PALMPARAMETER__H
#define __PALMPARAMETER__H

#include <vector>
#include <osg/Vec3>
#include <osg/Vec2>

/* A helper class for providing Palm tree parameters
 * Flow: a backbone is drawn and exported from Blender,
 * palm_parser.py parses the backbone and generate some 'key-frames'
 * this class inter-polates those 'key-frames', for a given Bezier curve
 *
 * Usage:
 * 1. PalmParameter()
 * 2. setBezier()
 * 3. getFrameAt() for each tail or
 * 3. getFrames() for all tail plus mirrored
 */
class PalmParameter
{
    public:
        PalmParameter();
        ~PalmParameter();

        /*
         * set the quadratic Bezier curve
         * ctr1: control point 1
         * ctr2: control point 2
         * ctr3: control point 3
         * steps: number of interpolation
         */
        void setBezier(osg::Vec3 ctr1, osg::Vec3 ctr2, osg::Vec3 ctr3, int steps=22);

        /* current size of _on_curve
         * for looping getFrameAt()
         */
        unsigned int onCurveSize();

        /* get the point on the interpolated curve
         */
        osg::Vec3 getOnCurve(int k);

        /*
         * get the frame at a give point
         * t: the t-th step, t = [0, _on_curve.size()-1]
         * mirror: whether mirror the tail or not
         */
        osg::Vec3 getFrameAt(int k, bool mirror = false);

        /* get all the frames for each k,
         * plus each tail is mirrored
         * return <head, tail1, head, tail2> quad-pair for each k
         */
        std::vector <osg::Vec3> getFrames();

        /* get all the leaf quads of this Bezier curve
         * aspect: texure height divided by texture width
         * debug: if true, stdout obj faces for all quads
         * debug_offset: the vertex offset for referencing vertices in obj
         */
        std::vector <osg::Vec3> getQuads(float aspect, bool debug = false, int debug_offset = 0);

        /* get all the corresponding texture coordinates
         * of getQuads
         */
        std::vector <osg::Vec2> getTexCoords();

    protected:
        /*
         * for setting up total culmulative distance of sample element
         */
        void setupKeyFrames();

        /*
         * unit tangent at k-th key or step
         */
        osg::Vec3 getTangent(int k);

    private:
        std::vector <osg::Vec3> _keys;//coords of each key
        std::vector <osg::Vec3> _key_frames;//basis coords of each key
        std::vector <osg::Vec3> _on_curve;//current interpolated Bezier curve
        float _keys_dist;//length of the sample element, i.e. keys
        float _target_dist;//length of the target
        float _scale;//_target_dist / _keys_dist
};

#endif
