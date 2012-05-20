#ifndef __BDLSG2BEZIER__H
#define __BDLSG2BEZIER__H

#include <vector>
#include <string>
#include <fstream>
#include <osg/Vec3>
#include "BDLPointGraph.h"

/* A converter class to convert a BDLSG to a list of Bezier control points
 * for importing in Blender.
 * The scheme of the output can be found at:
 * /Users/jacky/work/resources/branch_learner/blender_script/mesh/BDLSG2Bezier.scheme
 */
class BDLSG2Bezier
{
    public:
        BDLSG2Bezier(std::string path);
        ~BDLSG2Bezier();

        /* convert the BDLSG to a set of separated Bezier curves
		 * note: should not be transformed in here, should be stricly follow the root (for simplificity)
         */
        //void output(BDLSkeletonNode *root, osg::Vec3 origin = osg::Vec3(0.0, 0.0, 0.0), double height = 10.0, int angle = 0, bool maya = false, double simplication = 1.0);
        void output(BDLSkeletonNode *root);

        /* convert a palm data structure into set of separated Bezier curves
         * the lower part of the data structure is the same as usual, but 
         * the upper part is a cubic Bezier curve with at least 4 nodes for each branch
         * radius: main branch radius, other radii will be calculated automatically
         */
        void output_palm(BDLSkeletonNode *root, std::vector <float> radii);

        /* output the format used by XiaoPeng
         * note: just print the skeleton and the raidus of each node
         */
        void output_xiao(BDLSkeletonNode *root);

        /* need to solve the mystery of misalignment
         * solved: the misalignment is caused by the difference between the set of bezier curves and the connected mesh, so need to re-adjust the leaves
         */
        void blender_test();

    protected:
        /* convert a list of path points into a list of Cubic Bezier control points
         */
        inline std::vector <osg::Vec3> segment_controls(std::vector <osg::Vec3> path, osg::Vec3 hint);

        /* convert a list of 4 control points from each segment into Blender's style
         * i.e. a list of BezTriple
         */
        inline std::vector <osg::Vec3> seg2BezTriple(std::vector <osg::Vec3> ctrs);

        /* convert a list of path points into a list of BezTriple
         * each point would have 3 control points
         */
        inline void output_tube(std::vector <osg::Vec3> path, std::vector <double> radii, osg::Vec3 hint);

        /* for output_xiao
         */
        inline void smooth_radius(std::vector <double>& radii);
        inline std::vector <osg::Vec3> interpolate_segment_controls(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 d, int time = 11);
        inline void output_tube_xiao(std::vector <osg::Vec3> path, std::vector <double> radii, osg::Vec3 hint, int gen);

    private:
        std::string _output_path;
        FILE *_out;

        int _xiao_id;
};

#endif
