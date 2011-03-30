#ifndef __OSGMODELER__H
#define __OSGMODELER__H

#include <vector>
#include <osg/Geode>
#include <osgModeling/Loft>
#include <osgModeling/Bezier>
#include "OrientedCircle2.h"

/* Provide helper functions for modelling osg geometry
  */
class osgModeler
{
    public:
        osgModeler();

        /*create a NURBS circle with radius r*/
        static osg::ref_ptr <osgModeling::Curve> createNurbsCircle(float r=0.5f, int numPath=100);

        /*create a partial NURBS cirle and a straight line that joins the two ends
        * breaking_t: (0-1)
        */
        static void createPartialCircle(float r=0.5f, int numPath=100, double breaking_t=0.5, osgModeling::Curve *left=NULL, osgModeling::Curve *right=NULL);

        /* create a lofted surface with the given BezierCurve and cross-session
         * control_pts: Bezier's control points
         * degree: Bezier's degree
         * cross_r: Radius of the cross-session
         */
        static osg::ref_ptr <osg::Geode> createLoft(osg::Vec3Array *control_pts, unsigned int degree, std::vector <int> cross_r);

        /* create a stroke with broken segments defined by the breaking t
         * breaking_t: (0-1) parametric values on which the stroke should be broken
         * old return value: osg::ref_ptr <osg::Geode>
         * geom.update() for the whole curve has no big problem, but not for updating each segment
         * Todo: reimplementation is needed
         */
        static std::vector <osg::ref_ptr <osgModeling::Loft> > segmentedStroke(osg::Vec3Array *control_pts, unsigned int degree, std::vector <int> cross_r, std::vector <double> breaking_t);

        /* Reimplement the buggy osgModeler::Loft
         * in tube form, cubic Bezier
         * return each segmented stroke in OrientedCircle2 format sorted in accending order of t
         */
        //static osg::ref_ptr <osg::Group> createTube(osg::ref_ptr <osg::Vec3Array> controlPoints, std::vector <int> cross_r, std::vector <double> breaking_t);
        static std::vector <OrientedCircle2> createTube(osg::ref_ptr <osg::Vec3Array> controlPoints, std::vector <int> cross_r, std::vector <double> breaking_t);
        /* Given two OrientedCircle(s), create a tube that connects the two
         * depends on repositionOC2 and createTube above
         */
        //static osg::ref_ptr <osg::Vec3Array> createTube(OrientedCircle *a, OrientedCircle *c);
        static osg::ref_ptr <osg::Group> createTube(OrientedCircle *a, OrientedCircle *c);

        /* create a tube by the path and the radii
         * path: 3D points that the path passes through
         * radii: radius of the tube at each point
         * step: the number of divisions from one node to the other node
         * step_circle: the number of path of each circle
         */
        static osg::ref_ptr <osg::Geode> createTube(std::vector <osg::Vec3> path, std::vector <double> radii, int step=10, int step_circle = 100);
        //static osg::ref_ptr <osg::Vec3Array> createTube(std::vector <osg::Vec3> path, std::vector <double> radii, int step=10);

        /* createTube cannot create a tube with path == 2, should be an osg internal error, so LineBox is used instead
         * a: first end-point
         * b: second end-point
         * k: the thickness
         */
        static osg::ref_ptr <osg::Geode> createLineBox(osg::Vec3 a, osg::Vec3 b, double k=0.05);

        /* create a leaf model
         * by alpha texture a quad
         * pos: the tip of the leaf
         * gS: global scale returned by BDLSKeletonNode::rectify_skeleton_tree
         * return a osg::Geode
         */
        static osg::ref_ptr <osg::Geode> createLeaf(osg::Vec3 pos = osg::Vec3(500,0,500), osg::Vec3 to = osg::Vec3(1,1,1), double ang_z = 0.0, double gS = 1.0);

        /* same as the createLeaf() above but
         * this return the vertics and texture coords for the leaf
         * for batch processing
         * in current implementation, each time 6 vertices and texture coords will be pushed in v and tex if simple is false
         * if simple is true, only 4 vertices and texture coords are pushed
         */
        static void createLeaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::Vec3 pos = osg::Vec3(500,0,500), osg::Vec3 to = osg::Vec3(1,1,1), double ang_z = 0.0, double gS = 1.0, bool simple = false, bool constant_scale = false);

        /* same as above but this is specially designed for growing in the middle air
         * pos: the middle point of the rectangle
         * normal: the normal of the flat rectangle
         * scale: the x,y-scale of the rectangle, preferably function of Perlin noise
         */
        static void createFloatLeaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::Vec3 pos = osg::Vec3(0,0,0), osg::Vec3 normal = osg::Vec3(0,0,1), double scale = 1.0);

        static void perpendicularLeaf(osg::ref_ptr <osg::Vec3Array>& v, osg::ref_ptr <osg::Vec2Array>& tex, osg::ref_ptr <osg::Vec3Array>& all_n, osg::Vec3 cg);

        /* create a single osg::Geode by using the vertices and texture coords
         * generated by createLeaf() above
         * need simple to infer the size of each leaf
         */
        static osg::ref_ptr <osg::Geode> create_batch_leaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, bool simple = false);
        static osg::ref_ptr <osg::Geode> create_batch_leaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::ref_ptr <osg::Vec3Array> n);

        /* create a stateset for alpha texturing the leaf created by createLeaf
         * path is the filepath to the texture file
         */
        static osg::StateSet *leaf_stateset(const char *path = "/Users/jacky/work/resources/leaf/leaf3.png");

        static osg::ref_ptr <osg::Geometry> createSphere(double r=0.5);

        /* attach a sphere at each point on the given curve for visualization
         */
        static osg::ref_ptr <osg::Group> visualizeCurve(osg::ref_ptr<osg::Vec3Array>, double r=-1.0);

        /* create an oriented NURBS circle
         * Tx, Ty and Tz are directions, not points
         */
        static osg::ref_ptr <osg::Vec3Array> orientedCircle(osg::Vec3 center = osg::Vec3(0, 0, 0), osg::Vec3 Tx = osg::Vec3(1, 0, 0), osg::Vec3 Ty = osg::Vec3(0, 1, 0), osg::Vec3 Tz = osg::Vec3(0, 0, 1), int numPath = 100);

        /* create the joint in-between given three oriented circle
         */
        static osg::ref_ptr <osg::Group> branchJoint(osg::ref_ptr<osg::Vec3Array> a, osg::Vec3 ac, osg::Vec3 an, osg::ref_ptr<osg::Vec3Array> b, osg::Vec3 bc, osg::Vec3 bn, osg::ref_ptr<osg::Vec3Array> c, osg::Vec3 cc, osg::Vec3 cn);
        static osg::ref_ptr <osg::Group> branchJoint(OrientedCircle a, OrientedCircle b, OrientedCircle c);

        /* compute 3d center, tanget and normal of a cubic Bezier curve
         * t = [0..1]
         */
        static osg::Vec3 center_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t);
        static osg::Vec3 tangent_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t);
        static osg::Vec3 normal_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t);

        /* 3d orientation test
         * return posivite value if d is below the plane
         * the plane is formed by a, b, c counter-clockwisely
         */
        static double orient3D(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 d);

        /* determine if the given 3 OrientedCircle can form a well-shaped joint
         */
        static bool isGoodOC3(OrientedCircle a, OrientedCircle b, OrientedCircle c);
        //static osg::ref_ptr <osg::Vec3Array> isGoodOC3(OrientedCircle a, OrientedCircle b, OrientedCircle c);

        /* re-position the 3 OrientedCircle such that they are good
         * a and c are brothers, b is inserted, a != c != b
         * return the joint constructed by the 3 repositioned OC, c_down is the transformed oc with respect to c, which is the c_up
         */
        //static osg::ref_ptr <osg::Vec3Array> repositionOC3(OrientedCircle *a, OrientedCircle *b, OrientedCircle *c);
        static osg::ref_ptr <osg::Group> repositionOC3(OrientedCircle *a, OrientedCircle *b, OrientedCircle *c, OrientedCircle *c_down);

        /* re-position the 2 OrientedCircle such that a tube can be formed between them
         * a is to be moved, c is fixed, a's center will be projected to a plane spanned by c's center, normal and a's normal
         * a and c are allowed to be overlapped, but cannot share the same center
         * return the min index of a and c
         */
        static std::vector <int> repositionOC2(OrientedCircle *a, OrientedCircle *c);

        static osg::ref_ptr <osg::Geode> branchJointTriangle();
        static osg::ref_ptr <osg::Group> test();

    private:
        inline static double dist(osg::Vec3 a, osg::Vec3 b);
        
        /* return the middle control point of a quadradic Bezier curve given two endpoint and one point on the curve
         */
        inline static osg::Vec3 middle_ctr_pt(osg::Vec3 start, osg::Vec3 end, osg::Vec3 on_curve, double t);

        /* find intersection of two 3d vectors, if they do not intersect, bn will be projected to the plane
         * spanned by a, an and b
         */
        inline static osg::Vec3 intersect(osg::Vec3 a, osg::Vec3 an, osg::Vec3 b, osg::Vec3 bn);

        /* find the orthogonal projection of p on a plane spanned by a, b and c
         */
        inline static osg::Vec3 proj(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 p);

        /* modular in the positive sense
         */
        inline static int modular(int x, int size);

        /* generate cubic bezier curve that passes through the 3 given points, a normal is given for the first two points
         * it is used for generating c_start_opposite curve
         * return kcn, i.e. k times the projected c's normal
         */
        inline static osg::Vec3 cubic_ctr(osg::Vec3 start, osg::Vec3 opposite, osg::Vec3 on_curve, osg::Vec3 normal); 

        /* triangulate by the given curves for branch joint
         * cc is the center of curve c
         */
        inline static osg::ref_ptr <osg::Geode> triangulateCurve(std::vector <osg::ref_ptr <osgModeling::BezierCurve> > a, osg::Vec3 cc);

        /* triangulate a set of circular curves for createTube()
         * oc_list is the list of oriented circle
         */
        inline static osg::ref_ptr <osg::Geode> triangulateTubeCurve(std::vector <osg::ref_ptr <osg::Vec3Array> > oc_list, std::vector <osg::Vec3> oc_center_list);
};

#endif
