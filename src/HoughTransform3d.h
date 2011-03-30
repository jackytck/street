#ifndef __HOUGHTRANSFORM3D__H
#define __HOUGHTRANSFORM3D__H

#include <osg/Geode>
#include "BDL.h"
#include "BDLPointGraph.h"

/* The bin that contains the total count and
 * the back reference whom votes this bin.
 */
class HT_bin
{
    public:
        HT_bin();

        //vote by index and weight to this bin
        void vote(int index, double count);

        //delete the contribution of a particular vote
        void delete_vote_by_index(int index);

        //members
        //total count
        double _count;

        //who vote this bin
        std::vector <int> _back_index;

        //weighting of this vote
        std::vector <double> _back_weight;
};

typedef std::vector<HT_bin> binVector;
typedef std::vector<binVector> bin2dVector;
typedef std::vector<bin2dVector> bin3dVector;

/* Given a point in Primal Space, vote a plane(in spherical coordinates)
 * in Parameter Space. This class contains a 3d-box for storing the counts
 * of the vote in Parameter Space via the above HT_bin class. All points 
 * in the Primal Space are transformed to be within 0 <= x, y, z <= 100.
 * Usage:
 * 1. HoughTransform3d();
 * 2. set_steps(int, int, int);
 * 3. set_scale_and_vertex(osg::ref_ptr <osg::Vec3Array>, int);
 * 4. vote(double);
 * 5. extract_plane();
 */
class HoughTransform3d
{
    public:
        HoughTransform3d();
        //HoughTransform3d(osg::ref_ptr <osg::Vec3Array> v, step_theta = 5, step_phi = 5, step_rho = 10);
        ~HoughTransform3d();

        void set_steps(int step_theta, int step_phi, int step_rho);

        //clear or initialize the _box
        void clear();

        /* set the scale and vertices of all the polygon
         * v: all vertices of the polygon
         * offset: number of vertices per polygon
         */
        void set_scale_and_vertex(osg::ref_ptr <osg::Vec3Array> v, int offset);

        /* vote all polygons with a certain error
         * error: for translating along rho-axis
         * algorithm: for each polygon, vote all its vertices(with error offset) into a temp volume,
         * find the intersection of bins by all vertices. For each bin, find the weight, e.g. projected
         * area, this polygon should vote to this bin. Vote it in my class's volume.
         */
        //void vote(double error);

        /* extract the plane that has the highest response in the Hough Transform,
         * then delete all the corresponding votes given by the vertices of this plane
         * return XXX if all planes have already been extracted
         */
        //osg::ref_ptr <osg::Geode> extract_plane();

        static osg::ref_ptr <osg::Node> debug_vote();

    private:
        //steps for bins
        int _step_theta, _step_phi, _step_rho;

        /* plane space
         * Logically:
         *  -90 <= theta <= 90
         *    0 <= phi < 360
         * -300 <= rho <= 300
         * Index:
         * 0 <= theta <= 180
         * 0 <= phi < 360
         * 0 <= rho <= 600
         * _box[theta][phi][rho]
         */
        bin3dVector _box;

        //error
        double _error;

        /* original point P
         * new point P' = (P-P_min)*k such that P' is <= 100 and >= 0
         * controvesely P = P'*(1.0/k)+P_min
         */
        osg::Vec3 _P_min;
        osg::Vec3 _k;

        // all the vertices of the leaves
        osg::ref_ptr <osg::Vec3Array> _leaf_v;

        //number of vertex per polygon
        int _offset;
};

#endif
