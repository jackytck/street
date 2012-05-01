#ifndef __TRANSFORMER_H
#define __TRANSFORMER_H

#include <vector>
#include <osg/Vec3>
#include "BDLPointGraph.h"
#include "SLink.h"
#include "LibraryElement.h"

/* A helper class that provides some static methods for transforming the BDLSkeletonNode graph.
 */
class Transformer
{
    public:
        Transformer();

        /* transform a 3d point to the given link
         * such that the original z-axis is (Euclidean-ly) transformed to the link 
         * rotation: angle in degree
         * note: no scaling is performed
         */
        static osg::Vec3 transform(SLink link, float rotation, osg::Vec3 p);
        static osg::Vec3 transform(BDLSkeletonNode *par, BDLSkeletonNode *child, float rotation, osg::Vec3 p);

        /* transform a LibraryElement to the given link
         * note: does not 'new' anything, it just modifies the content of element
         * scaling is performed here
         */
        static void transform(SLink link, int rotation, LibraryElement& element);
        static void transform(BDLSkeletonNode *tail, int rotation, LibraryElement& element);

        /* rotate an element such that the supporting branch is aligned with the z-azis
         * note: does not 'new' anything, it just modifies the content of element
         */
        static void straight_up(LibraryElement& element);

        /* the closest distance from point c to line segment a b
         * return -1.0f if any of the pointer is NULL
         */
        static float point_segment_dist(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c);
        static float point_segment_dist(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *c);

        /* find the closest segment along the 'replacement branch' from tail to tail->_prev_support from a given query point
         * tail: the tail of the replacement branch
         * query: the query point
         */
        static SLink closest_segment(BDLSkeletonNode *tail, BDLSkeletonNode *query);

        /*convert BDLSkeletonNode * to a osg::Vec3
         */
        static osg::Vec3 toVec3(BDLSkeletonNode *node);

        /* compute the shortest distance between two given lines
         */
        static float line_dist(osg::Vec3 L1_P0, osg::Vec3 L1_P1, osg::Vec3 L2_P0, osg::Vec3 L2_P1);

        /* compute the shortest distance between two given line segments
         */
        static float segment_dist(osg::Vec3 S1_P0, osg::Vec3 S1_P1, osg::Vec3 S2_P0, osg::Vec3 S2_P1);

        /* compute the shortest distance between 
         * segment(a, b) and segment(c, d)
         * return -1.0f if a NULL pointer is found
         */
        static float segment_dist(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *c, BDLSkeletonNode *d);

        /* compute the shortest distance from a given link to a tree
         * return -1.0f if a NULL pointer is found
         * note: it only returns the non-zero shortest distance, since
         * the branch-replacement algorithm always generate zero as the shortest distance
         */
        static float link_tree_dist(BDLSkeletonNode *root, BDLSkeletonNode *a, BDLSkeletonNode *b);

		/* compute the shortest distance from a given point to a tree node
		 * return -1.0f if a NULL pointer is found
		 */
		static BDLSkeletonNode *point_tree_dist(BDLSkeletonNode *root, osg::Vec3 p, float& dist);
		static BDLSkeletonNode *point_tree_dist(std::vector <BDLSkeletonNode *> trees, osg::Vec3 p, float& dist);
		static float point_tree_dist(BDLSkeletonNode *root, osg::Vec3 p);

        /* convert a UNIT 3 vector p to <lat, lon> pair
         * return in degrees
         */
        static void cartesian_to_spherical(osg::Vec3 p, double& lat, double& lon);

        /* convert spherical to cartesian
         * lat and lon are in degrees
         */
        static osg::Vec3 spherical_to_cartesian(double r, double lat, double lon);
        static inline osg::Vec3 ellipsoid_to_cartesian(double rz, double rx, double ry, double lat, double lon);

        /* generate and append 4 leaf vertices to all_v
         * all_v: where 4 new vertices are appended
         * pos: the center position of this planar leaf
         * normal: normal of this plane
         * scale: scale or absolute width and height of this leaf
         * random: randomly and slightly perturb the normal
         * note: the leaf's aspect ratio is 1:1
         */
        static void points_to_leafs(std::vector <osg::Vec3>& all_v, osg::Vec3 pos, osg::Vec3 normal, float scale, bool random = false);

        /* given all the center positions of leaves, try to 
         * find all the leaf vertices
         * pts: all the center points
         * cg: the cg of all the leaves, for computing normal
         * scale: the absolute width and height of each leaf
         * random values will be added upon this scale
         * return all the vertices
         */
        static std::vector <osg::Vec3> pos_to_vertex(const std::vector <osg::Vec3>& pts, osg::Vec3 cg, float scale);

        /* covert std::vector to osg::Vec3Array
         */
        static osg::ref_ptr <osg::Vec3Array> std_to_osg_array(std::vector <osg::Vec3> v);
        static osg::ref_ptr <osg::Vec2Array> std_to_osg_array(std::vector <osg::Vec2> v);

		/* convert osg::Vec3Array to std::vector
		 */
		static std::vector <osg::Vec3> osg_to_std_array(osg::ref_ptr <osg::Vec3Array> v);
		static std::vector <osg::Vec2> osg_to_std_array(osg::ref_ptr <osg::Vec2Array> v);

        /* get cloud points inside a unit box by Billow noise
         * pts: all the generated cloud points
         * scalars: the weight associated with each cloud points
         * step: the number of steps along each axis
         * note: all pts lie within a unit box
         * i.e. 0 <= x,y,z <= 1
         */
        static void billow_noise(std::vector <osg::Vec3>& pts, std::vector <float>& scalars, int step = 6);

        /* given a reference frame and an origin,
         * find and return all the billow cloud points
         * origin: frame's origin
         * x: unit direction of x axis
         * y: unit direction of y axis
         * z: unit direction of z axis
         * w: the absolute width of each axis
         * return the transformed coords + the weights
         */
        static void billow_noise_transformed(std::vector <osg::Vec3>& transformed, std::vector <float>& weights, osg::Vec3 origin, osg::Vec3 x, osg::Vec3 y, osg::Vec3 z, float w);

        /* get points on the surface of unit sphere
         * pts: all the generated surface ppoints
         * weights: the weight of each point
         * coverage: latitude from 'coverage' to 90, -90 < coverage < 90
         * leaf_size_hint: the expected size of each leaf for a unit sphere
         * note: billow_noise is more random, this is more deterministic
         */
        static void sphere_points(std::vector <osg::Vec3>& pts, std::vector <float>& weights, int coverage = -89, float leaf_size_hint = 1.2f);
        static void ellipsoid_points(std::vector <osg::Vec3>& pts, std::vector <float>& weights, int coverage = -89, float leaf_size_hint = 1.2f);

        /* given a reference frame and an origin,
         * find and return all the surface points
         * origin: frame's origin
         * r: the radius of the sphere
         * z: unit direction of z axis
         * coverage: latitude from 'coverage' to 90, -90 < coverage < 90
         * leaf_size_hint: the expected size of each leaf for a unit sphere
         * return the transformed coords + the weights
         */
        static void sphere_points_transformed(std::vector <osg::Vec3>& transformed, std::vector <float>& weights, osg::Vec3 origin, float r, osg::Vec3 z, int coverage, float leaf_size_hint);

        /* normalize a list of float within a range [0..1]
         */
        static void normalize_float(std::vector <float>& in);

        /* compute the cg of the given points
         */
        static osg::Vec3 find_cg(std::vector <osg::Vec3> pts);

        /* for finding the divergent angle of a branch
         * return in degree, -1.0f if error
         */
        static float divergent_angle(BDLSkeletonNode *prev, BDLSkeletonNode *node, BDLSkeletonNode *child);

        /* for finding the most similar divergent angle in the tree
         * root: tree's root
         * angle: divergent angle I want to obtain from sub-tree
         * a: root of sub-tree
         * b: tip of sub-tree
         * return true if no error occurs
         */
        static bool similar_angle_in_tree(BDLSkeletonNode *root, float angle, BDLSkeletonNode *&a, BDLSkeletonNode *&b);

        /* extract the sub-tree given by the a and b from similar_angle_in_tree
         * a: root
         * b: tip
         * new_b: pointer to the copied b
         * full: if true, return the full sub-tree up to the same hop level, otherwise only the first level
         * of the a-->b path
         * all other nodes that are more further away than b to a are discarded
         * return the new root of the copied sub-tree
         */
        static BDLSkeletonNode *extract_subtree(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *&new_b, bool full = false);

        /* get all leaf node
         */
        static std::vector <BDLSkeletonNode *> terminal_nodes(BDLSkeletonNode *root);

        /* get the ATA of matrix M
         * M: input matrix, will be deleted
         * m: number of row of M
         * n: number of column of M
         * return a new 2d array
         */
        static float **ATA(float **M, int m, int n);

        /* given 4 2D points and the destination width and height,
         * compute the homography H
         * return by setting the 3x3 H
         * w: target width
         * h: target height
         * (ax,ay): source lower-left corner
         * (bx,by): source lower-right corner
         * (cx,cy): source upper-right corner
         * (dx,dy): source upper-left corner
         */
        static void homography(float **H, int w, int h, float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy);

        /* find out the quadrant of which a direction is lying
         * normal: direction, not necessarily unit vector
         */
        static int which_quadrant(osg::Vec3 normal);

        /* 3x3 matrix H pre-multiply a vector in
         */
        static osg::Vec3 mult_vec(float **H, osg::Vec3 in);

        /* bilinear interpolation
         * x: fraction in x direction [0,1]
         * y: fraction in y direction [0,1]
         */
        static unsigned char bilinear(float x, float y, const unsigned char p00, const unsigned char p01, const unsigned char p10, const unsigned char p11);

        /* find the length of the first hop of the initial skeleton
         * return -1.0f if error occurs
         */
        static float first_hop_dist(BDLSkeletonNode *root);

        /* convert a tree into array plus
         * adding the generation to it
         */
        static std::vector <BDLSkeletonNode *> bfs(BDLSkeletonNode *root);

        /* get all the leaves from a tree
         */
        static std::vector <BDLSkeletonNode *> leaves(BDLSkeletonNode *root);

        /* count the number of nodes of a tree
         */
        static int tree_size(BDLSkeletonNode *root);

        /* do clustering by annTree, Dijkstra and BDLPointGraph
         */
        static std::vector <std::vector <osg::Vec3> > ann_cluster(std::vector <osg::Vec3> pts, int k = 10);

        /* find the lowest height(z) in a set of osg::Vec3
        */
        static float min_z(std::vector <osg::Vec3> pts);

        /* find the highest height(z) in a set of osg::Vec3
        */
        static float max_z(std::vector <osg::Vec3> pts);

        /* extend end-points of initial tree such that
         * they are driven in the foliage
         * initial: initial skeleton
         * fol_c: the foliage center
         * threshold: the threshold for segmenting the initial points
         */
        static void extend_end_points(BDLSkeletonNode *initial, osg::Vec3 fol_c, float threshold);

        /* get the bounding box of the given points
         * return if the returned metrics is valid
         * x points right, y points inside, z points up
         * so right is max_x, in is max_y, top is max_z
         */
        static bool cloud_metrics(std::vector <osg::Vec3> pts, float& right, float& left, float& top, float& bottom, float& in, float& out);

        /* find the orientations of 3 2D points
         * refer to 754lects.pdf
         */
        static float orient(osg::Vec2 p, osg::Vec2 q, osg::Vec2 r);

        /* get the vertices of the sub-divided icosahedron
         */
        static std::vector <osg::Vec3> icosahedron(int depth);

        /* interpolate the quadratic bezier curve
         */
        static std::vector <osg::Vec3> interpolate_bezier_2(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, int time = 11);

        /* interpolate the cubic bezier curve
         */
        static std::vector <osg::Vec3> interpolate_bezier_3(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 d, int time = 11);

        /* interpolate the cubic bezier curve on 2d plane
         */
        static std::vector <osg::Vec2> interpolate_bezier_3_2d(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2 d, int time = 11);

        /* given 3 points a,b,c, construct d,e such that abde is a rectangle with width w lying on the plane
         * defined by a,b,c and {c,d,e} are on the same side (with respect to a,b)
         */
        static void rect_plane(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3& d, osg::Vec3& e, float w = -1.0f);

        /* find the average of the inter distance between a list of points
         * return -1.0f if list.size() is smaller than 2
         */
        static float average_inter_dist(std::vector <osg::Vec3> list);

        /* given 3 control points of a quadratic bezier curve,
         * tile rectangle planes along the interpolated path
         * ctr_pt1: 1st control point
         * ctr_pt2: 2nd control point
         * ctr_pt3: 3rd control point
         * start_a: starting point a
         * start_b: starting point b
         * return all the vertices of the planes in conventional <a,b,e,f> leaf order, 
         * including start_a and start_b
         */
        static std::vector <osg::Vec3> tile_plane_along_path(osg::Vec3 ctr_pt1, osg::Vec3 ctr_pt2, osg::Vec3 ctr_pt3, osg::Vec3 start_a, osg::Vec3 start_b);

        /* given a list of quads of a palm leaf,
         * most probably generated by Transformer::tile_plane_along_path(),
         * infer a list of texture coordinates, such that the middle part of the leaf is repeated if the texture is not long enough
         * all_v: all the vertices of quads
         * t_w: width of input palm texture
         * t_h: height of input palm texture
         */
        static std::vector <osg::Vec2> texture_coords_palm(std::vector <osg::Vec3> all_v, int t_w, int t_h);

        /*
         * given a list of floats, return the standard deviation of the series
         * last: only consider the last 'last' records, i.e. list[-last:] in python, -1 to consider all records
         * return -1.0f if the list is empty or last is 0
         * note: this is equivalent to Google Spreadsheet's STDEVP()
         */
        static float standard_deviation(std::vector <float> list, int last = -1);
};

#endif
