#ifndef __BILLBOARDTREE__H
#define __BILLBOARDTREE__H

#include <QtGui>
#include <osg/Group>
#include <vector>
#include <string>
#include "GPCA.h"
//#include "classes/controller/BDL.h"
//#include "BDLPointGraph.h"
//#include "classes/controller/HoughTransform3d.h"

/* Use GPCA to segment the 3D points (from the ImageGrower::perlin_displaced) into n number of 2D planes
 * and generate leaves texture for each plane
 * for clarity, mode 0 is for doing GPCA on the position, mode 1 is on each vertex of a leaf,
 * mode 2 is for doing k-means on each leaf's vertex, mode 3 for tangent-planes method
 * usage:
 * 1. BillboardTree(mode)
 * 2. do_work(no. of planes(mode 0-2) or no. of step of each side(mode 3))
 * 3. generate_textures()
 * 4. get_output()
 * 4. save_output(path) (optional)
 */
class BillboardTree
{
    public:
        /* Initialize the instance with a set of points
         * points: points from ImageGrower::perlin_displaced 
         * all_v: the vertices all_v for generating batch leaf
         * texs: the texture coordinates for each vertex
         */
        BillboardTree(int mode, std::vector <osg::Vec3> points, std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> texs, const char *tex_path);
        BillboardTree(int mode, osg::ref_ptr <osg::Vec3Array> points, osg::ref_ptr <osg::Vec3Array> all_v, osg::ref_ptr <osg::Vec2Array> texs, const char *tex_path);

        /* overloadded for visualization purpose only
         * no texture mapping
         */
        BillboardTree(std::vector <osg::Vec3> points);

        /* for deleting any new-ed data
         */
        ~BillboardTree();

        /* helper method to call all the necessary methods
         * n: parameters passed to do_GPCA
         * mode: 1 is for doing GPCA on each position, 2 is for ecah leaf's vertex
         */
        void do_work(int n = 10);

        /* do GPCA
         * n: the number of subspaces
         */
        void  do_GPCA(int n = 10);

        /* the second version of do_GPCA,
         * it ignores the points, and do work on each vertex of a leaf
         */
        void do_GPCA2(int n = 10);

        /* standard GPCA with n subspaces
         */
        Hyperplane_GPCA_Result *gpca(std::vector <osg::Vec3> v, int n = 10);

        /* give a set of vertices and their tex-coords
         * do a gpca and group them nicely
         */
        void gpca(std::vector <osg::Vec3> v, std::vector <osg::Vec2> tex, std::vector <std::vector <osg::Vec3> >& ret_v, std::vector <std::vector <osg::Vec2> >& ret_tex, std::vector <osg::Vec3>& ret_n, int n = 10);

        /* use k-means to segment all the leaf vertex points
         */
        std::vector <int> k_means(std::vector <osg::Vec3> all_v, int cluster_cnt);

        /* this is used to setup all the private required members of this instance,
         * it depends on the above k_means
         * iteration: the number of recursive clustering
         * cluster_size: number of clusters to separate
         * gpca_size: number of subspace to fit a cluster
         */
        void setup_k_means(int iteration = 1, int cluster_size = 10, int num_subspace = 5);

		/* do the t_planes() recursively for clustering any un-clustered points
		 * return all the un-done index of 'all_v' and 'tex'
		 */
		std::vector <int> t_planes_recursive(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step);
        void t_planes2(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step = 5);

        /* yet another method for clustering the points into planes,
         * this is my own synthetic approach of first constructing a set of tangent planes to cover the whole volume,
         * then cluster all points according to the distance to the planes,
         * note, each point may be assigned to different planes
         */
        void t_planes(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step = 5);

        /* setup all the private required members of this instance
         * v_step: the grid size of the volume, not the number of planes
         * note: depends on the above t_planes
         */
        void setup_t_planes(int v_step);

        /* return the number of subspaces
         * return -1 if _gpca_result == NULL
         * this method use indirection, instead of just asking the result from GPCA
         */
        int number_of_subspace();

        /* get the assignment of the point at index i
         * return 0 to number of subspaces
         * return -1 if _gpca_result == NULL
         * this method use indirection, instead of just asking the result from GPCA
         */
        int which_assignment(int pt_idx);

        /* get the normal of each subspace
         * this method use indirection, instead of just asking the result from GPCA
         */
        osg::Vec3 which_normal(int group_id);

        /* segment all the points in _points into different groups of osg::Vec3Array
         * and also its associated _texs if available
         */
        void create_segmented_groups();

        /* segment all the points in _all_v into different groups of osg::Vec3Array
         * and also its associated _texs if available
         */
        void create_segmented_groups2();

        /* given a set of points belonging to a subspace(plane)
         * find a bounding box to bound it
         * points: the point set
         * normal: the normal given by GPCA
         * return the four 3d corners of this box
         */
        std::vector <osg::Vec3> bounding_box(osg::ref_ptr <osg::Vec3Array> points, osg::Vec3 normal);

        /* visualize the GPCA segmentation result by assigning
         * different color to different segmentation
         */
        osg::ref_ptr <osg::Group> visualize_segmentation();

        /* find the cg of the subspace indexed at 'index'
         * currently useless
         */
        osg::Vec3 plane_cg(int index);

        /* generate all billboard textures for each plane
         * note: it will exit if the sizes of _group_ptrs and _group_ptrs_tex do not match or the _tex_path is wrong
         * generated textures will be stored in /tmp and in this instance _textures
		 * path: the temp path for storing the big tiled texture
         */
        void generate_textures(const char *path = "/tmp/texture_tiled.png", float multiplier = 1.8f);

        /* do bilinear interpolation on a triangle
         * triangle: a, b, c
         * coords in source imgage: ta, tb, tc
         * note:: not texture coords, but the absolute coords
         */
        inline void bilinear_interpolate(osg::Vec2 a, osg::Vec2 ta, osg::Vec2 b, osg::Vec2 tb, osg::Vec2 c, osg::Vec2 tc, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts);

        /* overloadded helper function of the above to bilinear interpolate a quad
         */
        inline void bilinear_interpolate(osg::Vec2 ap, osg::Vec2 bp, osg::Vec2 ep, osg::Vec2 fp, osg::Vec2 ta, osg::Vec2 tb, osg::Vec2 te, osg::Vec2 tf, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts);

		/* similar to the above bilinear_interpolate(), but this one will not project,
		 * it just do rigid transform, quad is mapped to quad, to retain the leaf shape
		 */
		inline void paste_sticker(osg::Vec2 ap, osg::Vec2 bp, osg::Vec2 ep, osg::Vec2 fp, osg::Vec2 ta, osg::Vec2 tb, osg::Vec2 te, osg::Vec2 tf, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts);

        /* get the billboard cloud with textures as a single osg::Geode
         */
        osg::ref_ptr <osg::Geode> get_output(bool maya = false, bool double_side = false);

        /* save the vertex array and texture array of all the billboards
		 * return the path of the newly created texture
         * note: no maya for simplicity
         */
		std::string save_output(std::string path, bool double_side = false);

        /* given all the group's pointers, find a texture scale that does not exceed max width or height for
         * all textures
         */
        inline float texture_scale_hint(std::vector <osg::ref_ptr <osg::Vec3Array> > points_ptrs, float max);

        /* tile all the textures into a single texture by BinPacker
         * rects: the [w0][h0][w1][h1][w2][h2]... sequence
         * size: the max width or height of the new texture
         * path: the file path of the new texture
         * note: this is called by generate_textures() at the end automatically
         */
        void tile_textures(std::vector <int> rects, int size, const char *path);
        
        /* make the billboard to avoid back face culling in Google Earth
         * by copying another quad with a different order
         */
        void double_side_it(osg::ref_ptr <osg::Vec3Array>& v, osg::ref_ptr <osg::Vec3Array>& n, osg::ref_ptr <osg::Vec2Array>& tex);
        void double_side_it(std::vector <osg::Vec3>& v, std::vector <osg::Vec3>& n, std::vector <osg::Vec2>& tex);

		/* run in verbose mode?
		 */
		bool _verbose;

    protected:
        /* check if the given texture-coordinates are in bound
         */
        inline bool in_bound(int x, int y, int w, int h);
        inline bool in_bound(osg::Vec2 v, int w, int h);
        inline bool in_bound(QPoint p, int w, int h);

        /* convert osg::Vec2 to QPoint
         */
        inline QPoint vec2_to_qpoint(osg::Vec2 v);

        /* given 4 integers, find a unique set from it
         */
        inline std::vector <int> unique_set(int a, int b, int c, int d);

        /* convert vector of osg::Vec3 to osg::Vec3Array
         */
        inline osg::ref_ptr <osg::Vec3Array> to_osg_Vec3Array(std::vector <osg::Vec3> v);

        /* convert vector of osg::Vec2 to osg::Vec2Array
         */
        inline osg::ref_ptr <osg::Vec2Array> to_osg_Vec2Array(std::vector <osg::Vec2> v);

        /* serialize vector of ref_ptr::osg::Vec3Array to vector of osg::Vec3
         */
        inline std::vector <osg::Vec3> to_std_vector(std::vector <osg::ref_ptr <osg::Vec3Array> > pts);

        /* serialize vector of ref_ptr::osg::Vec2Array to vector of osg::Vec2
         */
        inline std::vector <osg::Vec2> to_std_vector(std::vector <osg::ref_ptr <osg::Vec2Array> > pts);

    private:
        //only need to fill out the required members are enough for getting the output, no matter what methods are used
        int _mode;
        std::vector <osg::Vec3> _points; //all_pos
        Hyperplane_GPCA_Result *_gpca_result;
        int _number_of_points; //number of total points that are used for clustering (required)
        int _number_of_planes; //or number of groups or subspaces (required)
        std::vector <int> _assignments; //for storing the assignment of each points, each integer range from 0 to _number_of_planes-1 (required)
        std::vector <osg::Vec3> _normals; //for storing the normal of each plane or group or subspace (required)
        std::vector <osg::Vec3> _all_v; //all vertices (required) (input)
        std::vector <osg::Vec2> _texs; //texture coords for each points (required) (input)
        std::string _tex_path; //the file path to the clipped texture
        std::vector <osg::ref_ptr <osg::Vec3Array> > _group_ptrs; //the i-th group has normal which_normal(i)
        std::vector <osg::ref_ptr <osg::Vec3Array> > _group_ptrs_v; //each point above would have 4 vertices (required)
        std::vector <osg::ref_ptr <osg::Vec2Array> > _group_ptrs_tex; //one to one correspondance to the above (required)

        std::vector <QImage> _textures; //set of individual textures for each billboard, set after calling generate_textures()
        std::vector <osg::Vec2> _texture_coords; //4 tex-coords per subspace, stored in the order of a,b,e,f
        std::string _tiled_texture_path; //the file path for the tiled texture
};

#endif
