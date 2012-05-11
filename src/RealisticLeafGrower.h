#ifndef __REALISTICLEAFGROWER__H
#define __REALISTICLEAFGROWER__H

#include <string>
#include "BDLPointGraph.h"
#include <osg/Vec3>
#include <osg/Vec2>
#include <QImage>

/* A wrapper class of GenericLeafGrower for growing a more realistic leafs texture
 * with generic leaf shape, 
 * i.e. all leaf share the same leaf shape but not the small texture for a 
 * given BDLSkeleton graph, it could be use by either one photo or with the laser data
 * Usage:
 * 1. RealisticLeafGrower()
 * 2. setup()
 * 3. grow_single_image() or grow_laser()
 * 4. save()
 */
class RealisticLeafGrower
{
    public:
        RealisticLeafGrower();
        
        /* set the BDLSkeleton graph, in which where the leaves grow
         * root: the root node
		 * generic_path: the generic texture path
		 * scale: custom scale
         */
        void setup(BDLSkeletonNode *root, std::string generic_path, float scale);

        /* setting the scale, for re-using this LeafGrower object
         */
        void set_scale(float s);

        /* same as GenericLeafGrower
         */
        void set_parameters(float grow_zone, float radius_k, int pedal, float fuzziness);

		/* set the mode as verbose
		 */
		void set_verbose(bool flag);

		/* clear the vectors of this instance
		 */
		void clear();

        /* grow textured leaves from a single image
         * w: width of the resultant texture
         * h: height of the resultant texture
         * scale: the pruner scale used to grow this skeleton
         * root_x, root_y: img coords of the root in isp0
         * src_path: path of the source texture
         * src_seg_path: path of the source segmentaion
         */
        void grow_single_image(int w, int h, float scale, int root_x, int root_y, std::string src_path, std::string src_seg_path);
        void grow_single_image(int w, int h, float scale, std::string isp0);

        /* grow textured leaves from laser data
         * w: width of the resultant texture
         * h: height of the resultant texture
         * simple_pt: filepath of the simple point (for getting sp_cg)
         * cameras: filepath of cameras generated by script 'extract_camera'
         */
        void grow_laser(int w, int h, std::string simple_pt, std::string cameras);

        /* grow textured leaves from palm data
         * which is controlled by class SingleImagePalm
         * same input as grow_single_image()
         */
        void grow_single_palm(int w, int h, float scale, int root_x, int root_y, std::string src_path, std::string src_seg_path, std::vector <osg::Vec3> all_v);
;
        void grow_single_palm(int w, int h, float scale, std::string isp0, std::vector <osg::Vec3> all_v);

		/* output the grown leaves for external use, e.g. Blender
         * return the absolute path of the newly created texture
		 */
        std::string save(std::string path);

        /* used to tile the generic texture side by side in a big texture
         * generic_path: path of the generic leaf (should be alph-texture)
         * num_leaf: number of leaf to tile
         * w: width of resultant texture
         * h: height of resultant texture
         * tex: also return the texture coords
         */
        static QImage tile_texture(std::string generic_path, int num_leaf, int w, int h, std::vector <osg::Vec2>& tex); 

        /* infer the texture coords from the given 3d leaf points, 4 points per leaf
         * all_v: all vertex of leaves
         * cg: cg of all_v
         * scale: scale of the pruner
         * root_x, root_y: 2d root point in isp0
         * sw: width of source img
         * sh: height of source img
         */
        static std::vector <osg::Vec2> tex_coord_from_vertex(std::vector <osg::Vec3> all_v, osg::Vec3 cg, float scale, int root_x, int root_y, int sw, int sh);

		/* import back (not from Blender)
		 * path: file path of the import file
         * return the texture path
		 */
        std::string load(std::string path);

		/* convenient method for transfering texture sample color
		 * sample: the path to the small sample texture
		 * bill: the path to the giant titled texture board
		 * blend: if true, blend the sample and bill
		static void colorize(std::string sample, std::string bill, bool blend = false);
		 */

		/* for debugging purpose only,
		 * dump _all_pos, _all_v and _all_tex
		void dump(std::string path);
		 */

        //all the leaf vertices, four vertices per leaf
		std::vector <osg::Vec3> _all_pos;
        std::vector <osg::Vec3> _all_v;
        std::vector <osg::Vec2> _all_tex; //len(_all_tex) == len(_all_v) instead of 4 for simplicity
        std::vector <osg::Vec2> _all_tex_isp0; //the tex coords for the image in isp0

    private:
		//for finding the next non-transparent pixel
		//inline static void next_pixel(const QImage& img, int& x, int& y);

        //data members
        BDLSkeletonNode *_root;
		float _scale;
        float _grow_zone;
        float _radius_k;
        int _pedal;
        float _fuzziness;
		std::string _generic_texure;
		bool _verbose;
        QImage _tiled_texture;
};

#endif
