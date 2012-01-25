#ifndef __GENERICLEAFGROWER__H
#define __GENERICLEAFGROWER__H

#include <string>
#include "BDLPointGraph.h"
#include <osg/Vec3>
#include <osg/Vec2>
#include <QImage>

/* A simple and thin class for growing generic leafs, 
 * i.e. all leaf share the same small texture for a 
 * given BDLSkeleton graph
 * Usage:
 * 1. GenericLeafGrower()
 * 2. setup()
 * 3. set_parameters()
 * 4. grow()
 * 5. save()
 */
class GenericLeafGrower
{
    public:
        GenericLeafGrower();
        
        /* set the BDLSkeleton graph, in which where the leaves grow
         * root: the root node
		 * path: the generic texture path
		 * scale: custom scale
         */
        void setup(BDLSkeletonNode *root, std::string path, float scale);

        /* setting the scale, for re-using this LeafGrower object
         */
        void set_scale(float s);

        /* set other parameters
         * grow_zone: grow above this fraction of height only
         * radius_k: besides leaf node, grow if radius is smaller than this fraction
         * pedal: ~ expected number of leaf pedals
         * fuzziness: control the degree of unrestricted growing
         */
        void set_parameters(float grow_zone, float radius_k, int pedal, float fuzziness);

		/* set the mode as verbose
		 */
		void set_verbose(bool flag);

        /* ask if this instance is loaded, for caching
        bool is_loaded();
         */

		/* clear the vectors of this instance
		 */
		void clear();

        /* grow textured leaves
         */
        void grow();

        /* grow leaves for palm tree
         * by hard-coding a set of quadratic Bezier curves,
         * then tile quads onto each curve.
         * each quads represent a set of leaves
         */
        void grow_palm();

        /* improve the grow_palm() method by 
         * representing each leaf as a quad,
         * and properly align them in a more realistic way
         */
        void grow_palm2();

        /* instead of hard-coding the Bezier curve in grow_palm2(),
         * assume the outer part of the input skeleton represents sets of cubic Bezier curves,
         * the main skeleton stills represent the physical branch
         */
        void grow_palm3();

		/* output the grown leaves for external use, e.g. Blender
		 */
		void save(std::string path);

		/* import back from blender
		 * path: file path of the import file
		 */
		void load(std::string path);

		/* convenient method for transfering texture sample color
		 * sample: the path to the small sample texture
		 * bill: the path to the giant titled texture board
		 * blend: if true, blend the sample and bill
		 */
		static void colorize(std::string sample, std::string bill, bool blend = false);

		/* for debugging purpose only,
		 * dump _all_pos, _all_v and _all_tex
		 */
		void dump(std::string path);

        //all the leaf vertices, four vertices per leaf
		std::vector <osg::Vec3> _all_pos;
        std::vector <osg::Vec3> _all_v;
        std::vector <osg::Vec2> _all_tex; //len(_all_tex) == len(_all_v) instead of 4 for simplicity

    private:
		//for finding the next non-transparent pixel
		inline static void next_pixel(const QImage& img, int& x, int& y);

        //data members
        BDLSkeletonNode *_root;
		float _scale;
        float _grow_zone;
        float _radius_k;
        int _pedal;
        float _fuzziness;
		std::string _generic_texure;
		bool _verbose;
};

#endif
