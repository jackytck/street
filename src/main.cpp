#include <sys/time.h>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include <QString>
#include "BDLSkeletonLoader.h"
#include "SimpleSkeletonGrower.h"
#include "LaserSkeletonGrower.h"
#include "BDLSG2Bezier.h"
#include "LeafGrower.h"
#include "BillboardTree.h" 
#include "VolumeSurface.h" 
#include "GenericLeafGrower.h" 
#include "RealisticLeafGrower.h" 
#include "LaserProjector.h" 
#include "Transformer.h" 
#include "SkeletonSimplifier.h" 

namespace po = boost::program_options;
namespace fs = boost::filesystem;

/* To grow a skeleton from a set of elements and default skeletons
 * under the constraint of image-segmentation-pair.
 *
 * Output:
 * a. BDLSkeltonGraph
 * b. Blender BezierCurve
 * c. Cloud points (for caching)
 *
 * Command line options:
 * branch -L library -I initial -isp isp0 -cache cloud_pts -s skeleton_index -step 250 -out_bdlsg bdlsg -out_blender bezierCurve -save_cache path -sim simplification
 */

int main(int argc, char *argv[])
{
	struct timeval start;
	gettimeofday(&start, NULL);

	//inputs
	std::string library(""), initial(""), isp0(""), cache("");
	int skeleton_index, step;
	float simplification;
	bool maya = false, verbose = false, exhaust = false, potential = false;
	std::string in_bdlsg("");
	std::string in_gleaf(""); //generic leaf texture
	std::string in_gleaf_modifier(""); //modifier to modify the generic leaf texture
	std::string in_eleaf(""); //blender edited leaves
	float leaf_scale;
	float texture_multiplier;
	float root_radius;

	//inputs for laser data
	std::string in_laser_sp(""), in_laser_cam("");//simple point and cameras
	bool is_laser_raw = false;//by default the simple point is assume to be segmented
	float input_k_d;

    //generic leaf parameters
    float leaf_grow_zone;
    float leaf_radius_k;
    int leaf_pedal;
    float leaf_fuzziness;

	//inputs for leaves simplification
	bool is_simplify_leaf = false;//read realistic leaves from path: in_eleaf and output to: out_leaf
	bool is_simplify_skeleton = false;//read leaves from path: in_eleaf, read skeleton from path: in_bdlsg, and output to: out_bdlsg

    //palm tree
	bool is_palm_leaves = false;//grow palm leaves or not

	//outputs
	std::string out_bdlsg(""), out_bezier(""), out_xiao(""), out_cache(""), out_leaf(""), out_generic_leaf(""), out_realistic_leaf("");

	//program options
	po::options_description desc("Options");
	desc.add_options()
		("help,h", "Display this information")
		("version,V", "Display version number")
		("verbose,v", "Produce verbose output")
		("library,L", po::value <std::string>(&library), "Master file of all library elements")
		("initial,I", po::value <std::string>(&initial), "Master file of all initial skeletons")
		("isp,S", po::value <std::string>(&isp0), "Image-Segmentation-Pair")
		("cache,C", po::value <std::string>(&cache), "Cloud points cache")
		("index,i", po::value <int>(&skeleton_index)->default_value(0), "Use the i-th(zero-based) initial skeleton")
		("step,d", po::value <int>(&step)->default_value(250), "Number of branches to be replaced")
		("exhaust,x", "Subject to distance energy")
		("potential,u", "Subject to potential energy")
		("simplification,s", po::value <float>(&simplification)->default_value(1.0), "Simplification: [0,1]")
		("save-cache,c", po::value <std::string>(&out_cache), "Cache the cloud points")
		("output-bdlsg,b", po::value <std::string>(&out_bdlsg), "Output as BDLSkeletonGraph")
		("output-bezier,o", po::value <std::string>(&out_bezier), "Output as Blender's BezierTriple(s)")
		("output-xiao", po::value <std::string>(&out_xiao), "Output as XiaoPeng obj format")
		("input-bdlsg,G", po::value <std::string>(&in_bdlsg), "Input path of BDLSG for conversion")
		("input_leaf,E", po::value <std::string>(&in_eleaf), "Input path of (Blender edited) leaves")
		("leaf,l", po::value <std::string>(&out_leaf), "Output billboard leaf")
		("gleaf_texture", po::value <std::string>(&in_gleaf), "Texture of generic leaf")
		("texture_modifier", po::value <std::string>(&in_gleaf_modifier), "Modify the color of generic texture")
		("leaf_scale", po::value <float>(&leaf_scale)->default_value(0.0), "Leaf's scale")
		("leaf_grow_zone", po::value <float>(&leaf_grow_zone)->default_value(0.28), "Grow above this fraction")
		("leaf_radius_k", po::value <float>(&leaf_radius_k)->default_value(0.1), "Grow if node is small")
		("leaf_pedal", po::value <int>(&leaf_pedal)->default_value(13), "Number of leaf per node")
		("leaf_fuzziness", po::value <float>(&leaf_fuzziness)->default_value(0.0), "Leaf's fuzzy coverage")
		("texture_multiplier", po::value <float>(&texture_multiplier)->default_value(1.8f), "Control size of billboards inside tiled texture")
		("gleaf,g", po::value <std::string>(&out_generic_leaf), "Output generic leaf")
		("rleaf,r", po::value <std::string>(&out_realistic_leaf), "Output realistic leaf")
		("root_radius", po::value <float>(&root_radius)->default_value(0.0), "Root's radius")
		("laser_sp", po::value <std::string>(&in_laser_sp), "Input path of laser simple point data")
		("laser_cam", po::value <std::string>(&in_laser_cam), "Input path of laser camera data")
		("laser_raw", "Learn the initial skeleton from raw points")
		("kd", po::value <float>(&input_k_d)->default_value(-1.0f), "k_d in cache")
		("sim_leaf", "Simplify the leaves")
		("sim_ske", "Simplify the skeleton")
		("palm", "Generate palm tree's leaves")
	;

	//options parsing
	try
	{
		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);

		if(vm.count("help") || argc == 1)
		{
			std::cout << desc << "\n";
			return 1;
		}

		if(vm.count("version"))
		{
			printf("street version 1.3 (Jacky Tang, May 2011)\n");
			return 0;
		}

		if(vm.count("verbose"))
			verbose = true;

		if(vm.count("exhaust"))
			exhaust = true;

		if(vm.count("potential"))
			potential = true;

		if(vm.count("laser_raw"))
			is_laser_raw = true;

		if(vm.count("sim_leaf"))
			is_simplify_leaf = true;

		if(vm.count("sim_ske"))
			is_simplify_skeleton = true;

        if(vm.count("palm"))
            is_palm_leaves = true;

		//entry point for any generic test
		if(false)
		{
			goto Break;
		}

		//test for laser data
		if(!in_laser_sp.empty() && !in_laser_cam.empty() && !library.empty() && !initial.empty() && skeleton_index >=0 && step >= 0)
		//if(true)
		{
			/*
			*/
			LaserSkeletonGrower grower;
			grower.setup(library, initial, in_laser_sp, in_laser_cam, is_laser_raw);

			if(is_laser_raw)//construct initial skeleton only
				goto Break;

			grower.set_initial_skeleton(skeleton_index);
			if(!exhaust)
				grower.grow_potential(step);
			else
				grower.grow_exhaustive(step);

			BDLSkeletonNode *skeleton = grower._root;
			if(skeleton)
			{
				skeleton->_radius = BDLSkeletonNode::main_branch_length(skeleton) / 6.0f;
				//float height = std::max(10.0f, BDLSkeletonNode::max_height(skeleton));

				//rectify and simplify skeleton
				skeleton->_radius *= 9.5f;
				if(root_radius != 0.0f)
					skeleton->_radius = root_radius;
				//BDLSkeletonNode::rectify_skeleton_tree(skeleton, osg::Vec3(0.0f, 0.0f, 0.0f), height, 0, maya);

				//if(simplification != 1.0)
				//{
				//	int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
				//	for(int i=0; i<remaining_edges*(1-simplification); i++)
				//		BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
				//}

				//output bdlsg
				if(!out_bdlsg.empty())
				{
					BDLSkeletonNode::save_skeleton(skeleton, out_bdlsg.c_str());
				}
			}

			//test for Camera
			/*
			Camera camera;
			camera.set_id_type(123, 0);

			//camera.set_projection(-37.8251, 48.1455, 69.7248, -6367.84,
			//					  -57.1039, -46.8258, 80.4727, -14373.7,
			//					  -0.000843736, -0.00786523, 0.0662317, -1);
			//camera.set_parameters(1006.797119, 1006.560486, 959.997070, 1291.536865, -0.056101, -0.024881, 121.000000);
			//osg::Vec2 p = camera.project(osg::Vec3(-211.944, -40.4574, 10.9271));

			camera.set_projection(3.24502, 16.858, 1.85422, 1405.2, 
								  -7.13154, 6.68226, -15.3193, -1068.17,
								  -0.00565393, 0.00385524, 0.000354317, -1);
			camera.set_parameters(2309.771484, 2315.897461, 1007.366882, 1291.759888, -0.447955, 0.275281, 93.457802, -0.124844, 0.000125, 0.000585);
			osg::Vec2 p = camera.project(osg::Vec3(-211.093, -38.8421, 8.71338));

			printf("%f %f\n", p.x(), p.y());
			*/

			//test for LaserProjector
			/*
			LaserProjector lp;
			std::string matrix_path("/Users/jacky/quick/laser/cameras");
			if(lp.setup(matrix_path))
			{
				for(unsigned int i=0; i<lp._view.size(); i++)
					printf("%d: %s\n", lp._view[i], lp.view_name(i).c_str());

				for(unsigned int i=0; i<lp._view_img.size(); i++)
				{
					printf("%s\n", lp._view_img[i].c_str());
					printf("%s\n", lp._view_seg[i].c_str());
				}

				osg::Vec3 P;
				osg::Vec2 p;

				std::vector <osg::Vec3> test;
				test.push_back(osg::Vec3(-211.108, -39.1862, 9.5426));
				test.push_back(osg::Vec3(-211.345, -39.2353, 9.60331));
				test.push_back(osg::Vec3(-211.577, -39.282, 9.65142));
				test.push_back(osg::Vec3(-211.717, -39.305, 9.64124));
				test.push_back(osg::Vec3(-211.561, -39.2567, 9.47531));
				test.push_back(osg::Vec3(-211.647, -39.2667, 9.43598));
				test.push_back(osg::Vec3(-211.813, -39.295, 9.43151));
				test.push_back(osg::Vec3(-210.996, -39.0953, 8.98351));
				test.push_back(osg::Vec3(-210.913, -39.0678, 8.88269));
				test.push_back(osg::Vec3(-211.144, -39.1122, 8.91357));

				for(unsigned int i=0; i<test.size(); i++)
				{
					P = test[i];
					p = lp.project(P, 2, false);
					printf("%f %f\n", p.x(), p.y());
				}
			}
			*/
			
			goto Break;
		}

		//generation
		if(!library.empty() && !initial.empty() && !isp0.empty() && 
		   skeleton_index >= 0 && step > 0 && simplification >=0.0f && simplification <= 1.0f)
		{
			if(!out_bdlsg.empty() || !out_bezier.empty() || !out_cache.empty() || !out_leaf.empty())
			{
				SimpleSkeletonGrower grower;
				grower.setup(library, initial, isp0, cache);
				grower.set_initial_skeleton(skeleton_index);
				
				if(verbose)
				{
					if(!grower._root)
						printf("grower._root NULL error\n");
				}

				if(exhaust)
					grower.grow_exhaustive(step);
				else if(potential)
					grower.grow_potential(step);
				else
					grower.grow(step);

				BDLSkeletonNode *skeleton = grower._root;
				if(skeleton)
				{
					skeleton->_radius = BDLSkeletonNode::main_branch_length(skeleton) / 6.0f;
					float height = std::max(10.0f, BDLSkeletonNode::max_height(skeleton));

					//rectify and simplify skeleton
					skeleton->_radius *= 9.5f;
					if(root_radius != 0.0f)
						skeleton->_radius = root_radius;
					BDLSkeletonNode::rectify_skeleton_tree(skeleton, osg::Vec3(0.0f, 0.0f, 0.0f), height, 0, maya);
					if(simplification != 1.0)
					{
						int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
						for(int i=0; i<remaining_edges*(1-simplification); i++)
							BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
					}

					//output bdlsg
					if(!out_bdlsg.empty())
					{
						BDLSkeletonNode::save_skeleton(skeleton, out_bdlsg.c_str());
					}

					//output blender
					if(!out_bezier.empty())
					{
						if(verbose)
							printf("Root's radius is %f.", skeleton->_radius);
						BDLSG2Bezier bezier(out_bezier.c_str());
						bezier.output(skeleton);
					}

					//output cache
					if(!out_cache.empty())
					{
						grower._surface.save(out_cache);
					}

					//output leaf
					if(!out_leaf.empty())
					{
						LeafGrower leaf_grower;
						leaf_grower.setup(grower._root, isp0, grower._k_d, out_leaf);

						//grow leaves (should match with skeleton_node)
						std::vector <osg::Vec3> all_v = leaf_grower.grow_planes();
						std::vector <osg::Vec2> all_tex = leaf_grower.grow(all_v);
						std::vector <osg::Vec3> all_pos;

						leaf_grower.billboard_data(all_pos, all_v, all_tex);
						
						//consturct billboard
						std::string::size_type idx = out_leaf.rfind('.');
						std::string path_new = out_leaf.substr(0, idx) + "_texture.png";

						fs::path full_path = fs::system_complete(fs::path(out_leaf));

						BillboardTree billboard(3, all_pos, all_v, all_tex, out_leaf.c_str());
						billboard._verbose = verbose;
						billboard.do_work(std::max(1, int(height / 6.0f)));
						billboard.generate_textures(path_new.c_str(), texture_multiplier);
						billboard.save_output(full_path.string().c_str());

						if(verbose)
							printf("BillboardTree is done.\n");
					}
				}
			}
		}
		else
		{
			//conversion: bdlsg ==> blender's blezier
			if(!in_bdlsg.empty() && (!out_bdlsg.empty() || !out_bezier.empty() || !out_xiao.empty()))
			{
				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();

				if(skeleton)
				{
					skeleton->_radius = BDLSkeletonNode::main_branch_length(skeleton) / 5.0f;
					//float height = std::max(10.0f, BDLSkeletonNode::max_height(skeleton));

					//rectify and simplify skeleton
					skeleton->_radius *= 9.5f;
					if(root_radius != 0.0f)
						skeleton->_radius = root_radius;
					//BDLSkeletonNode::rectify_skeleton_tree(skeleton, osg::Vec3(0.0f, 0.0f, 0.0f), height, 0, maya);
					BDLSkeletonNode::rectify_skeleton_tree(skeleton, Transformer::toVec3(skeleton));
					if(simplification != 1.0)
					{
						int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
						for(int i=0; i<remaining_edges*(1-simplification); i++)
							BDLSkeletonNode::collaspe_skeleton_tree(skeleton);
					}

					//output bdlsg
					if(!out_bdlsg.empty())
					{
						BDLSkeletonNode::save_skeleton(skeleton, out_bdlsg.c_str());
					}

					//output blender
					if(!out_bezier.empty())
					{
						if(verbose)
							printf("Root's radius is %f.\n", skeleton->_radius);
						BDLSG2Bezier bezier(out_bezier.c_str());
						bezier.output(skeleton);
					}

					//output for XiaoPeng
					if(!out_xiao.empty())
					{
						if(verbose)
							printf("Root's radius is %f.\n", skeleton->_radius);
						BDLSG2Bezier bezier(out_xiao.c_str());
						bezier.output_xiao(skeleton);
					}

					BDLSkeletonNode::delete_this(skeleton);
				}
			}
			//conversion: bdlsg ==> billboard leaves
			if(!in_bdlsg.empty() && !isp0.empty() && !cache.empty() && !out_leaf.empty())
			{
				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();

				//get k_d
				float k_d = -1.0f;
				VolumeSurface volsurf;
				k_d = volsurf.load(cache); //must have any extension other than isp0

				if(skeleton && k_d != -1.0f)
				{
					LeafGrower leaf_grower;
					leaf_grower.setup(skeleton, isp0, k_d, out_leaf);
					float height = std::max(10.0f, BDLSkeletonNode::max_height(skeleton));

					//grow leaves (should match with skeleton_node)
					std::vector <osg::Vec3> all_v = leaf_grower.grow_planes();
					std::vector <osg::Vec2> all_tex = leaf_grower.grow(all_v);
					std::vector <osg::Vec3> all_pos;

					leaf_grower.billboard_data(all_pos, all_v, all_tex);

					//consturct billboard
					std::string::size_type idx = out_leaf.rfind('.');
					std::string path_new = out_leaf.substr(0, idx) + "_texture.png";

					fs::path full_path = fs::system_complete(fs::path(out_leaf));

					BillboardTree billboard(3, all_pos, all_v, all_tex, out_leaf.c_str());
					billboard._verbose = verbose;
					billboard.do_work(std::max(1, int(height / 6.0f)));
					billboard.generate_textures(path_new.c_str(), texture_multiplier);
					billboard.save_output(full_path.string().c_str());

					if(verbose)
						printf("BillboardTree is done.\n");

					BDLSkeletonNode::delete_this(skeleton);
				}
			}
			//conversion: bdlsg ==> generic leaves
			if(!in_bdlsg.empty() && !in_gleaf.empty() && !out_generic_leaf.empty())
			{
				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();

				GenericLeafGrower leaf_grower;
				leaf_grower.set_verbose(verbose);
				leaf_grower.setup(skeleton, in_gleaf, leaf_scale);

				bool use_load = !in_eleaf.empty();

				if(!use_load)
					leaf_grower.grow();
				else
					leaf_grower.load(in_eleaf);

				//each billboard has 1 leaf
				if(!use_load)
					leaf_grower.save(out_generic_leaf);

				//each billboard has >= 1 leaf
				//consturct billboard
				if(use_load)
				{
					float height = std::max(10.0f, BDLSkeletonNode::max_height(skeleton));

					std::string::size_type idx = out_generic_leaf.rfind('.');
					std::string path_new = out_generic_leaf.substr(0, idx) + "_texture.png";//absolute path of new texture
					fs::path full_path = fs::system_complete(fs::path(out_generic_leaf));//absolute path of leaf file

					if(true)
					{
						//this block of code is for simplifying the generic leaves generated by GenericLeafGrower,
						//and apply modifier if necessary
						std::vector <osg::Vec3> all_pos = leaf_grower._all_pos;
						std::vector <osg::Vec3> all_v = leaf_grower._all_v;
						std::vector <osg::Vec2> all_tex = leaf_grower._all_tex;

						BillboardTree billboard(3, all_pos, all_v, all_tex, in_gleaf.c_str());
						billboard._verbose = verbose;
						billboard.do_work(std::max(1, int(height / 6.0f)));
						billboard.generate_textures(path_new.c_str(), texture_multiplier);
						path_new = billboard.save_output(full_path.string().c_str(), true);

						if(!in_gleaf_modifier.empty())
							GenericLeafGrower::colorize(in_gleaf_modifier, path_new);
					}
					else
					{
						//this block is for simplififying the leaves generated by class RealisticLeafGrower
					}

					if(verbose)
						printf("BillboardTree is done.\n");
				}

				BDLSkeletonNode::delete_this(skeleton);
			}
			//conversion: bdlsg ==> realistic leaves
			//require in_bdlsg, in_gleaf, leaf_scale, input_k_d, isp0, out_realistic_leaf for 'single image mode'
			if(!in_bdlsg.empty() && !in_gleaf.empty() && leaf_scale != 0.0f && input_k_d != -1.0f && !isp0.empty() && !out_realistic_leaf.empty())
			{
				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();
				
				RealisticLeafGrower leaf_grower;
				leaf_grower.set_verbose(verbose);
				leaf_grower.setup(skeleton, in_gleaf, leaf_scale);
                leaf_grower.set_parameters(leaf_grow_zone, leaf_radius_k, leaf_pedal, leaf_fuzziness);

				int w = 2048*1;
				leaf_grower.grow_single_image(w, w, input_k_d, isp0);
				std::string real_tex_path = leaf_grower.save(out_realistic_leaf);//realistic texture path

				//simplification
				if(false)
				{
					out_realistic_leaf += "_sim";
					std::string::size_type idx = out_realistic_leaf.rfind('.');
					std::string path_new = out_realistic_leaf.substr(0, idx) + "_texture.png";//absolute path of billboard texture
					fs::path full_path = fs::system_complete(fs::path(out_realistic_leaf));//absolute path of new leaf file

					std::vector <osg::Vec3> all_pos = leaf_grower._all_pos;
					std::vector <osg::Vec3> all_v = leaf_grower._all_v;
					std::vector <osg::Vec2> all_tex = leaf_grower._all_tex;

					float height = BDLSkeletonNode::max_height(skeleton);

					BillboardTree billboard(3, all_pos, all_v, all_tex, real_tex_path.c_str());
					billboard._verbose = verbose;
					billboard.do_work(std::max(1, int(height / 6.0f)));
					billboard.generate_textures(path_new.c_str(), texture_multiplier);
					billboard.save_output(full_path.string().c_str(), true);
				}
			}
            //testing palm tree
            if(!in_bdlsg.empty() && !in_gleaf.empty() && leaf_scale != 0.0f && is_palm_leaves)
            {
                printf("testing palm tree:\n");

				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();

				GenericLeafGrower leaf_grower;
				leaf_grower.set_verbose(verbose);
				leaf_grower.setup(skeleton, in_gleaf, leaf_scale);

                leaf_grower.grow_palm();
            }
			//conversion: bdlsg laser ==> realistic leaves
			//require in_bdlsg, in_gleaf, leaf_scale, cameras (generated by 'extract_camera'
			if(!in_bdlsg.empty() && !in_gleaf.empty() && leaf_scale != 0.0f && !in_laser_sp.empty() && !in_laser_cam.empty() && !out_realistic_leaf.empty())
			{
				BDLSkeletonLoader loader;
				loader.load_file(QString(in_bdlsg.c_str()));
				BDLSkeletonNode *skeleton = loader.construct_bdl_skeleton_tree();
				BDLSkeletonNode::rectify_skeleton_tree(skeleton, Transformer::toVec3(skeleton));
				
				RealisticLeafGrower leaf_grower;
				leaf_grower.set_verbose(verbose);
				leaf_grower.setup(skeleton, in_gleaf, leaf_scale);
				int w = 2048*1;
				leaf_grower.grow_laser(w, w, in_laser_sp, in_laser_cam);
				std::string real_tex_path = leaf_grower.save(out_realistic_leaf);//realistic texture path

				//simplification
				//if(false)
				//{
				//	out_realistic_leaf += "_sim";
				//	std::string::size_type idx = out_realistic_leaf.rfind('.');
				//	std::string path_new = out_realistic_leaf.substr(0, idx) + "_texture.png";//absolute path of billboard texture
				//	fs::path full_path = fs::system_complete(fs::path(out_realistic_leaf));//absolute path of new leaf file

				//	std::vector <osg::Vec3> all_pos = leaf_grower._all_pos;
				//	std::vector <osg::Vec3> all_v = leaf_grower._all_v;
				//	std::vector <osg::Vec2> all_tex = leaf_grower._all_tex;

				//	float height = BDLSkeletonNode::max_height(skeleton);

				//	BillboardTree billboard(3, all_pos, all_v, all_tex, real_tex_path.c_str());
				//	billboard._verbose = verbose;
				//	//billboard.do_work(std::max(1, int(height / 6.0f)));
				//	billboard.do_work(std::max(1, 2));
				//	billboard.generate_textures(path_new.c_str(), texture_multiplier);
				//	billboard.save_output(full_path.string().c_str(), true);
				//}
			}
			//conversion: realistic leaves ==> simplified leaves
			if(is_simplify_leaf && !in_eleaf.empty() && !out_leaf.empty())
			{
				//load from RealisticLeafGrower
				RealisticLeafGrower leaf_grower;
				std::string in_real_tex = leaf_grower.load(in_eleaf);

				std::string tex_path = out_leaf + "_texture.png";//absolute path of billboard texture, 'z_leaves_texture.png'
				fs::path leaf_path = fs::system_complete(fs::path(out_leaf));//absolute path of new leaf file, 'z_leaves'

				//debug
				//printf("simplifying (%s with texture %s) to (%s with texture %s)\n", in_eleaf.c_str(), in_real_tex.c_str(), leaf_path.string().c_str(), tex_path.c_str());

				std::vector <osg::Vec3> all_pos = leaf_grower._all_pos;
				std::vector <osg::Vec3> all_v = leaf_grower._all_v;
				std::vector <osg::Vec2> all_tex = leaf_grower._all_tex;

				BillboardTree billboard(3, all_pos, all_v, all_tex, in_real_tex.c_str());
				billboard._verbose = verbose;
				billboard.do_work(6);
				billboard.generate_textures(tex_path.c_str(), texture_multiplier);
				billboard.save_output(leaf_path.string().c_str(), false);
			}
			//conversion: skeleton + leaves ==> simplified skeleton
			if(is_simplify_skeleton && !in_bdlsg.empty() && !in_eleaf.empty() && !out_bdlsg.empty())
			{
				SkeletonSimplifier simplifier;
				simplifier.setup(in_bdlsg, in_eleaf);
				int before = Transformer::tree_size(simplifier._root);
				simplifier.simplify(10);
				int after = Transformer::tree_size(simplifier._root);
				BDLSkeletonNode::save_skeleton(simplifier._root, out_bdlsg.c_str());
				if(verbose)
					printf("Skeleton Simplification: before(%d) after(%d) %f%%\n", before, after, float(after-before)/before*100);
			}
		}
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << "\n";
		return 1;
	}

	Break:
	//if no error and in verbose mode, print out the running time
	if(verbose)
	{
		struct timeval end;
		gettimeofday(&end, NULL);
		double t1 = start.tv_sec+(start.tv_usec/1000000.0);
		double t2 = end.tv_sec+(end.tv_usec/1000000.0);
		double elapsed = t2-t1;
		printf("Completed in %.12f seconds.\n", elapsed);
	}

	return 0;
}
