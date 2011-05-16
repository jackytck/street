#include "LaserSkeletonGrower.h"
#include <queue>
#include "Transformer.h"
#include "ConvexHull2D.h"
#include "LaserPruner.h"
#include "BDLTreeParser.h"

LaserSkeletonGrower::LaserSkeletonGrower(): _root(NULL), _approaching_done(false)
{
	srand(time(NULL));
    //srand(0);
}

LaserSkeletonGrower::~LaserSkeletonGrower()
{
    BDLSkeletonNode::delete_this(_root);
}

void LaserSkeletonGrower::setup(std::string master_element, std::string master_skeleton, std::string laser_sp, std::string laser_cam, bool is_laser_raw, std::string output_dir)
{
    if(is_laser_raw)
    {
        printf("start: learning initial skeleton\n");

        //1. segment lower initial branch from tree points first
        //2. save the foliage in an un-raw simple point format
        _pruner.setup(laser_sp, laser_cam);
        //std::vector <osg::Vec3> initial_pts = _pruner.segment_initial_branch(laser_sp + "_foliage");
        std::vector <osg::Vec3> foliage;
        std::vector <int> pts_img;
        std::vector <osg::Vec2> pts_uv;
        osg::Vec3 fol_c;
        float threshold;
        std::vector <osg::Vec3> initial_pts = _pruner.segment_initial_branch_plane_sweep(foliage, pts_img, pts_uv, fol_c, threshold);
        if(initial_pts.empty())
        {
            printf("LaserSkeletonGrower::setup():initial_pts empty error\n");
            return;
        }
        printf("initial_pts(%d) threshold(%f)\n", int(initial_pts.size()), threshold);

        BDLSkeletonNode *initial;

        //2.5 for connected trees only
        if(false)
        {
            //2.5a cluster pts by ANN tree
            std::vector <std::vector <osg::Vec3> > clusters = Transformer::ann_cluster(initial_pts, 12);

            //2.5b for each cluster, find initial tree
            float max_init = Transformer::max_z(initial_pts);
            float min_init = Transformer::min_z(initial_pts);

            std::vector <BDLSkeletonNode *> init_roots;
            for(unsigned int c=0; c<clusters.size(); c++)
            {
                float min_c = Transformer::min_z(clusters[c]);
                if(min_c < (max_init - min_init) * 0.25f + min_init)//tree4
                {
                    //2.5c convert osg vector to BDL
                    BDL bdl("");
                    bdl._pointSize = clusters[c].size();
                    bdl._points = (BDLPoint *) calloc(bdl._pointSize, sizeof(BDLPoint));
                    for(unsigned int i=0; i<clusters[c].size(); i++)
                    {
                        osg::Vec3 pts = clusters[c][i];
                        bdl._points[i].x = pts.x();
                        bdl._points[i].y = pts.y();
                        bdl._points[i].z = pts.z();
                        bdl._points[i].w = 1.0f;

                        //debug
                        printf("v %f %f %f\n", pts.x(), pts.y(), pts.z());
                    }
                    //debug
                    printf("#################################################\n");

                    //2.5d learn the initial skeleton by shortest path algorithm by BDLTreeParser
                    BDLTreeParser treeParser(&bdl);
                    treeParser.build_ann_tree(c==0?5:5);//tree4,10:(4,7),(10,7)
                    BDLSkeletonNode *c_initial = treeParser.build_skeleton_tree();
                    if(c_initial)
                        init_roots.push_back(c_initial);
                    else
                        printf("treeParser.build_skeleton_tree() return null error\n");
                }
            }

            if(init_roots.empty())
            {
                printf("no cluster is found error\n");
                return;
            }

            //2.5c add a logical lower root to each custer initial
            osg::Vec3 logical_root(0.0f, 0.0f, 0.0f);
            for(unsigned int i=0; i<init_roots.size(); i++)
            {
                BDLSkeletonNode *node = init_roots[i];
                logical_root.x() += node->_sx;
                logical_root.y() += node->_sy;
                logical_root.z() += node->_sz;
            }
            logical_root /= init_roots.size();
            logical_root.z() -= 1.0f;//just hard-coded

            //2.5d extend end-points of each tree
            for(unsigned int i=0; i<init_roots.size(); i++)
            {
                osg::Vec3 translate = Transformer::toVec3(init_roots[i]) - logical_root;
                translate.z() = 0.0f;//translate by projected distance only
                Transformer::extend_end_points(init_roots[i], fol_c + translate + osg::Vec3(0.0f, 0.0f, 2.0f), threshold);
            }

            //2.5e connect all the trees as one entity
            BDLSkeletonNode *tmpY = new BDLSkeletonNode(logical_root.x(), logical_root.y(), logical_root.z());
            for(unsigned int i=0; i<init_roots.size(); i++)
            {
                init_roots[i]->_prev = tmpY;
                tmpY->_children.push_back(init_roots[i]);
            }

            //this is for Blender only, not limitation of logic
            initial = new BDLSkeletonNode(logical_root.x(), logical_root.y(), logical_root.z()-0.01f);
            tmpY->_prev = initial;
            initial->_children.push_back(tmpY);
        }
        else
        {
            //3. convert osg vector to BDL
            BDL bdl("");
            bdl._pointSize = initial_pts.size();
            bdl._points = (BDLPoint *) calloc(bdl._pointSize, sizeof(BDLPoint));
            for(unsigned int i=0; i<initial_pts.size(); i++)
            {
                osg::Vec3 pts = initial_pts[i];
                bdl._points[i].x = pts.x();
                bdl._points[i].y = pts.y();
                bdl._points[i].z = pts.z();
                bdl._points[i].w = 1.0f;

                //debug
                printf("v %f %f %f\n", pts.x(), pts.y(), pts.z());
            }

            //4. learn the initial skeleton by shortest path algorithm by BDLTreeParser
            BDLTreeParser treeParser(&bdl);
            //treeParser.build_ann_tree(3);//tree2,3,5,6,7,8,9,11,12,19,20:7,3,11,5,7,8,7,4,4,5,5
            treeParser.build_threshold_graph(0.25);
            initial = treeParser.build_skeleton_tree();//may fail if the threshold in build_threshold_graph is too restrictive

            if(!initial)
            {
                printf("treeParser.build_skeleton_tree() return null error\n");
                return;
            }

            //5. by the foliage center, extend the end-points of initial so that they are diven in the foliage
            //Transformer::extend_end_points(initial, fol_c, threshold);
            //Transformer::extend_end_points(initial, fol_c-osg::Vec3(0.0f, 0.0f, 2.5f), threshold);//bare tree only
        }

        _fol_cut = threshold;

        //6. mirror the foliage over the root of initial skeleton
        std::vector <int> pts_img2;
        std::vector <osg::Vec2> pts_uv2;
        if(foliage.size() == pts_img.size() && foliage.size() == pts_uv.size())
        {
            osg::Vec3 init_root = Transformer::toVec3(initial);
            float init_root_x = init_root.x();
            std::vector <osg::Vec3> mirrored;
            for(unsigned int i=0; i<foliage.size(); i++)
            {
                osg::Vec3 p = foliage[i];
                if(p.x() < init_root_x)
                //if(p.x() > init_root_x)
                //if(false)//tree10
                {
                    osg::Vec3 mp(init_root_x+(init_root_x-p.x()), p.y(), p.z());
                    mirrored.push_back(p);
                    mirrored.push_back(mp);
                    pts_img2.push_back(pts_img[i]);
                    pts_uv2.push_back(pts_uv[i]);
                }
                else
                    mirrored.push_back(p);

                pts_img2.push_back(pts_img[i]);
                pts_uv2.push_back(pts_uv[i]);
            }

            _pruner.save_foliage(output_dir + "/foliage", mirrored, pts_img2, pts_uv2);
        }

        //7. export the the learnt skeleton
        std::string new_init = output_dir + "/initial";
        //printf("saving new_init(%s)\n", new_init.c_str());
        BDLSkeletonNode::save_skeleton(initial, new_init.c_str());
        BDLSkeletonNode::delete_this(initial);
    }
    else
    {
        //load library
        _library.load_default_elements(master_element);
        _library.load_default_skeleton(master_skeleton);

        if(_library._library_element.empty() || _library._library_skeleton.empty())
            printf("LaserSkeletonGrower::setup:_library is empty error\n");

        //setup laser pruner
        _pruner.setup(laser_sp, laser_cam);
    }
}

void LaserSkeletonGrower::set_initial_skeleton(int k)
{
    if(k >= 0 && k < int(_library._library_skeleton.size()))
    {
        BDLSkeletonNode *init_root = _library._library_skeleton[k];
        //float mb_length_d = BDLSkeletonNode::main_branch_length(init_root);

        //new root
        if(_root)
            BDLSkeletonNode::delete_this(_root);
        _root = BDLSkeletonNode::copy_tree(init_root);

        //set _close_threshold
        //_close_threshold = mb_length_d / 3.5f;//4.2f;//8.0f;
        //_close_threshold = mb_length_d / 15.0f;//laser
        //_close_threshold = 0.2f;//laser connected
        //_close_threshold = mb_length_d / 18.0f;//tree6
        _close_threshold = 0.15;//laser general

		if(!_root)
			printf("LaserSkeletonGrower::set_initial_skeleton():_root equals NULL error\n");
    }
}

BDLSkeletonNode *LaserSkeletonGrower::pick_branch(bool space_aware)
{
    BDLSkeletonNode *ret(NULL);
    if(!_root)
        return ret;

    //store all possible branches that can be replaced
    std::vector <BDLSkeletonNode *> replaced_candidates;
    std::vector <BDLSkeletonNode *> nodes = bfs();
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        if(nodes[i]->_prev_support)
            replaced_candidates.push_back(nodes[i]);
    }

    //evaulate the probability of each possible candidate
    //then pick the largest one

	//approach 1: grow node with the minimum generation first
	if(!space_aware)
	{
		std::vector <float> probabilities(replaced_candidates.size(), 1.0f);
		int min_gen = -1;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			int gen = replaced_candidates[i]->_generation;
			if(gen > 0)
				if(min_gen == -1 || gen < min_gen)
					min_gen = gen;
		}
		//all the min_gen replaced_candidates
		std::vector <BDLSkeletonNode *> min_gen_nodes;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
			if(replaced_candidates[i]->_generation <= min_gen+3)
				min_gen_nodes.push_back(replaced_candidates[i]);

		if(true)
			if(!min_gen_nodes.empty())
				ret = min_gen_nodes[rand()%min_gen_nodes.size()];

		//do it randomly
		if(false)
			if(!replaced_candidates.empty())
				ret = replaced_candidates[rand()%replaced_candidates.size()];
	}

	//approach 2: grow node which is farest from the volume bound + smaller generation
	if(false && space_aware && _pruner.is_loaded())
	{
		//cost = k1*cost_d + k2*cost_g
		//larger closest distance, smaller cost
		//smaller generation, smaller cost
		float min_cost = -1.0f; 
		float k1 = 0.0f, k2 = 1.5f;

		//O(n^2) to find the farest distance of the closest <node, cloud> pair
		//a. find cost_d
		std::vector <float> cost_ds = std::vector <float> (replaced_candidates.size(), 0.0f);
		float largest_d = -1.0f;

		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			osg::Vec3 cur_r = Transformer::toVec3(replaced_candidates[i]);

			float closest = -1.0f;
			for(unsigned int j=0; j<_pruner._surface_pts.size(); j++)
			{
				osg::Vec3 cur_s = _pruner._surface_pts[j];
				float cur_dist = (cur_r - cur_s).length2();

				if(closest == -1.0f || cur_dist < closest)
					closest = cur_dist;
			}
			cost_ds[i] = closest; 

			if(largest_d == -1.0f || closest > largest_d)
			{
				largest_d = closest;
				ret = replaced_candidates[i];
			}
		}

		//normalize cost_d
		if(largest_d == 0.0f)
			return ret;
		for(unsigned int i=0; i<cost_ds.size(); i++)
			cost_ds[i] /= largest_d;

		//b. find cost_g
		std::vector <float> cost_gs = std::vector <float> (replaced_candidates.size(), 0.0f);
		int largest_g = 0;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			int gen = replaced_candidates[i]->_generation;
			cost_gs[i] = gen;
			if(gen > 0)
				if(largest_g == -1 || gen > largest_g)
					largest_g = gen;
		}

		//normalize cost_g
		if(largest_g == 0)
			return ret;
		for(unsigned int i=0; i<cost_gs.size(); i++)
			cost_gs[i] /= float(largest_g);

		//c. caculate costs
		if(largest_g > 8) //hard-coded
			k1 = 0.8f;
		std::vector <float> costs = std::vector <float> (replaced_candidates.size(), 0.0f);
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
			costs[i] = k1 * (1.0f-cost_ds[i]) + k2 * cost_gs[i];

		//d. get min_cost
		for(unsigned int i=0; i<costs.size(); i++)
			if(min_cost == -1.0f || costs[i] < min_cost)
			{
				min_cost = costs[i];
				ret = replaced_candidates[i];
			}
	}

	//approach 3: find the least projected density among all nodes
	if(true && _pruner.is_loaded())
	{
		//a. find the bounds from all surface points
		float right = -1.0f, left = -1.0f, top = -1.0f, bottom = -1.0f;
		for(unsigned int i=0; i<_pruner._surface_pts.size(); i++)
		{
			osg::Vec3 cur_s = _pruner._surface_pts[i];
			if(right == -1.0f || cur_s.x() > right)
				right = cur_s.x();
			if(left == -1.0f || cur_s.x() < left)
				left = cur_s.x();
			if(top == -1.0f || cur_s.z() > top)
				top = cur_s.z();
			if(bottom == -1.0f || cur_s.z() < bottom)
				bottom = cur_s.z();
		}

		//b. new origin, width, height
		osg::Vec3 origin(left, 0.0f, 0.0f);
		int buff_offset = 50;
		int width = right - left + buff_offset;
		int height = top + buff_offset;
		//printf("w(%d) h(%d)\n", width, height);

		//c. density map
		std::vector <std::vector <int> > density_map(width, std::vector <int>(height, 0));

		//d1. initialize density map with all nodes
		for(unsigned int i=0; i<nodes.size(); i++)
		{
			BDLSkeletonNode *node = nodes[i];
			osg::Vec3 punch = Transformer::toVec3(node) - origin;

			if(punch.z() < bottom)
				continue;

			//vote around point punch
			for(int j=-3; j<6; j++)
				for(int k=-3; k<6; k++)
				{
					//printf("x(%d) y(%d)\n", int(punch.x()+j), int(punch.z()+k));
					int int_j = punch.x() + j;
					int int_k = punch.z() + k;

					if(int_j >= 0 && int_k >= 0)
						density_map[int_j][int_k]++;
				}
		}
		//d2. initialize density map with simple pruner
		for(int i=0; i<width; i++)
			for(int j=0; j<height; j++)
			{
				osg::Vec3 field = origin + osg::Vec3(i, 0.0f, j);
				if(!_pruner.is_inside(field))
					density_map[i][j] = -10;//99999; //means forbidden or infinitely dense
			}

		//test: obtain convex hull of replaced_candidates
		if(_approaching_done) //if approaching the end of the process
		{
			std::vector <osg::Vec2> r_can;
			for(unsigned int i=0; i<replaced_candidates.size(); i++)
				if(_pruner.is_inside(Transformer::toVec3(replaced_candidates[i])))
					r_can.push_back(osg::Vec2(replaced_candidates[i]->_sx, replaced_candidates[i]->_sz));

			ConvexHull2D hull;
			hull.load_points(r_can);
			std::vector <int> hull_idx = hull.work();
			std::vector <BDLSkeletonNode *> replaced_candidates_hull;
			for(unsigned int i=0; i<hull_idx.size(); i++)
				replaced_candidates_hull.push_back(replaced_candidates[hull_idx[i]]);
			replaced_candidates = replaced_candidates_hull;
		}

		//e. find the least density point
		std::vector <float> cost_ds = std::vector <float> (replaced_candidates.size(), 0.0f);
		float largest_d = -1.0f;

		long long least_dense_value = -1;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			BDLSkeletonNode *node = replaced_candidates[i];
			osg::Vec3 punch = Transformer::toVec3(node) - origin;

			if(punch.x() >= 0 && punch.x() < width-1 && punch.z() >= 0 && punch.z() < height-1)
				if(density_map[punch.x()][punch.z()] < 0)
					continue;

			long long sum = 0;

			//sum up the votes around the punch
			for(int j=-3; j<6; j++)
				for(int k=-3; k<6; k++)
				{

					int int_j = punch.x() + j;
					int int_k = punch.z() + k;

					if(int_j >= 0 && int_k >= 0)
						sum += density_map[int_j][int_k];
				}

			cost_ds[i] = sum;
			if(largest_d == -1.0f || sum > largest_d)
				largest_d = sum;

			if(least_dense_value == -1 || sum < least_dense_value)
			{
				least_dense_value = sum;
				ret = replaced_candidates[i];
			}
		}

		//f. normalize cost_ds
		if(largest_d == -1.0f)
			return ret;
		for(unsigned int i=0; i<cost_ds.size(); i++)
			cost_ds[i] /= float(largest_d);

		//g. find the lenghts of the replacement rods
		std::vector <float> cost_ls = std::vector <float> (replaced_candidates.size(), 0.0f);
		float largest_l = -1.0f;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			float len = (Transformer::toVec3(replaced_candidates[i]) - Transformer::toVec3(replaced_candidates[i]->_prev_support)).length2();
			cost_ls[i] = len;
			if(largest_l == -1.0f || len > largest_l)
				largest_l = len;
		}

		//h. normalize cost_gs
		if(largest_l == 0)
			return ret;
		for(unsigned int i=0; i<cost_ls.size(); i++)
			cost_ls[i] /= float(largest_l);

		//i. caculate costs
		int k1 = 0.4f, k2 = 0.6f;
		std::vector <float> costs = std::vector <float> (replaced_candidates.size(), 0.0f);
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
			costs[i] = k1 * (1.0f-cost_ls[i]) + k2 * cost_ds[i];

		//j. get min_cost
		float min_cost = -1.0f;
		for(unsigned int i=0; i<costs.size(); i++)
			if(min_cost == -1.0f || costs[i] < min_cost)
			{
				min_cost = costs[i];
				ret = replaced_candidates[i];
			}
	}

    //printf("return normally\n");

    return ret;
}

std::vector <BDLSkeletonNode *> LaserSkeletonGrower::replace_branch(BDLSkeletonNode *branch_tail, LibraryElement element, int angle)
{
    std::vector <BDLSkeletonNode *> ret;

    //debug
    if(false)
    {
        BDLSkeletonNode *ptr = branch_tail;
        BDLSkeletonNode *ptr2 = element._root;
        printf("%p %p replacing(%f %f %f) with root(%f %f %f)\n", ptr, ptr->_prev, ptr->_sx, ptr->_sy, ptr->_sz, ptr2->_sx, ptr2->_sy, ptr2->_sz);
    }

    if(!element.isValid() || !branch_tail || !branch_tail->_prev_support)
    {
        //printf("LaserSkeletonGrower::replace_branch():element.isValid(%d):branch_tail(%p)->_prev_support(%p) error\n", element.isValid(), branch_tail, branch_tail->_prev_support);
        printf("LaserSkeletonGrower::replace_branch():element._root(%p) sup_par(%p) sup_child(%p)\n", element._root, element._support_branch._par, element._support_branch._child);
        return ret;
    }

    //properly place the element into the replacement branch
    Transformer::transform(branch_tail, angle, element);

    //tranverse supporting branch in element and insert each node accordingly
    BDLSkeletonNode *cur = element._support_branch._child, *end = element._support_branch._par, *last = NULL;
    std::vector <BDLSkeletonNode *> intermediate_nodes;

    while(cur != end)
    {
        //first node
        if(!last)
        {
            //add all cur's children to branch_tail's children
            for(unsigned int i=0; i<cur->_children.size(); i++)
            {
                BDLSkeletonNode *child = cur->_children[i];

                //if not too close to existing tree, then grow it
                if(!is_too_close(cur, child))
                {
                    branch_tail->_children.push_back(child);
                    child->_prev = branch_tail;

                    if(child->_prev_support == cur)
                        child->_prev_support = branch_tail;

                    ret.push_back(child);
                }
            }
        }
        //intermediate nodes, not including last node
        else
        {
            //dis-integrate the chain for later use
            BDLSkeletonNode::remove_child(cur, last);
            intermediate_nodes.push_back(cur);
        }

        last = cur;
        cur = cur->_prev;
    }
    //last node
    //add all end's children (excluding last) to branch_tail->_prev_support
    for(unsigned int i=0; i<end->_children.size(); i++)
    {
        BDLSkeletonNode *child = end->_children[i];
        if(child != last)
        {
            if(!is_too_close(end, child))
            {
                branch_tail->_prev_support->_children.push_back(child);
                child->_prev = branch_tail->_prev_support;

                if(child->_prev_support == end)
                    child->_prev_support = branch_tail->_prev_support;

                ret.push_back(child);
            }
        }
    }

    //for each intermediate node, find the cloesest segment from resulting tree and add it
    for(unsigned int i=0; i<intermediate_nodes.size(); i++)
    {
        BDLSkeletonNode *node = intermediate_nodes[i];
        SLink link = Transformer::closest_segment(branch_tail, node);

        if(link._par && link._child)
        {
            BDLSkeletonNode::remove_child(link._par, link._child);

            link._par->_children.push_back(node);
            node->_prev = link._par;

            //add all old children of node to ret
            for(unsigned int j=0; j<node->_children.size(); j++)
                ret.push_back(node->_children[j]);

            node->_children.push_back(link._child);
            link._child->_prev = node;
        }
    }

    //part of the element is added to the tree, and should not be deleted entirely by its destructor
    element._is_added = true;
    //but the endpoints should be discarded
    delete element._support_branch._child;
    delete end;

    //experimental: each branch can only be replaced once
    branch_tail->_prev_support = NULL;

    //return all sub-tree for pruning
    return ret;
}

std::vector <BDLSkeletonNode *> LaserSkeletonGrower::replace_branch_unchanged(BDLSkeletonNode *branch_tail, LibraryElement element, int angle)
{
    std::vector <BDLSkeletonNode *> ret;

    if(!element.isValid() || !branch_tail || !branch_tail->_prev_support)
    {
        //printf("SimpleSkeletonGrower::replace_branch():element.isValid(%d):branch_tail(%p) error\n", element.isValid(), branch_tail);
        //printf("element._root(%p) sup_par(%p) sup_child(%p)\n", element._root, element._support_branch._par, element._support_branch._child);
        return ret;
    }

    //properly place the element into the replacement branch
    Transformer::transform(branch_tail, angle, element);

    //tranverse supporting branch in element and insert each node accordingly
    BDLSkeletonNode *cur = element._support_branch._child, *end = element._support_branch._par, *last = NULL;
    std::vector <BDLSkeletonNode *> intermediate_nodes;

    while(cur != end)
    {
        //first node
        if(!last)
        {
            //add all cur's children to branch_tail's children
            for(unsigned int i=0; i<cur->_children.size(); i++)
            {
                BDLSkeletonNode *child = cur->_children[i];

                //if not too close to existing tree, then grow it
                if(!is_too_close(cur, child))
                {
                    //branch_tail->_children.push_back(child);
                    //child->_prev = branch_tail;

                    //if(child->_prev_support == cur)
                    //    child->_prev_support = branch_tail;

                    ret.push_back(child);
                }
            }
        }
        //intermediate nodes, not including last node
        else
        {
            //dis-integrate the chain for later use
            BDLSkeletonNode::remove_child(cur, last);
            intermediate_nodes.push_back(cur);
        }

        last = cur;
        cur = cur->_prev;
    }
    //last node
    //add all end's children (excluding last) to branch_tail->_prev_support
    for(unsigned int i=0; i<end->_children.size(); i++)
    {
        BDLSkeletonNode *child = end->_children[i];
        if(child != last)
        {
            if(!is_too_close(end, child))
            {
                //branch_tail->_prev_support->_children.push_back(child);
                //child->_prev = branch_tail->_prev_support;

                //if(child->_prev_support == end)
                //    child->_prev_support = branch_tail->_prev_support;

                ret.push_back(child);
            }
        }
    }

    //for each intermediate node, find the cloesest segment from resulting tree and add it
    for(unsigned int i=0; i<intermediate_nodes.size(); i++)
    {
        BDLSkeletonNode *node = intermediate_nodes[i];
        SLink link = Transformer::closest_segment(branch_tail, node);

        if(link._par && link._child)
        {
            //BDLSkeletonNode::remove_child(link._par, link._child);

            //link._par->_children.push_back(node);
            //node->_prev = link._par;

            //add all old children of node to ret
            for(unsigned int j=0; j<node->_children.size(); j++)
                ret.push_back(node->_children[j]);

            //node->_children.push_back(link._child);
            //link._child->_prev = node;
        }
    }

    //part of the element is added to the tree, and should not be deleted entirely by its destructor
    element._is_added = true;
    //but the endpoints should be discarded
    delete element._support_branch._child;
    delete end;

    //experimental: each branch can only be replaced once
    //branch_tail->_prev_support = NULL;

    //return all sub-tree for pruning
    return ret;
}

std::vector <BDLSkeletonNode *> LaserSkeletonGrower::bfs()
{
    std::vector <BDLSkeletonNode *> ret;
    if(!_root)
        return ret;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    _root->_generation = 0;
    Queue.push(_root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        ret.push_back(front);

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            front->_children[i]->_generation = front->_generation + 1;
            Queue.push(front->_children[i]);
        }
    }

    return ret;
}

void LaserSkeletonGrower::prune(BDLSkeletonNode *root)
{
    if(!root || !_root)
        return;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        osg::Vec3 hit;
        bool hit_valid;
        //assume front->_prev exists
        if(!_pruner.is_inside(Transformer::toVec3(front), Transformer::toVec3(front->_prev), hit, hit_valid))
        {
            //cannot be replaced again
            if(true)
                if(front->_prev)
                    front->_prev->_prev_support = NULL;

			//should also cut the edge besides the vertex
            //only do so if hit is valid
            //since the ray cast is too unstable, just abandon it
			if(false && hit_valid && front->_prev)
			{
				//add back the intercepting point
				BDLSkeletonNode *jout = new BDLSkeletonNode(hit.x(), hit.y(), hit.z());
				jout->_prev = front->_prev;
				front->_prev->_children.push_back(jout);
			}

            //printf("deleting %p\n", front);
            BDLSkeletonNode::delete_this(front);
        }
        else
            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
    }
}

void LaserSkeletonGrower::prune(std::vector<BDLSkeletonNode *> roots)
{
    for(unsigned int i=0; i<roots.size(); i++)
        prune(roots[i]);
}

void LaserSkeletonGrower::grow(int no_branch)
{
    std::vector <LibraryElement> elements = _library._library_element;

    for(int i=0; i<no_branch; i++)
    {
        //prune(replace_branch(pick_branch(), elements[rand()%elements.size()], rand()%360));
        BDLSkeletonNode *tail = pick_branch();
        if(!tail)
        {
            printf("SimpleSkeletonGrower::grow():converged at %d-th step.\n", i+1);
            break;
        }
        std::vector <BDLSkeletonNode *> subtree = replace_branch(tail, elements[rand()%elements.size()], rand()%360);
        prune(subtree);
    }
}

void LaserSkeletonGrower::grow_potential(int no_branch)
{
	//overview: maintain a volume to represent the potential energy at each step,
	//then find the min_element_index and the min_rotate that require the minimum potential energy

	if(!_pruner.is_loaded() || !_root)
    {
        printf("LaserSkeletonGrower::grow_potential():_pruner not loaded or _root(%p) error\n", _root);
		return;
    }

	//find the dimensions of the volume
	float v_height = -1.0f;
	osg::Vec3 origin(0.0f, 0.0f, 0.0f);
	for(unsigned int i=0; i<_pruner._surface_pts.size(); i++)
	{
		float cur = (_pruner._surface_pts[i] - origin).length2();
		if(v_height == -1.0f || cur > v_height)
			v_height = cur;

        //debug
        //osg::Vec3 p = _pruner._surface_pts[i];
        //printf("%f %f %f\n", p.x(), p.y(), p.z());
	}
	if(v_height == -1.0f)
    {
        printf("LaserSkeletonGrower::grow_potential():v_height -1.0f error\n");
		return;
    }

	//v_height = sqrt(v_height) * 1.5f; //old
	v_height = sqrt(v_height) * 2.0f; //new
	float v_step = v_height / 100.0f;
	osg::Vec3 corner(-v_height/2.0f, -v_height/2.0f, 0.0f);
	SimpleVolumeFloat space(v_height, v_height, v_height, v_step, v_step, v_step);
	float energy_effect = v_step * 18;
    //printf("v_height(%f) v_step(%f) v_height/v_step(%f)\n", v_height, v_step, v_height/v_step);

	//negative charges on initial skeleton
	//bfs
	std::queue <BDLSkeletonNode *> Queue;
	Queue.push(_root);

    std::map <BDLSkeletonNode *, bool> except_from_pruned_dict;
    if(true)//single tree
    {
        std::vector <BDLSkeletonNode *> except_from_pruned = Transformer::leaves(_root);
        for(unsigned int i=0; i<except_from_pruned.size(); i++)
        {
            if(true && except_from_pruned[i]->_sz > _fol_cut * 0.85f)
                except_from_pruned_dict[except_from_pruned[i]] = true;
        }
    }
    else//connected trees
    {
        std::vector <BDLSkeletonNode *> nodes = Transformer::bfs(_root);
        for(unsigned int i=0; i<nodes.size(); i++)
            except_from_pruned_dict[nodes[i]] = true;
    }

	while(!Queue.empty())
	{
		BDLSkeletonNode *front = Queue.front();
		Queue.pop();

		osg::Vec3 cur = Transformer::toVec3(front) - corner;
		space.add_potential(cur.x(), cur.y(), cur.z(), -3, energy_effect);

		for(unsigned int i=0; i<front->_children.size(); i++)
			Queue.push(front->_children[i]);
	}

	//positive charges on volume surface
	for(unsigned int i=0; i<_pruner._surface_pts.size(); i++)
	{
		osg::Vec3 cur = _pruner._surface_pts[i] - corner;
		space.add_potential(cur.x(), cur.y(), cur.z(), 1, energy_effect*1.5f);
	}

    std::vector <LibraryElement> elements = _library._library_element;

    for(int i=0; i<no_branch; i++)
    {
		//pick a tail first
		//not by minimum generation, but by farest position from surface point
        BDLSkeletonNode *tail = pick_branch(true);
        if(!tail)
        {
            printf("LaserSkeletonGrower::grow():converged at %d-th step.\n", i+1);
            if(i<5)
            {
                std::vector <BDLSkeletonNode *> h_leafs = Transformer::leaves(_root);
                for(unsigned int l=0; l<h_leafs.size(); l++)
                    if(h_leafs[l]->_prev_support)
                    {
                        tail = h_leafs[l];
                        break;
                    }
            }
            else
                break;

            if(!tail)
                break;
        }

		//compute the node-->id dict
		std::map <BDLSkeletonNode *, int> dict;
		int id = 0;
		
		//bfs on source
		std::queue <BDLSkeletonNode *> Queue;
		Queue.push(_root);

		while(!Queue.empty())
		{
			BDLSkeletonNode *front = Queue.front();
			Queue.pop();

			dict[front] = id;

			for(unsigned int j=0; j<front->_children.size(); j++)
				Queue.push(front->_children[j]);

			id++;
		}

		int tail_id = dict[tail];

		//####trial starts####
		int min_element_index = -1;
		int min_rotate = -1;
		float min_energy = -1.0f;

		for(unsigned int e=0; e<elements.size(); e++)
			for(int r=0; r<360; r+=30)
			{
				LibraryElement element_trial = elements[e];

				if(false)
				{
					//copy the original(_root) tree and find the corresponding tail
					BDLSkeletonNode *copy_root = BDLSkeletonNode::copy_tree(_root);
					BDLSkeletonNode *copy_tail = NULL;

					//bfs on target, Queue is now empty at this point
					id = 0;
					Queue.push(copy_root);

					while(!Queue.empty())
					{
						BDLSkeletonNode *front = Queue.front();
						Queue.pop();

						if(id == tail_id)
							copy_tail = front;

						for(unsigned int j=0; j<front->_children.size(); j++)
							Queue.push(front->_children[j]);

						id++;
					}

					//branch replacement
					std::vector <BDLSkeletonNode *> subtree_trial = replace_branch(copy_tail, element_trial, r);
					std::vector <BDLSkeletonNode *> non_pruned_nodes = after_pruned(subtree_trial); //but logically
					//prune(subtree_trial); //no need to do it actually
				}

				std::vector <BDLSkeletonNode *> subtree_trial = replace_branch_unchanged(tail, element_trial, r);
				std::vector <BDLSkeletonNode *> non_pruned_nodes = after_pruned(subtree_trial); //but logically

				//compute the potential energy
				//float energy = non_pruned_nodes.empty() ? 23418715 : compute_potential_energy(subtree_trial, space);
				float energy = compute_potential_energy(non_pruned_nodes, space);

				if(energy == 23418715.0f) //to let it grow a little bit randomly even it's completely outside the segmentation
				{
					min_element_index = rand()%elements.size();
					min_rotate = rand()%360;
					min_energy = 0.0f;
				}
				else if(min_energy == -1.0f || energy > min_energy) //the max in value is the min potential energy for negative charge
				{
					min_element_index = e;
					min_rotate = r;
					min_energy = energy;
				}

				//delete the copied tree
				//BDLSkeletonNode::delete_this(copy_root);
				for(unsigned int j=0; j<subtree_trial.size(); j++)
					delete subtree_trial[j];
			}
		//####trial ends####

		//actual replacement
		if(min_energy != -1.0f)
		{
			//printf("tail(%p) min_energy(%f) min_element_index(%d) min_rotate(%d)\n", tail, min_energy, min_element_index, min_rotate);
			std::vector <BDLSkeletonNode *> subtree = replace_branch(tail, elements[min_element_index], min_rotate);
            std::vector <BDLSkeletonNode *> update_nodes = subtree;

            if(except_from_pruned_dict.find(tail) == except_from_pruned_dict.end())
            {
                update_nodes = after_pruned(subtree);
                prune(subtree);
            }
            //printf("added %d nodes\n", int(update_nodes.size()));

			//update space
			for(unsigned int j=0; j<update_nodes.size(); j++)
			{
				osg::Vec3 cur = Transformer::toVec3(update_nodes[j]) - corner;
				space.add_potential(cur.x(), cur.y(), cur.z(), -3, energy_effect); //negative charge
			}
		}
		//if(i > no_branch * 0.5f && i < no_branch * 0.75f)
		if(i > no_branch * 0.75f)
		{
			if(!_approaching_done)
				_close_threshold /= 2.0f;
			_approaching_done = true;
		}
		else if(i >= no_branch * 0.90f)
			_approaching_done = false;

        //printf("%d\n", i);
    }

	//aftermath: should be shared for different grow methods
	//prune strictly on entire tree
    if(true)
    {
        backward_grow();
        backward_grow(true);
        _pruner.back_trace_prune(_root);
    }
}

void LaserSkeletonGrower::grow_exhaustive(int no_branch)
{
    std::vector <LibraryElement> elements = _library._library_element;

    for(int i=0; i<no_branch; i++)
    {
		//pick a tail first
        BDLSkeletonNode *tail = pick_branch();
        if(!tail)
        {
            printf("LaserSkeletonGrower::grow():converged at %d-th step.\n", i+1);
            break;
        }

		//compute the node-->id dict
		std::map <BDLSkeletonNode *, int> dict;
		int id = 0;
		
		//bfs on source
		std::queue <BDLSkeletonNode *> Queue;
		Queue.push(_root);

		while(!Queue.empty())
		{
			BDLSkeletonNode *front = Queue.front();
			Queue.pop();

			dict[front] = id;

			for(unsigned int j=0; j<front->_children.size(); j++)
				Queue.push(front->_children[j]);

			id++;
		}

		int tail_id = dict[tail];

		//####trial starts####
		int min_element_index = -1;
		int min_rotate = -1;
		float min_energy = -1.0f;

		for(unsigned int e=0; e<elements.size(); e++)
			for(int r=0; r<360; r+=30)
			{
				LibraryElement element_trial = elements[e];
				//copy the original(_root) tree and find the corresponding tail
				BDLSkeletonNode *copy_root = BDLSkeletonNode::copy_tree(_root);
				BDLSkeletonNode *copy_tail = NULL;

				//bfs on target, Queue is now empty at this point
				id = 0;
				Queue.push(copy_root);

				while(!Queue.empty())
				{
					BDLSkeletonNode *front = Queue.front();
					Queue.pop();

					if(id == tail_id)
						copy_tail = front;

					for(unsigned int j=0; j<front->_children.size(); j++)
						Queue.push(front->_children[j]);

					id++;
				}

				//branch replacement
				std::vector <BDLSkeletonNode *> subtree_trial = replace_branch(copy_tail, element_trial, r);
				prune(subtree_trial);

				//find energy
				float energy = compute_energy(copy_root);
				if(energy >= 0.0f && (min_energy == -1.0f || energy < min_energy))
				{
					min_element_index = e;
					min_rotate = r;
					min_energy = energy;
				}

				//delete the copied tree
				BDLSkeletonNode::delete_this(copy_root);
			}
		//####trial ends####

		//actual replacement
		if(min_energy != -1.0f)
		{
			std::vector <BDLSkeletonNode *> subtree = replace_branch(tail, elements[min_element_index], min_rotate);
			prune(subtree);
		}
    }

    _pruner.back_trace_prune(_root);
}

float LaserSkeletonGrower::compute_energy(BDLSkeletonNode *root)
{
	float ret = -1.0f;
	if(!root || !_pruner.is_loaded())
		return ret;

	ret = 0.0f; // will it overflow?
	std::vector <osg::Vec3> attractors = _pruner._surface_pts;
	for(unsigned int i=0; i<attractors.size(); i++)
	{
		osg::Vec3 si = attractors[i];
		float min_dist = Transformer::point_tree_dist(root, si);
		if(min_dist >= 0.0f)
			ret += min_dist;
	}

	return ret;
}

/*
//1. bfs to mark all the already supported voxel
//2. for each unsupported voxel, find closest node in _root
//3. if dist between them is smaller than a 2*_close_threshold, directly grow it with straight line
//4. otherwise, if the edge E gives a divergent angle smaller than 30 degrees, replace it with a random element
//5. else search for the most similar existing sub-tree in terms of divergent angle by doing bfs inside a bfs
//6. construct an element out of that and replace the edge E with it
//7. for all newly grown nodes, mark the corresponding voxel as supported
//8. repeat step 2 until all voxels are supported
*/
void LaserSkeletonGrower::backward_grow(bool dense)
{
    //step 0: pre-check
    std::vector <LibraryElement> elements = _library._library_element;
    if(!_pruner.is_loaded() || !_root || elements.empty())
    {
        printf("LaserSkeletonGrower::backward_grow:_pruner(%d), _root(%p), elements(%d) error\n", _pruner.is_loaded(), _root, elements.empty());
        return;
    }

    //step 1: mark supported leaves
    std::queue <BDLSkeletonNode *> Queue;
	Queue.push(_root);

	while(!Queue.empty())
	{
		BDLSkeletonNode *front = Queue.front();
		Queue.pop();

        if(false && dense)
            _pruner._voxel_dict_supported[_pruner.key(Transformer::toVec3(front))] = true;
        else
            _pruner.set_support(Transformer::toVec3(front), front->_children.empty() ? 0.2f : 1.0f);

		for(unsigned int i=0; i<front->_children.size(); i++)
			Queue.push(front->_children[i]);
	}

    //some add-hoc logistics for more pretty growing
    std::vector <BDLSkeletonNode *> trees = bfs();
    BDLSkeletonNode *copied_root = BDLSkeletonNode::copy_tree(_root);
    std::map <BDLSkeletonNode *, int> grown_cnt;

    //step 2: loop each unsupported voxel
    //find a permuataion
    std::vector <int> unorder(_pruner._upper, 0);
    for(int i=0; i<_pruner._upper; i++)
        unorder[i] = i;
    for(unsigned int i=unorder.size(); i>1; i--)
    {
        int j = rand()%i;
        int tmp = unorder[j];
        unorder[j] = unorder[i-1];
        unorder[i-1] = tmp;
    }

    //for(int i=0; i<_pruner._upper; i++)//loop in asscending order
    for(unsigned int i=0; i<unorder.size(); i++)
    {
        if(!_pruner._voxel_dict_supported[i] && !_pruner._voxel_dict[i].empty())
        {
            //just pick a random one
            int select = rand() % _pruner._voxel_dict[i].size();
            osg::Vec3 p = _pruner._surface_pts[_pruner._voxel_dict[i][select]];
            BDLSkeletonNode *new_node = new BDLSkeletonNode(p.x(), p.y(), p.z());

            //find the closest node from tree to p
            float min_dist = -1.0f;
            //BDLSkeletonNode *min_node = Transformer::point_tree_dist(_root, p, min_dist);
            //BDLSkeletonNode *min_node = Transformer::point_tree_dist(trees, p, min_dist);//find the closest node in old tree
            BDLSkeletonNode *min_node = NULL;
            if(dense)
            {
                min_node = Transformer::point_tree_dist(_root, p, min_dist);
            }
            else
            {
                for(unsigned int t=0; t<trees.size(); t++)
                {
                    BDLSkeletonNode *tnode = trees[t];

                    //ensure N has _prev
                    if(!tnode->_prev)
                        continue;

                    //check if N has been used more than twice first
                    if(grown_cnt.find(tnode) != grown_cnt.end() && grown_cnt[tnode] > dense ? 2 : 0)
                        continue;

                    float cur_dist = (p - Transformer::toVec3(tnode)).length2();
                    if(min_dist == -1.0f || cur_dist < min_dist)
                    {
                        min_dist = cur_dist;
                        min_node = tnode;
                    }
                }
            }
            if(!min_node)
                return;

            //update book-keeping
            if(grown_cnt.find(min_node) != grown_cnt.end())
                grown_cnt[min_node]++;
            else
                grown_cnt[min_node] = 1;

            //step 3: directly grow a straight line
            if(min_dist <= 1.0f*_close_threshold)
            {
				new_node->_prev = min_node;
				min_node->_children.push_back(new_node);
            }
            else
            {
                //determine if the divergent angle exceed 30 degrees
                float angle = Transformer::divergent_angle(min_node->_prev, min_node, new_node);
                std::vector <BDLSkeletonNode *> update_nodes;

                //this is not pretty, so disable it
                //if(false && angle <= 30.0f)
                if(dense)
                {
                    //step 4: directly add new_node to it and replace it with a random element
                    new_node->_prev = min_node;
                    new_node->_prev_support = min_node;
                    min_node->_children.push_back(new_node);

                    update_nodes.push_back(new_node);

                    //maybe useless since it may be pruned away
                    //give more freedom to it
                    if(false)
                    {
                        std::vector <BDLSkeletonNode *> subtree = replace_branch(new_node, elements[rand()%elements.size()], rand()%360);
                        update_nodes = after_pruned(subtree);
                        prune(subtree);
                    }
                }
                else
                {
                    //step 5: search for the most similar divergent angle in the original tree
                    BDLSkeletonNode *a, *b;
                    if(Transformer::similar_angle_in_tree(copied_root, angle, a, b))
                    {
                        //printf("angle(%f) div(%f)\n", angle, Transformer::divergent_angle(a->_prev, a, b));
                        //step 6: construct an elemnt out of a->_prev and b
                        BDLSkeletonNode *support_tip;
                        BDLSkeletonNode *sub_tree = Transformer::extract_subtree(a->_prev, b, support_tip);
                        if(sub_tree && support_tip)
                        {
                            //setup element with replacing branch equal to new sub_tree and its first child
                            min_node->_prev_support = min_node->_prev;
                            LibraryElement element(sub_tree, SLink(sub_tree, sub_tree->_children[0]));

                            //find out best rotation of element
                            int min_tran_ang = -1;
                            float min_tran_ang_dist = -1.0f;
                            for(int ta=0; ta<360; ta+=12)
                            {
                                LibraryElement element_test = element;
                                Transformer::transform(min_node, ta, element_test);
                                std::vector <BDLSkeletonNode *> compare = Transformer::terminal_nodes(element_test._root);
                                for(unsigned int ter=0; ter<compare.size(); ter++)
                                {
                                    float cur_dist = BDLSkeletonNode::dist(compare[ter], new_node);
                                    if(min_tran_ang_dist == -1.0f || cur_dist < min_tran_ang_dist)
                                    {
                                        min_tran_ang_dist = cur_dist;
                                        min_tran_ang = ta;
                                    }
                                }
                            }

                            //repace <a->_prev,a>
                            std::vector <BDLSkeletonNode *> subtree = replace_branch(min_node, element, min_tran_ang);
                            update_nodes = after_pruned(subtree);
                            prune(subtree);
                        }
                        else
                            printf("LaserSkeletonGrower::backward_grow():extract_subtree error\n");

                        delete new_node;
                    }
                }
                //setp 7: update the newly added nodes
                for(unsigned int j=0; j<update_nodes.size(); j++)
                {
                    if(false && dense)
                    {
                        _pruner._voxel_dict_supported[_pruner.key(Transformer::toVec3(update_nodes[j]))] = true;
                    }
                    else
                        _pruner.set_support(Transformer::toVec3(update_nodes[j]), j<update_nodes.size()/9 ? 0.2f : 1.0f);
                }
            }
        }//end if
    }//end for (voxel)

    BDLSkeletonNode::delete_this(copied_root);

    if(false)
    {
        printf("hihi\n");
        printf("%d\n", int(unorder.size()));
        for(unsigned int i=0; i<unorder.size(); i++)
        {
            if(!_pruner._voxel_dict_supported[i] && !_pruner._voxel_dict[i].empty())
            {
                int select = rand() % _pruner._voxel_dict[i].size();
                osg::Vec3 p = _pruner._surface_pts[_pruner._voxel_dict[i][select]];
                printf("%f %f %f\n", p.x(), p.y(), p.z());
            }
        }
    }
}

bool LaserSkeletonGrower::is_too_close(BDLSkeletonNode *node, BDLSkeletonNode *child)
{
    bool ret = true;
    if(!_root || !node || !child)
        return ret;

    float d = Transformer::link_tree_dist(_root, node, child);
    if(d >= _close_threshold)
        ret = false;

    //printf("is_too_close(%10f)\n", d);
    return ret;
}

float LaserSkeletonGrower::compute_potential_energy(BDLSkeletonNode *root, const SimpleVolumeFloat& space)
{
	float ret = 23418715.0f;
	if(!root || space._box.empty())
		return ret;

	ret = 0.0f; // will it overflow?

	//overview: put negative charges at node and find the total potential energy of the tree
	float v_height = space._sizeX;
	osg::Vec3 corner(-v_height/2.0f, -v_height/2.0f, 0.0f);

	//bfs
	std::queue <BDLSkeletonNode *> Queue;
	Queue.push(root);

	while(!Queue.empty())
	{
		BDLSkeletonNode *front = Queue.front();
		Queue.pop();

		osg::Vec3 cur = Transformer::toVec3(front) - corner;
		ret += space.count(cur.x(), cur.y(), cur.z()); //negative charge

		for(unsigned int i=0; i<front->_children.size(); i++)
			Queue.push(front->_children[i]);
	}

	return ret;
}

float LaserSkeletonGrower::compute_potential_energy(std::vector <BDLSkeletonNode *> points, const SimpleVolumeFloat& space)
{
	float ret = 23418715.0f;
	if(points.empty() || space._box.empty())
		return ret;

	ret = 0.0f; // will it overflow?

	//overview: put negative charges at node and find the total potential energy of the tree
	float v_height = space._sizeX;
	osg::Vec3 corner(-v_height/2.0f, -v_height/2.0f, 0.0f);

	for(unsigned int i=0; i<points.size(); i++)
	{
		BDLSkeletonNode *front = points[i];
		osg::Vec3 cur = Transformer::toVec3(front) - corner;
		ret += space.count(cur.x(), cur.y(), cur.z()); //negative charge
	}

	return ret;
}

std::vector <BDLSkeletonNode *> LaserSkeletonGrower::after_pruned(BDLSkeletonNode *root)
{
	std::vector <BDLSkeletonNode *> ret;
    if(!root || !_root)
        return ret;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        if(!_pruner.is_inside(Transformer::toVec3(front)))
        {
			//should not modify anything
            //if(true)
            //    if(front->_prev)
            //        front->_prev->_prev_support = NULL;

            //BDLSkeletonNode::delete_this(front);
        }
        else
		{
			//if inside, save it
			ret.push_back(front);

            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
		}
    }

	return ret;
}

std::vector <BDLSkeletonNode *> LaserSkeletonGrower::after_pruned(std::vector <BDLSkeletonNode *> roots)
{
	std::vector <BDLSkeletonNode *> ret;

    for(unsigned int i=0; i<roots.size(); i++)
	{
		std::vector <BDLSkeletonNode *> tmp;
        tmp = after_pruned(roots[i]);

		for(unsigned int j=0; j<tmp.size(); j++)
			ret.push_back(tmp[j]);
	}

	return ret;
}

/*
void LaserSkeletonGrower::test_laser(std::string path)
{
	_root = new BDLSkeletonNode;
	LaserPruner lp;
	lp.setup(path);

	//test pruner
	//osg::Vec3 probe(1.285f+0.1f, 0.455f, 4.361f);
	osg::Vec3 probe(0.524f, -2.524f, 2.524f);
	osg::Vec3 hit;
    bool hit_valid;
	bool inside = lp.is_inside(probe, hit, hit_valid);

	printf("probe(%f %f %f) is %s and hit(%f %f %f)\n", probe.x(), probe.y(), probe.z(), inside ? "inside" : "outside", hit.x(), hit.y(), hit.z());
}
*/
