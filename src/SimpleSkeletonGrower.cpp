#include "SimpleSkeletonGrower.h"
#include <queue>
#include "Transformer.h"
#include "ConvexHull2D.h"

SimpleSkeletonGrower::SimpleSkeletonGrower(): _root(NULL), _approaching_done(false)
{
	srand(time(NULL));
}

SimpleSkeletonGrower::~SimpleSkeletonGrower()
{
    BDLSkeletonNode::delete_this(_root);
}

void SimpleSkeletonGrower::setup(std::string master_element, std::string master_skeleton, std::string isp0, std::string cache)
{
    //load library
    _library.load_default_elements(master_element);
    _library.load_default_skeleton(master_skeleton);

	if(_library._library_element.empty() || _library._library_skeleton.empty())
		printf("SimpleSkeletonGrower::setup:_library is empty error\n");

    //load surface points
    //from cache
    if(cache != "")
    {
        float k_d = _surface.load(cache);

        //setup pruner
        _pruner.setup(isp0, k_d);

        _k_d = k_d;

		//in case the provided cache is wrong
        _path_isp0 = isp0;
    }
    //from segmentation + root point
    else
    {
        //the mb_length_d depends on initial skeleton, which is not 
        //set and may change in the future, so need to store the path
        _path_isp0 = isp0;
    }
}

void SimpleSkeletonGrower::set_initial_skeleton(int k)
{
    if(k >= 0 && k < int(_library._library_skeleton.size()))
    {
        //mb_length_d
        BDLSkeletonNode *init_root = _library._library_skeleton[k];
        //float mb_length_d = BDLSkeletonNode::main_branch_length(init_root);
        float mb_length_d = Transformer::first_hop_dist(init_root);

        //mb_length_d *= 1.2f;
        //printf("mb_length_d(%f)\n", mb_length_d);

        //k_d is loaded from _surface, and be used to setup _pruner
        float k_d = -1.0f;

        //new surface
        if(!_surface.is_loaded())
        {
            k_d = _surface.load(_path_isp0, mb_length_d);
            //if the same initial skeleton is used again, pass this file to setup
            //_surface.save(); //should be called explicityly
        }

        if(!_pruner.is_loaded())
            _pruner.setup(_path_isp0, k_d);

        if(k_d != -1.0f)
            _k_d = k_d;

        //new root
        if(_root)
            BDLSkeletonNode::delete_this(_root);
        _root = BDLSkeletonNode::copy_tree(init_root);

        //set _close_threshold
        _close_threshold = mb_length_d / 5.0f;//4.2f;//8.0f;
        //printf("SimpleSkeletonGrower::set_initial_skeleton():_close_threshold(%f)\n", _close_threshold);
		if(!_root)
			printf("SimpleSkeletonGrower::set_initial_skeleton():_root equals NULL error\n");
    }
}

BDLSkeletonNode *SimpleSkeletonGrower::pick_branch(bool space_aware)
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

		return ret;
	}

	//approach 2: grow node which is farest from the volume bound + smaller generation
	if(false && space_aware && _surface.is_loaded())
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
			for(unsigned int j=0; j<_surface._surface_pts.size(); j++)
			{
				osg::Vec3 cur_s = _surface._surface_pts[j];
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

	//approach 3: FOR LATERAL GROWTH, find the longest support branch to replace
	if(false)
	{
		float max_sup_len = -1.0f;
		for(unsigned int i=0; i<replaced_candidates.size(); i++)
		{
			float cur = BDLSkeletonNode::dist(replaced_candidates[i], replaced_candidates[i]->_prev_support);
			if(max_sup_len == -1.0f || cur > max_sup_len)
			{
				max_sup_len = cur;
				ret = replaced_candidates[i];
			}
		}

		return ret;
	}

	//approach 3: find the least projected density among all nodes
	if(true && _surface.is_loaded())
	{
		//a. find the bounds from all surface points
		float right = -1.0f, left = -1.0f, top = -1.0f, bottom = -1.0f;
		for(unsigned int i=0; i<_surface._surface_pts.size(); i++)
		{
			osg::Vec3 cur_s = _surface._surface_pts[i];
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
				if(!_pruner.is_inside_ortho(field))
					density_map[i][j] = -10;//99999; //means forbidden or infinitely dense
			}

		//test: obtain convex hull of replaced_candidates
		if(_approaching_done) //if approaching the end of the process
		{
			std::vector <osg::Vec2> r_can;
			for(unsigned int i=0; i<replaced_candidates.size(); i++)
				if(_pruner.is_inside_ortho(Transformer::toVec3(replaced_candidates[i])))
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

    return ret;
}

std::vector <BDLSkeletonNode *> SimpleSkeletonGrower::replace_branch(BDLSkeletonNode *branch_tail, LibraryElement element, int angle)
{
    std::vector <BDLSkeletonNode *> ret;

    if(!element.isValid() || !branch_tail || !branch_tail->_prev_support)
    {
        //printf("SimpleSkeletonGrower::replace_branch():element.isValid(%d):branch_tail(%p) error\n", element.isValid(), branch_tail);
        printf("element._root(%p) sup_par(%p) sup_child(%p)\n", element._root, element._support_branch._par, element._support_branch._child);
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

std::vector <BDLSkeletonNode *> SimpleSkeletonGrower::replace_branch_unchanged(BDLSkeletonNode *branch_tail, LibraryElement element, int angle)
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

    //branch_tail->_prev_support = NULL;

    //return all sub-tree for pruning
    return ret;
}

std::vector <BDLSkeletonNode *> SimpleSkeletonGrower::bfs()
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

void SimpleSkeletonGrower::prune(BDLSkeletonNode *root)
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

        if(!_pruner.is_inside(front, true))
        {
            //cannot be replaced again
            if(true)
                if(front->_prev)
                    front->_prev->_prev_support = NULL;

			//should also cut the edge besides the vertex
			if(front->_prev)
			{
				int no_step = 20;
				osg::Vec3 out = Transformer::toVec3(front);
				osg::Vec3 in = Transformer::toVec3(front->_prev);
				osg::Vec3 go = out - in;
				float step_size = go.length() / no_step;
				go.normalize();

				osg::Vec3 just_outside;
				//by probing if it is inside the mask
				for(int i=0; i<no_step; i++)
				{
					osg::Vec3 probe = out + go * step_size * i;
					if(!_pruner.is_inside(probe))
					{
						just_outside = probe;
						break;
					}
				}

				//add back the intercepting point
				BDLSkeletonNode *jout = new BDLSkeletonNode(just_outside.x(), just_outside.y(), just_outside.z());
				jout->_prev = front->_prev;
				front->_prev->_children.push_back(jout);
			}

			//debug
			//printf("deleting:\n");
			//printf("%f %f %f\n", front->_sx, front->_sy, front->_sz);

            BDLSkeletonNode::delete_this(front);
        }
        else
		{
			//printf("%f %f %f\n", front->_sx, front->_sy, front->_sz);
            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
		}
    }
}

void SimpleSkeletonGrower::prune_strictly(BDLSkeletonNode *root)
{
    if(!root || !_root)
        return;

	//don't use generation, just use distance directly
	float mb_length_d = BDLSkeletonNode::main_branch_length(root);
	float exception = mb_length_d * 1.5f;//get the exception of checking within this distance
	//float exception = mb_length_d * 2.5f;//get the exception of checking within this distance

	//prune globally, but no effect within k-generations from root
	/*
	int k = 8;

	//bfs to initialize the generations
    std::queue <BDLSkeletonNode *> Queue;
	root->_generation = 0;
    Queue.push(root);

	while(!Queue.empty())
	{
		BDLSkeletonNode *front = Queue.front();
		Queue.pop();

		for(unsigned int i=0; i<front->_children.size(); i++)
		{
			BDLSkeletonNode *child = front->_children[i];
			child->_generation = front->_generation + 1;
			Queue.push(child);
		}
	}
	*/

    //bfs: pruned by sort of surface revolution
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        //if(front->_generation > k && !_pruner.is_inside(front))
        if(front->dist(root) > exception && !_pruner.is_inside(front))
        {
            //cannot be replaced again
			if(front->_prev)
				front->_prev->_prev_support = NULL;

			////should also cut the edge besides the vertex
			//if(front->_prev)
			//{
			//	int no_step = 20;
			//	osg::Vec3 out = Transformer::toVec3(front);
			//	osg::Vec3 in = Transformer::toVec3(front->_prev);
			//	osg::Vec3 go = out - in;
			//	float step_size = go.length() / no_step;
			//	go.normalize();

			//	osg::Vec3 just_outside;
			//	//by probing if it is inside the mask
			//	for(int i=0; i<no_step; i++)
			//	{
			//		osg::Vec3 probe = out + go * step_size * i;
			//		if(!_pruner.is_inside(probe))
			//		{
			//			just_outside = probe;
			//			break;
			//		}
			//	}

			//	//add back the intercepting point
			//	BDLSkeletonNode *jout = new BDLSkeletonNode(just_outside.x(), just_outside.y(), just_outside.z());
			//	jout->_prev = front->_prev;
			//	front->_prev->_children.push_back(jout);
			//}

            BDLSkeletonNode::delete_this(front);
        }
        else
            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
    }

    //bfs: pruned by orthogonal projection
	Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        //if(front->_generation > k && !_pruner.is_inside_ortho(Transformer::toVec3(front)))
        if(front->dist(root) > exception && !_pruner.is_inside_ortho(Transformer::toVec3(front)))
        {
            //cannot be replaced again
			if(front->_prev)
				front->_prev->_prev_support = NULL;

			////should also cut the edge besides the vertex
			//if(front->_prev)
			//{
			//	int no_step = 20;
			//	osg::Vec3 out = Transformer::toVec3(front);
			//	osg::Vec3 in = Transformer::toVec3(front->_prev);
			//	osg::Vec3 go = out - in;
			//	float step_size = go.length() / no_step;
			//	go.normalize();

			//	osg::Vec3 just_outside;
			//	//by probing if it is inside the mask
			//	for(int i=0; i<no_step; i++)
			//	{
			//		osg::Vec3 probe = out + go * step_size * i;
			//		if(!_pruner.is_inside(probe))
			//		{
			//			just_outside = probe;
			//			break;
			//		}
			//	}

			//	//add back the intercepting point
			//	BDLSkeletonNode *jout = new BDLSkeletonNode(just_outside.x(), just_outside.y(), just_outside.z());
			//	jout->_prev = front->_prev;
			//	front->_prev->_children.push_back(jout);
			//}

            BDLSkeletonNode::delete_this(front);
			//printf("deleting (%f,%f,%f)\n", front->_sx, front->_sy, front->_sz);
        }
        else
            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
    }
}

void SimpleSkeletonGrower::prune(std::vector<BDLSkeletonNode *> roots)
{
    for(unsigned int i=0; i<roots.size(); i++)
        prune(roots[i]);
}

void SimpleSkeletonGrower::grow(int no_branch)
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

void SimpleSkeletonGrower::grow_exhaustive(int no_branch)
{
    std::vector <LibraryElement> elements = _library._library_element;

    for(int i=0; i<no_branch; i++)
    {
		//pick a tail first
        BDLSkeletonNode *tail = pick_branch();
        if(!tail)
        {
            printf("SimpleSkeletonGrower::grow():converged at %d-th step.\n", i+1);
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
}

void SimpleSkeletonGrower::grow_potential(int no_branch)
{
	//overview: maintain a volume to represent the potential energy at each step,
	//then find the min_element_index and the min_rotate that require the minimum potential energy

	if(!_surface.is_loaded() || !_root)
	{
		printf("SimpleSkeletonGrower::surface error\n");
		return;
	}

	//find the dimensions of the volume
	float v_height = -1.0f;
	osg::Vec3 origin(0.0f, 0.0f, 0.0f);
	for(unsigned int i=0; i<_surface._surface_pts.size(); i++)
	{
		float cur = (_surface._surface_pts[i] - origin).length2();
		if(v_height == -1.0f || cur > v_height)
			v_height = cur;
	}
	if(v_height == -1.0f)
	{
		printf("SimpleSkeletonGrower::grow_potential():v_height(-1.0f) error\n");
		return;
	}

	//v_height = sqrt(v_height) * 1.5f; //old
	v_height = sqrt(v_height) * 2.0f; //new
	float v_step = v_height / 100.0f;
	osg::Vec3 corner(-v_height/2.0f, -v_height/2.0f, 0.0f);
	SimpleVolumeFloat space(v_height, v_height, v_height, v_step, v_step, v_step);
	float energy_effect = v_step * 18;

	//negative charges on initial skeleton
	//bfs
	std::queue <BDLSkeletonNode *> Queue;
	Queue.push(_root);

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
	for(unsigned int i=0; i<_surface._surface_pts.size(); i++)
	{
		osg::Vec3 cur = _surface._surface_pts[i] - corner;
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
            printf("SimpleSkeletonGrower::grow():converged at %d-th step.\n", i+1);
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
			std::vector <BDLSkeletonNode *> update_nodes = after_pruned(subtree);
			prune(subtree); //prune leniently

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
    }

	//aftermath: should be shared for different grow methods
	//prune strictly on entire tree
	prune_strictly(_root);
	if(true)
	{
		//printf("backward_grow()\n");
		backward_grow();
		prune_strictly(_root);
	}
}

void SimpleSkeletonGrower::backward_grow()
{
	if(_surface.is_loaded())
	{
		std::vector <osg::Vec3> backward_grow = _surface._relative_pts;

		//1. Initialization: setup a grow_map for book-keeping the backward growing
		//Key: rank of the <x,z> coords; Value: <low depth(y), high depth(y), isDone(1 or -1)>
		//this is used in addition to the relative bound for growing a dense foilage
		std::map <int, osg::Vec3> grow_map;

		for(unsigned int i=0; i<_surface._surface_pts.size(); i++)
		{
			osg::Vec3 pt = _surface._surface_pts[i];

			//only consider points that lay inside the boundary
			if(!_pruner.is_inside_ortho(pt))
				continue;

			int rank = (pt.x()+1000)*2500 + (pt.z()+1000);

			//check if exists, update its value
			if(grow_map.find(rank) != grow_map.end())
			{
				osg::Vec3 paras = grow_map[rank];
				osg::Vec3 new_paras = paras;

				if(pt.y() < paras.x())
					new_paras.x() = pt.y();
				if(pt.y() > paras.y())
					new_paras.y() = pt.y();

				grow_map[rank] = new_paras;
			}
			//new cloud point
			else
			{
				osg::Vec3 paras(pt.y(), pt.y(), -1);//new point has the same-depth and is un-done
				grow_map[rank] = paras;
			}
		}

		float inter_cons = 2.0f;
		float neighbor_dist = pow(inter_cons*1.05, 0.5) * _close_threshold;
		float neighbor_diag = neighbor_dist * 0.707f;
		//2. Loop each node pt in grow_map, un-rank it, and pretend to be a point in the relative bound
		for(std::map <int, osg::Vec3>::iterator it=grow_map.begin(); it!=grow_map.end(); it++)
		{
			//a. recover point, the y-coord is randomly picked between low and high
			int rank = it->first;
			//osg::Vec3 paras = it->second;
			//int y = rand()%(int(paras.y()-paras.x())) + paras.x();
			//osg::Vec3 pt(rank/2500-1000, y, rank%2500-1000);

			osg::Vec3 pt(rank/2500-1000, rank%2500-1000, 0.0f);
			backward_grow.push_back(pt);

			//add more neighbor points
			osg::Vec3 probe;
			if(false)
			{
				probe = osg::Vec3(pt.x()+neighbor_dist, pt.y(), 0.0f);//right
				if(!_pruner.is_inside_ortho(probe))
					backward_grow.push_back(probe);
				probe = osg::Vec3(pt.x()-neighbor_dist, pt.y(), 0.0f);//left
				if(!_pruner.is_inside_ortho(probe))
					backward_grow.push_back(probe);
				probe = osg::Vec3(pt.x(), pt.y()+neighbor_dist, 0.0f);//top
				if(!_pruner.is_inside_ortho(probe))
					backward_grow.push_back(probe);
				probe = osg::Vec3(pt.x(), pt.y()-neighbor_dist, 0.0f);//bottom
				if(!_pruner.is_inside_ortho(probe))
					backward_grow.push_back(probe);
			}

			probe = osg::Vec3(pt.x()+neighbor_diag, pt.y()+neighbor_diag, 0.0f);//top-right
			if(!_pruner.is_inside_ortho(probe))
				backward_grow.push_back(probe);
			probe = osg::Vec3(pt.x()-neighbor_diag, pt.y()+neighbor_diag, 0.0f);//top-left
			if(!_pruner.is_inside_ortho(probe))
				backward_grow.push_back(probe);
			probe = osg::Vec3(pt.x()+neighbor_diag, pt.y()-neighbor_diag, 0.0f);//bottom-right
			if(!_pruner.is_inside_ortho(probe))
				backward_grow.push_back(probe);
			probe = osg::Vec3(pt.x()-neighbor_diag, pt.y()-neighbor_diag, 0.0f);//bottom-left
			if(!_pruner.is_inside_ortho(probe))
				backward_grow.push_back(probe);
		}

		//given the non-even retaive contour points, backward grow it to the nearest tree nodes
		//each each node cannot grow more than twice, also the newly added retaive points are 
		//required to be mutually separated from previously added points, in addition to the old tree nodes
		std::vector <LibraryElement> elements = _library._library_element;

		std::vector <osg::Vec3> newly_added;
		std::map <BDLSkeletonNode *, int> grown_cnt;

		//put inside the loop becasue trees is updated, put outside for non-continuous effect
		std::vector <BDLSkeletonNode *> trees = bfs();
		BDLSkeletonNode *copied_root = BDLSkeletonNode::copy_tree(_root);

		for(unsigned int j=0; j<backward_grow.size(); j++)
		{
			osg::Vec3 temp = backward_grow[j];
			osg::Vec3 pt(temp.x(), 0.0f, temp.y());//add depth later

			//b. find min projected distance from tree
			float min_proj_dist = -1.0f;
			for(unsigned int i=0; i<trees.size(); i++)
			{
				osg::Vec3 T = Transformer::toVec3(trees[i]);
				float proj_dist = osg::Vec2(pt.x()-T.x(), pt.z()-T.z()).length2();
				if(min_proj_dist == -1.0f || proj_dist < min_proj_dist)
					min_proj_dist = proj_dist;
			}

			//c. find min projected distance from previously added points
			float min_proj_dist2 = -1.0f;
			for(unsigned int i=0; i<newly_added.size(); i++)
			{
				osg::Vec3 na = newly_added[i];
				float proj_dist = osg::Vec2(pt.x()-na.x(), pt.z()-na.z()).length2();
				if(min_proj_dist2 == -1.0f || proj_dist < min_proj_dist2)
					min_proj_dist2 = proj_dist;
			}

			//c. if minimum projected distance from trees T is larger than _close_threshold, then backward grow it
			float close_threshold2 = _close_threshold * _close_threshold;
			if(min_proj_dist > close_threshold2 && (min_proj_dist2 > inter_cons*close_threshold2 || newly_added.empty()))
			{
				//find the closest node N in trees T to grow
				BDLSkeletonNode *N = NULL;
				float min_tree_dist = -1.0f;
				for(unsigned int i=0; i<trees.size(); i++)
				{
					BDLSkeletonNode *tnode = trees[i];

					//ensure N has _prev
					if(!tnode->_prev)
						continue;

					//ensure N has no child
					//if(tnode->_children.size() > 1)
					//	continue;

					//check if N has been used more than twice first
					if(grown_cnt.find(tnode) != grown_cnt.end() && grown_cnt[tnode] > 0)
						continue;

					float cur_dist = (pt - Transformer::toVec3(tnode)).length2();
					if(min_tree_dist == -1.0f || cur_dist < min_tree_dist)
					{
						min_tree_dist = cur_dist;
						N = tnode;
					}
				}
				//fuel for backward grow is used up
				if(!N)
					return;

				//add depth of pt = 'depth of N' +/- 'half length of N and N->_prev'
				float par_len = BDLSkeletonNode::dist(N, N->_prev);
				if(par_len > 1.0f)
					pt.y() = N->_sy + rand()%int(par_len) - par_len/2.0f;
				else
					pt.y() = N->_sy + (rand()%3-1)*par_len/2.0f;

				//update book-keeping
				newly_added.push_back(pt);
				if(grown_cnt.find(N) != grown_cnt.end())
					grown_cnt[N]++;
				else
					grown_cnt[N] = 1;

				//shorten the N-pt segment by 10% to prevent unwanted pruning
				osg::Vec3 wrap = Transformer::toVec3(N);
				osg::Vec3 dir = pt - wrap;
				float desired_len = dir.length() * 0.95f;
				dir.normalize();
				pt = wrap + dir * desired_len;

				//new tip node
				BDLSkeletonNode *new_node = new BDLSkeletonNode(pt.x(), pt.y(), pt.z());

				//use advanced approach to grow it as in laser data
				if(true)
				{
					//before attaching, check if the divergent angle is too large
					float div_ang = Transformer::divergent_angle(N->_prev, N, new_node);
					if(div_ang > 30.0f)
					{
						//osg::Vec3 translateN = Transformer::toVec3(N->_prev) - wrap;
						//float len = translateN.length();
						//translateN.normalize();
						//translateN = wrap + translateN * len * 0.5f;

						//N->_sx = wrap.x();
						//N->_sy = wrap.y();
						//N->_sz = wrap.z();
						//if(N->_prev->_prev)
						//{
						//	N = N->_prev;
						//	div_ang = Transformer::divergent_angle(N->_prev, N, new_node);
						//}

						//search for the most similar divergent angle in the original tree
						BDLSkeletonNode *a, *b;
						if(Transformer::similar_angle_in_tree(copied_root, div_ang, a, b))
						{
							//printf("angle(%f) div(%f)\n", angle, Transformer::divergent_angle(a->_prev, a, b));
							//construct an elemnt out of a and b
							BDLSkeletonNode *support_tip = NULL;
							BDLSkeletonNode *sub_tree = Transformer::extract_subtree(a->_prev, b, support_tip);
							if(sub_tree && support_tip)
							{
								//should not add
								//new_node->_prev = N;
								//new_node->_prev_support = N;
								//N->_children.push_back(new_node);
								N->_prev_support = N->_prev;

								LibraryElement element(sub_tree, SLink(sub_tree, sub_tree->_children[0]));
								//rotation can't be random!
								int min_tran_ang = -1;
								float min_tran_ang_dist = -1.0f;
								for(int ta=0; ta<360; ta+=12)
								{
									LibraryElement element_test = element;
									Transformer::transform(N, ta, element_test);
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

								//printf("min_tran_ang_dist(%f)\n", min_tran_ang_dist);
								std::vector <BDLSkeletonNode *> subtree = replace_branch(N, element, min_tran_ang);
								prune(subtree);

								//printf("%f %f %f\n", pt.x(), pt.y(), pt.z());
								//pt = Transformer::toVec3(N);
								//printf("%f %f %f\n", pt.x(), pt.y(), pt.z());
								if(false || j==617)
								{
									Transformer::straight_up(element);
									char buffer[100];
									sprintf(buffer, "/tmp/log_sk/log_skeleton_%d", j);
									BDLSkeletonNode::save_skeleton(element._root, buffer);
								}
							}
							else
								printf("SimplSkeletonGrower::backward_grow():extract_subtree: a(%p) b(%p) error\n", a, b);
							delete new_node;
						}
						else
						{
							//if no suitable sub-tree can be found, just don't grow the node
							//printf("%f %f %f\n", pt.x(), pt.y(), pt.z());
							delete new_node;
						}
					}
					else
					{
						//directly attach it to produce a straight branch
						new_node->_prev = N;
						N->_children.push_back(new_node);

						//too far away, replace it
						//if(desired_len > _close_threshold * 2.0f)
						//{
						//	new_node->_prev_support = N;
						//	std::vector <BDLSkeletonNode *> subtree = replace_branch(new_node, elements[rand()%elements.size()], rand()%360);
						//	//prune(subtree); //prune leniently
						//}
					}
				}

				//visualize pt
				//printf("%f %f %f\n", pt.x(), pt.y(), pt.z());
			}
			//d. continue for the next point
		} //end for loop
		BDLSkeletonNode::delete_this(copied_root);
	}
}

bool SimpleSkeletonGrower::is_too_close(BDLSkeletonNode *node, BDLSkeletonNode *child)
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

void SimpleSkeletonGrower::test()
{
    if(false)
    {
        std::vector <osg::Vec3> pts;
        std::vector <float> scalars;
        Transformer::billow_noise(pts, scalars);

        for(unsigned int i=0; i<pts.size(); i++)
            printf("%d: (%f %f %f) -> (%f)\n", i, pts[i].x(), pts[i].y(), pts[i].z(), scalars[i]);
    }
}

float SimpleSkeletonGrower::compute_energy(BDLSkeletonNode *root)
{
	float ret = -1.0f;
	if(!root || !_surface.is_loaded())
		return ret;

	ret = 0.0f; // will it overflow?
	std::vector <osg::Vec3> attractors = _surface._surface_pts;
	for(unsigned int i=0; i<attractors.size(); i++)
	{
		osg::Vec3 si = attractors[i];
		float min_dist = Transformer::point_tree_dist(root, si);
		if(min_dist >= 0.0f)
			ret += min_dist;
	}

	return ret;
}

float SimpleSkeletonGrower::compute_potential_energy(BDLSkeletonNode *root, const SimpleVolumeFloat& space)
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

float SimpleSkeletonGrower::compute_potential_energy(std::vector <BDLSkeletonNode *> points, const SimpleVolumeFloat& space)
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

std::vector <BDLSkeletonNode *> SimpleSkeletonGrower::after_pruned(BDLSkeletonNode *root)
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

        if(!_pruner.is_inside(front, true))
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

std::vector <BDLSkeletonNode *> SimpleSkeletonGrower::after_pruned(std::vector <BDLSkeletonNode *> roots)
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
