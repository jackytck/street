#include <math.h>
#include <queue>
#include <stack>
#include <map>
#include <algorithm>
#include <utility>
#include <osg/Material>
#include "BDLPointGraph.h"
#include "osgModeler.h"
#include "HoughTransform3d.h"
#include "PointModel.h"
#include "LineModel.h"

BDLSkeletonNode::BDLSkeletonNode(): _sx(0.0), _sy(0.0), _sz(0.0), _radius(-1.0), _prev(NULL), _mesh_done(false), _inherited_radius(0.0), _radius_rectified(false), _supporting_length(0.0), _height(0.0), _pruned(false), _prev_support(NULL), _generation(-1)
{
}

BDLSkeletonNode::BDLSkeletonNode(double x, double y, double z): _sx(x), _sy(y), _sz(z), _radius(-1.0), _prev(NULL), _mesh_done(false), _inherited_radius(0.0), _radius_rectified(false), _supporting_length(0.0), _height(0.0), _pruned(false), _prev_support(NULL), _generation(-1)
{
}

double BDLSkeletonNode::dist(BDLSkeletonNode *b)
{
    if(!b)
        return -1.0;

    return sqrt(pow(this->_sx-b->_sx, 2) + pow(this->_sy-b->_sy, 2) + pow(this->_sz-b->_sz, 2));
}

BDLSkeletonNode *BDLSkeletonNode::ts2bdlsn(TopologicalSkeleton *ts)
{
    if(!ts)
        return NULL;
    TopologicalNode *ts_root = ts->root();
    if(!ts_root)
        return NULL;
    //just copy the positions and topology first
    //tranverse all and new each nodoe
    //todo: handle the new-ed object
    BDLSkeletonNode *ret = new BDLSkeletonNode();
    ret->_sx = ts_root->x();
    ret->_sy = ts_root->y();
    ret->_sz = ts_root->z();

    //BFS on a pair
    std::pair <TopologicalNode *, BDLSkeletonNode *> root = std::make_pair(ts_root, ret);

    std::queue <std::pair <TopologicalNode *, BDLSkeletonNode *> > Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        std::pair <TopologicalNode *, BDLSkeletonNode *> front = Queue.front(); 
        Queue.pop();

        TopologicalNode *tn = front.first;
        BDLSkeletonNode *sn = front.second;

        //for each child of tn, new a copy for sn
        for(int i=0; i<tn->childenSize(); i++)
        {
            TopologicalNode *tn_child = tn->child(i);
            BDLSkeletonNode *sn_child = new BDLSkeletonNode();

            sn_child->_sx = tn_child->x();
            sn_child->_sy = tn_child->y();
            sn_child->_sz = tn_child->z();

            sn_child->_prev = sn;
            sn->_children.push_back(sn_child);

            std::pair <TopologicalNode *, BDLSkeletonNode *> next = std::make_pair(tn_child, sn_child);
            Queue.push(next);
        }
    }

    //set the radius of the root, because all the other nodes depend on it
    ret->_radius = 150.0;

    //set the random set in constructor
    srand(time(NULL));

    return ret;
}

double BDLSkeletonNode::compute_supporting_length(BDLSkeletonNode *node)
{
    double ret = 0.0;
    if(!node)
        return ret;

    for(unsigned int i=0; i<node->_children.size(); i++)
    {
        BDLSkeletonNode *child = node->_children[i];
        double dist = sqrt(pow(child->_sx - node->_sx, 2) + pow(child->_sy - node->_sy, 2) + pow(child->_sz - node->_sz, 2)); 

        ret += compute_supporting_length(child) + dist;
    }

    //set back the result in node
    node->_supporting_length = ret;

    //debug
    //printf("supporting_length = %f\n", ret);

    return ret;
}

double BDLSkeletonNode::rectify_skeleton_tree(BDLSkeletonNode *root, osg::Vec3 origin, double height, int angle, bool maya)
{
    double ret = 1.0;

    if(!root)
        return ret;

    //bfs each point
    //find the highest point(highest_pt), assume the z-axis is the height
    //and rotate each point over the root by a 'angle'
    BDLSkeletonNode *highest_pt = root;
    std::queue <BDLSkeletonNode *> Queue3;
    Queue3.push(root);

    osg::Matrix rotate;
    rotate.makeRotate((angle%360)/180.0*M_PI, 0, 0, 1);

    while(!Queue3.empty())
    {
        BDLSkeletonNode *node = Queue3.front();
        Queue3.pop();

        if(node != root && node->_sz > highest_pt->_sz)
            highest_pt = node;

        if(angle)
        {
            osg::Vec3 node_vec3(node->_sx-root->_sx, node->_sy-root->_sy, node->_sz-root->_sz);
            node_vec3 = node_vec3 * rotate;

            node->_sx = node_vec3.x() + root->_sx;
            node->_sy = node_vec3.y() + root->_sy;
            node->_sz = node_vec3.z() + root->_sz;
        }

        //push all of the children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue3.push(node->_children[i]);
    }

    //bfs each point
    //scale it uniformly such that the highest z-coord - root's z-coord = height
    if(height != -1.0 && highest_pt->_sz - root->_sz)
    {
        double scale = height / (highest_pt->_sz - root->_sz);
        ret = scale;

        //by default the root->_radius = 150.0, so need to scale it too
        root->_radius *= scale;

        std::queue <BDLSkeletonNode *> Queue4;
        Queue4.push(root);

        while(!Queue4.empty())
        {
            BDLSkeletonNode *node = Queue4.front();
            Queue4.pop();

            //need to translate to the object's origin fisrt
            node->_sx -= root->_sx;
            node->_sy -= root->_sy;
            node->_sz -= root->_sz;

            //then scale
            node->_sx *= scale;
            node->_sy *= scale;
            node->_sz *= scale;

            //then translate back
            node->_sx += root->_sx;
            node->_sy += root->_sy;
            node->_sz += root->_sz;

            //push all of the children
            for(unsigned int i=0; i<node->_children.size(); i++)
                Queue4.push(node->_children[i]);
        }
    }

    //translate all points by move_by = origin - root
    osg::Vec3 move_by = origin - osg::Vec3(root->_sx, root->_sy, root->_sz);

    //if maya is true, then should swap origin, 
    //because origin is the real point, but now is not the real coords system
    if(maya)
    {
        double tmp_y = move_by.y();
        move_by.y() = move_by.z();
        move_by.z() = tmp_y;
    }

    //bfs each point
    //then translate to the new origin, should translate after the above scaling, not before it
    std::queue <BDLSkeletonNode *> Queue5;

    Queue5.push(root);

    while(!Queue5.empty())
    {
        BDLSkeletonNode *node = Queue5.front();
        Queue5.pop();

        node->_sx += move_by.x();
        node->_sy += move_by.y();
        node->_sz += move_by.z();

        //push all of the children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue5.push(node->_children[i]);
    }

    //finally, bfs each point
    //exchange the y and z-axis if maya is true
    if(maya)
    {
        std::queue <BDLSkeletonNode *> Queue6;
        Queue6.push(root);

        while(!Queue6.empty())
        {
            BDLSkeletonNode *node = Queue6.front();
            Queue6.pop();

            double tmp = node->_sy;
            node->_sy = node->_sz;
            node->_sz = tmp;

            //ugly fix for fitting the inverted Pittsburbh plane created last year
            if(false)
                node->_sy = -node->_sy;

            //push all of the children
            for(unsigned int i=0; i<node->_children.size(); i++)
                Queue6.push(node->_children[i]);
        }
    }

    //then find the radius of each node first

    //compute the supporting length
    BDLSkeletonNode::compute_supporting_length(root);

    //BFS from root to all other nodes, re-setting the radius by this biological model
    //assume the radius of root is correct now
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        double r_p = node->_radius;

        //this node is done, now set its children
        //if only one child
        if(node->_children.size() == 1)
        {
            double l_p = node->_supporting_length;
            double l_c = node->_children[0]->_supporting_length;

            //node->_children[0]->_radius = r_p * pow(l_c / l_p, 1.5);
            double expected = r_p * pow(l_c / l_p, 1.5);//from other paper
            //double expected = r_p * pow(l_c / l_p, 1.3);//mine
            //printf("single: expected(%lf) = rp(%lf) * (%lf)^(%lf)\n", expected, r_p, l_c/l_p, 1.3);
            //node->_children[0]->_radius = 0.9 * expected + 0.1 * node->_children[0]->_radius;

            //prevent radius == 0.0
            //node->_children[0]->_radius = 0.9 * std::max(expected, 0.001);
            
            //don't want to reduce too fast
            //node->_children[0]->_radius = 0.93 * std::max(expected, 0.0001);
            //node->_children[0]->_radius = 0.959 * std::max(expected, 0.00001);
            //node->_children[0]->_radius = 0.982 * std::max(expected, 0.3);//last
            //node->_children[0]->_radius = 0.972 * std::max(expected, 0.4);//current
            node->_children[0]->_radius = 0.982 * expected;//last 2
        }
        else
        {
            double sum_l_c = 0.0;
            for(unsigned int i=0; i<node->_children.size(); i++)
                sum_l_c += node->_children[i]->_supporting_length;

            for(unsigned int i=0; i<node->_children.size(); i++)
            {
                double l_c = node->_children[i]->_supporting_length;

                double l_c_ratio = 1.0 / node->_children.size() * 0.3;
                if(sum_l_c != 0.0 && l_c != 0.0)
                    l_c_ratio = l_c / sum_l_c;

                //node->_children[i]->_radius = r_p * pow(l_c / sum_l_c, 1.0/2.49);
                //double expected = r_p * pow(l_c_ratio, 1.0/2.49);//from other paper
                double expected = r_p * pow(l_c_ratio, 1.2/3.10);//mine
                //double expected = r_p * pow(l_c_ratio, 1.2/3.50);//mine2
                //printf("multi: expected(%lf) = rp(%lf) * (%lf)^(%lf)\n", expected, r_p, l_c_ratio, 1.2/3.1);
                //node->_children[i]->_radius = 0.9 * expected + 0.1 * node->_children[i]->_radius;
                //node->_children[i]->_radius = 0.9 * expected;

                //don't want to reduce too fast
                node->_children[i]->_radius = 0.99 * expected;
                //node->_children[i]->_radius = 1.0 * std::max(expected, 0.25);
            }
        }

        //push all of the children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    //then find the height or distance of each node from the root

    //bfs
    //me is done, my child's _height = me + dist(me, child), push my child for the next iteration
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(root);

    while(!Queue2.empty())
    {
        BDLSkeletonNode *node = Queue2.front();
        Queue2.pop();

        for(unsigned int i=0; i<node->_children.size(); i++)
        {
            BDLSkeletonNode *child = node->_children[i];
            double dist = pow(node->_sx - child->_sx, 2) + pow(node->_sy - child->_sy, 2) + pow(node->_sz - child->_sz, 2);
            dist = pow(dist, 0.5);
            child->_height = node->_height + dist;

            Queue2.push(child);
        }
    }

    return ret;
}

void BDLSkeletonNode::debug_radius(BDLSkeletonNode *root)
{
    if(!root)
        return;

    //bfs once
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        printf("radius: %lf\n", node->_radius);

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }
}

void BDLSkeletonNode::debug_support_length(BDLSkeletonNode *root)
{
    if(!root)
        return;

    //bfs once
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        printf("support %d child: %lf\n", int(node->_children.size()), node->_supporting_length);

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }
}

void BDLSkeletonNode::compress_skeleton_tree(BDLSkeletonNode *root)
{
    //bfs all points
    //if a trivial node(not a root and has one and only one child) is found,
    //delete it
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        //no matter if this node is trivial or not,
        //push all of its children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);

        //check if it is trivial
        if(node != root && node->_prev && node->_children.size() == 1)
        {
            //perparations before dieing, 
            //i.e. construct a new child list for its parent
            //and update the new parent for its child

            //ask its parent to delete the reference to itself
            //or only add other child to the new list
            std::vector <BDLSkeletonNode *> _newList;
            for(unsigned int i=0; i<node->_prev->_children.size(); i++)
                if(node->_prev->_children[i] != node)
                    _newList.push_back(node->_prev->_children[i]);

            //pass its only child to its parent
            _newList.push_back(node->_children[0]);

            //finally, update the new child list
            node->_prev->_children = _newList;

            //update the new parent for its child
            node->_children[0]->_prev = node->_prev;

            //die
            delete(node);
        }
    }
}

int BDLSkeletonNode::collaspe_skeleton_tree(BDLSkeletonNode *root, int iteration)
{
    //traverses all nodes, find the cost of collapsing an edge,
    //sort the costs, extract the min-cost edge, delete that edge and update any affected edges
    //repeat the extraction, deletion and updating until the convergent limit is reached or the iteration quota is used up

    //BFS all skeleton nodes
    //for finding the main branch, i.e. the longest branch
    BDLSkeletonNode *highest_node = root;
    double highest_height = root->_height;
    std::vector <BDLSkeletonNode *> main_branch;
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        if(front->_height > highest_height)
        {
            highest_node = front;
            highest_height = front->_height;
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue.push(child);
        }
    }

    //store the main_branch in a vector
    while(highest_node)
    {
        main_branch.push_back(highest_node);
        highest_node = highest_node->_prev;
    }
    //highest_node should be equal to NULL at this point
    //printf("main_branch size = %d\n", main_branch.size());

    //store all the feasible edges
    std::vector <std::pair <BDLSkeletonNode *, BDLSkeletonNode *> > edges;
    //store all the associated costs of the above edges, edge_costs should have the same size as edges
    std::vector <double> edge_costs, sorted_edge_costs;

    //BFS all skeleton nodes
    //for finding the edges and the associated costs
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(root);

    while(!Queue2.empty())
    {
        BDLSkeletonNode *front = Queue2.front();
        Queue2.pop();
 
        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue2.push(child);
        }

        //push all feasible edges
        //an edge is feasible if it dose not belong to the main branch and it does not end at a leaf

        //iterate all edges
        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            //if the child is not a leaf node, then a potential edge is found
            if(front->_children[i]->_children.size() != 0)
            {
                //check if both nodes A, B do not belong to the main branch
                bool Ain = false, Bin = false;
                for(unsigned int j=0; j<main_branch.size(); j++)
                {
                    if(front == main_branch[j])
                        Ain = true;
                    if(front->_children[i] == main_branch[j])
                        Bin = true;
                }
                
                //if no, then a feasible edge is found
                if(!(Ain && Bin))
                {
                    std::pair <BDLSkeletonNode *, BDLSkeletonNode *> edge = std::make_pair(front, front->_children[i]);
                    edges.push_back(edge);

                    //compute and push edge cost
                    double cost = 0.0, k1 = 1.0, k2 = 1.0;

                    //displacement cost of this edge
                    osg::Vec3 A(edge.first->_sx, edge.first->_sy, edge.first->_sz);
                    osg::Vec3 B(edge.second->_sx, edge.second->_sy, edge.second->_sz);
                    osg::Vec3 AB = B-A;
                    double Ra = edge.first->_radius, Rb = edge.second->_radius;
                    cost += k1 * (Ra + Rb) * AB.length();

                    //displacement costs for all the affected child
                    for(unsigned int c=0; c<edge.second->_children.size(); c++)
                    {
                        BDLSkeletonNode *child = edge.second->_children[c];
                        osg::Vec3 C(child->_sx, child->_sy, child->_sz);
                        osg::Vec3 CB = B-C;
                        osg::Vec3 CA = A-C;
                        CB.normalize();
                        CA.normalize();
                        double Rc = child->_radius;

                        double CB_CA = 1 - CB*CA;
                        if(CB_CA < 0.0)
                            CB_CA = 0.0;

                        cost += k2 * CB_CA * abs((Ra+Rc)*((C-A).length()) - (Rb+Rc)*((C-B).length()));
                    }

                    edge_costs.push_back(cost);
                }
            }
        }
    }
    sorted_edge_costs = edge_costs;
    sort(sorted_edge_costs.begin(), sorted_edge_costs.end());

    //printf("edges size = %d\n", edges.size());
    if(int(edges.size()) < iteration)
        return 0;
    //for(unsigned int i=0; i<sorted_edge_costs.size(); i++)
    //  printf("%lf\n", sorted_edge_costs[i]);
    
    //record which edge is done
    std::vector <bool> done_edge(edges.size(), false);

    for(int iter=0; iter<iteration; iter++)
    {
        //extract the min cost edge
        int min_edge_index = 0;
        for(unsigned int i=0; i<edges.size(); i++)
        {
            //if(sorted_edge_costs.size() == 0)
            //  return;

            //if this is a valid edge and matches the current minimum
            if(!done_edge[i] && edge_costs[i] == sorted_edge_costs[iter])
            {
                min_edge_index = i;
                break;
            }
        }

        //delete the min edge, i.e. delete the node minB
        BDLSkeletonNode *minA, *minB;
        minA = edges[min_edge_index].first;
        minB = edges[min_edge_index].second;

        //new child list of minA
        std::vector <BDLSkeletonNode *> minA_new_child_list;

        //delete minB by not including it
        for(unsigned int i=0; i<minA->_children.size(); i++)
            if(minA->_children[i] != minB)
                minA_new_child_list.push_back(minA->_children[i]);

        //pass all children of minB to minA
        for(unsigned int i=0; i<minB->_children.size(); i++)
        {
            minA_new_child_list.push_back(minB->_children[i]);
            //update minB
            minB->_children[i]->_prev = minA;
        }

        //update minA
        minA->_children = minA_new_child_list;

        //hihi if minB is a leaf
        if(minB->_children.size() == 0)
            printf("minB hihi\n");

        //minB die
        delete(minB);//if iteration > 1, may have segmentation because minB is deleted, but minB pointer is still in edges
        done_edge[min_edge_index] = true;
    }

    return edges.size()-iteration;
}

osg::ref_ptr <osg::Group> BDLSkeletonNode::skeleton_mesh(BDLSkeletonNode *root, osg::Vec3 origin, double height, int angle, bool output_branch, bool output_leaf, bool maya, double simplication)
{
    //output_leaf = false;
    //maya = true;

    osg::ref_ptr <osg::Group> ret = new osg::Group;
    if(!root)
        return ret;

    //test osgModeler::createLineBox
    //ret->addChild(osgModeler::createLineBox(osg::Vec3(0,0,0), osg::Vec3(1,1,1)));
    //return ret;

    //translate and scale the tree, find height and radius of each node, get the scale
    //double scale = BDLSkeletonNode::rectify_skeleton_tree(root, osg::Vec3(0.0, 0.0, 10.0));
    //the returned scale is used to add an ugly node below the root and for scaling the leaf
    double scale = BDLSkeletonNode::rectify_skeleton_tree(root, origin, height, angle, maya);

    //printf("rectify scale = %lf\n", scale);
    //the scale above is only the rectifying scale for adjusting the height, the local scale is below
    float leaf_scale = BDLSkeletonNode::leaf_scale_hint(root);
    //printf("leaf scale = %lf\n", leaf_scale);

    //debug
    //BDLSkeletonNode::debug_radius(root);
    //BDLSkeletonNode::debug_support_length(root);
    //return NULL;

    //if there's simplification, the leaf scale should be increased
    scale += (1.0-simplication) * scale * 3.2;
    leaf_scale += (1.0-simplication) * leaf_scale * 1.9;

    //remove trivial nodes
    BDLSkeletonNode::compress_skeleton_tree(root);

    //simplify the trees by collasping some edges
    /*
    */
    if(simplication != 1.0)
    {
        int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(root);
        for(int i=0; i<remaining_edges*(1-simplication); i++)
            BDLSkeletonNode::collaspe_skeleton_tree(root);
    }

    //####################

    //try adding color
    //does not work, should add it in geometry
    //osg::Vec4Array *colors = new osg::Vec4Array;
    //colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
    //ret->setColorArray(colors);
    //ret->setColorBinding(osg::Geometry::BIND_OVERALL);

    //construct the path from longest to shortest
    //stores all the leaf skeleton nodes
    std::vector <BDLSkeletonNode *> leafs;
    //distance between leafs and root
    std::vector <double> dists;
    //sorted dists
    std::vector <double> sorted_dists;

    //BFS all skeleton nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();
        
        if(front->_children.size() == 0)
        {
            leafs.push_back(front);
            dists.push_back(front->_height);
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue.push(child);
        }
    }

    //sort the dists
    sorted_dists = dists;
    sort(sorted_dists.begin(), sorted_dists.end());
    reverse(sorted_dists.begin(), sorted_dists.end());

    //debug
    //printf("no. of branches = %d\n", sorted_dists.size());

    for(unsigned int branch=0; branch<sorted_dists.size(); branch++)
    //for(unsigned int branch=0; branch<1; branch++)
    {
        double cur_dist = sorted_dists[branch];
        int cur_index = 0;

        //find the corresponding skeleton leaf
        for(unsigned int l=0; l<dists.size(); l++)
            if(dists[l] == cur_dist)
            {
                cur_index = l;
                break;
            }

        BDLSkeletonNode *cur_leaf = leafs[cur_index];

        std::vector <osg::Vec3> path;
        std::vector <double> radii;

        BDLSkeletonNode *node = cur_leaf;

        //while(node->_prev)
        while(node)
        {
            path.push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));
            radii.push_back(node->_radius);
            //radii.push_back(25);

            if(node->_mesh_done)
                break;
            node->_mesh_done = true;
            
            node = node->_prev;
        }

        //if(branch == 0)
        //   radii[radii.size()-1] = radii[radii.size()-2];

        //shorten the tip and end of the branch for all path, except the root
        if(!radii.empty())
        {
            radii[0] *= 0.75;
            if(branch != 0)
                radii[radii.size()-1] *= 0.75;
        }

        //go for 0.1 more step
        /*
        */
        if(node)
        {
            BDLSkeletonNode *next = node->_prev;
            if(next)
            {
                osg::Vec3 now = osg::Vec3(node->_sx, node->_sy, node->_sz);
                osg::Vec3 to = osg::Vec3(next->_sx, next->_sy, next->_sz);
                osg::Vec3 dir = to - now;
                //no need to normalize since the gap may be very small, 0.5 may be very large
                //dir.normalize();
                path.push_back(now + dir*0.1);
                radii.push_back(node->_radius*0.1);
            }
        }

        //ugly fix, really don't know why
        /*
        */
        if(branch == 0)
        {
            //cannot hard-code here since 0.2 may be very long for this tree
            //path.push_back(osg::Vec3(root->_sx+0.2, root->_sy+0.3, root->_sz-5.0));
            if(maya)
            {
                //ugly fix for fitting the inverted Pittsburbh plane created last year
                if(false)
                    path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy+5.0*scale, root->_sz+0.3*scale));
                else
                    path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy-5.0*scale, root->_sz+0.3*scale));
            }
            else
                //path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy+0.3*scale, root->_sz-5.0*scale));
                path.push_back(osg::Vec3(root->_sx+0.1*scale, root->_sy+0.1*scale, root->_sz-8.0*scale));
            //radii.push_back(25);
            radii.push_back(root->_radius);
        }

        /*
        if(branch != 0)
        {
            path.push_back(path[path.size()-1]);
            radii.push_back(radii[radii.size()-1]);
        }
        else
        {
            path.push_back(osg::Vec3(0.2, 0.3, -10));
            radii.push_back(25);
            path.push_back(osg::Vec3(0.3, 0.2, -10));
            radii.push_back(25);
        }
        */

        //the step_circle size should decreases with the largest radius in radii
        //most nodes have radius much smaller than the root branch
        if(int(radii.size()) == 0)
            continue;
        int step_circle = 12;
        double step_circle_d = step_circle*(radii[radii.size()-1]/root->_radius);
        if(step_circle_d < 0.5)
            step_circle = 4;//if 3, then some branches may looked transparent
        else if(step_circle_d <= 1.0)
            step_circle = 5;
        else
            step_circle = std::max(int(step_circle_d), 8);

        //old step_circle spec
        //int step_circle = 12;
        //double step_circle_d = step_circle*(radii[radii.size()-1]/root->_radius);
        //if(step_circle_d < 1.0)
        //    step_circle = 4;

        //printf("branch %d: step_circle(%d)\n", branch, step_circle);

        //real work
        if(output_branch)
            ret->addChild(osgModeler::createTube(path, radii, 2, step_circle));
            //ret->addChild(osgModeler::createTube(path, radii, 10, step_circle));
    }
    //printf("branch is done\n");

    //####################
    //leaf prototype 
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 0));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 45));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 90));
    srand(time(NULL));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 135));
    //return ret;

    //todo
    //add leaves
    //bfs once, find the approximate tangent by (current - prev)
    //add a leaf or leaves if this node has no children
    //or its radius is smaller than a fraction of the radius of the root
    srand(time(NULL));
    double radius_thresold = root->_radius * 0.03;
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(root);

    //want to combine all leaf osg::Geode into one giant Geode
    //so use this vector to store all vertices and texture coords
    osg::ref_ptr <osg::Vec3Array> all_v = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> all_tex = new osg::Vec2Array;

    //all leafs share the same stateset
    osg::StateSet *leaf_state = osgModeler::leaf_stateset();

    //is flat leaf?
    bool flat_leaf = simplication == 1.0 ? false : true;

    while(!Queue2.empty())
    {
        BDLSkeletonNode *front = Queue2.front();
        Queue2.pop();

        //push all of my children
        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue2.push(front->_children[i]);

        //no parent, i.e. a root, should not has any leaf
        if(!front->_prev)
            continue;

        //graph's leafs || branch with small radius
        if(front->_children.size() == 0 || front->_radius <= radius_thresold)
        //if(front->_children.size() == 0)
        {
            osg::Vec3 pos(front->_sx, front->_sy, front->_sz);
            osg::Vec3 pre(front->_prev->_sx, front->_prev->_sy, front->_prev->_sz); 
            osg::Vec3 approxi_tangent = pos - pre;
            approxi_tangent.normalize();

            //consider a plane with approxi_tangent as the normal and contains point pos
            //find basis u, v of this plane
            osg::Vec3 u(approxi_tangent.y(), -approxi_tangent.x(), 0.0);
            osg::Vec3 v = approxi_tangent ^ u;

            //skipped: v'=u*cos(phi)+v*sin(phi)

            //consider another plane with u as normal and contains point pos
            //the basis of this plane is chosen as approxi_tangent and v
            //div_angle is the angle angle spanned by approxi_tangent and the root of the leaf
            double div_angle = (rand()%30-15+70)/180.0*M_PI;

            //number of leaf of this node has
            int no_leaf = rand()%4+1;
            for(int i=1; i<=no_leaf; i++)
            {
                double angle = 360.0/no_leaf*i/180.0*M_PI;
                osg::Vec3 div = u*cos(angle) + v*sin(angle);
                div = approxi_tangent * cos(div_angle) + div * sin(div_angle);

                //one osg::Geode for one leaf
                //osg::ref_ptr <osg::Geode> leaf_geode = osgModeler::createLeaf(pos, div, 360.0/no_leaf*i);

                //the expensive texture step
                //leaf_geode->setStateSet(leaf_state);

                //ret->addChild(leaf_geode);
                ////ret->addChild(osgModeler::createLeaf(pos, div, 0.0));

                //preparing for one giant geode that contains all leaves
                osgModeler::createLeaf(all_v, all_tex, pos, div, 360.0/no_leaf*i, leaf_scale, flat_leaf);
            }

            //debug u and v
            //ret->addChild(osgModeler::createLeaf(pos, u));
            //ret->addChild(osgModeler::createLeaf(pos, v));
        }
    }
    osg::ref_ptr <osg::Geode> leaf_geode = osgModeler::create_batch_leaf(all_v, all_tex, flat_leaf);
    leaf_geode->setStateSet(leaf_state);

    if(output_leaf)
       ret->addChild(leaf_geode);
    //ret->addChild(HoughTransform3d::debug_vote());

    return ret;
}

osg::ref_ptr <osg::Group> BDLSkeletonNode::skeleton_mesh(BDLSkeletonNode *root, osg::Group *&branch_group, osg::Geode *&leave_geode, int& triangle_cnt, osg::Vec3 origin, double height, int angle, bool output_branch, bool output_leaf, bool maya, double simplication)
{
    //output_leaf = false;
    //maya = true;

    osg::ref_ptr <osg::Group> ret = new osg::Group;
    if(!root)
        return ret;

    //setup branch_group smart pointer for storage
    osg::ref_ptr <osg::Group> branch_group_ptr = new osg::Group;

    //reset triangle_cnt counter
    //int triangle_cnt = 0;
    triangle_cnt = 0;

    //test osgModeler::createLineBox
    //ret->addChild(osgModeler::createLineBox(osg::Vec3(0,0,0), osg::Vec3(1,1,1)));
    //return ret;

    //translate and scale the tree, find height and radius of each node, get the scale
    //double scale = BDLSkeletonNode::rectify_skeleton_tree(root, osg::Vec3(0.0, 0.0, 10.0));
    //the returned scale is used to add an ugly node below the root and for scaling the leaf
    double scale = BDLSkeletonNode::rectify_skeleton_tree(root, origin, height, angle, maya);

    //printf("rectify scale = %lf\n", scale);
    //the scale above is only the rectifying scale for adjusting the height, the local scale is below
    float leaf_scale = BDLSkeletonNode::leaf_scale_hint(root);
    //printf("leaf scale = %lf\n", leaf_scale);

    //debug
    //BDLSkeletonNode::debug_radius(root);
    //BDLSkeletonNode::debug_support_length(root);
    //return NULL;

    //if there's simplification, the leaf scale should be increased
    scale += (1.0-simplication) * scale * 3.2;
    leaf_scale += (1.0-simplication) * leaf_scale * 1.9;

    //remove trivial nodes
    BDLSkeletonNode::compress_skeleton_tree(root);

    //simplify the trees by collasping some edges
    /*
    */
    if(simplication != 1.0)
    {
        int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(root);
        for(int i=0; i<remaining_edges*(1-simplication); i++)
            BDLSkeletonNode::collaspe_skeleton_tree(root);
    }

    //####################

    //try adding color
    //does not work, should add it in geometry
    //osg::Vec4Array *colors = new osg::Vec4Array;
    //colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
    //ret->setColorArray(colors);
    //ret->setColorBinding(osg::Geometry::BIND_OVERALL);

    //construct the path from longest to shortest
    //stores all the leaf skeleton nodes
    std::vector <BDLSkeletonNode *> leafs;
    //distance between leafs and root
    std::vector <double> dists;
    //sorted dists
    std::vector <double> sorted_dists;

    //BFS all skeleton nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();
        
        if(front->_children.size() == 0)
        {
            leafs.push_back(front);
            dists.push_back(front->_height);
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue.push(child);
        }
    }

    //sort the dists
    sorted_dists = dists;
    sort(sorted_dists.begin(), sorted_dists.end());
    reverse(sorted_dists.begin(), sorted_dists.end());

    //debug
    //printf("no. of branches = %d\n", sorted_dists.size());

    //create a material for the branches
    osg::ref_ptr <osg::Material> material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.25f, 0.15, 0.0f, 1.0f));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.05f, 0.05f, 0.05f, 1.0f));
    material->setShininess(osg::Material::FRONT, 5.0f);

    for(unsigned int branch=0; branch<sorted_dists.size(); branch++)
    //for(unsigned int branch=0; branch<1; branch++)
    {
        double cur_dist = sorted_dists[branch];
        int cur_index = 0;

        //find the corresponding skeleton leaf
        for(unsigned int l=0; l<dists.size(); l++)
            if(dists[l] == cur_dist)
            {
                cur_index = l;
                break;
            }

        BDLSkeletonNode *cur_leaf = leafs[cur_index];

        std::vector <osg::Vec3> path;
        std::vector <double> radii;

        BDLSkeletonNode *node = cur_leaf;

        //while(node->_prev)
        while(node)
        {
            path.push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));
            radii.push_back(node->_radius);
            //radii.push_back(25);

            if(node->_mesh_done)
                break;
            node->_mesh_done = true;
            
            node = node->_prev;
        }

        //if(branch == 0)
        //   radii[radii.size()-1] = radii[radii.size()-2];


        //go for 1 more node, to solve the problem of degenerated case for step == 2 where
       //the binding circle is cutting the outgoing direction
        /*
        if(node)
        {
            BDLSkeletonNode *next = node->_prev;
            if(next)
            {
                //avoid singularity that extend to the root
                if(next->_prev)
                {
                    path.push_back(osg::Vec3(next->_sx, next->_sy, next->_sz));
                    radii.push_back(next->_radius);
                }
            }
        }
        */

        //shorten the tip and end of the branch for all path, except the root
        if(!radii.empty())
        {
            radii[0] *= 0.55;
            if(branch != 0)
            {
                radii[radii.size()-1] *= 0.55;

                if(int(radii.size())-2 >= 0)
                    radii[radii.size()-2] *= 0.85;
            }
        }

        //go for 0.1 more step
        /*
        */
        if(node && branch != 0)
        {
            BDLSkeletonNode *next = node->_prev;
            if(next)
            {
                osg::Vec3 now = osg::Vec3(node->_sx, node->_sy, node->_sz);
                osg::Vec3 to = osg::Vec3(next->_sx, next->_sy, next->_sz);
                osg::Vec3 dir = to - now;
                //no need to normalize since the gap may be very small, 0.5 may be very large
                //dir.normalize();
                path.push_back(now + dir*0.1);
                radii.push_back(node->_radius*0.1);
            }
        }

        //ugly fix, really don't know why, for case where the root and its child lie on a straight line
        if(branch == 0)
        {
            //cannot hard-code here since 0.2 may be very long for this tree
            //path.push_back(osg::Vec3(root->_sx+0.2, root->_sy+0.3, root->_sz-5.0));
            if(maya)
            {
                //ugly fix for fitting the inverted Pittsburbh plane created last year
                if(false)
                    path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy+5.0*scale, root->_sz+0.3*scale));
                else
                    path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy-5.0*scale, root->_sz+0.3*scale));
            }
            else
                //path.push_back(osg::Vec3(root->_sx+0.2*scale, root->_sy+0.3*scale, root->_sz-5.0*scale));
                path.push_back(osg::Vec3(root->_sx+0.1*scale, root->_sy+0.1*scale, root->_sz-4.0*scale));
            //radii.push_back(25);
            radii.push_back(root->_radius);
        }

        /*
        if(branch != 0)
        {
            path.push_back(path[path.size()-1]);
            radii.push_back(radii[radii.size()-1]);
        }
        else
        {
            path.push_back(osg::Vec3(0.2, 0.3, -10));
            radii.push_back(25);
            path.push_back(osg::Vec3(0.3, 0.2, -10));
            radii.push_back(25);
        }
        */

        //the step_circle size should decreases with the largest radius in radii
        //most nodes have radius much smaller than the root branch
        if(int(radii.size()) == 0)
            continue;
        int step_circle = 12;
        double step_circle_d = step_circle*(radii[radii.size()-1]/root->_radius);
        if(step_circle_d < 0.5)
            step_circle = 4;//if 3, then some branches may looked transparent
        else if(step_circle_d <= 1.0)
            step_circle = 5;
        else
            step_circle = std::max(int(step_circle_d), 8);
        //printf("%d: %d\n", branch, step_circle);

        //real work
        if(output_branch)
        {
            double radius_cut = radii[radii.size()-1] / root->_radius;

            //todo: determine value of radius_cut
            if(simplication == 1.0 || radius_cut > 0.008)
            {
                osg::ref_ptr <osg::Geode> b_child = osgModeler::createTube(path, radii, 2, step_circle);
                //osg::ref_ptr <osg::Geode> b_child = osgModeler::createTube(path, radii, 10, step_circle);

                //old
                //ret->addChild(osgModeler::createTube(path, radii, 10, step_circle));

                //set material to this geode
                osg::StateSet *state = b_child->getOrCreateStateSet();
                state->setAttribute(material.get());

                //ret->addChild(b_child);
                branch_group_ptr->addChild(b_child);

                //update triangle_cnt

                //printf("path(%d)\tstep_circle(%d)\tradius(%lf)(%lf)\n", int(path.size()), step_circle, radii[radii.size()-1], radius_cut);
                triangle_cnt += path.size() == 2 ? 8 : (path.size()-1) * step_circle * 2;
            }
        }
    }
    //printf("branch is done\n");
    
    //try adding a material to branch_group_ptr
    //osg::StateSet *state = branch_group_ptr->getOrCreateStateSet();
    //osg::ref_ptr <osg::Material> mat = new osg::Material;
    //mat->setDiffuse(osg::Material::FRONT, osg::Vec4(0.25f, 0.15, 0.0f, 1.0f));
    //mat->setSpecular(osg::Material::FRONT, osg::Vec4(0.05f, 0.05f, 0.05f, 1.0f));
    //mat->setShininess(osg::Material::FRONT, 5.0f);
    //state->setAttribute(mat.get());

    //add back the branch_group to ret
    ret->addChild(branch_group_ptr);

    //set the branch_group pointer
    branch_group = branch_group_ptr.get();

    //####################
    //leaf prototype 
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 0));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 45));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 90));
    srand(time(NULL));
    //ret->addChild(osgModeler::createLeaf(osg::Vec3(0,0,0), osg::Vec3(1,1,1), 135));
    //return ret;

    //todo
    //add leaves
    //bfs once, find the approximate tangent by (current - prev)
    //add a leaf or leaves if this node has no children
    //or its radius is smaller than a fraction of the radius of the root
    srand(time(NULL));
    double radius_thresold = root->_radius * 0.03;
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(root);

    //want to combine all leaf osg::Geode into one giant Geode
    //so use this vector to store all vertices and texture coords
    osg::ref_ptr <osg::Vec3Array> all_v = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> all_tex = new osg::Vec2Array;

    //all leafs share the same stateset
    osg::StateSet *leaf_state = osgModeler::leaf_stateset();

    //is flat leaf?
    bool flat_leaf = simplication == 1.0 ? false : true;

    while(!Queue2.empty())
    {
        BDLSkeletonNode *front = Queue2.front();
        Queue2.pop();

        //push all of my children
        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue2.push(front->_children[i]);

        //no parent, i.e. a root, should not has any leaf
        if(!front->_prev)
            continue;

        //graph's leafs || branch with small radius
        if(front->_children.size() == 0 || front->_radius <= radius_thresold)
        //if(front->_children.size() == 0)
        {
            osg::Vec3 pos(front->_sx, front->_sy, front->_sz);
            osg::Vec3 pre(front->_prev->_sx, front->_prev->_sy, front->_prev->_sz); 
            osg::Vec3 approxi_tangent = pos - pre;
            approxi_tangent.normalize();

            //consider a plane with approxi_tangent as the normal and contains point pos
            //find basis u, v of this plane
            osg::Vec3 u(approxi_tangent.y(), -approxi_tangent.x(), 0.0);
            osg::Vec3 v = approxi_tangent ^ u;

            //skipped: v'=u*cos(phi)+v*sin(phi)

            //consider another plane with u as normal and contains point pos
            //the basis of this plane is chosen as approxi_tangent and v
            //div_angle is the angle angle spanned by approxi_tangent and the root of the leaf
            double div_angle = (rand()%30-15+70)/180.0*M_PI;

            //number of leaf of this node has
            int no_leaf = rand()%4+1;
            for(int i=1; i<=no_leaf; i++)
            {
                double angle = 360.0/no_leaf*i/180.0*M_PI;
                osg::Vec3 div = u*cos(angle) + v*sin(angle);
                div = approxi_tangent * cos(div_angle) + div * sin(div_angle);

                //one osg::Geode for one leaf
                //osg::ref_ptr <osg::Geode> leaf_geode = osgModeler::createLeaf(pos, div, 360.0/no_leaf*i);

                //the expensive texture step
                //leaf_geode->setStateSet(leaf_state);

                //ret->addChild(leaf_geode);
                ////ret->addChild(osgModeler::createLeaf(pos, div, 0.0));

                //preparing for one giant geode that contains all leaves
                osgModeler::createLeaf(all_v, all_tex, pos, div, 360.0/no_leaf*i, leaf_scale, flat_leaf);
            }

            triangle_cnt += flat_leaf ? no_leaf * 2 : no_leaf * 4;

            //debug u and v
            //ret->addChild(osgModeler::createLeaf(pos, u));
            //ret->addChild(osgModeler::createLeaf(pos, v));
        }
    }
    osg::ref_ptr <osg::Geode> leaf_geode = osgModeler::create_batch_leaf(all_v, all_tex, flat_leaf);
    leaf_geode->setStateSet(leaf_state);

    if(output_leaf)
    {
       ret->addChild(leaf_geode);

       //set the leave_group pointer
       leave_geode = leaf_geode;
    }
    //ret->addChild(HoughTransform3d::debug_vote());

    //debug triangle_cnt
    //printf("triangle_cnt = %d\n", triangle_cnt);

    return ret;
}

osg::ref_ptr <osg::Group> BDLSkeletonNode::skeleton_node(BDLSkeletonNode *root)
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;
    if(!root)
        return ret;

    //find all the vertices for the point model by BFS once
    osg::ref_ptr <osg::Vec3Array> vertices = new osg::Vec3Array;

    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        vertices->push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));

        //push all of its children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    //find all the line_vertices for the line model by DFS once
    //this is very similar to TopologicalSkeleton::lineVertices()
    osg::ref_ptr <osg::Vec3Array> line_vertices = new osg::Vec3Array;
    std::stack <BDLSkeletonNode *> Stack;
    Stack.push(root);

    while(!Stack.empty())
    {
        BDLSkeletonNode *node = Stack.top();
        Stack.pop();

        //for each node, store <myself,my child> as a pair
        for(unsigned int i=0; i<node->_children.size(); i++)
        {
            BDLSkeletonNode *child = node->_children[i];
            line_vertices->push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));
            line_vertices->push_back(osg::Vec3(child->_sx, child->_sy, child->_sz));

            Stack.push(child);
        }
    }

    PointModel pm;
    pm.setColor(1.0, 0.0, 0.0, 1.0);

    LineModel lm(line_vertices);

    //add the points
    ret->addChild(pm.createScene(vertices));
    pm.setSize(2.0);
    //add the lines
    ret->addChild(lm.createScene());

    return ret;
}

BDLSkeletonNode *BDLSkeletonNode::copy_tree(BDLSkeletonNode *root)
{
    //copy_tree is very similar to ts2bdlsn
    if(!root)
        return NULL;

    //just copy the positions and topology first
    //tranverse all and new each nodoe
    //todo: handle the new-ed object
    BDLSkeletonNode *ret = new BDLSkeletonNode();
    ret->_sx = root->_sx;
    ret->_sy = root->_sy;
    ret->_sz = root->_sz;

    //BFS on a pair to copy pos and child
    std::pair <BDLSkeletonNode *, BDLSkeletonNode *> base = std::make_pair(root, ret);
    //for copying the _prev_support pointers
    std::map <BDLSkeletonNode *, int> source_dict; //for storing source pointer => id
    std::map <int, BDLSkeletonNode *> target_dict; //for storing id => target pointer
    int node_id = 0;

    std::queue <std::pair <BDLSkeletonNode *, BDLSkeletonNode *> > Queue;
    Queue.push(base);

    while(!Queue.empty())
    {
        std::pair <BDLSkeletonNode *, BDLSkeletonNode *> front = Queue.front(); 
        Queue.pop();

        //tn is the source, sn is the target
        BDLSkeletonNode *tn = front.first;
        BDLSkeletonNode *sn = front.second;

        source_dict[tn] = node_id;
        target_dict[node_id] = sn;

        //for each child of tn, new a copy for sn
        for(unsigned int i=0; i<tn->_children.size(); i++)
        {
            BDLSkeletonNode *tn_child = tn->_children[i];
            BDLSkeletonNode *sn_child = new BDLSkeletonNode();

            //pos
            sn_child->_sx = tn_child->_sx;
            sn_child->_sy = tn_child->_sy;
            sn_child->_sz = tn_child->_sz;

            //prev, child
            sn_child->_prev = sn;
            sn->_children.push_back(sn_child);

            std::pair <BDLSkeletonNode *, BDLSkeletonNode *> next = std::make_pair(tn_child, sn_child);
            Queue.push(next);
        }

        node_id++;
    }

    //bfs again, copy the _prev_support pointers
    Queue.push(base);

    while(!Queue.empty())
    {
        std::pair <BDLSkeletonNode *, BDLSkeletonNode *> front = Queue.front(); 
        Queue.pop();

        //tn is the source, sn is the target
        BDLSkeletonNode *tn = front.first;
        BDLSkeletonNode *sn = front.second;

        if(tn->_prev_support)
            sn->_prev_support = target_dict[source_dict[tn->_prev_support]];

        //for each child of tn, new a copy for sn
        for(unsigned int i=0; i<tn->_children.size(); i++)
        {
            BDLSkeletonNode *tn_child = tn->_children[i];
            BDLSkeletonNode *sn_child = sn->_children[i];

            std::pair <BDLSkeletonNode *, BDLSkeletonNode *> next = std::make_pair(tn_child, sn_child);
            Queue.push(next);
        }
    }

    //set the radius of the root, because all the other nodes depend on it
    //ret->_radius = 150.0;
    ret->_radius = root->_radius;

    return ret;
}

void BDLSkeletonNode::delete_this(BDLSkeletonNode *sn)
{
    if(!sn)
        return;
    
    //check if _prev exists
    BDLSkeletonNode *par = sn->_prev;
    if(par)
    {
        std::vector <BDLSkeletonNode *> children;
        for(unsigned int i=0; i<par->_children.size(); i++)
            if(par->_children[i] != sn)
                children.push_back(par->_children[i]);

        par->_children = children;
    }

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(sn);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);

        delete(front);
    }
}

float BDLSkeletonNode::dist(BDLSkeletonNode *a, BDLSkeletonNode *b)
{
    if(!a || !b)
        return -1.0;
    return pow((a->_sx-b->_sx)*(a->_sx-b->_sx) + (a->_sy-b->_sy)*(a->_sy-b->_sy) + (a->_sz-b->_sz)*(a->_sz-b->_sz), 0.5);
}

float BDLSkeletonNode::leaf_scale_hint(BDLSkeletonNode *root)
{
    if(!root)
        return 0.0f;

    //bfs once, find the average radius of the leaves
    /*
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    float avg_radius = 0.0f;
    int avg_cnt = 0;

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        if(node->_children.size() == 0)
        {
            if(node->_prev)
            {
                avg_radius += node->_radius;
                avg_cnt++;
            }
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    if(avg_cnt != 0)
        avg_radius /= avg_cnt;

    //printf("avg_radius = %f\n", avg_radius);
    //printf("0.04 / avg_radius = %f\n", 0.04 / avg_radius);

    float hint = avg_radius * 0.28636117386;
    printf("root's radius = %lf\n", root->_radius);
    */
    float hint = root->_radius * 0.0095;//0.01465;

    return hint;
}

bool BDLSkeletonNode::is_singular(BDLSkeletonNode *root)
{
    if(!root)
        return false;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        if(node->_children.size() > 1)
            return false;

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    return true;
}

float BDLSkeletonNode::max_height(BDLSkeletonNode *root)
{
    float ret = -1.0;

    if(!root)
        return ret;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        float cur_dist = BDLSkeletonNode::dist(node, root);
        if(ret == -1.0 || cur_dist > ret)
            ret = cur_dist;

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    return ret;
}

int BDLSkeletonNode::max_height_hops(BDLSkeletonNode *root)
{
    int ret = -1;

    if(!root)
        return ret;

    float max_dist = -1.0f;
    BDLSkeletonNode *max_node = NULL;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        float cur_dist = BDLSkeletonNode::dist(node, root);
        if(max_dist == -1.0f || cur_dist > ret)
        {
            max_dist = cur_dist;
            max_node = node;
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    if(max_node)
    {
        ret = 0;

        while(max_node)
        {
            max_node = max_node->_prev;
            ret++;
        }
    }

    return ret;
}

int BDLSkeletonNode::tree_size(BDLSkeletonNode *root)
{
    int ret = 0;
    
    if(!root)
        return ret;

    //bfs once and count the number of nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        ret++;

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    return ret;
}

void BDLSkeletonNode::save_skeleton(BDLSkeletonNode *root, const char *path)
{
    if(!root || !path)
        return;

    FILE *out = fopen(path, "w");
    if(!out)
        return;

    //all nodes and links
    std::vector <BDLSkeletonNode *> nodes;
    std::vector <std::pair <int, int> > links;

    //helper
    std::vector <std::pair <BDLSkeletonNode *, BDLSkeletonNode *> > links_helper;

    //bfs once and push all nodes and links_helper
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        nodes.push_back(node);

        for(unsigned int i=0; i<node->_children.size(); i++)
        {
            BDLSkeletonNode *child = node->_children[i];
            Queue.push(child);
            links_helper.push_back(std::make_pair(node, child));
        }
    }

    //for each pair in links_helper, find back the index in nodes, and push it
    for(unsigned int i=0; i<links_helper.size(); i++)
    {
        BDLSkeletonNode *par = links_helper[i].first;
        BDLSkeletonNode *child = links_helper[i].second;

        int par_index = 0, child_index = 0;

        for(unsigned int j=0; j<nodes.size(); j++)
        {
            BDLSkeletonNode *node = nodes[j];
            if(node == par)
                par_index = j;
            if(node == child)
                child_index = j;
        }

        links.push_back(std::make_pair(par_index, child_index));
    }

    //file IO
    fprintf(out, "%d\n", int(nodes.size()));
    
    for(unsigned int i=0; i<nodes.size(); i++)
        fprintf(out, "%f %f %f\n", nodes[i]->_sx, nodes[i]->_sy, nodes[i]->_sz);

    fprintf(out, "%d\n", int(links.size()));

    for(unsigned int i=0; i<links.size(); i++)
        fprintf(out, "%d %d\n", links[i].first, links[i].second);
}

float BDLSkeletonNode::main_branch_length(BDLSkeletonNode *root)
{
    float ret = -1.0f;
    
    if(!root)
        return ret;

    //bfs once to find the first branching node of the main branch
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    BDLSkeletonNode *first_branching_node = NULL;

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        if(node->_children.size() > 1)
        {
            first_branching_node = node;
            break;
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    return dist(root, first_branching_node);
}

void BDLSkeletonNode::rectify_default_skeleton(BDLSkeletonNode *root, float mb_length, float b_h, int g)
{
    if(!root || mb_length <= 0 || b_h <= 0 || g <= 0)
        return;

    //default skeleton's height
    float mt_h = BDLSkeletonNode::max_height(root);
    
    //default skelton's mainbranch height
    float mb_h = BDLSkeletonNode::main_branch_length(root);

    if(mt_h <= 0 || mb_h <= 0)
        return;

    //rectify the upper and lower part respectively
    float k_d_up = (b_h * g / 100.) / (mt_h - mb_h);
    float k_d_down = mb_length / mb_h;

    //printf("k_d_up(%f) b_h(%f) mt_h(%f) mb_h(%f)\n", k_d_up, b_h, mt_h, mb_h);

    //bfs once and scale
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    bool main_branch = true;
    osg::Vec3 middle;

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        if(main_branch && node->_children.size() > 1)
        {
            main_branch = false;
            middle = osg::Vec3(node->_sx, node->_sy, node->_sz);

            //printf("middle(%f %f %f)\n", middle.x(), middle.y(), middle.z());
        }

        if(main_branch)
        {
            node->_sx = node->_sx * k_d_down;
            node->_sy = node->_sy * k_d_down;
            node->_sz = node->_sz * k_d_down;
        }
        else
        {
            node->_sx = (node->_sx - middle.x()) * k_d_up + middle.x();
            node->_sy = (node->_sy - middle.y()) * k_d_up + middle.y();
            node->_sz = (node->_sz - middle.z()) * k_d_up + middle.z();
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }
}

void BDLSkeletonNode::remove_child(BDLSkeletonNode *root, BDLSkeletonNode *child)
{
    if(!root || !child)
        return;

    //new child
    std::vector <BDLSkeletonNode *> new_children;
    for(unsigned int i=0; i<root->_children.size(); i++)
    {
        BDLSkeletonNode *node = root->_children[i];
        if(node != child)
            new_children.push_back(node);
    }

    //assign back the new vector
    root->_children = new_children;

    //update the parent of child
    child->_prev = NULL;
}

BDLPointGraphNode::BDLPointGraphNode(): _done(false), _dist(-1.0), _prev(NULL), _this(NULL), _sub_group(-1), _bin(-1), _skeleton_king(NULL), _pigegon_visited(false)
{
}

const bool BDLPointGraphNode::operator < (const BDLPointGraphNode& node) const
{
    return !(_dist < node._dist);
}

BDLPointGraph::BDLPointGraph(): _root_node(NULL), _quantization_step(1.0), _group_size(0)
{
}

BDLPointGraph::~BDLPointGraph()
{
    for(unsigned int i=0; i<_nodes.size(); i++)
        delete _nodes[i];
}

BDLPointGraphNode *BDLPointGraph::new_node()
{
    BDLPointGraphNode *ret = new BDLPointGraphNode;
    _nodes.push_back(ret);
    return ret;
}

void BDLPointGraph::new_nodes(int num)
{
    for(int i=0; i<num; i++)
    {
        BDLPointGraphNode *ret = new BDLPointGraphNode;
        _nodes.push_back(ret);
    }
}

void BDLPointGraph::find_root()
{
    BDLPointGraphNode *lowest = NULL, *highest = NULL;

    //printf("_nodes.size() = %d\n", _nodes.size());

    for(unsigned int i=0; i<_nodes.size(); i++)
    {
        if(!lowest || !highest)
        {
            lowest = _nodes[i];
            highest = _nodes[i];
            continue;
        }
        BDLPointGraphNode *node = _nodes[i];
        BDLPoint bdlPoint = node->_self;
        
        //find the lowest node
        if(bdlPoint.z / bdlPoint.w < lowest->_self.z / lowest->_self.w)
            lowest = _nodes[i];

        //find the higest node
        if(bdlPoint.z / bdlPoint.w > highest->_self.z / highest->_self.w)
            highest = _nodes[i];
    }
    
    _lowest_node = lowest;
    _highest_node = highest;

    //just the lowest point
    _root_node = lowest;

    //hard-coded
    if(false)
    {
        for(unsigned int i=0; i<_nodes.size(); i++)
        {
            BDLPointGraphNode *node = _nodes[i];
            BDLPoint bdlPoint = node->_self;
            if(fabs(bdlPoint.x+1.11f) < 0.001f && fabs(bdlPoint.y+0.193f) < 0.001f && fabs(bdlPoint.z-0.029f) < 0.001f)
            {
                _root_node = node;
                printf("hard-coded success\n");
                printf("root(%f %f %f)\n", bdlPoint.x, bdlPoint.y, bdlPoint.z);
                return;
            }
        }
    }

    //alternatively: find the average of the lowest 10%
    float min_height = lowest->_self.z / lowest->_self.w;
    float max_height = highest->_self.z / highest->_self.w;
    float range = max_height - min_height;
    float threshold = min_height + range * 0.1f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    int cnt = 0;

    for(unsigned int i=0; i<_nodes.size(); i++)
    {
        BDLPointGraphNode *node = _nodes[i];
        BDLPoint bdlPoint = node->_self;

        float cur = bdlPoint.z / bdlPoint.w;
        if(cur < threshold)
        {
            ax += bdlPoint.x / bdlPoint.w;
            ay += bdlPoint.y / bdlPoint.w;
            az += bdlPoint.z / bdlPoint.w;
            cnt++;
        }
    }

    if(cnt != 0)
    {
        ax /= cnt;
        ay /= cnt;
        az /= cnt;
        _root_node->_self.x = ax;
        _root_node->_self.y = ay;
        //_root_node->_self.z = az;
        _root_node->_self.w = 1.0f;
    }
}

void BDLPointGraph::run_dijkstra()
{
    if(!_root_node)
        return;

    std::priority_queue<BDLPointGraphNode> pq;
    
    //the seed
    _root_node->_dist = 0.0;
    BDLPointGraphNode root = *_root_node;
    root._this = _root_node;
    pq.push(root);

    while(!pq.empty())
    {
        BDLPointGraphNode node = pq.top();
        pq.pop();

        if(node._this->_done)
            continue;

        //push all un-done neighbors
        for(unsigned int i=0; i<node._edges.size(); i++)
        {
            if(node._edges[i]->_done)
                continue;

            BDLPointGraphNode neighbor = *node._edges[i];
            neighbor._this = node._edges[i];
            neighbor._dist = node._dist + node._costs[i];
            neighbor._prev = node._this;
            
            pq.push(neighbor);
        }

        //update the original instance
        node._this->_dist = node._dist;
        node._this->_prev = node._prev;
        node._this->_sub_group = 0;//do the root sub-group first

        node._this->_done = true;
    }

    //do the root sub-group first
    _group_size++;
}

BDLSkeletonNode * BDLPointGraph::build_skeleton()
{
    if(!_root_node)
        return NULL;

    //the root of the skeleton node graph is the root of the point graph
    BDLPoint bdl_root = _root_node->_self;
    BDLSkeletonNode *ret = new BDLSkeletonNode(bdl_root.x/bdl_root.w, bdl_root.y/bdl_root.w, bdl_root.z/bdl_root.w);
    _root_node->_skeleton_king = ret;
    ret->_slave_nodes.push_back(_root_node);

    for(int group=0; group<_group_size; group++)
    {
        double group_avg = 0.0;
        int group_size = 0;

        //find the quantization step for this group
        for(unsigned int i=0; i<_nodes.size(); i++)
        {
            BDLPointGraphNode *node = _nodes[i];
            if(node->_sub_group == group)
            {
                group_avg += node->_dist;
                group_size++;
                node->_bin = node->_dist / _quantization_step;
            }
        }

        group_avg /= group_size;
        _quantization_step = group_avg / 12.;

        int max_bin_id = -1;

        //assign bin id for this group
        for(unsigned int i=0; i<_nodes.size(); i++)
        {
            BDLPointGraphNode *node = _nodes[i];
            if(node->_sub_group == group)
            {
                node->_bin = node->_dist / _quantization_step;
                if(node->_bin > max_bin_id || max_bin_id == -1)
                    max_bin_id = node->_bin;

                //put the first 4 bins into one (for creating a straight branch)
                if(true)
                {
                    if(node->_bin < 6)
                        node->_bin = 0;
                }
            }
        }

        _bin_sizes.push_back(max_bin_id);

        //the pigeon holes
        std::vector <std::vector <BDLPointGraphNode *> > pigeon_holes(max_bin_id+1, std::vector <BDLPointGraphNode *>());

        //put the node with _bin = n to the n-th pigeon holes
        for(unsigned int n=0; n<_nodes.size(); n++)
        {
            //for all nodes for this sub_group
            BDLPointGraphNode *node = _nodes[n];
            if(node->_sub_group != group)
                continue;

            //put the node to the corresponding bin
            pigeon_holes[node->_bin].push_back(node);
        }

        //printf("pigeon_holes.size = %d\n", pigeon_holes.size());

        //for all nodes in each pigeon holes, find all the connected components by DFS
        std::map <BDLSkeletonNode *, bool> deleted_dict;//keep track of which node is deleted
        for(unsigned int hole=0; hole<pigeon_holes.size(); hole++)
        {
            std::vector <BDLPointGraphNode *> nodes = pigeon_holes[hole];

            //no pigeon
            if(nodes.size() == 0)
                continue;

            //printf("pigeon size = %d\n", nodes.size());

            //each skeleton node corresponds to one connected component
            std::vector <BDLSkeletonNode *> skeleton_nodes;

            //standard DFS
            for(unsigned int pigeon=0; pigeon<nodes.size(); pigeon++)
            {
                if(nodes[pigeon]->_pigegon_visited)
                    continue;

                //found new connected component
                BDLSkeletonNode *s_node = new BDLSkeletonNode;
                skeleton_nodes.push_back(s_node);

                std::stack <BDLPointGraphNode *> Stack;
                Stack.push(nodes[pigeon]);

                while(!Stack.empty())
                {
                    BDLPointGraphNode *top = Stack.top();
                    Stack.pop();

                    if(top->_pigegon_visited)
                        continue;

                    s_node->_slave_nodes.push_back(top);
                    top->_skeleton_king = s_node;

                    //push all of my childen that is also belong to this bin
                    for(unsigned int c=0; c<top->_edges.size(); c++)
                        if(top->_edges[c]->_bin == top->_bin)
                            Stack.push(top->_edges[c]);

                    top->_pigegon_visited = true;
                }
            }

            //printf("\n%d\n", skeleton_nodes.size());

            //for each skeleton node, update its 3D position and radius
            for(unsigned int s_node=0; s_node<skeleton_nodes.size(); s_node++)
            {
                BDLSkeletonNode *node = skeleton_nodes[s_node];
                double cx = 0.0, cy = 0.0, cz = 0.0;
                unsigned int sample_size = node->_slave_nodes.size();
                for(unsigned int i=0; i<sample_size; i++)
                {
                    BDLPoint slave = node->_slave_nodes[i]->_self;
                    cx += slave.x / slave.w;
                    cy += slave.y / slave.w;
                    cz += slave.z / slave.w;
                }
                if(sample_size > 0)
                {
                    cx /= sample_size;
                    cy /= sample_size;
                    cz /= sample_size;
                }
                node->_sx = cx;
                node->_sy = cy;
                node->_sz = cz;

                //find the radius of this skeleton node
                std::vector <double> radius_samples;
                for(unsigned int i=0; i<sample_size; i++)
                {
                    BDLPoint slave = node->_slave_nodes[i]->_self;
                    double radius = sqrt(pow(cx-slave.x/slave.w, 2) + pow(cy-slave.y/slave.w, 2) + pow(cz-slave.z/slave.w, 2)); 
                    radius_samples.push_back(radius);
                }
                //sort the samples to get the median
                sort(radius_samples.begin(), radius_samples.end());
                node->_radius = radius_samples[radius_samples.size()/2];

                //printf("%d: r(%f)\n", s_node, node->_radius);
                //printf("%d: (%f,%f,%f)\n", s_node, cx, cy, cz);
            }

            //connect the new skeleton_nodes with the previous skeleton nodes
            for(unsigned int s_node=0; s_node<skeleton_nodes.size(); s_node++)
            {
                //for each s_node, use one of its slave, tranverse down to a node via shortest path,
                //until the node's king is different, then the parent is this king
                BDLSkeletonNode *node = skeleton_nodes[s_node];
                BDLPointGraphNode *slave = node->_slave_nodes[rand() % node->_slave_nodes.size()];

                //printf("slave_nodes.size = %d\n", node->_slave_nodes.size());
                
                //tranverse
                while(slave->_prev)
                {
                    BDLSkeletonNode *king = slave->_prev->_skeleton_king;
                    if(deleted_dict.find(king) == deleted_dict.end() && king != slave->_skeleton_king)
                    {
                        //only add a new skeleton node if it is higher
                        //if(node->_sz > king->_sz*1.05f)
                        //if(node->_sz - king->_sz > 0.15f && (node->_sz-king->_sz)/(node->_sy-king->_sy) > 0.3f)
                        if(node->_sz - king->_sz > 0.15f)
                        {
                            node->_prev = king;
                            king->_children.push_back(node);
                            //printf("%p king->_children(%d)\n", king, int(king->_children.size()));
                        }
                        else
                        {
                            //delete node;
                            deleted_dict[node] = true;
                        }

                        break;
                    }
                    else
                        slave = slave->_prev;
                }
                //for the first batch of skeleton's node that have to be connected to the root
                if(!slave->_prev)
                {
                    node->_prev = ret;
                    ret->_children.push_back(node);
                }
            }

            //debug
            //for(unsigned int s_node=0; s_node<skeleton_nodes.size(); s_node++)
            //{
            //    BDLSkeletonNode *node = skeleton_nodes[s_node];
            //    printf("%p node->_children(%d)\n", node, int(node->_children.size()));
            //}
        }
        //true delete
        std::map <BDLSkeletonNode *, bool>::iterator it;
        for(it=deleted_dict.begin(); it!=deleted_dict.end(); it++)
            delete it->first;
    }

    return ret;
}
