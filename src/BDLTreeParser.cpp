#include <queue>
#include <set>
#include <ANN.h>
#include "BDLTreeParser.h"
#include "ThresholdGraph.h"

BDLTreeParser::BDLTreeParser(BDL *bdl): _bdl(NULL), _k(0), _bdlPointGraph(NULL), _skeleton_root(NULL)
{
    if(bdl)
        _bdl = bdl;
}

BDLTreeParser::~BDLTreeParser()
{
    if(_bdlPointGraph)
        delete _bdlPointGraph;
    //also need to delete the _skeleton_root by asking BDLSkeletonNode to do so
}

void BDLTreeParser::build_ann_tree(int k)
{
    if(k <= 0 || !_bdl)
        return;
    _k = k;

    //parameters
    int dim = 3;
    double eps = 0;			                                // error bound


    //data points
    int pointSize = _bdl->getPointSize();
	ANNpointArray annDataPts = annAllocPts(pointSize, dim);

    for(int i=0; i<pointSize; i++)
    {
        BDLPoint p = _bdl->getPoint(i);
        ANNpoint annP = annDataPts[i];
        annP[0] = p.x/p.w;
        annP[1] = p.y/p.w;
        annP[2] = p.z/p.w;
    }

    //allocate ann index, dist and kdTree
	ANNidxArray	nnIdx = new ANNidx[k];						// allocate near neigh indices
	ANNdistArray dists = new ANNdist[k];		            // allocate near neighbor dists
    ANNkd_tree *kdTree = new ANNkd_tree(annDataPts, pointSize, dim);					

    //construct a BDLPointGraph
    if(_bdlPointGraph)
        delete _bdlPointGraph;
    _bdlPointGraph = new BDLPointGraph;

    //wrape the node in BDLPointGraphNode format
    _bdlPointGraph->new_nodes(pointSize);

    for(int i=0; i<pointSize; i++)
    {
        //query point
        ANNpoint annQueryPt = annDataPts[i];

        kdTree->annkSearch(annQueryPt, k, nnIdx, dists, eps);						

        //BDLPointGraphNode *bdlGraphNode = _bdlPointGraph->new_node();
        BDLPointGraphNode *bdlGraphNode = _bdlPointGraph->_nodes[i];

        //fill the relationships of this node with others
        for(int j=0; j<k; j++)
        {
            if(j==0)
            {
                bdlGraphNode->_self = _bdl->getPoint(nnIdx[j]);
            }
            else
            {
                //push bdl point
                //bdlGraphNode->_edges.push_back(_bdl->getPoint(nnIdx[j]));

                //push BDLPointGraphNode
                bdlGraphNode->_edges.push_back(_bdlPointGraph->_nodes[nnIdx[j]]);
                bdlGraphNode->_costs.push_back(sqrt(dists[j]));
            }
        }

        //print results
        /*
        std::cout << "\n\tNN:\tIndex\tDistance\n";
        for (int i = 0; i < k; i++) 
        {			
            dists[i] = sqrt(dists[i]);			// unsquare distance
            std::cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
        }
        */
    }

    //set root of BDLPointGraph
    _bdlPointGraph->find_root();

    //run dijkstra
    _bdlPointGraph->run_dijkstra();

    // clean things up
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    //annDeallocPts(annDataPts);
    //annDeallocPt(annQueryPt);
	annClose();					                           // done with ANN
}

void BDLTreeParser::build_threshold_graph(float threshold)
{
    if(threshold <= 0.0f || !_bdl)
        return;

    //1. convert to array of Vec3 points
    int pointSize = _bdl->getPointSize();
    std::vector <osg::Vec3> dataPts(pointSize, osg::Vec3());

    for(int i=0; i<pointSize; i++)
    {
        BDLPoint p = _bdl->getPoint(i);
        osg::Vec3 pt;
        pt.x() = p.x/p.w;
        pt.y() = p.y/p.w;
        pt.z() = p.z/p.w;
        dataPts[i] = pt;
    }

    //2. do clustering
    ThresholdGraph tg(threshold, dataPts);
    std::vector <std::vector <int> > links = tg.construct_graph();

    //3. construct a BDLPointGraph
    if(_bdlPointGraph)
        delete _bdlPointGraph;
    _bdlPointGraph = new BDLPointGraph;

    //wrape the node in BDLPointGraphNode format
    _bdlPointGraph->new_nodes(pointSize);

    //4. build graph
    for(int i=0; i<pointSize; i++)
    {
        //BDLPointGraphNode *bdlGraphNode = _bdlPointGraph->new_node();
        BDLPointGraphNode *bdlGraphNode = _bdlPointGraph->_nodes[i];
        bdlGraphNode->_self = _bdl->getPoint(i);

        //fill the relationships of this node with others
        int k = links[i].size();
        for(int j=0; j<k; j++)
        {
            //push BDLPointGraphNode
            bdlGraphNode->_edges.push_back(_bdlPointGraph->_nodes[links[i][j]]);

            osg::Vec3 self = dataPts[i];
            osg::Vec3 child = dataPts[links[i][j]];
            bdlGraphNode->_costs.push_back((self-child).length());
        }
    }

    //5. post-processing
    //set root of BDLPointGraph
    _bdlPointGraph->find_root();

    //run dijkstra
    _bdlPointGraph->run_dijkstra();
}

BDLSkeletonNode *BDLTreeParser::build_skeleton_tree()
{
    BDLSkeletonNode *ret = NULL;

    if(_bdlPointGraph)
    {
        ret = _bdlPointGraph->build_skeleton();
        _skeleton_root = ret;
    }

    return ret;
}

double BDLTreeParser::compute_supporting_length(BDLSkeletonNode *node)
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

void BDLTreeParser::rectify_skeleton_tree()
{
    if(!_skeleton_root)
        return;

    //stores all the leaf skeleton nodes
    std::set <BDLSkeletonNode *> leafs;

    //BFS all skeleton nodes for finding the leafs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_skeleton_root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();
        
        if(front->_children.size() == 0)
            leafs.insert(front);

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);
    }

    std::queue <std::set <BDLSkeletonNode *> > branch_queue;
    branch_queue.push(leafs);

    //topological sort for finding the inherited radius and setting an initial estimate of each radius
    while(!branch_queue.empty())
    {
        std::set <BDLSkeletonNode *> front = branch_queue.front();
        branch_queue.pop();

        std::set <BDLSkeletonNode *> next_set;
        while(!front.empty())
        {
            std::set <BDLSkeletonNode *>::iterator it = front.begin();

            //for each node, transverse down until the branching point
            BDLSkeletonNode *node = *it;

            double inherited_avg = -1.0;
            //if this node is not a leaf, it will have a valid _inherited_radius
            if(node->_children.size() > 1)
                inherited_avg = node->_inherited_radius;

            //sum of all radius along the path, excluding the branching point
            double sum_radii = 0.0;
            //number of skeleton nodes involved in sum_radii
            int num_nodes = 0;
            //store all the nodes that I have seen
            std::vector <BDLSkeletonNode *> nodes_seen;
            
            //should not break for the first node
            bool first = true;

            while(node->_prev)
            {
                if(!first && node->_children.size() > 1)
                    break;

                sum_radii += node->_radius;
                num_nodes++;
                nodes_seen.push_back(node);
                
                if(first)
                    first = false;

                node = node->_prev;
            }

            //find the average of all the nodes I have seen
            double avg = sum_radii / num_nodes;
            //weighted average with _inherited_radius
            if(inherited_avg != -1.0)
                avg = 0.1 * inherited_avg + 0.9 * avg;
            
            //set the avg to every node that I have seen
            for(unsigned int n=0; n<nodes_seen.size(); n++)
            {
                BDLSkeletonNode *node_seen = nodes_seen[n];
                node_seen->_radius = avg;
                node_seen->_radius_rectified = true;
            }

            //now node points to a branching node or it points to root(not NULL)
            if(node != _skeleton_root)
            {
                //check if all of its childen is rectified
                bool all_rectified = true;
                for(unsigned int child=0; child<node->_children.size(); child++)
                    if(!node->_children[child]->_radius_rectified)
                    {
                        all_rectified = false;
                        break;
                    }

                node->_inherited_radius += avg;

                if(all_rectified)
                    next_set.insert(node);
            }
            //if node points to root
            else
            {
                node->_radius = avg;
                node->_radius_rectified = true;
            }

            front.erase(it);
            //printf("branch_queue.size()=%d front.size()=%d next_set.size()=%d\n", branch_queue.size(), front.size(), next_set.size());
        }

        if(!next_set.empty())
            branch_queue.push(next_set);
    }

    //compute the supporting length
    compute_supporting_length(_skeleton_root);

    //BFS from root to all other nodes, re-setting the radius by this biological model
    //assume the radius of root is correct now
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(_skeleton_root);

    while(!Queue2.empty())
    {
        BDLSkeletonNode *node = Queue2.front();
        Queue2.pop();

        double r_p = node->_radius;

        //this node is done, now set its children
        //if only one child
        if(node->_children.size() == 1)
        {
            double l_p = node->_supporting_length;
            double l_c = node->_children[0]->_supporting_length;

            //node->_children[0]->_radius = r_p * pow(l_c / l_p, 1.5);
            double expected = r_p * pow(l_c / l_p, 1.5);
            //node->_children[0]->_radius = 0.9 * expected + 0.1 * node->_children[0]->_radius;
            node->_children[0]->_radius = 0.80 * expected + 0.1 * node->_children[0]->_radius;
        }
        else
        {
            double sum_l_c = 0.0;
            for(unsigned int i=0; i<node->_children.size(); i++)
                sum_l_c += node->_children[i]->_supporting_length;

            for(unsigned int i=0; i<node->_children.size(); i++)
            {
                double l_c = node->_children[i]->_supporting_length;

                //node->_children[i]->_radius = r_p * pow(l_c / sum_l_c, 1.0/2.49);
                double expected = r_p * pow(l_c / sum_l_c, 1.0/2.49);
                //node->_children[i]->_radius = 0.9 * expected + 0.1 * node->_children[i]->_radius;
                node->_children[i]->_radius = 0.85 * expected + 0.1 * node->_children[i]->_radius;
            }
        }

        //push all of the children
        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue2.push(node->_children[i]);
    }
}
