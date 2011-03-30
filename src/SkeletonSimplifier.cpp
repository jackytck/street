#include "SkeletonSimplifier.h"
#include "BDLSkeletonLoader.h"
#include "RealisticLeafGrower.h"
#include "Transformer.h"
#include "AdvancedVolume.h"
#include <queue>

SkeletonSimplifier::SkeletonSimplifier(): _root(NULL)
{
}

SkeletonSimplifier::~SkeletonSimplifier()
{
    BDLSkeletonNode::delete_this(_root);
}

void SkeletonSimplifier::setup(std::string skeleton, std::string leaves)
{
    //load skeleton
    BDLSkeletonLoader loader;
    loader.load_file(QString(skeleton.c_str()));

    if(_root)
        BDLSkeletonNode::delete_this(_root);
    _root = loader.construct_bdl_skeleton_tree();

    //load leaves
    RealisticLeafGrower leaf_grower;
    std::string in_real_tex = leaf_grower.load(leaves);
    _leaves = leaf_grower._all_v;
}

void SkeletonSimplifier::simplify(int volume_step, int depth)
{
    //0. pre-checking all parameters
    if(depth < 0 || !_root || _leaves.empty())
        return;

    //1. setup foliage volume
    AdvancedVolume volume(volume_step, _leaves);

    //2. find the cg and largest distance from cg to the 8 corners of the volume
    //cg of bounding box, not cg of _leaves
    osg::Vec3 cg((volume._left+volume._right)/2.0f, (volume._in+volume._out)/2.0f, (volume._top+volume._bottom)/2.0f);
    float radius = (cg - volume._origin).length();//bounding radius

    //3. setup points of icosahedron
    std::vector <osg::Vec3> norm_vertices = Transformer::icosahedron(depth);//unit normals
    std::vector <osg::Vec3> vertices;//scaled and translated
    for(unsigned int i=0; i<norm_vertices.size(); i++)
    {
        osg::Vec3 p = (cg + norm_vertices[i]*radius);
        vertices.push_back(p);
    }

    //4. ray-tracing
    std::vector <BDLSkeletonNode *> nodes = Transformer::bfs(_root);
    float ray_step = volume._voxel.length() / 2.0f;
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        osg::Vec3 node = Transformer::toVec3(nodes[i]);
        int node_key = volume.key(node);

        //node is out of the bound of leaves(very rare), so must be visible
        if(node_key == -1)
            continue;

        for(unsigned int j=0; j<vertices.size(); j++)
        {
            //4a. ray-traced from each vertex with step size be half of the voxel size
            osg::Vec3 from = vertices[j];
            osg::Vec3 ray = node - from;
            int step = ray.length() / ray_step;
            ray.normalize();
            ray *= ray_step;

            //4b. transverse until it hits the first non-empty voxel
            for(int s=0; s<step; s++)
            {
                osg::Vec3 probe = from + ray*s;

                //debug
                //if(i==nodes.size()/2)
                //    printf("v %f %f %f\n", probe.x(), probe.y(), probe.z());

                //4c. test if this voxel is the same voxel which contains the node, 
                //if no, node is invisible, otherwise visible
                int probe_key = volume.key(probe);
                if(probe_key != -1 && volume._voxel_dict_visited[probe_key])
                {
                    //visible: mark 'mesh_done' as true
                    if(probe_key == node_key)
                        nodes[i]->_mesh_done = true;//_mesh_done here means is_visible?
                    //invisible: do nothing

                    break;
                }
            }
        }

        //debug, after ray-tracing from all the vertices, if a node still has mesh_done == false, then it is invisible
        //if(!nodes[i]->_mesh_done)
        //    printf("v %f %f %f\n", node.x(), node.y(), node.z());
    }

    //5. mark the path from visible (mesh_done) nodes to the root as protected
    for(unsigned int i=0; i<nodes.size(); i++)
    {
        BDLSkeletonNode *node = nodes[i];
        if(node->_mesh_done)
        {
            while(node)
            {
                node->_mesh_done = true;
                node = node->_prev;
            }
        }
    }

    //6. delete all invisible and unprotected nodes, i.e. nodes whose mesh_done is false
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_root);
    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        if(front->_mesh_done)
        {
            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
        }
        else
            BDLSkeletonNode::delete_this(front);
    }

    //debug
    //leaves
    if(false)
    {
        for(unsigned int i=0; i<_leaves.size(); i++)
        {
            osg::Vec3 p = _leaves[i];
            printf("v %f %f %f\n", p.x(), p.y(), p.z());
        }
        printf("SkeletonSimplifier::simplify():hihi\n");
        //ray
        for(unsigned int i=0; i<vertices.size(); i++)
        {
            osg::Vec3 p = vertices[i];
            printf("v %f %f %f\n", p.x(), p.y(), p.z());
        }
    }
}
