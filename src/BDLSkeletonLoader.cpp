#include <fstream>
#include "BDLSkeletonLoader.h"

BDLSkeletonLoader::BDLSkeletonLoader(): _number_of_node(-1), _number_of_link(-1)
{
}

BDLSkeletonLoader::~BDLSkeletonLoader()
{
}

void BDLSkeletonLoader::load_file(QString filename)
{
    std::ifstream fs(filename.toStdString().c_str());
    std::string s;

    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &_number_of_node);

    for(int i=0; i<_number_of_node; i++)
    {
        float px, py, pz;

        getline(fs, s);
        sscanf(s.c_str(), "%f %f %f\n", &px, &py, &pz);

        _nodes.push_back(osg::Vec3(px, py, pz));
    }

    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &_number_of_link);

    for(int i=0; i<_number_of_link; i++)
    {
        int parent, child;
        
        getline(fs, s);
        sscanf(s.c_str(), "%d %d\n", &parent, &child);

        _parents.push_back(parent);
        _childs.push_back(child);
    }
}

BDLSkeletonNode *BDLSkeletonLoader::construct_bdl_skeleton_tree()
{
    BDLSkeletonNode *ret = NULL;

    std::vector <BDLSkeletonNode *> nodes;

    for(int i=0; i<_number_of_node; i++)
    {
        BDLSkeletonNode *node = new BDLSkeletonNode;
        node->_sx = _nodes[i].x();
        node->_sy = _nodes[i].y();
        node->_sz = _nodes[i].z();

        nodes.push_back(node);
    }

    for(int i=0; i<_number_of_link; i++)
    {
        int parent = _parents[i];
        int child = _childs[i];

        nodes[parent]->_children.push_back(nodes[child]);
        nodes[child]->_prev = nodes[parent];
    }

    if(!nodes.empty())
    {
        nodes[0]->_radius = 5.0;
        ret = nodes[0];
    }

    return ret;
}

void BDLSkeletonLoader::print_self()
{
    //print back the file in the same format

    printf("%d\n", _number_of_node);
    for(int i=0; i<_number_of_node; i++)
        printf("%f %f %f\n", _nodes[i].x(), _nodes[i].y(), _nodes[i].z());

    printf("%d\n", _number_of_link);
    for(int i=0; i<_number_of_link; i++)
        printf("%d %d\n", _parents[i], _childs[i]);
}
