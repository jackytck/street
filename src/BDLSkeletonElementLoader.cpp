#include "BDLSkeletonElementLoader.h"
#include <fstream>

BDLSkeletonElementLoader::BDLSkeletonElementLoader()
{
}

void BDLSkeletonElementLoader::load_file(QString filename)
{
    clear();

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

    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &_num_sec_sup);

    for(int i=0; i<_num_sec_sup; i++)
    {
        int parent, child;

        getline(fs, s);
        sscanf(s.c_str(), "%d %d\n", &parent, &child);

        _sec_sup_branches.push_back(std::make_pair(parent, child));
    }

    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &_num_pri_sup);

    for(int i=0; i<_num_pri_sup; i++)
    {
        int parent, child;

        getline(fs, s);
        sscanf(s.c_str(), "%d %d\n", &parent, &child);

        _pri_sup_branches.push_back(std::make_pair(parent, child));
        _sup_par = parent;
        _sup_child = child;
    }

    fs.close();
}

BDLSkeletonNode *BDLSkeletonElementLoader::construct_bdl_skeleton_tree()
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

        if(i == _sup_par)
            _support_branch._par = node;
        else if(i == _sup_child)
            _support_branch._child = node;
    }

    for(int i=0; i<_number_of_link; i++)
    {
        int parent = _parents[i];
        int child = _childs[i];

        nodes[parent]->_children.push_back(nodes[child]);
        nodes[child]->_prev = nodes[parent];
    }

    for(int i=0; i<_num_sec_sup; i++)
    {
        int parent = _sec_sup_branches[i].first;
        int child = _sec_sup_branches[i].second;

        nodes[child]->_prev_support = nodes[parent];
    }

    if(!nodes.empty())
    {
        nodes[0]->_radius = 5.0;
        ret = nodes[0];
    }

    return ret;
}

SLink BDLSkeletonElementLoader::support_branch()
{
    return _support_branch;
}

void BDLSkeletonElementLoader::clear()
{
    _nodes.clear();
    _parents.clear();
    _childs.clear();
    _sec_sup_branches.clear();
    _pri_sup_branches.clear();
}

void BDLSkeletonElementLoader::print_self()
{
    BDLSkeletonLoader::print_self();

    printf("%d\n", _num_sec_sup);
    for(int i=0; i<_num_sec_sup; i++)
        printf("%d %d\n", _sec_sup_branches[i].first, _sec_sup_branches[i].second);

    printf("%d\n", _num_pri_sup);
    for(int i=0; i<_num_pri_sup; i++)
        printf("%d %d\n", _pri_sup_branches[i].first, _pri_sup_branches[i].second);
}
