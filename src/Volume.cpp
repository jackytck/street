#include "Volume.h"
#include <queue>
#include "PointModel.h"

VolumeIndex::VolumeIndex(int x, int y, int z): _vx(x), _vy(y), _vz(z)
{
}

void VolumeIndex::bound(int maxX, int maxY, int maxZ)
{
    if(_vx < 0)
        _vx = 0;
    if(_vy < 0)
        _vy = 0;
    if(_vz < 0)
        _vz = 0;

    if(_vx >= maxX)
        _vx = maxX-1;
    if(_vy >= maxY)
        _vy = maxY-1;
    if(_vz >= maxZ)
        _vz = maxZ-1;
}

bool VolumeIndex::is_equal(VolumeIndex index)
{
    bool ret = false;
    
    if(index._vx == _vx && index._vy == _vy && index._vz == _vz)
        ret = true;

    return ret;
}

float VolumeIndex::dist(VolumeIndex a, VolumeIndex b)
{
    float ret = -1.0;

    ret = pow(pow(a._vx - b._vx, 2) + pow(a._vy - b._vy, 2) + pow(a._vz - b._vz, 2), 0.5);

    return ret;
}

Volume::Volume(int sizeX, int sizeY, int sizeZ, int stepX, int stepY, int stepZ): _sizeX(sizeX), _sizeY(sizeY), _sizeZ(sizeZ), _stepX(stepX), _stepY(stepY), _stepZ(stepZ), _root(NULL), _status_bar(NULL)
{
    clear();
}

void Volume::clear()
{
    intVector x(_sizeX/_stepX, 0);
    int2dVector y(_sizeY/_stepY, x);
    int3dVector z(_sizeZ/_stepZ, y);
    _box = z;
}

void Volume::initialize_tree(BDLSkeletonNode *root)
{
    if(!root)
        return;

    clear();
    _root = root;
    vote_tree(_root);
}

void Volume::update_tree()
{
    if(_root)
    {
        clear();
        vote_tree(_root);
    }
}

void Volume::add_point(float x, float y, float z)
{
    if(check_bounds(x, y, z))
    {
        _box[int(x)/_stepX][int(y)/_stepY][int(z)/_stepZ]++;
    }
}

void Volume::add_point(BDLSkeletonNode *node)
{
    if(node)
    {
        add_point(_sizeX/2 + node->_sx, _sizeY/2 + node->_sy, node->_sz);
    }
}

void Volume::add_point(VolumeIndex index)
{
    index.bound(_sizeX / _stepX, _sizeY / _stepY, _sizeZ / _stepZ);
    _box[index._vx][index._vy][index._vz]++;
}

int Volume::count(float x, float y, float z)
{
    int ret = -1;

    if(check_bounds(x, y, z))
        ret = _box[int(x)/_stepX][int(y)/_stepY][int(z)/_stepZ];

    return ret;
}

int Volume::count(VolumeIndex index)
{
    int ret = 0;

    index.bound(_sizeX / _stepX, _sizeY / _stepY, _sizeZ / _stepZ);
    ret = _box[index._vx][index._vy][index._vz];

    if(ret < 0)
        printf("Volume: hihi, count(%d) < 0\n", ret);

    return ret;
}

bool Volume::is_empty(float x, float y, float z)
{
    bool ret = true;

    if(count(x, y, z) > 0)
        ret = false;

    return ret;
}

bool Volume::is_empty(VolumeIndex index)
{
    int cnt = count(index);
    return cnt > 0 ? false : true;
}

bool Volume::is_empty(BDLSkeletonNode *node)
{
    return node ? is_empty(_sizeX/2 + node->_sx, _sizeY/2 + node->_sy, node->_sz) : false;
}

void Volume::vote_tree(BDLSkeletonNode *root)
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

        //no translation
        //add_point(node->_sx, node->_sy, node->_sz);

        for(unsigned int i=0; i<node->_children.size(); i++)
        {
            //translation
            add_point(node);

            Queue.push(node->_children[i]);
        }
    }
}

float Volume::density_at(float x, float y, float z, int t)
{
    float ret = -1.0f;

    if(!check_bounds(x, y, z))
        return ret;

    //reset ret to zero
    ret = 0.0f;

    //find the index first
    VolumeIndex center(int(x)/_stepX, int(y)/_stepY, int(z)/_stepZ);

    //then find the 3d ranges
    std::vector <VolumeIndex> neighbors = t_neighbor(center, t);

    //sum up the densities
    for(unsigned int i=0; i<neighbors.size(); i++)
    {
        VolumeIndex index = neighbors[i];
        //float cnt = count(index);
        int cnt = !is_empty(index);

        if(cnt >= 0)
            ret += cnt;
    }

    //divide
    if(!neighbors.empty())
        ret /= neighbors.size();

    //debug
    //printf("Volume::density_at: %f\n", ret);

    return ret;
}

float Volume::density_at(BDLSkeletonNode *node, int t)
{
    return density_at(_sizeX/2 + node->_sx, _sizeY/2 + node->_sy, node->_sz, t);
}

std::vector <VolumeIndex> Volume::t_neighbor(VolumeIndex center, int t)
{
    std::vector <VolumeIndex> ret;

    if(t <= 0)
        return ret;

    t /= 2;

    for(int i=-t; i<t; i++)
        for(int j=-t; j<t; j++)
            for(int k=-t; k<t; k++)
            {
                VolumeIndex n(center._vx+i, center._vy+j, center._vz+k);
                n.bound(_sizeX / _stepX, _sizeY / _stepY, _sizeZ / _stepZ);

                ret.push_back(n);
            }

    return ret;
}

std::vector <BDLSkeletonNode *> Volume::lowest_density_3()
{
    std::vector <BDLSkeletonNode *> ret;
    
    if(!_root)
        return ret;

    ret.push_back(NULL);
    ret.push_back(NULL);
    ret.push_back(NULL);

    float first, second, third;

    //bfs once to get 3 nodes of the lowest density
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        float cur_density = density_at(node);

        //if _img_bound is valid, should only use the points that fall inside the bound
        if(_img_bound.isValid() && !_img_bound.isInside(node))
            goto Break;

        if(cur_density != -1.0f && node->_children.size() != 0 && node->_prev && !is_main_branch(node) && node->_children.size() == 1 && !node->_pruned)
        {
            //compare cur with the strongest, then 2nd and the 3rd
            if(!ret[0])
            {
                ret[0] = node;
                first = cur_density;
                goto Break;
            }
            else
            {
                if(cur_density < first)
                {
                    ret[0] = node;
                    first = cur_density;
                    goto Break;
                }
            }

            if(!ret[1])
            {
                ret[1] = node;
                second = cur_density;
                goto Break;
            }
            else
            {
                if(cur_density < second)
                {
                    ret[1] = node;
                    second = cur_density;
                    goto Break;
                }
            }

            if(!ret[2])
            {
                ret[2] = node;
                third = cur_density;
                goto Break;
            }
            else
            {
                if(cur_density < third)
                {
                    ret[2] = node;
                    third = cur_density;
                    goto Break;
                }
            }
        }

        Break:

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    //debug
    //printf("Volume: first = %f\tsecond = %f\tthird = %f\n", first, second, third);
    if(_status_bar)
        _status_bar->showMessage(QString("Volume: first = %1\tsecond = %2\tthird = %3").arg(first).arg(second).arg(third));

    return ret;
}

BDLSkeletonNode *Volume::intersection_cut(BDLSkeletonNode *element)
{
    BDLSkeletonNode *ret = NULL;

    if(!element)
        return ret;

    ret = element;

    //bfs once and find if the voxel is already occupied
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(element);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        if(node != element && !is_empty(node))
        {
            BDLSkeletonNode::delete_this(node);
            continue;
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    return ret;
}

VolumeIndex Volume::voxel_index(float x, float y, float z)
{
    VolumeIndex ret(-1, -1, -1);

    //need to translate to the bottom center first as in density_at
    x = _sizeX/2 + x;
    y = _sizeY/2 + y;

    if(!check_bounds(x, y, z))
        return ret;

    VolumeIndex center(int(x)/_stepX, int(y)/_stepY, int(z)/_stepZ);

    return center;
}

osg::ref_ptr <osg::Group> Volume::visualize_volume_index(std::vector <VolumeIndex> index, bool maya)
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;

    //point model vertices
    osg::ref_ptr <osg::Vec3Array> pm_vertices = new osg::Vec3Array;

    //for each index, use a blue point to visualize
    for(unsigned int i=0; i<index.size(); i++)
    {
        VolumeIndex vi = index[i];
        vi._vx -= _sizeX/2/_stepX;
        vi._vy -= _sizeY/2/_stepY;

        osg::Vec3 p(vi._vx * _stepX, vi._vy * _stepY, vi._vz * _stepZ);

        if(maya)
        {
            float tmp = p.y();
            p.y() = p.z();
            p.z() = -tmp;
        }

        pm_vertices->push_back(p);
    }

    PointModel pm;
    //pm.setColor(0.0, 0.0, 1.0, 1.0);
    pm.setColor(0.0, 0.0, 0.0, 1.0);
    pm.createScene(pm_vertices);
    pm.setSize(1);

    ret->addChild(pm.get());

    return ret;
}

osg::ref_ptr <osg::Group> Volume::visualize_volume_index_perlin(std::vector <osg::Vec3> index, bool maya)
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;

    //point model vertices
    osg::ref_ptr <osg::Vec3Array> pm_vertices = new osg::Vec3Array;

    //for each index, use a blue point to visualize
    for(unsigned int i=0; i<index.size(); i++)
    {
        osg::Vec3 p = index[i];

        if(maya)
        {
            float tmp = p.y();
            p.y() = p.z();
            p.z() = -tmp;
        }

        pm_vertices->push_back(p);
    }

    PointModel pm;
    pm.setColor(0.0, 0.0, 1.0, 1.0);
    //pm.setColor(0.0, 0.0, 0.0, 1.0);
    pm.createScene(pm_vertices);
    pm.setSize(1);

    ret->addChild(pm.get());

    return ret;
}

osg::ref_ptr <osg::Group> Volume::visualize_volume_index_color(std::vector <osg::Vec3> index, std::vector <int> segments, bool maya)
{
    osg::ref_ptr <osg::Group> ret = new osg::Group;

    //point model vertices
    std::vector <osg::ref_ptr <osg::Vec3Array> > point_ptrs;
    osg::ref_ptr <osg::Vec3Array> pm_vertices = new osg::Vec3Array;

    int color_index = 0;

    for(unsigned int i=0; i<index.size(); i++)
    {
        osg::Vec3 p = index[i];

        if(maya)
        {
            float tmp = p.y();
            p.y() = p.z();
            p.z() = -tmp;
        }

        pm_vertices->push_back(p);

        if(int(i) > segments[color_index])
        {
            color_index++;
            point_ptrs.push_back(pm_vertices);
            pm_vertices = new osg::Vec3Array;
        }
    }

    for(unsigned int i=0; i<point_ptrs.size(); i++)
    {
        float r = (rand()%200 + 55.0f) / 255.0f;
        float g = (rand()%200 + 55.0f) / 255.0f;
        float b = (rand()%200 + 55.0f) / 255.0f;

        PointModel pm;
        pm.setColor(r, g, b, 1.0);
        pm.createScene(point_ptrs[i]);
        pm.setSize(2);

        ret->addChild(pm.get());
    }

    return ret;
}

bool Volume::is_main_branch(BDLSkeletonNode *node)
{
    bool ret = false;

    if(!_root)
        return ret;

    BDLSkeletonNode *go = _root;

    while(go)
    {
        //found supsect
        if(go == node)
        {
            ret = true;
            break;
        }

        if(go->_children.size() == 1)
            go = go->_children[0];
        else
            go = NULL;
    }

    return ret;
}

bool Volume::check_bounds(float x, float y, float z)
{
    bool ret = false;

    if(x >= 0 && x < _sizeX && y >= 0 && y < _sizeY && z >= 0 && z < _sizeZ)
        ret = true;

    if(!ret)
        printf("outbound:p(%f,%f,%f) size(%d,%d,%d)\n", x, y, z, _sizeX, _sizeY, _sizeZ);

    return ret;
}
