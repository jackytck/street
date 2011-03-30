#include "AdvancedVolume.h"
#include "Transformer.h"

AdvancedVolume::AdvancedVolume(int step, std::vector <osg::Vec3> pts): _step(step), _points(pts)
{
    if(step <= 0)
    {
        printf("AdvancedVolume::AdvancedVolume():step size(%d) error\n", step);
        return;
    }

    //step 1: find cloud metrics, x points right, y points inside, z points up
    bool valid = Transformer::cloud_metrics(pts, _right, _left, _top, _bottom, _in, _out);
    if(!valid)
    {
        printf("AdvancedVolume::AdvancedVolume():cloud_metrics not valid error\n");
        return;
    }

    //step 2: find the left-out-bottom-most corner and other metircs
    _origin = osg::Vec3(_left, _out, _bottom);
    float w = _right - _left;
    float h = _top - _bottom;
    float d = _in - _out;
    if(w == 0.0f || h == 0.0f || d == 0.0f)
        printf("AdvancedVolume::AdvancedVolume():bounding size error\n");
    _voxel = osg::Vec3(w/step, d/step, h/step);

    //step 3: initialize the _voxel_dict_visited to false for all, total number of voxels == (step ** 3)
    // key ranges from [0,(step+1)^3], if step == 20, 20kB will be used
    int upper = step+1;
    upper *= upper * upper;
    _upper = upper;
    for(int i=0; i<=upper; i++)
        _voxel_dict_visited[i] = false;

    //step 4: visit each point and set the dict to true
    //at this point, key() should be valid
    for(unsigned int i=0; i<pts.size(); i++)
        _voxel_dict_visited[key(pts[i])] = true;
}

void AdvancedVolume::clear()
{
    _points.clear();
    _step = -1;
}

osg::Vec3 AdvancedVolume::cg()
{
    osg::Vec3 ret(-23418.715f, -23418.715f, -23418.715f);
    if(_points.empty())
        return ret;
    return Transformer::find_cg(_points);
}

std::vector <osg::Vec3> AdvancedVolume::get_non_empty_voxels()
{
    std::vector <osg::Vec3> ret;

    for(int i=0; i<=_upper; i++)
        if(_voxel_dict_visited[i])
            ret.push_back(unkey(i));

    //debug
    //for(unsigned int i=0; i<ret.size(); i++)
    //{
    //    osg::Vec3 p = ret[i];
    //    printf("v %f %f %f\n", p.x(), p.y(), p.z());
    //}

    return ret;
}

int AdvancedVolume::key(osg::Vec3 p)
{
    int ret = -1;

    int p1 = _step + 1;
    int p2 = p1 * p1;

    //a. check bounds
    if(p.x()>=_left && p.x()<=_right && p.y()>=_out && p.y()<=_in && p.z()>=_bottom && p.z()<=_top)
    {
        osg::Vec3 d = p - _origin;
        //digit ranges from [0,_step]
        d.x() /= _voxel.x();
        d.y() /= _voxel.y();
        d.z() /= _voxel.z();

        ret = p2 * round(d.x()) + p1 * round(d.y()) + round(d.z());

        //debug
        //printf("p(%f %f %f) d(%f %f %f) key(%d)\n", p.x(), p.y(), p.z(), d.x(), d.y(), d.z(), ret);
    }

    return ret;
}

osg::Vec3 AdvancedVolume::unkey(int k)
{
    osg::Vec3 ret(-23418.715f, -23418.715f, -23418.715f);
    if(k < 0 || k > _upper)
        return ret;

    int p1 = _step + 1;
    int p2 = p1 * p1;

    //get back which voxel first
    int dx, dy, dz;
    dx = k / p2;
    dy = (k-dx*p2) / p1;
    dz = k % p1;

    //get back the center point of the voxel relative to the corner
    osg::Vec3 d(_voxel.x() * dx, _voxel.y() * dy, _voxel.z() * dz);

    //get back the world coordinates
    ret = d + _origin;

    //debug
    //printf("d(%f %f %f)\n", d.x(), d.y(), d.z());
    //printf("ret(%f %f %f)\n", ret.x(), ret.y(), ret.z());

    return ret;
}
