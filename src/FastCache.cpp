#include "FastCache.h"
#include <set>
#include <cstdlib>
#include <stdio.h>

FastCache::FastCache(std::vector <osg::Vec3> relative_bounds, int step)
{
    if(!relative_bounds.empty() && step > 0)
    {
        _step = step;
        _rbounds = relative_bounds;
        _more_margin = 1.2f;
        inferCacheBound();
    }
}

void FastCache::inferCacheBound()
{
    //a. find the 2d rect bound first
    float minX, minY, maxX, maxY;

    for(unsigned int i=0; i<_rbounds.size(); i++)
    {
        osg::Vec3 p = _rbounds[i];

        if(i==0)
        {
            minX = maxX = p.x();
            minY = maxY = p.y();
        }

        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
    }

    //b. find 3d metrics
    float width = maxX - minX;
    float height = maxY - minY;
    float depth = width * 0.7f;

    //more offsets
    width *= _more_margin;
    height *= _more_margin;

    //c. set origin
    _origin = osg::Vec3(minX, -depth, minY);

    /*
    printf("bounds:\n");
    printf("v %f %f %f\n", _origin.x(), _origin.y(), _origin.z());
    printf("v %f %f %f\n", width+minX, -depth, minY);
    printf("v %f %f %f\n", minX, depth, minY);
    printf("v %f %f %f\n", width+minX, depth, minY);
    printf("v %f %f %f\n", minX, -depth, height+minY);
    printf("v %f %f %f\n", width+minX, -depth, height+minY);
    printf("v %f %f %f\n", minX, depth, height+minY);
    printf("v %f %f %f\n", width+minX, depth, height+minY);
    printf("end bounds:\n");
    */

    //d. set bounds
    _w = width;
    _h = height;
    _d = depth;

    //e. set sub-division voxels
    if(check_instance())
    {
        _voxel.x() = _w / _step;
        _voxel.y() = _h / _step;
        _voxel.z() = _d / _step;

        //for performance
        _step_p1 = _step + 100;
        _step_p2 = _step_p1 * _step_p1;
    }

    //f. set cg of relative bounds
    _cg = center_mass(_rbounds);

    //g. find the bound-metircs
    bound_metrics(_rbounds, _cg, _b_h, _w_r, _w_l);
}

bool FastCache::check_instance()
{
    bool ret = false;
    
    if(_w > 0 && _h > 0 && _d > 0 && _step > 0)
        ret = true;
    else
        printf("FastCache::check_instance():_w(%f) _h(%f) _d(%f) _step(%d)\n", _w, _h, _d, _step);

    return ret;
}

int FastCache::key(osg::Vec3 p)
{
    int ret = -1;

    if(check_instance())
    {
        int p1 = _step_p1;
        int p2 = _step_p2;

        osg::Vec3 d = p - _origin;

        //each digit ranges from [0,_pruner_cube_step]
        d.x() /= _voxel.x();
        d.y() /= _voxel.y();
        d.z() /= _voxel.z();

        ret = p2 * round(d.x()) + p1 * round(d.y()) + round(d.z());

        //debug
        //printf("p(%f %f %f) d(%f %f %f) key(%d)\n", p.x(), p.y(), p.z(), d.x(), d.y(), d.z(), ret);
    }

    return ret;
}

osg::Vec3 FastCache::unkey(int k)
{
    osg::Vec3 ret;

    if(k >= 0)
    {
        int p1 = _step_p1;
        int p2 = _step_p2;

        int x, y, z;
        z = k % p1;
        y = ((k-z) % p2) / p1;
        x = (k - z - p1 * y) / p2;

        //actual coords
        ret.x() = x * _voxel.x();
        ret.y() = y * _voxel.y();
        ret.z() = z * _voxel.z();

        //return the center
        ret += _voxel * 0.5f;

        //with respect to origin
        ret += _origin;
    }

    return ret;
}

std::vector <osg::Vec3> FastCache::surface_points()
{
    std::vector <osg::Vec3> ret;
    std::set <int> keys;

    //a. for each point in relative bounds, construct a sweeping ellipsoid
    for(unsigned int i=0; i<_rbounds.size(); i++)
    {
        osg::Vec3 rb = _rbounds[i];

        //printf("v %f %f %f\n", rb.x(), 0.0f, rb.y());

        //radius
        float r = rb.x() - _cg.x();

        //fit an ellipse
        osg::Vec3 R(rb.x()-_cg.x(), 0.0f, rb.y()-_cg.z());

        if(r > 0)
            r = R.length() * _w_r/_b_h;
        else
            r = R.length() * _w_r/_b_h;

        //empirical constant
        float rk = 2*_w_r > _b_h ? 1.2f : 1.5f;

        osg::Vec3 R_perp(0.0f, -r*rk, 0.0f);

        //debug one slice
        unsigned int size = _rbounds.size();
        if(true || i==size/6 || i==4*size/6)
        {
            //rotate by half-circle
            for(int theta=-90; theta<90; theta+=8)
            {
                //osg::Vec3 rb_t(r*cos(theta*M_PI/180.0), r*sin(theta*M_PI/180.0), rb.y());
                osg::Vec3 rb_t = _cg + R_perp * cos((theta+90)*M_PI/180.0) + R * sin((theta+90)*M_PI/180.0);

                //printf("v %f %f %f\n", rb_t.x(), rb_t.y(), rb_t.z());

                keys.insert(key(rb_t));
            }
        }
    }

    //b. un-key all the unique elements in the set
    for(std::set<int>::iterator it=keys.begin(); it!=keys.end(); it++)
        ret.push_back(unkey(*it));

    //debug
    //printf("FastCache::surface_points()\n");
    //for(unsigned int i=0; i<ret.size(); i++)
    //    printf("v %f %f %f\n", ret[i].x(), ret[i].y(), ret[i].z());
    //printf("End of FastCache::surface_points()\n");

    return ret;
}

osg::Vec3 FastCache::center_mass(std::vector <osg::Vec3> bounds)
{
    osg::Vec3 ret(-1.0f, -1.0f, -1.0f);

    osg::Vec3 sum(0.0f, 0.0f, 0.0f); //assume no overflow
    int cnt = 0;

    for(unsigned int i=0; i<bounds.size(); i++)
    {
        sum += bounds[i];
        cnt++;
    }

    if(cnt > 0)
    {
        sum = sum * (1.0f/cnt);
        ret = osg::Vec3(sum.x(), 0, sum.y());
    }

    return ret;
}

void FastCache::bound_metrics(std::vector <osg::Vec3> rbound, osg::Vec3 cg, float& height, float& width_right, float& width_left)
{
    width_right = -1.0f;
    width_left = -1.0f;
    height = -1.0f;

    for(unsigned int i=0; i<rbound.size(); i++)
    {
        osg::Vec3 cur_p = rbound[i];
        float cur_d = abs(cur_p.x() - cg.x());

        //max width right
        if(cur_p.x() >= cg.x())
        {
            if(width_right == -1.0f || cur_d > width_right)
                width_right = cur_d;
        }
        //max width left
        else
        {
            if(width_left == -1.0f || cur_d > width_left)
                width_left = cur_d;
        }

        //max height
        for(unsigned int j=0; j<rbound.size(); j++)
            if(i != j)
            {
                float cur_h = abs(rbound[i].y() - rbound[j].y());
                if(height == -1.0f || cur_h > height)
                    height = cur_h;
            }
    }
}
