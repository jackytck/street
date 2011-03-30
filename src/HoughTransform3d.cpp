#include "HoughTransform3d.h"
#include "PointModel.h"

HT_bin::HT_bin()
{
}

void HT_bin::vote(int index, double count)
{
    _back_index.push_back(index);
    _back_weight.push_back(count);
    _count += count;
}

void HT_bin::delete_vote_by_index(int index)
{
    std::vector <int> new_index;
    std::vector <double> new_weight;

    for(unsigned int i=0; i<_back_index.size(); i++)
    {
        if(_back_index[i] != index)
        {
            new_index.push_back(_back_index[i]);
            new_weight.push_back(_back_weight[i]);
        }
        else
            _count -= _back_weight[i];
    }

    _back_index = new_index;
    _back_weight = new_weight;
}

HoughTransform3d::HoughTransform3d(): _step_theta(5), _step_phi(5), _step_rho(10)
{
    clear();
}

HoughTransform3d::~HoughTransform3d()
{
}

void HoughTransform3d::set_steps(int step_theta, int step_phi, int step_rho)
{
    _step_theta = step_theta;
    _step_phi = step_phi;
    _step_rho = step_rho;
}

void HoughTransform3d::clear()
{
    /*
    intVector r(255/_stepR+1, 0);
    int2dVector g(255/_stepG+1, r);
    int3dVector b(255/_stepB+1, g);
    _box = b;
    */
    binVector rho(600/_step_rho+1, HT_bin());
    bin2dVector phi(359/_step_phi+1, rho);
    bin3dVector theta(180/_step_theta+1, phi);
    _box = theta;
}

void HoughTransform3d::set_scale_and_vertex(osg::ref_ptr <osg::Vec3Array> v, int offset)
{
    _offset = offset;
    _leaf_v = v;

    /* original point P
     * new point P' = (P-P_min)*k such that P' is <= 100 and >= 0
     * controvesely P = P'*(1.0/k)+P_min
     */
    //find vectors P_min and P_max
    osg::Vec3 P_min, P_max;
    for(unsigned int i=0; i<v->size(); i++)
    {
        osg::Vec3 cur = (*v)[i];
        if(i==0)
            P_min = P_max = cur;
        else
        {
            if(cur.x() < P_min.x())
                P_min.x() = cur.x();
            if(cur.y() < P_min.y())
                P_min.y() = cur.y();
            if(cur.z() < P_min.z())
                P_min.z() = cur.z();

            if(cur.x() > P_max.x())
                P_max.x() = cur.x();
            if(cur.y() > P_max.y())
                P_max.y() = cur.y();
            if(cur.z() > P_max.z())
                P_max.z() = cur.z();
        }
    }

    //find scales k, i.e. 100.0 / (P_max - P_min)
    osg::Vec3 k = P_max - P_min;
    if(k.x() > 0)
        k.x() = 100.0 / k.x();
    if(k.y() > 0)
        k.y() = 100.0 / k.y();
    if(k.z() > 0)
        k.z() = 100.0 / k.z();

    //store these results for inverse, postitions of original points will be lost
    _P_min = P_min;
    _k = k;

    //scale all the points
    for(unsigned int i=0; i<_leaf_v->size(); i++)
    {
        (*_leaf_v)[i].x() = ((*_leaf_v)[i].x() - _P_min.x()) * _k.x();
        (*_leaf_v)[i].y() = ((*_leaf_v)[i].y() - _P_min.y()) * _k.y();
        (*_leaf_v)[i].z() = ((*_leaf_v)[i].z() - _P_min.z()) * _k.z();
    }
}

osg::ref_ptr <osg::Node> HoughTransform3d::debug_vote()
{
    osg::ref_ptr <osg::Vec3Array> plot = new osg::Vec3Array;

    for(int i=-90; i<=90; i++)
    {
        for(int j=0; j<360; j++)
        {
            double theta = i/180.0*M_PI;
            double phi = j/180.0*M_PI;

            double x = 2.0, y = 4.0, z = 6.0;
            double rho = x*(sin(theta)*cos(phi)) + y*(sin(theta)*sin(phi)) + z*cos(theta);

            plot->push_back(osg::Vec3(i, j, 5.0*rho));
            plot->push_back(osg::Vec3(i, j, 0.0));
        }
    }

    PointModel pm;
    return pm.createScene(plot);
}
