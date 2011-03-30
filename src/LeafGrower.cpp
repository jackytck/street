#include "LeafGrower.h"
#include "ISPLoader.h"
#include "Transformer.h"
#include <queue>

LeafGrower::LeafGrower(): _root(NULL), _rootX(-1), _rootY(-1), _scale(-1.0f)
{
}

void LeafGrower::setup(BDLSkeletonNode *root, std::string isp0, float s, std::string path)
{
    _root = root;

    ISPLoader loader;
    loader.load(isp0);
    loader.construct_min_texture(path, _cg, _bound_bottomLeft, _bound_width, _bound_height);
    _rootX = loader._rootX;
    _rootY = loader._rootY;

    _scale = s;

    //convert the absolute _cg in image_coords to skeleton space under this scale
    _cg = (_cg - osg::Vec3(_rootX, _rootY, 0.0f)) * _scale;
    _cg.y() *= -1.0f;
    _cg = osg::Vec3(_cg.x(), 0.0f, _cg.y());

    //clear previous cache
    _all_v.clear();
    _all_tex.clear();

    //setup pruner for grow_planes()
    _pruner.setup(isp0, s);
}

void LeafGrower::set_scale(float s)
{
    _scale = s;
}

bool LeafGrower::is_loaded()
{
    bool ret = false;

    if(_root && _rootX != -1 && _rootY != -1 && _scale != -1.0f)
        ret = true;

    return ret;
}

std::vector <osg::Vec2> LeafGrower::grow(const std::vector <osg::Vec3>& all_v)
{
    std::vector <osg::Vec2> ret;

    _all_v = all_v;
    ret = texture_coords_from_vertex(all_v);
    _all_tex = ret;

    return ret;
}

//std::vector <osg::Vec3> LeafGrower::grow_planes(osg::Vec3 origin, double height, int angle, bool maya)
std::vector <osg::Vec3> LeafGrower::grow_planes()
{
    std::vector <osg::Vec3> ret;
    if(!_root)
        return ret;

    //copy to avoid modification
    //BDLSkeletonNode *root = BDLSkeletonNode::copy_tree(_root);
    //BDLSkeletonNode::rectify_skeleton_tree(root, origin, height, angle, maya);
	BDLSkeletonNode *root = _root;

    //set the expected leaf size be 0.35 of the trunk's radius
    float leaf_size_hint = root->_radius * 0.35;

    //store for billboard data
    _all_pos.clear();

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    int ball_cnt = 0;
    float outgrow = 1.3f;

    //grow leaves only at tips or small radius nodes
    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);

        //skip the root
        if(!front->_prev)
            continue;

        //graph's leafs only
        if(front->_children.size() == 0)
        {
            osg::Vec3 pos(front->_sx, front->_sy, front->_sz);
            osg::Vec3 pre(front->_prev->_sx, front->_prev->_sy, front->_prev->_sz); 
            osg::Vec3 mid = (pos+pre) / 2.0f;
            osg::Vec3 tangent = pos - pre;

            float cloud_width = tangent.length() * 0.50 * outgrow;
            tangent.normalize();

            //consider a plane with tangent as the normal and contains point pos
            //find basis u, v of this plane
            osg::Vec3 u(tangent.y(), -tangent.x(), 0.0);
            osg::Vec3 v = tangent ^ u;

            //consider another plane with u as normal and contains point pos
            //the basis of this plane is chosen as tangent and v
            //div_angle is the angle angle spanned by tangent and the root of the leaf
            double div_angle = (rand()%30-15+70)/180.0*M_PI;

            //number of ball of this node
            int no_leaf = 1;
            for(int i=1; i<=no_leaf; i++)
            {
                double angle = 360.0/no_leaf*i/180.0*M_PI;
                osg::Vec3 div = u*cos(angle) + v*sin(angle);
                div = tangent * cos(div_angle) + div * sin(div_angle);

                //frame: fu, fv, div, at pos
                osg::Vec3 fu(div.y(), -div.x(), 0.0f);
                fu.normalize();
                osg::Vec3 fv = div ^ fu;

                //cloud around this node
                std::vector <osg::Vec3> b_cloud;
                std::vector <float> weights;
                Transformer::sphere_points_transformed(b_cloud, weights, mid, cloud_width, tangent, 10-rand()%20, leaf_size_hint);
                //Transformer::sphere_points_transformed(b_cloud, weights, mid, cloud_width, tangent, -89, leaf_size_hint);

                //from points to leaf vertices
                for(unsigned j=0; j<b_cloud.size(); j++)
                {
                    osg::Vec3 b_p = b_cloud[j];

                    osg::Vec3 normal = b_p - mid;
                    normal.normalize();

                    osg::Vec3 normal2 = b_p - pos;
                    normal2.normalize();

                    //displace
                    float d_ratio = weights[j] > 0.5 ? 0.40 : 1.0f/outgrow;
                    b_p += normal2 * cloud_width * d_ratio * (weights[j]-0.5f);
                    float leaf_s = leaf_size_hint * (1.0-0.8*std::max(0.0f, weights[j]-0.5f));

                    //cloud pt inside segmentation?
                    if(false || _pruner.is_inside_ortho(b_p))
                    {
                        Transformer::points_to_leafs(ret, b_p, normal, leaf_s, true);

                        //for billboard data
                        _all_pos.push_back(b_p);
                    }
                }
            }

            ball_cnt++;
        }
        if(false)
            if(ball_cnt > 5)
                break;
    }
    if(false)
        printf("ball_cnt(%d)\n", ball_cnt);

    //delete copy
    //BDLSkeletonNode::delete_this(root);

    return ret;
}

void LeafGrower::billboard_data(std::vector <osg::Vec3>& all_pos, std::vector <osg::Vec3>& all_v, std::vector <osg::Vec2>& all_tex)
{
    //check if all the sizes match
    if(_all_pos.empty() || _all_v.size() != _all_pos.size() * 4 || _all_v.size() != _all_tex.size())
    {
        printf("LeafGrower::billboard_data() _all_pos(%d) _all_v(%d) _all_tex(%d) not match\n", int(_all_pos.size()), int(_all_v.size()), int(_all_tex.size()));
        return;
    }

    //clear inputs
    all_pos.clear();
    all_v.clear();
    all_tex.clear();

    //Method 1: Do nothing and dump back
    if(false)
    {
        all_pos = _all_pos;
        all_v = _all_v;
        all_tex = _all_tex;
    }

    //Method 2: Skip all the core points in _all_pos and return the rest
    if(true)
    {
        //the cg of all_pos
        osg::Vec3 cg = Transformer::find_cg(_all_pos);

        //the average square distance from cg
        float avg_dist = 0.0f;
        for(unsigned int i=0; i<_all_pos.size(); i++)
        {
            float cur_dist = (_all_pos[i] - cg).length2();
            avg_dist += cur_dist;
        }
        avg_dist /= _all_pos.size();

        //cutoff threshold
        float cutoff = 0.5f;
        float cutoff_dist = avg_dist * cutoff;

        //only accept all the data if the associated point is > cutoff_dist away from cg
        for(unsigned int i=0; i<_all_pos.size(); i++) //will not overflow, i guess
        {
            float cur_dist = (_all_pos[i] - cg).length2();
            if(cur_dist > cutoff_dist)
            {
                //all_pos
                all_pos.push_back(_all_pos[i]);

                //all_v + all_tex
                for(int j=0; j<4; j++)
                {
                    all_v.push_back(_all_v[i*4+j]);
                    all_tex.push_back(_all_tex[i*4+j]);
                }
            }
        }
    }
}

osg::Vec2 LeafGrower::obj_to_img_space(osg::Vec3 v)
{
    osg::Vec2 ret;

    if(_scale == 0.0f || _scale < 0.0f)
    {
        printf("LeafGrower::obj_to_img_space:scale(%f) error.\n", _scale);
        return ret;
    }

    //project by just discarding the y-coords
    ret.x() = v.x();
    ret.y() = v.z();
 
    //find the image coordinates by inverting the processes
    ret.y() *= -1.0f;
    ret = ret / _scale;

    ret.x() += _rootX;
    ret.y() += _rootY;

    return ret;
}

int LeafGrower::which_quadrant(osg::Vec3 normal)
{
    int ret = -1;

    double lat, lon;

    //change to spherical coordinates first
    Transformer::cartesian_to_spherical(normal, lat, lon);

    //top
    if(lat > 80)
    //if(lat > 40)
        return 3;

    //bottom
    if(lat < -55)
    //if(lat < -35)
        return 4;

    //front
    if(lon <= -30 && lon >= -150)
        return 0;

    //right
    if(lon > -30 && lon < 90)
        return 1;

    //left
    if((lon >= 90 && lon <= 180) || (lon < -150 && lon > -180))
        return 2;

    return ret;
}

std::vector <osg::Vec2> LeafGrower::texture_coords_from_vertex(const std::vector <osg::Vec3>& all_v)
{
    std::vector <osg::Vec2> ret;
    if(!is_loaded())
        return ret;

    std::vector <osg::Vec3> ABEF(4, osg::Vec3(0, 0, 0));//planar leaves

    //for each vertex, find its texture coordinates
    for(unsigned int i=0; i<all_v.size(); i++)
    {
        osg::Vec3 v = all_v[i];

        //find which quadrant does it lay
        osg::Vec3 normal = v - _cg;
        int region = which_quadrant(normal);

        //for rotation
        osg::Matrixf R;

        //front
        if(region == 0)
            R.makeIdentity();

        //right, rotate to front by -120
        if(region == 1)
            R.makeRotate(osg::Vec3(0, 1, 0), osg::Vec3(0.866025403, -0.5, 0));

        //left, rotate to front by +120
        if(region == 2)
            R.makeRotate(osg::Vec3(0, 1, 0), osg::Vec3(-0.866025403, -0.5, 0));

        //top, rotate to front by -60
        if(region == 3)
            //R.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(0, -1, 0));
            R.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(0, -0.866025403, 0.5));

        //bottom, rotate to front by +60
        if(region == 4)
            //R.makeRotate(osg::Vec3(0, 0, -1), osg::Vec3(0, -1, 0));
            R.makeRotate(osg::Vec3(0, 0, -1), osg::Vec3(0, -0.866025403, -0.5));

        if(i%4 != 3)
        {
            ABEF[i%4] = v;

            continue;
        }
        else
        {
            ABEF[3] = v;

            //to prevent different corner points multiple to different rotations
            //so use the last R to apply to all
            for(int j=0; j<4; j++)
            {
                ABEF[j] = (ABEF[j]-_cg) * R + _cg;

                osg::Vec2 img_space = obj_to_img_space(ABEF[j]);
                //further transform it to real texture space by bound
                osg::Vec2 tex_space = img_space - _bound_bottomLeft;//for texture mapping, origin is at bottomLeft
                tex_space.y() *= -1.0f;

                //normalized it to within 0 <= x <= 1
                osg::Vec2 tex_coord(tex_space.x()/float(_bound_width), tex_space.y()/float(_bound_height));

                ret.push_back(tex_coord);
            }
        }
    }

    return ret;
}
