#include "Transformer.h"
#include "eigen.h"
#include <queue>
#include <noise.h>
#include <ANN.h>
#include "Triangle.h"

Transformer::Transformer()
{
}

osg::Vec3 Transformer::transform(SLink link, float rotation, osg::Vec3 p)
{
    osg::Vec3 ret;
    
    if(!link._par || !link._child)
        return ret;

    //construct the frame w.r.t the link
    osg::Vec3 link_par(link._par->_sx, link._par->_sy, link._par->_sz);
    osg::Vec3 link_child(link._child->_sx, link._child->_sy, link._child->_sz);

    osg::Vec3 link_z = link_child - link_par;
    link_z.normalize();

    osg::Vec3 link_x(link_z.y(), -link_z.x(), 0.0);
    link_x.normalize();

    osg::Vec3 link_y = link_z ^ link_x;

    rotation = rotation*M_PI/180.0f;
    osg::Vec3 rotated_link_x = link_x*cos(rotation) + link_y*sin(rotation);
    
    osg::Vec3 rotated_link_y = link_z ^ rotated_link_x;

    //p rotation
    ret = rotated_link_x*p.x() + rotated_link_y*p.y() + link_z*p.z();

    //translation
    ret = ret + link_par;

    return ret;
}

osg::Vec3 Transformer::transform(BDLSkeletonNode *par, BDLSkeletonNode *child, float rotation, osg::Vec3 p)
{
    return transform(SLink(par, child), rotation, p);
}

void Transformer::transform(SLink link, int rotation, LibraryElement& element)
{
    if(!element.isValid() || !link._par || !link._child)
        return;
    //directly modify the element

    //straight up the element such that the supporting branch is aligned with z-axis
    straight_up(element);

    //first determine the scale by inspecting the primary-branch
    float target_height = BDLSkeletonNode::dist(link._par, link._child);
    float source_height = element.support_branch_length();
    if(target_height == 0.0f || source_height == 0.0f)
        return;

    float k_st = target_height / source_height * 1.0;
    rotation %= 360;

    //bfs once
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(element._root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        osg::Vec3 nodeV = toVec3(node);

        //scale before calling transform
        nodeV = nodeV * k_st;

        nodeV = transform(link, rotation, nodeV);

        node->_sx = nodeV.x();
        node->_sy = nodeV.y();
        node->_sz = nodeV.z();

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }
}

void Transformer::transform(BDLSkeletonNode *tail, int rotation, LibraryElement& element)
{
    transform(SLink(tail->_prev_support, tail), rotation, element);
}

void Transformer::straight_up(LibraryElement& element)
{
    if(!element.isValid())
        return;

    osg::Vec3 z = toVec3(element._support_branch._child) - toVec3(element._support_branch._par);
    z.normalize();

    if(z.x() == 0.0f && z.y() == 0.0f)
        return;

    osg::Vec3 x(z.y(), -z.x(), 0.0f);
    x.normalize();

    osg::Vec3 y = z ^ x;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(element._root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        osg::Vec3 wrap = toVec3(front);
        osg::Vec3 straighted(wrap * x, wrap * y, wrap * z);

        front->_sx = straighted.x();
        front->_sy = straighted.y();
        front->_sz = straighted.z();

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);
    }
}

float Transformer::point_segment_dist(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c)
{
    const float l2 = (b-a).length2();

    // a == b
    if(l2 == 0.0)
        return (c-a).length();

    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of point c onto the line. 
    // It falls where t = [(c-a) . (b-a)] / |b-a|^2
    const float t = (c-a) * (b-a) / l2;

    // Beyond the 'a' end of the segment
    if(t < 0.0)
        return (c-a).length();

    // Beyond the 'b' end of the segment
    if(t > 1.0)
        return (c-b).length();

    // Projection falls on the segment
    const osg::Vec3 proj = a + (b-a)*t;
    return (c-proj).length();
}

float Transformer::point_segment_dist(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *c)
{
    if(!a || !b || !c)
        return -1.0f;

    osg::Vec3 aw(a->_sx, a->_sy, a->_sz);
    osg::Vec3 bw(b->_sx, b->_sy, b->_sz);
    osg::Vec3 cw(c->_sx, c->_sy, c->_sz);

    return point_segment_dist(aw, bw, cw);
}

float Transformer::point_segment_dist(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2& per)
{
    per = osg::Vec2(0.0f, 0.0f);
    const float l2 = (b-a).length2();

    // a == b
    if(l2 == 0.0)
        return (c-a).length();

    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of point c onto the line.
    // It falls where t = [(c-a) . (b-a)] / |b-a|^2
    const float t = (c-a) * (b-a) / l2;

    // Beyond the 'a' end of the segment
    if(t < 0.0)
        return (c-a).length();

    // Beyond the 'b' end of the segment
    if(t > 1.0)
        return (c-b).length();

    // Projection falls on the segment
    const osg::Vec2 proj = a + (b-a)*t;
    per = proj - c;
    return per.length();
}

SLink Transformer::closest_segment(BDLSkeletonNode *tail, BDLSkeletonNode *query)
{
    SLink ret;
    if(!tail || !tail->_prev_support || !query)
        return ret;

    BDLSkeletonNode *cur = tail, *end = tail->_prev_support, *last = NULL;
    float min_dist = -1.0f;
    BDLSkeletonNode *min_a = NULL, *min_b = NULL;

    while(last != end)
    {
        if(last)
        {
            float cur_dist = point_segment_dist(cur, last, query);
            if(min_dist == -1.0f || cur_dist < min_dist)
            {
                min_dist = cur_dist;
                min_a = cur;
                min_b = last;
            }
        }

        last = cur;
        cur = cur->_prev;
    }
    
    if(min_a && min_b)
    {
        ret._par = min_a;
        ret._child = min_b;
    }

    return ret;
}

osg::Vec3 Transformer::toVec3(BDLSkeletonNode *node)
{
    osg::Vec3 ret;

    if(!node)
        return ret;

    ret.x() = node->_sx;
    ret.y() = node->_sy;
    ret.z() = node->_sz;

    return ret;
}

float Transformer::line_dist(osg::Vec3 L1_P0, osg::Vec3 L1_P1, osg::Vec3 L2_P0, osg::Vec3 L2_P1)
{
    osg::Vec3 u = L1_P1 - L1_P0;
    osg::Vec3 v = L2_P1 - L2_P0;
    osg::Vec3 w = L1_P0 - L2_P0;
    float a = u * u;        // always >= 0
    float b = u * v;
    float c = v * v;        // always >= 0
    float d = u * w;
    float e = v * w;
    float D = a*c - b*b;    // always >= 0
    float sc, tc;

    // compute the line parameters of the two closest points
    if(D < 0.00000001)            // the lines are almost parallel
    {
        sc = 0.0;
        tc = (b>c ? d/b : e/c);   // use the largest denominator
    }
    else 
    {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // get the difference of the two closest points
    osg::Vec3 dP = w + (u * sc) - (v * tc);  // = L1(sc) - L2(tc)

    return dP.length();     // return the closest distance
}

float Transformer::segment_dist(osg::Vec3 S1_P0, osg::Vec3 S1_P1, osg::Vec3 S2_P0, osg::Vec3 S2_P1)
{
    osg::Vec3 u = S1_P1 - S1_P0;
    osg::Vec3 v = S2_P1 - S2_P0;
    osg::Vec3 w = S1_P0 - S2_P0;
    float a = u * u;              // always >= 0
    float b = u * v;
    float c = v * v;              // always >= 0
    float d = u * w;
    float e = v * w;
    float D = a*c - b*b;          // always >= 0
    float sc, sN, sD = D;         // sc = sN / sD, default sD = D >= 0
    float tc, tN, tD = D;         // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < 0.00000001) {         // the lines are almost parallel
        sN = 0.0;                 // force using point P0 on segment S1
        sD = 1.0;                 // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                        // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {           // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {       // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {               // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {           // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < 0.00000001 ? 0.0 : sN / sD);
    tc = (abs(tN) < 0.00000001 ? 0.0 : tN / tD);

    // get the difference of the two closest points
    osg::Vec3 dP = w + (u * sc) - (v * tc);  // = S1(sc) - S2(tc)

    return dP.length();           // return the closest distance
}

float Transformer::segment_dist(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *c, BDLSkeletonNode *d)
{
    float ret = -1.0f;
    if(!a || !b || !c || !d)
        return ret;

    return segment_dist(toVec3(a), toVec3(b), toVec3(c), toVec3(d));
}

float Transformer::link_tree_dist(BDLSkeletonNode *root, BDLSkeletonNode *a, BDLSkeletonNode *b)
{
    float ret = -1.0f;
    if(!root || !a || !b)
        return ret;

    //bfs to find all links
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];

            float cur = segment_dist(front, child, a, b);
            //printf("link_tree_dist(%f)\n", cur);
            if(cur > 0.0001f) 
                if(ret == -1.0f || cur < ret)
                    ret = cur;

            Queue.push(child);
        }
    }

    return ret;
}

BDLSkeletonNode *Transformer::point_tree_dist(BDLSkeletonNode *root, osg::Vec3 p, float& dist)
{
    float min_dist = -1.0f;
    BDLSkeletonNode *ret = NULL;
    if(!root)
        return ret;

    //bfs to find all nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

		osg::Vec3 f = toVec3(front);
		float cur = (f-p).length2();
		if(min_dist == -1.0f || cur < min_dist)
        {
			min_dist = cur;
            ret = front;
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
			Queue.push(front->_children[i]);
    }

    dist = sqrt(min_dist);
    return ret;
}

BDLSkeletonNode *Transformer::point_tree_dist(std::vector <BDLSkeletonNode *> trees, osg::Vec3 p, float& dist)
{
    float min_dist = -1.0f;
    BDLSkeletonNode *ret = NULL;
    if(trees.empty())
        return ret;

    for(unsigned int i=0; i<trees.size(); i++)
    {
        BDLSkeletonNode *node = trees[i];

        osg::Vec3 f = toVec3(node);
		float cur = (f-p).length2();
		if(min_dist == -1.0f || cur < min_dist)
        {
			min_dist = cur;
            ret = node;
        }
    }

    dist = sqrt(min_dist);
    return ret;
}

float Transformer::point_tree_dist(BDLSkeletonNode *root, osg::Vec3 p)
{
    float dist;
    point_tree_dist(root, p, dist);
    return dist;
}

void Transformer::cartesian_to_spherical(osg::Vec3 p, double& lat, double& lon)
{
    p.normalize();

    //-90 <= lat <= 90
    lat = asin(p.z());

    //-180 <= lon <= 180
    lon = atan2(p.y(), p.x());

    //to degrees
    lat *=  180.0 / M_PI;
    lon *=  180.0 / M_PI;
}

osg::Vec3 Transformer::spherical_to_cartesian(double r, double lat, double lon)
{
    osg::Vec3 ret;

    //to radians
    lat *= M_PI / 180.0;
    lon *= M_PI / 180.0;

    ret.x() = r * cos(lat) * cos(lon);
    ret.y() = r * cos(lat) * sin(lon);
    ret.z() = r * sin(lat);

    return ret;
}

osg::Vec3 Transformer::ellipsoid_to_cartesian(double rz, double rx, double ry, double lat, double lon)
{
    osg::Vec3 ret;

    //to radians
    lat *= M_PI / 180.0;
    lon *= M_PI / 180.0;

    ret.x() = rx * cos(lat) * cos(lon);
    ret.y() = ry * cos(lat) * sin(lon);
    ret.z() = rz * sin(lat);

    return ret;
}

void Transformer::points_to_leafs(std::vector <osg::Vec3>& all_v, osg::Vec3 pos, osg::Vec3 normal, float scale, bool random)
{
    //initial vertices values, depends on the texture
    osg::Vec3 a, b, e, f, from;
    a = osg::Vec3(-0.5f, -0.5f, 0.0f);
    b = osg::Vec3(-0.5f, 0.5f, 0.0f);
    e = osg::Vec3(0.5f, -0.5f, 0.0f);
    f = osg::Vec3(0.5f, 0.5f, 0.0f);
    from = osg::Vec3(0.0f, 0.0f, 1.0f);

    //perturb normal
    if(random)
    {
        //osg::Vec3 d = spherical_to_cartesian(0.4, rand()%180-90, rand()%360);
        osg::Vec3 d = spherical_to_cartesian(1.0, rand()%180-90, rand()%360);
        normal += d;
        normal.normalize();
    }

    osg::Matrixf S, T, R;
    S.makeScale(scale, scale, scale);
    T.makeTranslate(pos);
    R.makeRotate(from, normal);

    //resultant matrix, osg use post multiplication, not the same as OpenGL
    osg::Matrix M;
    M = S * R * T;

    //note: order is important for the right normal
    all_v.push_back(b * M);//b
    all_v.push_back(a * M);//a
    all_v.push_back(f * M);//f
    all_v.push_back(e * M);//e
}

std::vector <osg::Vec3> Transformer::pos_to_vertex(const std::vector <osg::Vec3>& pts, osg::Vec3 cg, float scale)
{
    std::vector <osg::Vec3> ret;

    osg::Vec3 normal;
    for(unsigned int i=0; i<pts.size(); i++)
    {
        osg::Vec3 pos = pts[i];
        normal = pos - cg;
        normal.normalize();

        points_to_leafs(ret, pos, normal, scale);
    }

    return ret;
}

osg::ref_ptr <osg::Vec3Array> Transformer::std_to_osg_array(std::vector <osg::Vec3> v)
{
    osg::ref_ptr <osg::Vec3Array> ret = new osg::Vec3Array;

    for(unsigned int i=0; i<v.size(); i++)
        ret->push_back(v[i]);

    return ret;
}

osg::ref_ptr <osg::Vec2Array> Transformer::std_to_osg_array(std::vector <osg::Vec2> v)
{
    osg::ref_ptr <osg::Vec2Array> ret = new osg::Vec2Array;

    for(unsigned int i=0; i<v.size(); i++)
        ret->push_back(v[i]);

    return ret;
}

std::vector <osg::Vec3> Transformer::osg_to_std_array(osg::ref_ptr <osg::Vec3Array> v)
{
	std::vector <osg::Vec3> ret;

	for(unsigned int i=0; i<v->size(); i++)
		ret.push_back((*v)[i]);

	return ret;
}

std::vector <osg::Vec2> Transformer::osg_to_std_array(osg::ref_ptr <osg::Vec2Array> v)
{
	std::vector <osg::Vec2> ret;

	for(unsigned int i=0; i<v->size(); i++)
		ret.push_back((*v)[i]);

	return ret;
}

void Transformer::billow_noise(std::vector <osg::Vec3>& pts, std::vector <float>& scalars, int step)
{
    if(step <= 0)
        return;

    //clear inputs
    pts.clear();
    scalars.clear();

    //setup noise
    noise::module::Billow billow;
    billow.SetSeed(time(NULL));

    float grid = 1.0f / step;

    for(float i=0; i<=1; i+=grid)
        for(float j=0; j<=1; j+=grid)
            for(float k=0; k<=1; k+=grid)
            {
                double n = billow.GetValue(i, j, k);
                if(n > 0.5)
                {
                    pts.push_back(osg::Vec3(i, j, k));
                    scalars.push_back(n);
                }
            }
}

void Transformer::billow_noise_transformed(std::vector <osg::Vec3>& transformed, std::vector <float>& weights, osg::Vec3 origin, osg::Vec3 x, osg::Vec3 y, osg::Vec3 z, float w)
{
    //make origin be the center
    origin = origin - osg::Vec3(0.5f, 0.5f, 0.5f) * w;

    //canonical billow noise
    billow_noise(transformed, weights);

    for(unsigned int i=0; i<transformed.size(); i++)
    {
        osg::Vec3 p = transformed[i] * w;
        transformed[i] = origin + x*p.x() + y*p.y() + z*p.z();
    }
}

void Transformer::sphere_points(std::vector <osg::Vec3>& pts, std::vector <float>& weights, int coverage, float leaf_size_hint)
{
    if(leaf_size_hint <= 0.0f || coverage <= -90 || coverage >= 90)
        return;

    //clear inputs
    pts.clear();
    weights.clear();

    //setup noise for weights only
    noise::module::Billow billow;
    billow.SetSeed(time(NULL));

    float step = M_PI / leaf_size_hint;
    float lat_step = 180.0f / step;

    for(float i=coverage; i<90; i+=lat_step)
    {
        //0.9f is hard-coded to decrease the step for more samples
        float lon_step = 360.0f / (2 * step * cos(i/180.0f*M_PI)) * 0.9f;

        for(float j=0; j<360; j+=lon_step)
        {
            osg::Vec3 p = spherical_to_cartesian(1.0f, i, j);
            double n = billow.GetValue(p.x(), p.y(), p.z());

            pts.push_back(p);
            weights.push_back(n);
        }
    }

    Transformer::normalize_float(weights);
}

void Transformer::ellipsoid_points(std::vector <osg::Vec3>& pts, std::vector <float>& weights, int coverage, float leaf_size_hint)
{
    if(leaf_size_hint <= 0.0f || coverage <= -90 || coverage >= 90)
        return;

    //clear inputs
    pts.clear();
    weights.clear();

    //setup noise for weights only
    noise::module::Billow billow;
    billow.SetSeed(time(NULL));

    float step = M_PI / leaf_size_hint;
    float lat_step = 180.0f / step;

    for(float i=coverage; i<90; i+=lat_step)
    {
        //0.9f is hard-coded to decrease the step for more samples
        float lon_step = 360.0f / (2 * step * cos(i/180.0f*M_PI)) * 0.9f;

        for(float j=0; j<360; j+=lon_step)
        {
            osg::Vec3 p = ellipsoid_to_cartesian(1.0f, 0.7f, 0.7f, i, j);
            double n = billow.GetValue(p.x(), p.y(), p.z());

            pts.push_back(p);
            weights.push_back(n);
        }
    }

    Transformer::normalize_float(weights);
}

void Transformer::sphere_points_transformed(std::vector <osg::Vec3>& transformed, std::vector <float>& weights, osg::Vec3 origin, float r, osg::Vec3 z, int coverage, float leaf_size_hint)
{
    if(r <= 0.0f)
        return;

    //frame
    osg::Vec3 x(z.y(), -z.x(), 0.0f);
    x.normalize();
    osg::Vec3 y = z ^ x;

    //deduce the leaf_size_hint from r, divide r because it's an unit sphere
    leaf_size_hint /= r;

    //canonical billow noise
    //sphere_points(transformed, weights, coverage, leaf_size_hint);
    ellipsoid_points(transformed, weights, coverage, leaf_size_hint);

    for(unsigned int i=0; i<transformed.size(); i++)
    {
        osg::Vec3 p = transformed[i] * r;
        transformed[i] = origin + x*p.x() + y*p.y() + z*p.z();
    }
}

void Transformer::normalize_float(std::vector <float>& in)
{
    //find min and max first
    float min = -1.0f, max = 1.0f;
    for(unsigned int i=0; i<in.size(); i++)
    {
        float cur = in[i];

        if(min == -1.0f || cur < min)
            min = cur;
        if(max == -1.0f || cur > max)
            max = cur;
    }

    //normalize to [0,1]
    float delta = max-min;
    if(delta == 0.0f)
        return;

    for(unsigned int i=0; i<in.size(); i++)
        in[i] = (in[i]-min) / delta;
}

osg::Vec3 Transformer::find_cg(std::vector <osg::Vec3> pts)
{
    osg::Vec3 ret(0.0f, 0.0f, 0.0f);
    if(pts.empty())
        return ret;
    
    for(unsigned int i=0; i<pts.size(); i++)
        ret += pts[i];

    return ret * 1.0f/pts.size();
}

float Transformer::divergent_angle(BDLSkeletonNode *prev, BDLSkeletonNode *node, BDLSkeletonNode *child)
{
    float ret = -1.0;
    if(!prev || !node || !child)
        return ret;

    osg::Vec3 p = toVec3(prev), n = toVec3(node), c = toVec3(child);
    osg::Vec3 nc = c - n;
    osg::Vec3 pn = n - p;

    nc.normalize();
    pn.normalize();

    //if nc points in the same direction as pn, dot = 1
    //if nc points in the perpendicular direction as pn, dot = 0
    //if nc points in the opposite direction as pn, dot = -1
    float dot = nc * pn;
    ret = acos(dot);
    ret *= 180.0f / M_PI;//degree

    return ret;
}

bool Transformer::similar_angle_in_tree(BDLSkeletonNode *root, float angle, BDLSkeletonNode *&a, BDLSkeletonNode *&b)
{
    if(!root || angle < 0.0f)
        return false;

    //to find the most similar angle in tree, just bfs inside a bfs
    //to find all the divergent angle and compared with 'angle'
    a = NULL, b = NULL;
    float min_diff = -1.0f;

    //bfs
    std::queue <BDLSkeletonNode *> Queue1;
    Queue1.push(root);
    
    //debug
    int work = 0;
    
    while(!Queue1.empty())
    {
        BDLSkeletonNode *front1 = Queue1.front();
        Queue1.pop();

        if(front1->_prev)
        {
            //bfs
            std::queue <BDLSkeletonNode *> Queue2;
            Queue2.push(front1);

            while(!Queue2.empty())
            {
                BDLSkeletonNode *front2 = Queue2.front();
                Queue2.pop();

                if(front2 != front1)
                {
                    float ang = divergent_angle(front1->_prev, front1, front2);
                    float diff = abs(ang - angle);
                    
                    if(min_diff == -1.0f || diff < min_diff)
                    {
                        min_diff = diff;
                        a = front1;
                        b = front2;
                    }
                }

                for(unsigned int i=0; i<front2->_children.size(); i++)
                    Queue2.push(front2->_children[i]);

                work++;
            }
        }

        for(unsigned int i=0; i<front1->_children.size(); i++)
            Queue1.push(front1->_children[i]);
    }

    //printf("Transformer::similar_angle_in_tree:work(%d) min_diff(%f)\n", work, min_diff);

    //return (a != NULL && b != NULL && min_diff < 20.0f);
    return (a != NULL && b != NULL);
}

BDLSkeletonNode *Transformer::extract_subtree(BDLSkeletonNode *a, BDLSkeletonNode *b, BDLSkeletonNode *&new_b, bool full)
{
    BDLSkeletonNode *ret = NULL;
    if(!a || !b || a == b)
        return ret;

    //check if a could led to b and find the number of hop from a to b
    int hop = 0;
    ret = b;
    bool valid = false;
    while(ret)
    {
        ret = ret->_prev;
        hop++;

        if(ret == a)
        {
            valid = true;
            break;
        }
    }
    if(!valid)
        return ret;

    if(full)//todo: make sure the a has only one child
    {
        BDLSkeletonNode *a_prev = a->_prev;
        a->_prev = NULL;
        ret = BDLSkeletonNode::copy_tree(a);
        ret->_prev = NULL;
        new_b = NULL;
        a->_prev = a_prev;

        //bfs old tree to find id of b
        int id = 0;
        int b_id = -1;
        std::queue <BDLSkeletonNode *> Queue;
        Queue.push(a);
        while(!Queue.empty())
        {
            BDLSkeletonNode *front = Queue.front();
            Queue.pop();

            if(front == b)
                b_id = id;

            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
            id++;
        }

        //bfs new tree to find new_b
        id = 0;
        Queue.push(ret);
        while(!Queue.empty())
        {
            BDLSkeletonNode *front = Queue.front();
            Queue.pop();

            if(id == b_id)
            {
                new_b = front;
                break;
            }

            for(unsigned int i=0; i<front->_children.size(); i++)
                Queue.push(front->_children[i]);
            id++;
        }

        //bfs to delete any node that has a larger hop
        std::queue <BDLSkeletonNode *> Queue2;
        Queue2.push(ret);
        ret->_generation = 0;

        while(!Queue2.empty())
        {
            BDLSkeletonNode *front = Queue2.front();
            Queue2.pop();

            if(front->_generation > hop)
                BDLSkeletonNode::delete_this(front);
            else
            {
                for(unsigned int i=0; i<front->_children.size(); i++)
                {
                    BDLSkeletonNode *child = front->_children[i];
                    child->_generation = front->_generation + 1;
                    Queue2.push(child);
                }
            }
        }
    }
    else
    {
        //tranverse from b to a, add the first level nodes to this path only
        new_b = new BDLSkeletonNode(b->_sx, b->_sy, b->_sz);
        BDLSkeletonNode *cur = b;
        ret = new_b;
        while(cur)
        {
            cur = cur->_prev;
            if(cur)
            {
                BDLSkeletonNode *tmp = new BDLSkeletonNode(cur->_sx, cur->_sy, cur->_sz);
                ret->_prev = tmp;
                tmp->_children.push_back(ret);

                if(false && cur!=a)
                {
                    if(cur->_children.size() > 1)
                    {
                        for(unsigned int i=0; i<cur->_children.size(); i++)
                        {
                            BDLSkeletonNode *cur_child = cur->_children[i];
                            if(BDLSkeletonNode::dist(cur_child, ret) > 0.01f)
                            {
                                BDLSkeletonNode *tmp2 = new BDLSkeletonNode(cur_child->_sx, cur_child->_sy, cur_child->_sz);
                                tmp->_children.push_back(tmp2);
                                break;//only one extra child
                            }
                        }
                    }
                }

                ret = tmp;
            }

            if(cur == a)
            {
                break;
            }
        }
    }

    //bfs to make ret placed at the origin
    std::queue <BDLSkeletonNode *> Queue2;
    Queue2.push(ret);
    while(!Queue2.empty())
    {
        BDLSkeletonNode *front = Queue2.front();
        Queue2.pop();

        front->_sx -= ret->_sx;
        front->_sy -= ret->_sy;
        front->_sz -= ret->_sz;

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue2.push(front->_children[i]);
    }

    return ret;
}

std::vector <BDLSkeletonNode *> Transformer::terminal_nodes(BDLSkeletonNode *root)
{
    std::vector <BDLSkeletonNode *> ret;
    if(!root)
        return ret;

    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        if(front->_children.empty())
            ret.push_back(front);
        
        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);
    }

    return ret;
}

float **Transformer::ATA(float **M, int m, int n)
{
	float **ATA = (float**) calloc (n, sizeof(float*));
	for(int i=0; i<n; i++)
		ATA[i] = (float *) calloc (n, sizeof(float));
	float **MT = (float**) calloc (n, sizeof(float*));
	for(int i=0; i<n; i++)
		MT[i] = (float *) calloc (m, sizeof(float));
	for(int i=0; i<n; i++)
		for(int j=0; j<m; j++)
			MT[i][j] = M[j][i];
	for(int i=0; i<n; i++)
		for(int j=0; j<n; j++)
			for(int k=0; k<m; k++)
					ATA[i][j] += M[k][j] * MT[i][k];
	for(int i=0; i<n; i++)
		free(MT[i]);
	free(MT);
	for(int i=0; i<m; i++)
		free(M[i]);
	free(M);
	return ATA;
}

void Transformer::homography(float **H, int w, int h, float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
{
    if(!H)
        return;

    float e[2] = {ax, ay};
    float f[2] = {bx, by};
    float g[2] = {cx, cy};
    float k[2] = {dx, dy};
    
    //construct system of linear equations
    int m = 8, n = 9; //4 points, so 8 equations, and 9 unknowns for H
    float **A = (float**) calloc (m, sizeof(float*));
    for(int i=0; i<m; i++)
        A[i] = (float *) calloc (n, sizeof(float));
    A[0][0] = 0;	A[0][1] = 0;	A[0][2] = 1;	A[0][3] = 0;	A[0][4] = 0;	A[0][5] = 0;	A[0][6] = 0;	    A[0][7] = 0;	    A[0][8] = -e[0];
    A[1][0] = 0;	A[1][1] = 0;	A[1][2] = 0;	A[1][3] = 0;	A[1][4] = 0;	A[1][5] = 1;	A[1][6] = 0;	    A[1][7] = 0;	    A[1][8] = -e[1];
    A[2][0] = w;	A[2][1] = 0;	A[2][2] = 1;	A[2][3] = 0;	A[2][4] = 0;	A[2][5] = 0;	A[2][6] = -f[0]*w;	A[2][7] = 0;	    A[2][8] = -f[0];
    A[3][0] = 0;	A[3][1] = 0;	A[3][2] = 0;	A[3][3] = w;	A[3][4] = 0;	A[3][5] = 1;	A[3][6] = -f[1]*w;	A[3][7] = 0;	    A[3][8] = -f[1];
    A[4][0] = w;	A[4][1] = h;	A[4][2] = 1;	A[4][3] = 0;	A[4][4] = 0;	A[4][5] = 0;	A[4][6] = -g[0]*w;	A[4][7] = -g[0]*h;	A[4][8] = -g[0];
    A[5][0] = 0;	A[5][1] = 0;	A[5][2] = 0;	A[5][3] = w;	A[5][4] = h;	A[5][5] = 1;	A[5][6] = -g[1]*w;	A[5][7] = -g[1]*h;	A[5][8] = -g[1];
    A[6][0] = 0;	A[6][1] = h;	A[6][2] = 1;	A[6][3] = 0;	A[6][4] = 0;	A[6][5] = 0;	A[6][6] = 0;	    A[6][7] = -k[0]*h;	A[6][8] = -k[0];
    A[7][0] = 0;	A[7][1] = 0;	A[7][2] = 0;	A[7][3] = 0;	A[7][4] = h;	A[7][5] = 1;	A[7][6] = 0;	    A[7][7] = -k[1]*h;	A[7][8] = -k[1];

    //use eigen decomposition to solve
    A = ATA(A, m, n);
    float *eig_val = (float *) calloc(n, sizeof(float));
    float **eig_vec = (float **) calloc(n, sizeof(float *));
    for(int i=0; i<n; i++)
        eig_vec[i] = (float *) calloc(n, sizeof(float));
    eig_sys(n, A, eig_vec, eig_val);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            H[i][j] = eig_vec[n-1][i*3+j];

    //debug
    //printf("H = [%f %f %f\n %f %f %f\n %f %f %f]\n", H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0], H[2][1], H[2][2]);

    //free up
    for(int i=0; i<n; i++)
        free(eig_vec[i]);
    free(eig_vec);
    free(eig_val);
	for(int i=0; i<n; i++)
        free(A[i]);
    free(A);
}

int Transformer::which_quadrant(osg::Vec3 normal)
{
    int ret = -1;

    double lat, lon;

    //change to spherical coordinates first
    cartesian_to_spherical(normal, lat, lon);

    //can ignore top and bottom
    if(false)
    {
        //top
        if(lat > 80)
        //if(lat > 40)
            return 3;

        //bottom
        if(lat < -55)
        //if(lat < -35)
            return 4;
    }

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

osg::Vec3 Transformer::mult_vec(float **H, osg::Vec3 in)
{
    osg::Vec3 t = osg::Vec3(
                            H[0][0]*in[0] + H[0][1]*in[1] + H[0][2]*in[2],
                            H[1][0]*in[0] + H[1][1]*in[1] + H[1][2]*in[2],
                            H[2][0]*in[0] + H[2][1]*in[1] + H[2][2]*in[2]
                            );
	return osg::Vec3(t[0]/t[2], t[1]/t[2], 1.0f);
}

unsigned char Transformer::bilinear(float x, float y, const unsigned char p00, const unsigned char p01, const unsigned char p10, const unsigned char p11)
{
	return p00*(1-x)*(1-y) + p10*x*(1-y) + p01*(1-x)*y + p11*x*y;
} 

float Transformer::first_hop_dist(BDLSkeletonNode *root)
{
    float ret = -1.0f;

    if(!root || root->_children.empty() || root->_children.size() > 1)
        return ret;

    ret = BDLSkeletonNode::dist(root, root->_children[0]);

    return ret;
}

std::vector <BDLSkeletonNode *> Transformer::bfs(BDLSkeletonNode *root)
{
    std::vector <BDLSkeletonNode *> ret;
    if(!root)
        return ret;

    //bfs
    std::queue <BDLSkeletonNode *> Queue;
    root->_generation = 0;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        ret.push_back(front);

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            front->_children[i]->_generation = front->_generation + 1;
            Queue.push(front->_children[i]);
        }
    }

    return ret;
}

  
std::vector <BDLSkeletonNode *> Transformer::leaves(BDLSkeletonNode *root)
{
    std::vector <BDLSkeletonNode *> leaves;

    if(root)
    {
        std::vector <BDLSkeletonNode *> s_tree = Transformer::bfs(root);

        for(unsigned int i=0; i<s_tree.size(); i++)
        {
            BDLSkeletonNode *node = s_tree[i];
            if(node->_children.empty())
                leaves.push_back(node);
        }
    }

    return leaves;
}

int Transformer::tree_size(BDLSkeletonNode *root)
{
    return int(bfs(root).size());
}

std::vector <std::vector <osg::Vec3> > Transformer::ann_cluster(std::vector <osg::Vec3> pts, int k)
{
    std::vector <std::vector <osg::Vec3> > ret;
    if(pts.empty() || k <= 0)
        return ret;

    /////////////ANN TREE///////////////////
    //parameters
    int dim = 3;
    double eps = 0;			                                // error bound

    //ann + bdl data points
    int pointSize = pts.size();
	ANNpointArray annDataPts = annAllocPts(pointSize, dim);
    std::vector <BDLPoint> bdlNodes;

    for(int i=0; i<pointSize; i++)
    {
        osg::Vec3 p = pts[i];
        ANNpoint annP = annDataPts[i];
        annP[0] = p.x();
        annP[1] = p.y();
        annP[2] = p.z();

        BDLPoint bdlP;
        bdlP.x = p.x();
        bdlP.y = p.y();
        bdlP.z = p.z();
        bdlP.w = 1.0f;
        bdlNodes.push_back(bdlP);
    }

    //allocate ann index, dist and kdTree
	ANNidxArray	nnIdx = new ANNidx[k];						// allocate near neigh indices
	ANNdistArray dists = new ANNdist[k];		            // allocate near neighbor dists
    ANNkd_tree *kdTree = new ANNkd_tree(annDataPts, pointSize, dim);					

    //construct a BDLPointGraph
    BDLPointGraph bdlPointGraph;

    //wrape the node in BDLPointGraphNode format
    bdlPointGraph.new_nodes(pointSize);

    for(int i=0; i<pointSize; i++)
    {
        //query point
        ANNpoint annQueryPt = annDataPts[i];

        kdTree->annkSearch(annQueryPt, k, nnIdx, dists, eps);						

        BDLPointGraphNode *bdlGraphNode = bdlPointGraph._nodes[i];

        //fill the relationships of this node with others
        for(int j=0; j<k; j++)
        {
            if(j==0)
            {
                bdlGraphNode->_self = bdlNodes[nnIdx[j]];
            }
            else
            {
                //push BDLPointGraphNode
                bdlGraphNode->_edges.push_back(bdlPointGraph._nodes[nnIdx[j]]);
                bdlGraphNode->_costs.push_back(sqrt(dists[j]));
            }
        }

        //debug
        //std::cout << "\n\tNN:\tIndex\tDistance\n";
        //for (int i = 0; i < k; i++) 
        //{			
        //    dists[i] = sqrt(dists[i]);			// unsquare distance
        //    std::cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
        //}
    }

    // clean things up
    delete [] nnIdx;
    delete [] dists;
    delete kdTree;
    //annDeallocPts(annDataPts);
    //annDeallocPt(annQueryPt);
	annClose();					                           // done with ANN

    /////////////END ANN TREE///////////////////

    //////////////BFS//////////////////////////
    int total = 0;
    for(unsigned n=0; n<bdlPointGraph._nodes.size(); n++)
    {
        BDLPointGraphNode *root = bdlPointGraph._nodes[n];
        if(root->_done)
            continue;

        std::queue <BDLPointGraphNode *> Queue;
        Queue.push(root);
        std::vector <osg::Vec3> cluster;

        while(!Queue.empty())
        {
            BDLPointGraphNode *node = Queue.front();
            Queue.pop();

            if(node->_done)
                continue;

            node->_done = true;
            osg::Vec3 osgP(node->_self.x, node->_self.y, node->_self.z);
            cluster.push_back(osgP);
            total++;

            for(unsigned int i=0; i<node->_edges.size(); i++)
            {
                BDLPointGraphNode *neighbor = node->_edges[i];
                if(!neighbor->_done)
                    Queue.push(neighbor);
            }
        }

        ret.push_back(cluster);
    }
    //////////////END BFS/////////////////////

    //debug
    //printf("bdlPointGraph._nodes(%d)\n", int(bdlPointGraph._nodes.size()));
    //printf("total(%d)\n", total);
    //for(unsigned int i=0; i<ret.size(); i++)
    //    printf("%d: cluster(%d)\n", i, int(ret[i].size()));

    return ret;
}

float Transformer::min_z(std::vector <osg::Vec3> pts)
{
    float ret = -23418.715f;
    for(unsigned int i=0; i<pts.size(); i++)
    {
        if(ret == -23418.715f || pts[i].z() < ret)
            ret = pts[i].z();
    }

    return ret;
}

float Transformer::max_z(std::vector <osg::Vec3> pts)
{
    float ret = -23418.715f;
    for(unsigned int i=0; i<pts.size(); i++)
    {
        if(ret == -23418.715f || pts[i].z() > ret)
            ret = pts[i].z();
    }

    return ret;
}

void Transformer::extend_end_points(BDLSkeletonNode *initial, osg::Vec3 fol_c, float threshold)
{
    std::vector <BDLSkeletonNode *> leaves = Transformer::leaves(initial);
    for(unsigned int i=0; i<leaves.size(); i++)
    {
        BDLSkeletonNode *node = leaves[i];
        //if(!node->_prev || node->_sz < threshold*0.85f)
        if(!node->_prev || node->_sz < threshold*0.55f)
            continue;

        osg::Vec3 o_node = Transformer::toVec3(node);
        osg::Vec3 par_c = o_node - Transformer::toVec3(node->_prev);
        float par_len = par_c.length();
        par_c.normalize();

        //add two branches to each end-points
        osg::Vec3 to_fc = fol_c - o_node;

        float scale = 1.0f;
        osg::Vec3 new_pos = (to_fc*0.2f + par_c*par_len*1.5f)*scale + o_node;

        //first branch
        BDLSkeletonNode *new_node = new BDLSkeletonNode(new_pos.x(), new_pos.y(), new_pos.z());
        new_node->_prev = node;
        node->_children.push_back(new_node);

        //second branch (by mirroring)
        osg::Vec3 new_pos2 = new_pos;
        new_pos2.x() = -1.0f * (new_pos.x() - o_node.x()) + o_node.x();
        BDLSkeletonNode *new_node2 = new BDLSkeletonNode(new_pos2.x(), new_pos2.y(), new_pos2.z());
        new_node2->_prev = node;
        node->_children.push_back(new_node2);

        //increase depth difference of first and second branches
        float d1 = new_node->_sx, d2 = new_node2->_sx;
        float offset = d1 - d2;
        if(false)
        {
            if(offset > 0.0f)
            {
                if(offset < 0.5f)
                {
                    offset *= 0.025f;
                    new_node->_sx += offset;
                    new_node2->_sx -= offset;
                }
            }
            else
            {
                if(offset > -0.5f)
                {
                    offset -= 0.025f;
                    new_node->_sx -= offset;
                    new_node2->_sx += offset;
                }
            }
        }

        //5c. extend all end-points by 10%
        osg::Vec3 elongated = o_node + par_c * par_len * 1.2f;
        //osg::Vec3 elongated = o_node + par_c * par_len * 1.0f;
        node->_sx = elongated.x();
        node->_sy = elongated.y();
        node->_sz = elongated.z();
    }
}

bool Transformer::cloud_metrics(std::vector <osg::Vec3> pts, float& right, float& left, float& top, float& bottom, float& in, float& out)
{
    bool ret = false;
    if(pts.empty())
        return ret;

    //copied from LaserPruner
    float _left = -1.0f;
    float _right = -1.0f;
    float _top = -1.0f;
    float _bottom = -1.0f;
    float _in = -1.0f;
    float _out = -1.0f;

    for(unsigned int i=0; i<pts.size(); i++)
    {
        osg::Vec3 p = pts[i];
        if(_left == -1.0f || p.x() < _left)
            _left = p.x();
        if(_right == -1.0f || p.x() > _right)
            _right = p.x();
        if(_top == -1.0f || p.z() > _top)
            _top = p.z();
        if(_bottom == -1.0f || p.z() < _bottom)
            _bottom = p.z();
        if(_in == -1.0f || p.y() > _in)
            _in = p.y();
        if(_out == -1.0f || p.y() < _out)
            _out = p.y();
    }

    //set back results
    if(_left!=-1.0f && _right!=-1.0f && _top!=-1.0f && _bottom!=-1.0f && _in!=-1.0f && _out!=-1.0f)
    {
       ret = true; 
       right = _right;
       left = _left;
       top = _top;
       bottom = _bottom;
       in = _in;
       out = _out;
    }

    //debug
    //printf("_left(%f) _right(%f) _top(%f) _bottom(%f) _in(%f) _out(%f)\n", _left, _right, _top, _bottom, _in, _out);

    return ret;
}

float Transformer::orient(osg::Vec2 p, osg::Vec2 q, osg::Vec2 r)
{
    float det = q.x()*r.y()-q.y()*r.x() - (p.x()*r.y()-p.y()*r.x()) + p.x()*q.y()-p.y()*q.x();
    return det;
}

std::vector <osg::Vec3> Transformer::icosahedron(int depth)
{
    std::vector <osg::Vec3> ret;
    if(depth < 0)
        return ret;

    std::set <osg::Vec3> light_set;

    //hard-code all triangle faces first
    std::vector <Triangle> icosa;
	float t = 1.6180339887498949;
	icosa.push_back(Triangle(-1, t, 0, 1, t, 0, 0, 1, t));
	icosa.push_back(Triangle(1, t, 0, t, 0, 1, 0, 1, t));
	icosa.push_back(Triangle(t, 0, 1, 0, -1, t, 0, 1, t));
	icosa.push_back(Triangle(t, 0, 1, 1, -t, 0, 0, -1, t));
	icosa.push_back(Triangle(-1, -t, 0, 0, -1, t, 1, -t, 0));
	icosa.push_back(Triangle(-1, -t, 0, -t, 0, 1, 0, -1, t));
	icosa.push_back(Triangle(-t, 0, 1, 0, 1, t, 0, -1, t));
	icosa.push_back(Triangle(-t, 0, 1, -1, t, 0, 0, 1, t));

	icosa.push_back(Triangle(-1, t, 0, 1, t, 0, 0, 1, -t));
	icosa.push_back(Triangle(1, t, 0, t, 0, -1, 0, 1, -t));
	icosa.push_back(Triangle(t, 0, -1, 0, -1, -t, 0, 1, -t));
	icosa.push_back(Triangle(t, 0, -1, 1, -t, 0, 0, -1, -t));
	icosa.push_back(Triangle(-1, -t, 0, 0, -1, -t, 1, -t, 0));
	icosa.push_back(Triangle(-1, -t, 0, -t, 0, -1, 0, -1, -t));
	icosa.push_back(Triangle(-t, 0, -1, 0, 1, -t, 0, -1, -t));
	icosa.push_back(Triangle(-t, 0, -1, -1, t, 0, 0, 1, -t));

	icosa.push_back(Triangle(1, t, 0, t, 0, -1, t, 0, 1));
	icosa.push_back(Triangle(t, 0, -1, 1, -t, 0, t, 0, 1));
	icosa.push_back(Triangle(-1, t, 0, -t, 0, -1, -t, 0, 1));
	icosa.push_back(Triangle(-t, 0, -1, -1, -t, 0, -t, 0, 1));

    //normalize each vertex
    for(unsigned int i=0; i<icosa.size(); i++)
    {
        icosa[i]._a.normalize();
        icosa[i]._b.normalize();
        icosa[i]._c.normalize();

        light_set.insert(icosa[i]._a);
        light_set.insert(icosa[i]._b);
        light_set.insert(icosa[i]._c);
    }
        
    //sub-divide
    for(int d=0; d<depth; d++)
	{
        std::vector <Triangle> icosaNext;
        for(unsigned int i=0; i<icosa.size(); i++)
		{
			Triangle tri = icosa[i];
            osg::Vec3 a = tri._a, b = tri._b, c = tri._c, ab = (a+b)/2, bc = (b+c)/2, ac = (a+c)/2;
            ab.normalize();
            bc.normalize();
            ac.normalize();

            light_set.insert(ab);
            light_set.insert(bc);
            light_set.insert(ac);

			icosaNext.push_back(Triangle(a, ab, ac));
			icosaNext.push_back(Triangle(ab, b, bc));
			icosaNext.push_back(Triangle(ab, bc, ac));
			icosaNext.push_back(Triangle(ac, bc, c));
		}
		icosa = icosaNext;
	}

    //return results
    std::set <osg::Vec3>::iterator it;
    for(it=light_set.begin(); it!=light_set.end(); it++)
        ret.push_back(*it);

    //debug
	//printf("No. of Vertex = %u\t\n", int(light_set.size()));
    //for(unsigned int i=0; i<ret.size(); i++)
    //{
    //    osg::Vec3 p = ret[i];
    //    printf("v %f %f %f\n", p.x(), p.y(), p.z());
    //}

    return ret;
}

std::vector <osg::Vec3> Transformer::interpolate_bezier_2(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, int time)
{
    std::vector <osg::Vec3> ret;
    for(int i=0; i<time; i++)
    {
        float t = float(i)/time;
        osg::Vec3 cur = a*(1-t)*(1-t) + b*2*(1-t)*t + c*t*t;
        ret.push_back(cur);
    }
    return ret;
}

std::vector <osg::Vec3> Transformer::interpolate_bezier_3(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 d, int time)
{
    std::vector <osg::Vec3> ret;
    for(int i=0; i<=time; i++)
    {
        float t = float(i)/time;
        osg::Vec3 cur = a*(1-t)*(1-t)*(1-t) + b*3*(1-t)*(1-t)*t + c*3*(1-t)*t*t + d*t*t*t;
        ret.push_back(cur);
    }
    return ret;
}

std::vector <osg::Vec2> Transformer::interpolate_bezier_3_2d(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2 d, int time)
{
    std::vector <osg::Vec2> ret;
    for(int i=0; i<=time; i++)
    {
        float t = float(i)/time;
        osg::Vec2 cur = a*(1-t)*(1-t)*(1-t) + b*3*(1-t)*(1-t)*t + c*3*(1-t)*t*t + d*t*t*t;
        ret.push_back(cur);
    }
    return ret;
}

void Transformer::rect_plane(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3& d, osg::Vec3& e, float w)
{
    //normal, u, v of plane
    osg::Vec3 v = a - b;
    if(w == -1.0f)
        w = v.length();
    v.normalize();

    osg::Vec3 n = (c-b) ^ v;
    n.normalize();

    osg::Vec3 u = v ^ n;

    d = b + u * w;
    e = a + u * w;
}

float Transformer::average_inter_dist(std::vector <osg::Vec3> list)
{
    float ret = 0.0f;
    int cnt = 0;

    for(unsigned int i=0; i<list.size()-1; i++)
    {
        osg::Vec3 cur = list[i];
        osg::Vec3 next = list[i+1];

        ret += (cur - next).length();
        cnt++;
    }

    if(cnt > 0)
        ret /= cnt;

    return ret;
}

std::vector <osg::Vec3> Transformer::tile_plane_along_path(osg::Vec3 ctr_pt1, osg::Vec3 ctr_pt2, osg::Vec3 ctr_pt3, osg::Vec3 start_a, osg::Vec3 start_b)
{
    std::vector <osg::Vec3> ret;

    //a. interpolate the curve
    float inter_width = 0.0f;
    std::vector <osg::Vec3> on_curve = Transformer::interpolate_bezier_2(ctr_pt1, ctr_pt2, ctr_pt3);
    on_curve.push_back(ctr_pt3);

    //set length between hops
    if(on_curve.size() > 1)
        inter_width = Transformer::average_inter_dist(on_curve);
    if(inter_width == 0.0f)
        return ret;

    //b. tile square planes along the path
    osg::Vec3 a = start_a;
    osg::Vec3 b = start_b;

    for(unsigned int i=1; i<on_curve.size(); i++)
    {
        ret.push_back(b);
        ret.push_back(a);

        osg::Vec3 c = on_curve[i];
        osg::Vec3 d, e;

        Transformer::rect_plane(a, b, c, d, e, (a-b).length()/2.5);
        a = e;
        b = d;

        ret.push_back(b);
        ret.push_back(a);
    }

    return ret;
}

std::vector <osg::Vec2> Transformer::texture_coords_palm(std::vector <osg::Vec3> all_v, int t_w, int t_h)
{
    std::vector <osg::Vec2> ret;
    if(all_v.size() < 4 || all_v.size()%4 != 0 || t_w <= 0 || t_h <= 0 || (all_v[1]-all_v[0]).length() == 0.0f)
    {
        printf("Transformer::texture_coords_palm:input error\n");
        return ret;
    }

    //0. allocate memory
    ret.resize(all_v.size());

    //todo(low pirority): seems not significant since the scale up factor is not big
    //a. find curve's width and height, assume each quad has the same size
    //float c_w = (all_v[1] - all_v[0]).length();
    //float c_h = (all_v[2] - all_v[0]).length() * (all_v.size()/4);
    //float n_h = t_h / float(t_w) * c_w;//normalized height of texture

    //b. repeat texture in the middle (to avoid scaling up)
    //if(n_h < c_h)
    //{
    //}

    ////c. simply scale down
    //else
    {
        int n = all_v.size() / 4;
        float k = 1.0f / n;
        for(unsigned int i=0; i<ret.size(); i+=4)
        {
            ret[i+0] = osg::Vec2(1.0f, k * (i/4+0));
            ret[i+1] = osg::Vec2(0.0f, k * (i/4+0));
            ret[i+2] = osg::Vec2(1.0f, k * (i/4+1));
            ret[i+3] = osg::Vec2(0.0f, k * (i/4+1));
        }
    }

    return ret;
}

float Transformer::standard_deviation(std::vector <float> list, int last)
{
    float std = -1.0f;
    if(list.empty() || last == 0)
        return std;

    std = 0.0f;
    double n = list.size(), mean = 0;
    int len = list.size();
    if(last < 0 || last > len)
        last = len;
    else
        n = last;
    for(int i=len-1; i>= len-last; i--)
        mean += list[i];
    mean /= n;
    for(int i=len-1; i>= len-last; i--)
        std += (list[i] - mean) * (list[i] - mean);
    std = sqrt(std / float(n));
    return std;
}
