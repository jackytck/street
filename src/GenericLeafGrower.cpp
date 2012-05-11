#include "GenericLeafGrower.h"
#include <queue>
#include <stack>
#include "osgModeler.h"
#include "Transformer.h"
#include <fstream>
#include <QImage>
#include "PalmParameter.h"

GenericLeafGrower::GenericLeafGrower(): _root(NULL), _scale(-1.0f), _grow_zone(0.286f), _radius_k(0.1f), _pedal(13), _fuzziness(0.0f), _verbose(false)
{
    srand(time(NULL));
}

void GenericLeafGrower::setup(BDLSkeletonNode *root, std::string path, float scale)
{
	if(!root || path.empty())
		return;

	_root = root;
	_scale = scale;
	_generic_texure = path;
}

void GenericLeafGrower::set_scale(float s)
{
	_scale = s;
}

void GenericLeafGrower::set_parameters(float grow_zone, float radius_k, int pedal, float fuzziness)
{
    _grow_zone = grow_zone;
    _radius_k = radius_k;
    _pedal = pedal;
    _fuzziness = fuzziness;
}

void GenericLeafGrower::set_verbose(bool flag)
{
	_verbose = flag;
}

void GenericLeafGrower::clear()
{
	_all_pos.clear();
	_all_v.clear();
	_all_tex.clear();
}

void GenericLeafGrower::grow()
{
    if(!_root)
        return;

	const double gold_angle = 137.50776405003785; //Golden Angle
    float leaf_scale = _scale <= 0.0f ? BDLSkeletonNode::leaf_scale_hint(_root)/5.5f : _scale;

	if(_verbose)
		printf("Current leaf-scale is %f.\n", leaf_scale);

    //add leaves
    //bfs once, find the approximate tangent by (current - prev)
    //add a leaf or leaves if this node has no children
    //or its radius is smaller than a fraction of the radius of the root
    double radius_thresold = _root->_radius * _radius_k;
	float height = BDLSkeletonNode::max_height(_root);

    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_root);

    //use this vector to store all vertices and texture coords
    osg::ref_ptr <osg::Vec3Array> all_v = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> all_tex = new osg::Vec2Array;
	_all_pos.clear();

	bool flat_leaf = true;

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        //push all of my children
        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);

        //no parent, i.e. a root, should not has any leaf
        if(!front->_prev)
            continue;

        //graph's leafs || branch with small radius
        if(true || front->_children.empty() || front->_radius <= radius_thresold)//lateral
        {
			osg::Vec3 pos = Transformer::toVec3(front);
			//don't grow leaf near the root
			if((pos-Transformer::toVec3(_root)).length() < height * _grow_zone)
				continue;

			osg::Vec3 pre = Transformer::toVec3(front->_prev);
            osg::Vec3 approxi_tangent = pos - pre;
			float par_child_dist = approxi_tangent.length();
            approxi_tangent.normalize();

            //consider a plane with approxi_tangent as the normal and contains point pos
            //find basis u, v of this plane
            osg::Vec3 u(approxi_tangent.y(), -approxi_tangent.x(), 0.0);
			u.normalize();
            osg::Vec3 v = approxi_tangent ^ u;

            //consider another plane with u as normal and contains point pos
            //the basis of this plane is chosen as approxi_tangent and v
            //div_angle is the angle spanned by approxi_tangent and the root of the leaf
            double div_angle = (rand()%30-15+50)/180.0*M_PI;

            //number of leaf of this node has
            int no_leaf = rand()%_pedal + _pedal;
			if(front->_children.size() == 0)
				no_leaf += 2;

            for(int i=1; i<=no_leaf; i++)
            {
				//this angle should be multiple of Golden Angle, not uniformly
                //double angle = 360.0/no_leaf*i/180.0*M_PI;
				double angle = i*gold_angle;
				double angle_int;
				angle = modf(angle, &angle_int);
				angle += int(angle_int) % 360;
				float cycle = angle_int / 360.0f;

				//the more turn the larger div_angle
                osg::Vec3 div = u*cos(angle) + v*sin(angle);
                div = approxi_tangent * cos(div_angle+cycle*0.140) + div * sin(div_angle+cycle*0.140);

				//the more turn the closer to the prev
				//float offset = cycle*leaf_scale*4.0f;
				float offset = cycle*par_child_dist*0.9f / (no_leaf/2.3f);//non-lateral growth
				//float offset = cycle*par_child_dist*1.0f / (no_leaf/2.0f);//lateral growth or high-density foliage
				if(offset > par_child_dist)
					offset = par_child_dist;
				osg::Vec3 tmp_pos = pos - approxi_tangent*offset;
				
				//add some fuzziness for more seamless coverage
				tmp_pos += div * front->_radius * (front->_children.empty() ? _fuzziness*0.8f : _fuzziness);

                //preparing for 1 pos, 4 vertices and 4 tex-coords
                osgModeler::createLeaf(all_v, all_tex, tmp_pos, div, 360.0/no_leaf*i, leaf_scale, flat_leaf);
				_all_pos.push_back(tmp_pos);
            }
        }
    }

	_all_v = Transformer::osg_to_std_array(all_v);
	_all_tex = Transformer::osg_to_std_array(all_tex);

    return;
}

void GenericLeafGrower::grow_palm()
{
    if(!_root)
        return;

    //todo: fine tune other parameters by configs in settings
    float leaf_scale = _scale <= 0.0f ? BDLSkeletonNode::leaf_scale_hint(_root)/5.5f : _scale;
	if(_verbose)
		printf("Current leaf-scale is %f.\n", leaf_scale);

    //checking: assume its geometry is a stick, otherwise just choose the first terminal node
    BDLSkeletonNode *terminal = _root;
    while(!terminal->_children.empty())
        terminal = terminal->_children[0];
    if(terminal == _root)
    {
        printf("GenericLeafGrower::grow_palm():incorrect input skeleton\n");
        return;
    }
    _all_v.clear();
    _all_tex.clear();

    //a. setup frame and parameters for quadratic bezier curvers
    osg::Vec3 ter = Transformer::toVec3(terminal);
    osg::Vec3 normal = ter - Transformer::toVec3(terminal->_prev);
    normal.normalize();
    float height = terminal->dist(_root);
    osg::Vec3 plane_pt = ter + normal * height * 0.20f;

    osg::Vec3 u(normal.y(), -normal.x(), 0.0f);
    u.normalize();
    osg::Vec3 v = normal ^ u;

    int level = 3;
    int no_leaf = 5;
    float plane_r = height * 0.25f;
    float thetha = 0.0f;

    //b. infer list of 2nd and 3rd control points (1st control point is shared by all)
    std::vector <osg::Vec3> sec_pts, third_pts;
    for(int l=0; l<level; l++)
    {
        for(int i=0; i<no_leaf; i++)
        {
            osg::Vec3 ctr_pt_2 = u * plane_r * cos(thetha) + v * plane_r * sin(thetha) + plane_pt;
            sec_pts.push_back(ctr_pt_2);

            float drop = l > 0 ? -0.05f : 0.15f;
            float extend = l > 0 ? 1.1f : 1.2f;

            osg::Vec3 ctr_pt_3 = ctr_pt_2 - normal * height * drop + (ctr_pt_2 - plane_pt) * extend;
            third_pts.push_back(ctr_pt_3);

            thetha += 1.0f / no_leaf * 2 * M_PI;
        }

        //higher and closer to the main trunk for the next level
        plane_r *= (l == level-2) ? 0.6f : 0.80f;
        plane_pt = ter + normal * (plane_pt - ter).length() * (1.2f + float(l)/level + (l == level-2 ? 0.4f : 0.0f));
        thetha = 1.0f / no_leaf * 2 * M_PI / (l+2);
    }

    //c. infer the width of a leaf
    std::vector <osg::Vec3> on_curve = Transformer::interpolate_bezier_2(ter, sec_pts[0], third_pts[0]);
    float inter_width = Transformer::average_inter_dist(on_curve);

    //d. infer the starting vertices of each curve
    std::vector <osg::Vec3> start_as;
    std::vector <osg::Vec3> start_bs;
    thetha = 0.0f;

    for(int l=0; l<level; l++)
    {
        for(int i=0; i<no_leaf; i++)
        {
            start_as.push_back(u * inter_width * 1.20f * cos(thetha+M_PI/2) + v * inter_width * 1.20f * sin(thetha+M_PI/2) + ter);
            start_bs.push_back(u * inter_width * 1.20f * cos(thetha-M_PI/2) + v * inter_width * 1.20f * sin(thetha-M_PI/2) + ter);

            thetha += 1.0f / no_leaf * 2 * M_PI;
        }

        thetha = 1.0f / no_leaf * 2 * M_PI / (l+2);
    }

    //e. read width and height of input texture
    QImage img(_generic_texure.c_str());
    if(img.isNull())
    {
        printf("GenericLeafGrower::grow_palm():_img(%s) error\n", _generic_texure.c_str());
        return;
    }
    int t_w = img.width(), t_h = img.height();

    //f. tile square planes along each bezier path, ab:ef = 2.5:1
    for(int l=0; l<level; l++)
    {
        for(int i=0; i<no_leaf; i++)
        {
            osg::Vec3 ctr_pt1 = ter;
            osg::Vec3 ctr_pt2 = sec_pts[l*no_leaf + i];
            osg::Vec3 ctr_pt3 = third_pts[l*no_leaf + i];

            osg::Vec3 a = start_as[l*no_leaf + i];
            osg::Vec3 b = start_bs[l*no_leaf + i];

            std::vector <osg::Vec3> leafs = Transformer::tile_plane_along_path(ctr_pt1, ctr_pt2, ctr_pt3, a, b);
            std::vector <osg::Vec2> tex = Transformer::texture_coords_palm(leafs, t_w, t_h);

            for(unsigned int j=0; j<leafs.size(); j++)
            {
                _all_v.push_back(leafs[j]);
                _all_tex.push_back(tex[j]);
            }
        }
    }

    //debug
    if(false)
    {
        for(unsigned int i=0; i<sec_pts.size(); i++)
            printf("v %f %f %f\n", sec_pts[i].x(), sec_pts[i].y(), sec_pts[i].z());

        for(unsigned int i=0; i<third_pts.size(); i++)
            printf("v %f %f %f\n", third_pts[i].x(), third_pts[i].y(), third_pts[i].z());

        for(unsigned int i=0; i<_all_v.size(); i++)
            printf("v %f %f %f\n", _all_v[i].x(), _all_v[i].y(), _all_v[i].z());
    }

    return;
}

void GenericLeafGrower::grow_palm2()
{
    if(!_root)
        return;

    //todo: fine tune other parameters by configs in settings
    float leaf_scale = _scale <= 0.0f ? BDLSkeletonNode::leaf_scale_hint(_root)/5.5f : _scale;
	if(_verbose)
		printf("Current leaf-scale is %f.\n", leaf_scale);

    //checking: assume its geometry is a stick, otherwise just choose the first terminal node
    BDLSkeletonNode *terminal = _root;
    while(!terminal->_children.empty())
        terminal = terminal->_children[0];
    if(terminal == _root)
    {
        printf("GenericLeafGrower::grow_palm2():incorrect input skeleton\n");
        return;
    }
    _all_v.clear();
    _all_tex.clear();

    //a. setup frame and parameters for quadratic bezier curvers
    osg::Vec3 ter = Transformer::toVec3(terminal);
    osg::Vec3 normal = ter - Transformer::toVec3(terminal->_prev);
    normal.normalize();
    float height = terminal->dist(_root);
    osg::Vec3 plane_pt = ter + normal * height * 0.20f;

    osg::Vec3 u(normal.y(), -normal.x(), 0.0f);
    u.normalize();
    osg::Vec3 v = normal ^ u;

    int level = 2;
    int no_leaf = 4;
    float plane_r = height * 0.1f;
    float thetha = 0.0f;

    //b. infer list of 2nd and 3rd control points (1st control point is shared by all)
    std::vector <osg::Vec3> sec_pts, third_pts;
    for(int l=0; l<level; l++)
    {
        for(int i=0; i<no_leaf; i++)
        {
            osg::Vec3 ctr_pt_2 = u * plane_r * cos(thetha) + v * plane_r * sin(thetha) + plane_pt;
            sec_pts.push_back(ctr_pt_2);

            float drop = l > 0 ? -0.15f : -0.17f;
            float extend = l > 0 ? 2.1f : 2.8f;

            osg::Vec3 ctr_pt_3 = ctr_pt_2 - normal * height * drop + (ctr_pt_2 - plane_pt) * extend;
            third_pts.push_back(ctr_pt_3);

            thetha += 1.0f / no_leaf * 2 * M_PI;
        }

        //higher and closer to the main trunk for the next level
        plane_r *= (l == level-2) ? 0.8f : 0.80f;
        plane_pt = ter + normal * (plane_pt - ter).length() * (1.2f + float(l)/level + (l == level-2 ? 0.4f : 0.0f));
        thetha = 1.0f / no_leaf * 2 * M_PI / (l+2);
        no_leaf -= 2;
    }


    //c. read width and height of input texture (which represents a single leaf now) to compute its aspect ratio, 
    QImage img(_generic_texure.c_str());
    if(img.isNull() || img.width() == 0 || img.height() == 0)
    {
        printf("GenericLeafGrower::grow_palm2():_img(%s) error\n", _generic_texure.c_str());
        return;
    }
    float aspect = float(img.height()) / img.width();

    //d. get quads and tex coords for each bezier curve
    PalmParameter pp;//this class interpolate all the palm parameters
    int density = 25;//number of growing point for each bezier curve, each growing point grows 2 leaves, each leaf has 4 vertices
    int debug_cnt = 0;
    bool debug = false;
    
    for(unsigned int i=0; i<sec_pts.size(); i++)
    {
        osg::Vec3 ctr_pt1 = ter;
        osg::Vec3 ctr_pt2 = sec_pts[i];
        osg::Vec3 ctr_pt3 = third_pts[i];

        pp.setBezierQuadratic(ctr_pt1, ctr_pt2, ctr_pt3);
        std::vector <osg::Vec3> leafs = pp.getQuads(aspect, debug, debug_cnt);
        std::vector <osg::Vec2> tex = pp.getTexCoords();

        debug_cnt += density * 8;

        for(unsigned int j=0; j<leafs.size(); j++)
        {
            _all_v.push_back(leafs[j]);
            _all_tex.push_back(tex[j]);
        }
    }

    return;
}

/* the idea is to let the tree data-structure represent both the branching
 * structure and the leaf arrangements via Bezier control points
 */
void GenericLeafGrower::grow_palm3()
{
    if(!_root)
        return;

    //todo: fine tune other parameters by configs in settings
    float leaf_scale = _scale <= 0.0f ? BDLSkeletonNode::leaf_scale_hint(_root)/5.5f : _scale;
	if(_verbose)
		printf("Current leaf-scale is %f.\n", leaf_scale);

    //checking: assume its geometry is a stick, plus a set of curly branches representing the cubic Bezier curves
    //the terminal node is defined as the first node whose child count is larger than 1
    BDLSkeletonNode *terminal = _root;
    while(!terminal->_children.empty())
    {
        if(terminal->_children.size() > 1)
            break;
        terminal = terminal->_children[0];
    }
    if(terminal == _root || terminal->_children.empty())
    {
        printf("GenericLeafGrower::grow_palm3():incorrect input skeleton\n");
        return;
    }
    _all_v.clear();
    _all_tex.clear();

    //a. dfs from terminal and infer set of cubic bezier control points
    //1 st control point is the terminal node and is shared by all curves
    osg::Vec3 ter = Transformer::toVec3(terminal);
    std::vector <osg::Vec3> sec_pts, third_pts, forth_pts;

    std::vector <osg::Vec3> tmp_list;
    std::stack <BDLSkeletonNode *> Stack;
    Stack.push(terminal);
    while(!Stack.empty())
    {
        BDLSkeletonNode *top = Stack.top();
        Stack.pop();

        //assume the structure is fixed and defined as:
        //       __
        //      /
        // ter -----
        //      \__
        if(top != terminal)
            tmp_list.push_back(Transformer::toVec3(top));

        if(top->_children.empty())
        {
            if(tmp_list.size() >= 3)
            {
                sec_pts.push_back(tmp_list[0]);
                third_pts.push_back(tmp_list[tmp_list.size()-2]);
                forth_pts.push_back(tmp_list[tmp_list.size()-1]);
                tmp_list.clear();
            }
        }

        for(unsigned int i=0; i<top->_children.size(); i++)
            Stack.push(top->_children[i]);
    }

    if(sec_pts.empty())
    {
        printf("GenericLeafGrower::grow_palm3():incorrect input skeleton\n");
        return;
    }

    //b. read width and height of input texture (which represents a single leaf now) to compute its aspect ratio, 
    QImage img(_generic_texure.c_str());
    if(img.isNull() || img.width() == 0 || img.height() == 0)
    {
        printf("GenericLeafGrower::grow_palm2():_img(%s) error\n", _generic_texure.c_str());
        return;
    }
    float aspect = float(img.height()) / img.width();

    //c. get quads and tex coords for each bezier curve
    float noise = 0.5f;
    PalmParameter pp(noise);//this class interpolate all the palm parameters
    pp.enableGravity();
    
    for(unsigned int i=0; i<sec_pts.size(); i++)
    {
        osg::Vec3 ctr_pt1 = ter;
        osg::Vec3 ctr_pt2 = sec_pts[i];
        osg::Vec3 ctr_pt3 = third_pts[i];
        osg::Vec3 ctr_pt4 = forth_pts[i];

        pp.setBezierCubic(ctr_pt1, ctr_pt2, ctr_pt3, ctr_pt4);
        std::vector <osg::Vec3> leafs = pp.getQuads(aspect);
        std::vector <osg::Vec2> tex = pp.getTexCoords();

        for(unsigned int j=0; j<leafs.size(); j++)
        {
            _all_v.push_back(leafs[j]);
            _all_tex.push_back(tex[j]);
        }

        //if(i >= 2)
        //    break;//debug
    }

    return;
}

void GenericLeafGrower::save(std::string path)
{
	if(path.empty())
		return;

    //double side?
    //if(double_side)
    //    double_side_it(v, n, tex);

    //check data before saving
    if(_all_v.size() != _all_tex.size() || _generic_texure.empty())
    {
        printf("GenericLeafGrower::save():_all_v.size(%d) _all_tex.size(%d) _generic_texure(%s) error\n", int(_all_v.size()), int(_all_tex.size()), _generic_texure.c_str());
        return;
    }

	//scheme is exactly the same as BillboardTree
    //SAVING
    FILE *out = fopen(path.c_str(), "w");

    //first line is number of vertice
    fprintf(out, "%d\n", int(_all_v.size()));

    //_all_v.size()'s lines of vertice
    for(unsigned int i=0; i<_all_v.size(); i++)
        fprintf(out, "%f %f %f\n", _all_v[i].x(), _all_v[i].y(), _all_v[i].z());

    //_all_v.size()'s lines of texure coords
    for(unsigned int i=0; i<_all_tex.size(); i++)
        fprintf(out, "%f %f\n", _all_tex[i].x(), _all_tex[i].y());

    //file path of the generic texture
    fprintf(out, "%s\n", _generic_texure.c_str());

    fclose(out);
}

void GenericLeafGrower::load(std::string path)
{
	if(path.empty())
		return;

    std::ifstream fs(path.c_str());
    std::string s;

	int total;
    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &total);

	//must be equal to multiple of 4
	//this may happen if one carelessly erases individual vertex instead of a quad automatically
	if(total%4 != 0)
		return;

	clear();

	//load the vertices
    for(int i=0; i<total; i++)
    {
        float px, py, pz;

        getline(fs, s);
        sscanf(s.c_str(), "%f %f %f\n", &px, &py, &pz);

		_all_v.push_back(osg::Vec3(px, py, pz));
    }

	int quad = total/4;

	//infer all position by approximated by the cg of the quad
	osg::Vec3 a, b, c, d;
	for(int i=0; i<quad; i++)
	{
		a = _all_v[i*4];
		b = _all_v[i*4+1];
		c = _all_v[i*4+2];
		d = _all_v[i*4+3];

		osg::Vec3 cg = (a+b+c+d)*0.25f;
		_all_pos.push_back(cg);
	}

	//infer the texture-coordinates
	//<0,0> <0,1> <1,0> <1,1>
	for(int i=0; i<quad; i++)
	{
		_all_tex.push_back(osg::Vec2(0.0f, 0.0f));
		_all_tex.push_back(osg::Vec2(0.0f, 1.0f));
		_all_tex.push_back(osg::Vec2(1.0f, 0.0f));
		_all_tex.push_back(osg::Vec2(1.0f, 1.0f));
	}

	fs.close();
}

void GenericLeafGrower::colorize(std::string s, std::string b, bool blend)
{
	QImage sample(s.c_str()), bill(b.c_str());
	if(sample.isNull() || bill.isNull())
	{
		printf("sample(%s) bill(%s) path error\n", s.c_str(), b.c_str());
		return;
	}

	//transfer color from sample to bill
	//by blending the non-transparent pixel in bill with the current pixel in sample
	//the non-transparent sample pixels will be looped forever until all pixels in bill are exhausted
	//note that x means height, y means width
	/*
		QRgb color = img.pixel(QPoint(bi_texs[k].x(), bi_texs[k].y()));
		texture.setPixel(p, color);
	*/
	int h = bill.height(), w = bill.width();
	int nx = 0, ny = 0;

	for(int i=0; i<h; i++)
		for(int j=0; j<w; j++)
		{
			QRgb target = bill.pixel(j, i);
			if(qAlpha(target) != 0) // if not fully transparent
			{
				next_pixel(sample, nx, ny);
				if(nx == -1 || ny == -1)
				{
					printf("GenericLeafGrower::colorize:sample image errro\n");
					return;
				}

				QRgb source = sample.pixel(ny, nx);

				if(blend)
				{
					int br = (qRed(source) + qRed(target)) / 2;
					int bg = (qGreen(source) + qGreen(target)) / 2;
					int bb = (qBlue(source) + qBlue(target)) / 2;
					int ba = (qAlpha(source) + qAlpha(target)) / 2;

					QRgb blend = qRgba(br, bg, bb, ba);
					bill.setPixel(j, i, blend);
				}
				else
					bill.setPixel(j, i, source);
			}
		}

	//save back
    bill.save(QString(b.c_str()), "PNG", 100);
}

void GenericLeafGrower::dump(std::string path)
{
	if(path.empty())
		return;

    //SAVING
    FILE *out = fopen(path.c_str(), "w");

    //first line is number of pos
    fprintf(out, "%d\n", int(_all_pos.size()));

    //_all_pos.size()'s lines of pos
    for(unsigned int i=0; i<_all_pos.size(); i++)
        fprintf(out, "%f %f %f\n", _all_pos[i].x(), _all_pos[i].y(), _all_pos[i].z());

    //number of v
    fprintf(out, "%d\n", int(_all_v.size()));

    //_all_v.size()'s lines of vertice
    for(unsigned int i=0; i<_all_v.size(); i++)
        fprintf(out, "%f %f %f\n", _all_v[i].x(), _all_v[i].y(), _all_v[i].z());

    //number of tex
    fprintf(out, "%d\n", int(_all_tex.size()));

    //_all_v.size()'s lines of texure coords
    for(unsigned int i=0; i<_all_tex.size(); i++)
        fprintf(out, "%f %f\n", _all_tex[i].x(), _all_tex[i].y());

    fclose(out);
}

//note that x means height, y means width
void GenericLeafGrower::next_pixel(const QImage& img, int& x, int& y)
{
	if(img.isNull())
		return;

	int retX = -1, retY = -1;
	int h = img.height(), w = img.width();
	bool first = true, has_alpha = img.hasAlphaChannel();

	for(int i=0; i<h; i++)
		for(int j=0; j<w; j++)
		{
			if(first)
			{
				i = x;
				j = y;
				first = false;
				continue;
			}

			if(has_alpha)
			{
				QRgb color = img.pixel(j, i);
				if(qAlpha(color) != 0) //if not fully transparent
				{
					retX = i;
					retY = j;
					goto Return;
				}
			}
			else
			{
				retX = i;
				retY = j;
				goto Return;
			}
		}

	//loop over again
	if(retX == -1 || retY == -1)
	{
		for(int i=0; i<h; i++)
			for(int j=0; j<w; j++)
			{
				if(has_alpha)
				{
					QRgb color = img.pixel(j, i);
					if(qAlpha(color) != 0) //if not fully transparent
					{
						retX = i;
						retY = j;
						break;
					}
				}
				else
				{
					retX = i;
					retY = j;
					break;
				}
			}
	}

	//return -1 if not found, so caller needs to check
	Return:
	x = retX;
	y = retY;
}
