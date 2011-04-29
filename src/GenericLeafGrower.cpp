#include "GenericLeafGrower.h"
#include <queue>
#include "osgModeler.h"
#include "Transformer.h"
#include <fstream>

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
