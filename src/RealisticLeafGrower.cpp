#include "RealisticLeafGrower.h"
#include "GenericLeafGrower.h"
#include "ISPLoader.h"
#include <queue>
#include "osgModeler.h"
#include "Transformer.h"
#include <fstream>
#include "LaserProjector.h"

RealisticLeafGrower::RealisticLeafGrower(): _root(NULL), _scale(-1.0f), _verbose(false)
{
    srand(time(NULL));
}

void RealisticLeafGrower::setup(BDLSkeletonNode *root, std::string generic_path, float scale)
{
	if(!root || generic_path.empty())
		return;

	_root = root;
	_scale = scale;
	_generic_texure = generic_path;

    if(_scale <= 0.0f)
        _scale = BDLSkeletonNode::leaf_scale_hint(_root)/5.5f;
}

void RealisticLeafGrower::set_scale(float s)
{
	_scale = s;
}

void RealisticLeafGrower::set_verbose(bool flag)
{
	_verbose = flag;
}

void RealisticLeafGrower::clear()
{
	_all_pos.clear();
	_all_v.clear();
	_all_tex.clear();
}

//1. ask the GenericLeafGrower to grow generic leafs with specified skeleton and scale
//2. tile n number of generic leaves side by side to form a w by h texture and get target texture coords
//3. generate source texture coords
//4. from the generated source and target texture coords pair, backward wrapping the texture color from source to target
void RealisticLeafGrower::grow_single_image(int w, int h, float scale, int root_x, int root_y, std::string src_path, std::string src_seg_path)
{
    //step 1: use GenericLeafGrower to grow pos and all_v
    GenericLeafGrower leaf_grower;
    leaf_grower.set_verbose(_verbose);
    leaf_grower.setup(_root, _generic_texure, _scale);
    leaf_grower.grow();

    _all_pos = leaf_grower._all_pos;
    _all_v = leaf_grower._all_v;
    _all_tex = leaf_grower._all_tex;

    //step 2: tile texture and get the target texture coord
    _tiled_texture = tile_texture(_generic_texure, _all_pos.size(), w, h, _all_tex);

    //step 3: generate source texture coords
    osg::Vec3 cg = Transformer::find_cg(_all_pos);
    QImage src_img(src_path.c_str());
    if(src_img.isNull())
    {
        printf("RealisticLeafGrower::grow_single_image():src_img(%s) error\n", src_path.c_str());
        return;
    }
    int sw = src_img.width(), sh = src_img.height();
    _all_tex_isp0 = tex_coord_from_vertex(_all_v, cg, scale, root_x, root_y, sw, sh);

    //step 4: re-paint the new-texture by projectively backward transform the color
    if(_all_tex.size() == _all_tex_isp0.size() && _all_tex.size() % 4 == 0)
    {
        //open seg img too
        QImage src_seg(src_seg_path.c_str());
        if(src_seg.isNull())
        {
            printf("RealisticLeafGrower::grow_single_image():src_seg(%s) error\n", src_seg_path.c_str());
            return;
        }

        //allocate H
        float **H = (float**) calloc (3, sizeof(float*));
        for(int i=0; i<3; i++)
        H[i] = (float *) calloc (3, sizeof(float));

        //store each un-touched target leaf for post-processing
        std::vector <int> untouched;
        int max_paint_pixel = 0;//store the maximum number of pixels to be paint
        std::vector <int> full_touched;//store the target leaf which are fully touched

        //for each target leaf
        for(unsigned int i=0; i<_all_tex.size(); i+=4)
        {
            //_all_tex and _all_tex_isp0 are in order (0,0),(0,1),(1,0),(1,1)

            //targets
            osg::Vec2 at = _all_tex[i];
            osg::Vec2 bt = _all_tex[i+1];
            osg::Vec2 et = _all_tex[i+2];
            osg::Vec2 ft = _all_tex[i+3];

            at.x() *= w;
            bt.x() *= w;
            et.x() *= w;
            ft.x() *= w;

            at.y() = h - at.y() * h;
            bt.y() = h - bt.y() * h;
            et.y() = h - et.y() * h;
            ft.y() = h - ft.y() * h;

            int tw = int(et.x()) - int(at.x());
            int th = int(at.y()) - int(bt.y());

            int tstartx = int(bt.x());
            int tstarty = int(bt.y());

            //sources
            osg::Vec2 as = _all_tex_isp0[i];
            osg::Vec2 bs = _all_tex_isp0[i+1];
            osg::Vec2 es = _all_tex_isp0[i+2];
            osg::Vec2 fs = _all_tex_isp0[i+3];

            as.x() *= sw;
            bs.x() *= sw;
            es.x() *= sw;
            fs.x() *= sw;

            as.y() = sh - as.y() * sh;
            bs.y() = sh - bs.y() * sh;
            es.y() = sh - es.y() * sh;
            fs.y() = sh - fs.y() * sh;

            //H are in order (0,0),(1,0),(1,1),(0,1)
            Transformer::homography(H, tw, th, as.x(), as.y(), es.x(), es.y(), fs.x(), fs.y(), bs.x(), bs.y());

            //number of pixels painted
            int pixel_painted = 0;

            //backward-wrapping
            for(int j=0; j<tw; j++)
                for(int k=0; k<th; k++)
                {
                    //set pixel coords
                    int set_x = tstartx + j; 
                    int set_y = tstarty + k;

                    //only set pixels inside the leaf area
                    QRgb leaf_color = _tiled_texture.pixel(set_x, set_y);
                    if(qAlpha(leaf_color) != 0) //if not fully transparent
                    {
                        osg::Vec3 dir = Transformer::mult_vec(H, osg::Vec3(j, k, 1.0f));
                        float fx = dir.x();
                        float fy = dir.y();

                        //prepare for bilinear interpolation
                        float px = fx - floor(fx);
                        float py = fy - floor(fy);
                        int ix = (int)floor(fx);
                        int iy = (int)floor(fy);

                        if(ix < 0 || ix >= sw-1 || iy < 0 || iy >= sh-1)
                            continue;

                        //only use pixels inside the mask
                        QRgb seg_color = src_seg.pixel(ix, iy);
                        if(qRed(seg_color) != 0 || qGreen(seg_color) != 0 || qBlue(seg_color) != 0)
                        {
                            //color at 4 corners 
                            QRgb ptr00 = src_img.pixel(ix, iy);
                            QRgb ptr01 = src_img.pixel(ix, iy + 1);
                            QRgb ptr10 = src_img.pixel(ix + 1, iy);
                            QRgb ptr11 = src_img.pixel(ix + 1, iy + 1);

                            //bilinear interpolations
                            unsigned char r, g, b;
                            r = Transformer::bilinear(px, py, qRed(ptr00), qRed(ptr01), qRed(ptr10), qRed(ptr11));
                            g = Transformer::bilinear(px, py, qGreen(ptr00), qGreen(ptr01), qGreen(ptr10), qGreen(ptr11));
                            b = Transformer::bilinear(px, py, qBlue(ptr00), qBlue(ptr01), qBlue(ptr10), qBlue(ptr11));

                            //blend source and target
                            int br = (1.0f * r + 0.0f * qRed(leaf_color));
                            int bg = (1.0f * g + 0.0f * qGreen(leaf_color));
                            int bb = (1.0f * b + 0.0f * qBlue(leaf_color));

                            //blend or not is subject to data preference
                            QRgb blend = qRgb(br, bg, bb);
                            //blend = qRgb(r, g, b);

                            _tiled_texture.setPixel(set_x, set_y, blend);
                            pixel_painted++;
                        }

                        //store the maximum number of pixels to be painted
                        if(i==0)
                            max_paint_pixel++;
                    }
                }

            //append un-touched and fully-touched leaves for post-processing
            if(pixel_painted < max_paint_pixel * 0.95)
                untouched.push_back(i);
            if(pixel_painted == max_paint_pixel)
                full_touched.push_back(i);

        }//end for this target leaf

        //debug
        //printf("untouched(%d) full_touched(%d)\n", int(untouched.size()), int(full_touched.size()));

        //post-processing: copy a random full_touched leaf to each untouched one
        if(!full_touched.empty())
        {
            for(unsigned int i=0; i<untouched.size(); i++)
            {
                int src = full_touched[rand()%full_touched.size()];
                int des = untouched[i];

                //source
                osg::Vec2 as = _all_tex[src];
                osg::Vec2 bs = _all_tex[src+1];
                osg::Vec2 es = _all_tex[src+2];
                osg::Vec2 fs = _all_tex[src+3];

                as.x() *= w;
                bs.x() *= w;
                es.x() *= w;
                fs.x() *= w;

                as.y() = h - as.y() * h;
                bs.y() = h - bs.y() * h;
                es.y() = h - es.y() * h;
                fs.y() = h - fs.y() * h;

                int sstartx = int(bs.x());
                int sstarty = int(bs.y());

                //destination
                osg::Vec2 ad = _all_tex[des];
                osg::Vec2 bd = _all_tex[des+1];
                osg::Vec2 ed = _all_tex[des+2];
                osg::Vec2 fd = _all_tex[des+3];

                ad.x() *= w;
                bd.x() *= w;
                ed.x() *= w;
                fd.x() *= w;

                ad.y() = h - ad.y() * h;
                bd.y() = h - bd.y() * h;
                ed.y() = h - ed.y() * h;
                fd.y() = h - fd.y() * h;

                int dw = int(ed.x()) - int(ad.x());
                int dh = int(ad.y()) - int(bd.y());

                int dstartx = int(bd.x());
                int dstarty = int(bd.y());

                //copy
                for(int j=0; j<dw; j++)
                    for(int k=0; k<dh; k++)
                    {
                        //set pixel coords
                        int set_x = dstartx + j; 
                        int set_y = dstarty + k;

                        //get pixel coords
                        int get_x = sstartx + j;
                        int get_y = sstarty + k;

                        QRgb color = _tiled_texture.pixel(get_x, get_y);
                        //QRgb red = qRgba(255, 0, 0, qAlpha(color));//demo
                        _tiled_texture.setPixel(set_x, set_y, color);
                    }
            }//end each untouched
        }//end post-processing

        //deallocate H
        for(int i=0; i<3; i++)
            free(H[i]);
        free(H);
    }
    else
    {
        printf("RealisticLeafGrower::grow_single_image():_all_tex(%d) _all_tex_isp0(%d) size error\n", int(_all_tex.size()), int(_all_tex_isp0.size()));
        return;
    }
}

//same as above except now the texture coordinates are calculated by LaserProjector
//and blended with n-images
void RealisticLeafGrower::grow_laser(int w, int h, std::string simple_pt, std::string cameras)
{
    //step 1: use GenericLeafGrower to grow pos and all_v
    GenericLeafGrower leaf_grower;
    leaf_grower.set_verbose(_verbose);
    leaf_grower.setup(_root, _generic_texure, _scale);
    leaf_grower.grow();

    _all_pos = leaf_grower._all_pos;
    _all_v = leaf_grower._all_v;
    _all_tex = leaf_grower._all_tex;

    //step 2: tile texture and get the target texture coord
    _tiled_texture = tile_texture(_generic_texure, _all_pos.size(), w, h, _all_tex);

    //step 3: setup LaserProjector to get source texture coords
    LaserProjector lp;
    if(!lp.setup(cameras))
    {
        printf("RealisticLeafGrower::grow_laser():lp.setup error\n");
        return;
    }
    int sw = 1936, sh = 2592;

    //read cg of simple_point
    std::ifstream fs(simple_pt.c_str());
    std::string s;
    float sp_cg_x, sp_cg_y, sp_cg_z;
    getline(fs, s);
    sscanf(s.c_str(), "%f %f %f\n", &sp_cg_x, &sp_cg_y, &sp_cg_z);
    osg::Vec3 sp_cg(sp_cg_x, sp_cg_y, sp_cg_z);
    fs.close();

    //store the logistics for post-processing
    //store each un-touched target leaf for post-processing
    std::vector <int> touched_stat(_all_tex.size(), 0);//only use multiple of 4 elements
    int max_paint_pixel = 0;//store the maximum number of pixels to be paint on each generic leaf
    //store the target leaf which are fully touched, to be qualified, this leaf has to be almost fully painted by all views
    std::vector <int> full_touched;

    //step 4: for each view, re-paint the new-texture by projecting to it and get the color if inside segmentation mask
    for(unsigned int v=0; v<lp._view.size(); v++)
    //for(unsigned int v=1; v<2; v++)
    {
        QImage src_img(lp._view_img[v].c_str());
        QImage src_seg(lp._view_seg[v].c_str());
        if(src_img.isNull() || src_seg.isNull())
        {
            printf("RealisticLeafGrower::grow_laser():src_img(%s)\nsrc_seg(%s) error\n", lp._view_img[v].c_str(), lp._view_seg[v].c_str());
            return;
        }

        //for each target leaf
        for(unsigned int i=0; i<_all_tex.size(); i+=4)
        {
            //_all_tex are in order (0,0),(0,1),(1,0),(1,1)

            //targets
            osg::Vec2 at = _all_tex[i];
            osg::Vec2 bt = _all_tex[i+1];
            osg::Vec2 et = _all_tex[i+2];
            osg::Vec2 ft = _all_tex[i+3];

            at.x() *= w;
            bt.x() *= w;
            et.x() *= w;
            ft.x() *= w;

            at.y() = h - at.y() * h;
            bt.y() = h - bt.y() * h;
            et.y() = h - et.y() * h;
            ft.y() = h - ft.y() * h;

            int tw = int(et.x()) - int(at.x());
            int th = int(at.y()) - int(bt.y());

            int tstartx = int(bt.x());
            int tstarty = int(bt.y());

            //sources are inferred by each (j,k) to get a 3D point
            //so construct source 3D frame from top-left corner
            osg::Vec3 as = _all_v[i];
            osg::Vec3 bs = _all_v[i+1];
            osg::Vec3 es = _all_v[i+2];
            osg::Vec3 fs = _all_v[i+3];

            osg::Vec3 bfs = fs - bs;
            osg::Vec3 bas = as - bs;

            //number of pixels painted
            int pixel_painted = 0;

            //backward-wrapping
            for(int j=0; j<tw; j++)
                for(int k=0; k<th; k++)
                {
                    //set pixel coords
                    int set_x = tstartx + j; 
                    int set_y = tstarty + k;

                    //only set pixels inside the leaf area
                    QRgb leaf_color = _tiled_texture.pixel(set_x, set_y);
                    if(qAlpha(leaf_color) != 0) //if not fully transparent
                    {
                        //osg::Vec3 dir = Transformer::mult_vec(H, osg::Vec3(j, k, 1.0f));
                        osg::Vec3 P = bs + bfs*(float(j)/tw) + bas*(float(k)/th);
                        osg::Vec2 dir = lp.project(P + sp_cg, v, false, true);
                        float fx = dir.x();
                        float fy = dir.y();

                        //prepare for bilinear interpolation
                        float px = fx - floor(fx);
                        float py = fy - floor(fy);
                        int ix = (int)floor(fx);
                        int iy = (int)floor(fy);

                        if(ix < 0 || ix >= sw-1 || iy < 0 || iy >= sh-1)
                            continue;

                        //only use pixels inside the mask
                        QRgb seg_color = src_seg.pixel(ix, iy);
                        if(true || qRed(seg_color) != 0 || qGreen(seg_color) != 0 || qBlue(seg_color) != 0)
                        {
                            //color at 4 corners 
                            QRgb ptr00 = src_img.pixel(ix, iy);
                            QRgb ptr01 = src_img.pixel(ix, iy + 1);
                            QRgb ptr10 = src_img.pixel(ix + 1, iy);
                            QRgb ptr11 = src_img.pixel(ix + 1, iy + 1);

                            //bilinear interpolations
                            unsigned char r, g, b;
                            r = Transformer::bilinear(px, py, qRed(ptr00), qRed(ptr01), qRed(ptr10), qRed(ptr11));
                            g = Transformer::bilinear(px, py, qGreen(ptr00), qGreen(ptr01), qGreen(ptr10), qGreen(ptr11));
                            b = Transformer::bilinear(px, py, qBlue(ptr00), qBlue(ptr01), qBlue(ptr10), qBlue(ptr11));

                            //blend or not is subject to data preference
                            QRgb blend;
                            //blend = qRgb(r, g, b);

                            //blend source and target
                            if(v == 0)
                            {
                                //blend with generic leaf
                                int br = (1.0f * r + 0.0f * qRed(leaf_color));
                                int bg = (1.0f * g + 0.0f * qGreen(leaf_color));
                                int bb = (1.0f * b + 0.0f * qBlue(leaf_color));
                                blend = qRgb(br, bg, bb);
                            }
                            else
                            {
                                //blend with previous view
                                int br = (1.0f * r + 0.0f * qRed(leaf_color));
                                int bg = (1.0f * g + 0.0f * qGreen(leaf_color));
                                int bb = (1.0f * b + 0.0f * qBlue(leaf_color));
                                blend = qRgb(br, bg, bb);
                            }

                            if(set_x >= 0 && set_x < w && set_y >=0 and set_y < h)
                            {
                                _tiled_texture.setPixel(set_x, set_y, blend);
                                pixel_painted++;
                            }
                        }
                    }
                    //store the maximum number of pixels to be painted on each leaf
                    if(v==0 && i==0)
                        max_paint_pixel++;
                }

            //update the statistics each leaf
            touched_stat[i] += pixel_painted;
        }//end for this target leaf
    }//end for this view

    for(unsigned int i=0; i<touched_stat.size(); i+=4)
        if(touched_stat[i] >= max_paint_pixel)
        {
            full_touched.push_back(i);//full_touched is sorted
            //printf("%d\n", touched_stat[i]);
        }
    //printf("full_touched(%d) max_paint_pixel(%d)\n", int(full_touched.size()), max_paint_pixel);

    //post-processing: copy a random full_touched leaf to each untouched one
    //untouched is defined as being filled by less than 0.05f, i.e. at least 0.95f has to be filled
    if(!full_touched.empty())
    {
        for(unsigned int i=0; i<touched_stat.size(); i+=4)
            if(touched_stat[i] < max_paint_pixel * 0.95)
            {
                //random
                //int src = full_touched[rand()%full_touched.size()];

                //find the closest index from full_touched
                int src = -1, min_diff = 0;
                for(unsigned int j=0; j<full_touched.size(); j++)
                {
                    int diff = abs(full_touched[j]-i);
                    if(src == -1 || diff < min_diff)
                    {
                        min_diff = diff;
                        src = j;
                    }
                    if(diff > min_diff)
                        break;
                }
                int des = i;

                //source
                osg::Vec2 as = _all_tex[src];
                osg::Vec2 bs = _all_tex[src+1];
                osg::Vec2 es = _all_tex[src+2];
                osg::Vec2 fs = _all_tex[src+3];

                as.x() *= w;
                bs.x() *= w;
                es.x() *= w;
                fs.x() *= w;

                as.y() = h - as.y() * h;
                bs.y() = h - bs.y() * h;
                es.y() = h - es.y() * h;
                fs.y() = h - fs.y() * h;

                int sstartx = int(bs.x());
                int sstarty = int(bs.y());

                //destination
                osg::Vec2 ad = _all_tex[des];
                osg::Vec2 bd = _all_tex[des+1];
                osg::Vec2 ed = _all_tex[des+2];
                osg::Vec2 fd = _all_tex[des+3];

                ad.x() *= w;
                bd.x() *= w;
                ed.x() *= w;
                fd.x() *= w;

                ad.y() = h - ad.y() * h;
                bd.y() = h - bd.y() * h;
                ed.y() = h - ed.y() * h;
                fd.y() = h - fd.y() * h;

                int dw = int(ed.x()) - int(ad.x());
                int dh = int(ad.y()) - int(bd.y());

                int dstartx = int(bd.x());
                int dstarty = int(bd.y());

                //copy
                for(int j=0; j<dw; j++)
                    for(int k=0; k<dh; k++)
                    {
                        //set pixel coords
                        int set_x = dstartx + j; 
                        int set_y = dstarty + k;

                        //get pixel coords
                        int get_x = sstartx + j;
                        int get_y = sstarty + k;

                        QRgb color = _tiled_texture.pixel(get_x, get_y);
                        //QRgb red = qRgba(255, 0, 0, qAlpha(color));//demo
                        _tiled_texture.setPixel(set_x, set_y, color);
                    }
            }//end each untouched
    }//end post-processing
}

void RealisticLeafGrower::grow_single_image(int w, int h, float scale, std::string isp0)
{
    ISPLoader loader;
    loader.load(isp0);
    grow_single_image(w, h, scale, loader._rootX, loader._rootY, loader._img_path, loader._seg_path);
}

QImage RealisticLeafGrower::tile_texture(std::string generic_path, int num_leaf, int w, int h, std::vector <osg::Vec2>& tex)
{
    QImage ret(w, h, QImage::Format_ARGB32);
	QImage generic_shape(generic_path.c_str());
	if(generic_shape.isNull())
	{
		printf("RealisticLeafGrower::tile_texture():generic_shape(%s) path error\n", generic_path.c_str());
		return ret;
    }

    //step 1: tiling maths and parameters
    int n_x, n_y; //number of leaves alonge x and y direction
    int g_w, g_h; //width and height of generic leaf
    float k; //scale multiplier from source generic to target leaf
    int l_w, l_h; //actual leaf width and height in big texture

    g_w = generic_shape.width();
    g_h = generic_shape.height();
    k = pow(float(w * h) / (g_w * g_h * num_leaf), 0.5);
    n_x = ceil(w / (k * g_w));
    n_y = ceil(h / (k * g_h));
    l_w = w / n_x; //integer division to get a smaller size
    l_h = h / n_y; //aspect ratio may have been changed a little bit by round-off error

    //step 2: fill the big texture with generic leaves regularly
    int cnt = 0;
    ret.fill(0);
    for(int i=0; i<n_x; i++)
        for(int j=0; j<n_y; j++)
        {
            int c_x = i * l_w, c_y = j * l_h; //left-top corner of each leaf

            for(int k=0; k<l_w; k++)
                for(int l=0; l<l_h; l++)
                {
                    int source_x = int(float(k) / l_w * g_w); //source pixel coords
                    int source_y = int(float(l) / l_h * g_h);

                    QRgb color = generic_shape.pixel(source_x, source_y);

                    int set_x = c_x + k; //set pixel coords
                    int set_y = c_y + l;

                    ret.setPixel(set_x, set_y, color);
                }

            cnt++;
            if(cnt == num_leaf)
                goto Break1;
        }

    Break1:
    //step 3: reset all the texture coords, in order (0,0),(0,1),(1,0),(1,1)
    std::vector <osg::Vec2> all_tex;
    cnt = 0;
    for(int i=0; i<n_x; i++)
        for(int j=0; j<n_y; j++)
        {
            int c_x = i * l_w, c_y = j * l_h; //left-top corner of each leaf

            osg::Vec2 a(c_x, h - (c_y + l_h));
            osg::Vec2 b(c_x, h - c_y);
            osg::Vec2 e(c_x + l_w, h - (c_y + l_h));
            osg::Vec2 f(c_x + l_w, h - c_y);

            a.x() *= 1.0f/w;
            b.x() *= 1.0f/w;
            e.x() *= 1.0f/w;
            f.x() *= 1.0f/w;

            a.y() *= 1.0f/h;
            b.y() *= 1.0f/h;
            e.y() *= 1.0f/h;
            f.y() *= 1.0f/h;

            all_tex.push_back(a);
            all_tex.push_back(b);
            all_tex.push_back(e);
            all_tex.push_back(f);

            cnt++;
            if(cnt == num_leaf)
                goto Break2;
            //printf("a(%f %f) b(%f %f) e(%f %f) f(%f %f)\n", a.x(), a.y(), b.x(), b.y(), e.x(), e.y(), f.x(), f.y());
        }
    Break2:
    if(int(all_tex.size()) == num_leaf * 4)
        tex = all_tex;

    //debug
    //printf("g_w(%d) g_h(%d) k(%f) n_x(%d) n_y(%d) l_w(%d) l_h(%d)\n", g_w, g_h, k, n_x, n_y, l_w, l_h);
    //printf("num_leaf(%d) all_tex(%d)\n", num_leaf, int(all_tex.size()));
    //ret.save(QString("/tmp/debug_tile_texture.png"), "PNG", 100);

    return ret;
}

std::vector <osg::Vec2> RealisticLeafGrower::tex_coord_from_vertex(std::vector <osg::Vec3> all_v, osg::Vec3 cg, float scale, int root_x, int root_y, int sw, int sh)
{
    std::vector <osg::Vec2> ret;
    //check
    if(all_v.empty() || scale <= 0.0f || sw <= 0 || sh <= 0 || root_x < 0 || root_x >= sw || root_y < 0 || root_y >= sh)
        return ret;

    std::vector <osg::Vec3> ABEF(4, osg::Vec3(0, 0, 0));//planar leaves

    //for each vertex, find its texture coordinates
    for(unsigned int i=0; i<all_v.size(); i++)
    {
        osg::Vec3 v = all_v[i];

        //find which quadrant does it lay
        osg::Vec3 normal = v - cg;
        int region = Transformer::which_quadrant(normal);

        //for rotation
        osg::Matrixf R;
        

        //actually no need to worry about aliasing, because it's H to a new texture, this even solves the transparency problem
        if(false)
        {
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
        }
        else
            R.makeIdentity();

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
                ABEF[j] = (ABEF[j]-cg) * R + cg;

                //convert from object space to image space
                float qX = ABEF[j].x(), qY = ABEF[j].z();
                qY *= -1;
                
                qX /= scale;
                qY /= scale;
                qX += root_x;
                qY += root_y;

                //further transform it to real texture space, for texture mapping, origin is at bottomLeft
                qY = sh - qY;

                //normalized it
                osg::Vec2 tex_coord(qX / sw, qY / sh);

                ret.push_back(tex_coord);
            }
        }
    }

    //debug
    //for(unsigned int i=0; i<ret.size(); i++)
    //    printf("%f %f\n", ret[i].x(), ret[i].y());

    return ret;
}

std::string RealisticLeafGrower::save(std::string path)
{
    std::string ret;
	if(path.empty())
		return ret;

    //double side?
    //if(double_side)
    //    double_side_it(v, n, tex);

    //check data before saving
    if(_all_v.size() != _all_tex.size() || _generic_texure.empty())
    {
        printf("GenericLeafGrower::save():_all_v.size(%d) _all_tex.size(%d) _generic_texure(%s) error\n", int(_all_v.size()), int(_all_tex.size()), _generic_texure.c_str());
        return ret;
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

    //debug for _all_tex_isp0
    //for(unsigned int i=0; i<_all_tex_isp0.size(); i++)
    //    fprintf(out, "%f %f\n", _all_tex_isp0[i].x(), _all_tex_isp0[i].y());

    //file path of the generic texture
    std::string name = path + "_texture.png";
    fprintf(out, "%s\n", name.c_str());

    fclose(out);

    _tiled_texture.save(QString(name.c_str()), "PNG", 50);

    return name;
}

std::string RealisticLeafGrower::load(std::string path)
{
    std::string ret("");

	if(path.empty())
		return ret;

    std::ifstream fs(path.c_str());
    std::string s;

	int total;
    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &total);

	//must be equal to multiple of 4
	//this may happen if one carelessly erases individual vertex instead of a quad automatically
	if(total%4 != 0)
		return ret;

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

	//infer leaf position by a and e
	osg::Vec3 a, e;
	for(int i=0; i<quad; i++)
	{
		a = _all_v[i*4];
		e = _all_v[i*4+2];

        osg::Vec3 front = a*54.0f/128.0f + e*74.0f/128.0f;
		_all_pos.push_back(front);
	}

	//load the texture-coordinates
	//<0,0> <0,1> <1,0> <1,1>
	for(int i=0; i<total; i++)
	{
        float u, v;

        getline(fs, s);
        sscanf(s.c_str(), "%f %f\n", &u, &v);
        _all_tex.push_back(osg::Vec2(u, v));
	}

    //read and return the path of texture
    getline(fs, s);
    if(!s.empty())
    {
       if(s[s.length()-1] == '\n')
           s.erase(s.length()-1);
       ret = s;
    }

	fs.close();
    return ret;
}

/*
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
		//QRgb color = img.pixel(QPoint(bi_texs[k].x(), bi_texs[k].y()));
		//texture.setPixel(p, color);

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
*/

/*
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
*/

/*
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
*/
