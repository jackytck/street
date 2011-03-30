#include "LaserPruner.h"
#include <fstream>
#include "Transformer.h"
#include <highgui.h>
#include <queue>

LaserPruner::LaserPruner(): _verbose(false), _left(-1.0f), _right(-1.0f), _top(-1.0f), _bottom(-1.0f), _in(-1.0f), _out(-1.0f), _sd1(-1.0f), _pruner_cube_step(20), _ray_trace_step(-1.0f)
{
}

LaserPruner::LaserPruner(bool verbose): _verbose(verbose), _left(-1.0f), _right(-1.0f), _top(-1.0f), _bottom(-1.0f), _in(-1.0f), _out(-1.0f), _sd1(-1.0f), _pruner_cube_step(20), _ray_trace_step(-1.0f)
{
}

LaserPruner::~LaserPruner()
{
    if(_projector.is_loaded())
        for(unsigned int i=0; i<_segmentations.size(); i++)
            cvReleaseImage(&_segmentations[i]);
}

void LaserPruner::clear()
{
    _surface_pts.clear();
    _voxel_dict.clear();
    _voxel_dict_supported.clear();
    _raw_pts.clear();
    _pts_img.clear();
    _pts_uv.clear();
    for(unsigned int i=0; i<_segmentations.size(); i++)
        cvReleaseImage(&_segmentations[i]);
    _segmentations.clear();
}

void LaserPruner::setup(std::string sp_path, std::string cam_path)
{
    std::ifstream fs(sp_path.c_str());
    std::string s;
    float cg_x, cg_y, cg_z;

    //original cg, for projector
    getline(fs, s);
    sscanf(s.c_str(), "%f %f %f\n", &cg_x, &cg_y, &cg_z);
    _sp_cg = osg::Vec3(cg_x, cg_y, cg_z);

    //num of surface points
    int n;
    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &n);

    for(int i=0; i<n; i++)
    {
        float x, y, z;
        getline(fs, s);
        sscanf(s.c_str(), "%f %f %f\n", &x, &y, &z);
        _raw_pts.push_back(osg::Vec3(x, y, z));
    }

    //n parameters <img_id, u, v>
    for(int i=0; i<n; i++)
    {
        int x, y, z;
        getline(fs, s);
        sscanf(s.c_str(), "%d %d %d\n", &x, &y, &z);
        _pts_img.push_back(x);
        _pts_uv.push_back(osg::Vec2(y, z));
    }
    fs.close();

    //setup a LaserProjector for projecting 3D points for me
    if(_projector.setup(cam_path))
    {
        for(unsigned int i=0; i<_projector._view_seg.size(); i++)
        {
            std::string seg_path = _projector._view_seg[i];
            IplImage *seg = cvLoadImage(seg_path.c_str(), 0);//0 == CV_LOAD_IMAGE_GRAYSCALE
            if(!seg)
                printf("LaserPruner::setup: Error loading %s.\n", seg_path.c_str());
            else
                _segmentations.push_back(seg);
        }
    }
    else
        printf("cam_path(%s) error\n", cam_path.c_str());

    //mirror points
    //mirror();
    set_surface_pts();
    _cg = find_cg(_surface_pts);
    if(cloud_metrics(_surface_pts))
        setup_cube_pruner();

    //debug on one point
    //printf("_voxel(%f %f %f) max_dist(%f)\n", _voxel.x(), _voxel.y(), _voxel.z(), _voxel.length());
    //osg::Vec3 probe = osg::Vec3(-1.3f, -2.1f, 3.2f);
    //std::vector <int> collide = _voxel_dict[key(probe)];
    //for(unsigned int i=0; i<collide.size(); i++)
    //{
    //    osg::Vec3 p = _surface_pts[collide[i]];
    //    printf("collide(%f %f %f) dist(%f)\n", p.x(), p.y(), p.z(), (probe-p).length());
    //}
}

bool LaserPruner::is_loaded()
{
    bool ret = false;
    if(!_raw_pts.empty() && _raw_pts.size() == _pts_img.size() && _pts_img.size() == _pts_uv.size() && _projector.is_loaded())
        ret = true;
    return ret;
}

bool LaserPruner::cloud_metrics(std::vector <osg::Vec3> data)
{
    bool ret = false;

    if(is_loaded())
    {
        _left = -1.0f;
        _right = -1.0f;
        _top = -1.0f;
        _bottom = -1.0f;
        _in = -1.0f;
        _out = -1.0f;

        for(unsigned int i=0; i<data.size(); i++)
        {
            osg::Vec3 p = data[i];
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

        if(_left!=-1.0f && _right!=-1.0f && _top!=-1.0f && _bottom!=-1.0f && _in!=-1.0f && _out!=-1.0f)
           ret = true; 

        //further pushed away a little bit
        _left *= 1.1f;
        _right *= 1.1f;
        _top *= 1.1f;
        _bottom *= 0.9f;
        _in *= 1.1f;
        _out *= 1.1f;
        
        //debug
        //printf("_left(%f) _right(%f) _top(%f) _bottom(%f) _in(%f) _out(%f)\n", _left, _right, _top, _bottom, _in, _out);
    }

    return ret;
}

void LaserPruner::find_sd()
{
    if(is_loaded())
    {
        osg::Vec3 cg = find_cg(_raw_pts);
        float sum = 0.0f;
        int cnt = 0;
        for(unsigned int i=0; i<_raw_pts.size(); i++)
        {
            osg::Vec3 p = _raw_pts[i];
            if(p.x() < cg.x())
            {
                sum += pow(p.x() - cg.x(), 2);
                cnt++;
            }
        }
        sum /= cnt;
        _sd1 = cg.x() - pow(sum, 0.5);

        //debug
        //printf("sd1(%f)\n", _sd1);
    }
}

void LaserPruner::setup_cube_pruner()
{
    //a. find the left-out-bottom-most corner and other metircs
    _origin = osg::Vec3(_left, _out, _bottom);
    float w = _right - _left;
    float h = _top - _bottom;
    float d = _in - _out;
    if(w == 0.0f || h == 0.0f || d == 0.0f)
        printf("LaserPruner::setup_cube_pruner():step size error\n");
    _voxel = osg::Vec3(w/_pruner_cube_step, d/_pruner_cube_step, h/_pruner_cube_step);
    _ray_trace_step = _voxel.length() / 2.0f;
    if(_ray_trace_step <= 0.0f)
        printf("LaserPruner::setup_cube_pruner():ray trace step size error\n");

    //b. initialize the _voxel_dict by associating an empty list to every voxel (_pruner_cube_step ** 3)
    // key ranges from [0,(_pruner_cube_step+1)^3]
    int upper = _pruner_cube_step+1;
    upper *= upper * upper;
    _upper = upper;
    for(int i=0; i<=upper; i++)
    {
        std::vector <int> empty;
        _voxel_dict[i] = empty;
        _voxel_dict_supported[i] = false;
    }

    //c. add each cloud point into the respective vector
    //at this point, key() is valid just in time
    for(unsigned int i=0; i<_surface_pts.size(); i++)
        _voxel_dict[key(_surface_pts[i])].push_back(i);

    //debug
    //printf("_voxel(%f %f %f)\n", _voxel.x(), _voxel.y(), _voxel.z());
}

int LaserPruner::key(osg::Vec3 p)
{
    int ret = -1;

    int p1 = _pruner_cube_step + 1;
    int p2 = p1 * p1;

    //a. check bounds
    if(p.x()>=_left && p.x()<=_right && p.y()>=_out && p.y()<=_in && p.z()>=_bottom && p.z()<=_top)
    {
        osg::Vec3 d = p - _origin;
        //eacg digit ranges from [0,_pruner_cube_step]
        d.x() /= _voxel.x();
        d.y() /= _voxel.y();
        d.z() /= _voxel.z();

        ret = p2 * round(d.x()) + p1 * round(d.y()) + round(d.z());

        //debug
        //printf("p(%f %f %f) d(%f %f %f) key(%d)\n", p.x(), p.y(), p.z(), d.x(), d.y(), d.z(), ret);
    }

    return ret;
}

void LaserPruner::set_support(osg::Vec3 p, float scale)
{
    _voxel_dict_supported[key(p)] = true;

    //too big
    /*
    if(false)
        for(int x=-effect; x<effect; x++)
            for(int y=-effect; y<effect; y++)
                for(int z=-effect; z<effect; z++)
                {
                    osg::Vec3 neighbor = p + osg::Vec3(x*_voxel.x(), y*_voxel.y(), z*_voxel.z());
                    _voxel_dict_supported[key(neighbor)] = true;
                }
    */

    if(true)
    {
        //a cross
        osg::Vec3 neighbor;
        float s = scale;
        neighbor = p + osg::Vec3(-s*_voxel.x(), 0*_voxel.y(), 0*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
        neighbor = p + osg::Vec3(s*_voxel.x(), 0*_voxel.y(), 0*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
        neighbor = p + osg::Vec3(0*_voxel.x(), s*_voxel.y(), 0*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
        neighbor = p + osg::Vec3(0*_voxel.x(), -s*_voxel.y(), 0*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
        neighbor = p + osg::Vec3(0*_voxel.x(), 0*_voxel.y(), s*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
        neighbor = p + osg::Vec3(0*_voxel.x(), 0*_voxel.y(), -s*_voxel.z());
        _voxel_dict_supported[key(neighbor)] = true;
    }
}

void LaserPruner::back_trace_prune(BDLSkeletonNode *root)
{
    if(!root)
        return;

    //1. find the cg of tree, plus extracting the potential leaves
    osg::Vec3 cg(0.0f, 0.0f, 0.0f);
    std::vector <BDLSkeletonNode *> trees = Transformer::bfs(root);
    std::vector <BDLSkeletonNode *> leaves;

    if(trees.empty())
        return;
    for(unsigned int i=0; i<trees.size(); i++)
    {
        BDLSkeletonNode *node = trees[i];
        
        //sum all coords values
        cg.x() += node->_sx;
        cg.y() += node->_sy;
        cg.z() += node->_sz;

        //put leaves
        if(node->_children.empty() && node->_prev)
            leaves.push_back(node);
    }
    cg /= trees.size();
    //debug
    //printf("cg(%f %f %f)\n", cg.x(), cg.y(), cg.z());

    //2. for each leaf, ray-trace back to a point either
    //a. the central core zone is hit or
    //b. a non-empty voxel is hit
    std::vector <BDLSkeletonNode *> pending_delete;
    std::vector <osg::Vec3> pending_add;//size of add should be equal to delete
    std::map <BDLSkeletonNode *, bool> is_pushed;//pending_delete may have duplicates
    for(unsigned int i=0; i<leaves.size(); i++)
    {
        BDLSkeletonNode *cur = leaves[i];
        //tranverse down to the direct neighbors of root
        while(cur && cur->_prev)
        {
            osg::Vec3 node = Transformer::toVec3(cur);
            //a1. test if node has reached the safe zone, i.e. the central core
            osg::Vec3 diff = node-cg;
            //if(diff.x()*diff.x() + diff.y()*diff.y() < (_voxel.x()*_voxel.x() + _voxel.y()*_voxel.y()) * 1.5f)
            if(diff.x()*diff.x() + diff.y()*diff.y() < (_voxel.x()*_voxel.x() + _voxel.y()*_voxel.y()) * 1.0f)
                goto NEXTLEAF;

            //a2. or has reached a safe generation
            //if(cur->_generation < 6)
            if(cur->_generation < 3)
                goto NEXTLEAF;

            //b. ray-trace if segment node <--> prev, hit a non-empty voxel
            osg::Vec3 prev = Transformer::toVec3(cur->_prev);
            osg::Vec3 ray = prev - node;
            float ray_len = ray.length();
            ray.normalize();
            int steps = ray_len / _ray_trace_step;

            for(int j=0; j<steps; j++)
            {
                osg::Vec3 probe = node + ray * j * _ray_trace_step;//probe does not include prev because prev will be queried in next loop
                int key_p = key(probe);

                //make a hit
                if(key_p != -1 && !_voxel_dict[key_p].empty())
                {
                    //but don't want to be too close to prev
                    if((prev - probe).length() > ray_len * 0.2f)
                    {
                        //to prevent duplicate
                        if(is_pushed.find(cur) == is_pushed.end())
                        {
                            //book-keeping
                            pending_delete.push_back(cur);//cur and its subtree will be deleted
                            pending_add.push_back(probe);//probe will be added to cur->_prev
                            is_pushed[cur] = true;
                            goto NEXTLEAF;
                        }
                    }
                }
            }
            //next node
            cur = cur->_prev;
        }
        NEXTLEAF:
            i = i;//dummy
    }

    //debug
    //printf("pending_delete(%d) pending_add(%d)\n", int(pending_delete.size()), int(pending_add.size()));

    //3. delete each pending_delete node and add back the corresponding pending_add node
    //from largest generation to the lowest
    std::vector <bool> done(pending_delete.size(), false);
    int done_cnt = 0;
    std::vector <BDLSkeletonNode *> ordered_delete;//the first one should be deleted first

    while(done_cnt < int(pending_delete.size()))
    {
        int max_index = -1;//max hop index of all un-done nodes
        int max_gen = -1;
        for(unsigned int i=0; i<pending_delete.size(); i++)
        {
            if(done[i])
                continue;

            BDLSkeletonNode *cur = pending_delete[i];
            if(max_gen == -1 || cur->_generation > max_gen)
            {
                max_gen = cur->_generation;
                max_index = i;
            }
        }

        if(max_index != -1)
        {
            BDLSkeletonNode *d = pending_delete[max_index];
            osg::Vec3 a = pending_add[max_index];
            BDLSkeletonNode *new_node = new BDLSkeletonNode(a.x(), a.y(), a.z());

            //add
            new_node->_prev = d->_prev;
            d->_prev->_children.push_back(new_node);
            new_node->_mesh_done = true;

            //delete
            //BDLSkeletonNode::delete_this(d);

            //don't delete the subtree directly, it will overkill
            ordered_delete.push_back(d);
            while(d)
            {
                //protect it by setting its _mesh_done 
                d->_mesh_done = true;//_mesh_done here means is_protected?
                d = d->_prev;
            }

            done[max_index] = true;
            done_cnt++;
        }
    }

    //4. bfs to delete any subtree under d and is not under protection
    for(unsigned int pi=0; pi<ordered_delete.size(); pi++)
    {
        BDLSkeletonNode *d = ordered_delete[pi];

        //bfs until a safe point to call BDLSkeletonNode::delete_this()
        std::queue <BDLSkeletonNode *> Queue;
        Queue.push(d);

        while(!Queue.empty())
        {
            BDLSkeletonNode *front = Queue.front();
            Queue.pop();

            if(front->_mesh_done)
            {
                for(unsigned int i=0; i<front->_children.size(); i++)
                    Queue.push(front->_children[i]);
            }
            else
                BDLSkeletonNode::delete_this(front);
        }
    }
}

void LaserPruner::mirror()
{
    if(is_loaded())
    {
        find_sd();
        //mirror the points at the left-plane, i.e. x = _sd1, (y and z remain the same)
        for(unsigned int i=0; i<_raw_pts.size(); i++)
        {
            osg::Vec3 p = _raw_pts[i];
            if(p.x() > _sd1)
            {
                osg::Vec3 mp(_sd1-p.x()+_sd1, p.y(), p.z());
                _surface_pts.push_back(p);
                _surface_pts.push_back(mp);
            }
            else
                _surface_pts.push_back(p);
        }

        //printf("_sd1(%f)\n", _sd1);

        //debug
        //printf("%d\n", int(_surface_pts.size()));
        //for(unsigned int i=0; i<_surface_pts.size(); i++)
        //{
        //    osg::Vec3 p = _surface_pts[i];
        //    printf("v %f %f %f\n", p.x(), p.y(), p.z());
        //}
    }
}

void LaserPruner::set_surface_pts()
{
    if(is_loaded())
    {
        _surface_pts.clear();
        for(unsigned int i=0; i<_raw_pts.size(); i++)
        {
            osg::Vec3 p = _raw_pts[i];
            _surface_pts.push_back(p);
        }
    }
}

osg::Vec3 LaserPruner::find_cg(std::vector <osg::Vec3> data)
{
    osg::Vec3 ret(0.0f, 0.0f, 0.0f);

    if(is_loaded())
    {
        for(unsigned int i=0; i<data.size(); i++)
            ret += data[i];
        ret /= data.size();
    }

    //debug
    //printf("cg(%f %f %f\n", ret.x(), ret.y(), ret.z());

    return ret;
}

bool LaserPruner::is_inside(osg::Vec3 node, osg::Vec3 prev, osg::Vec3& hit, bool& hit_valid)
{
    //to determine if the line is inside the volume, 3 necessary conditions are checked, (but not sufficiently obviously)
    bool condition_a = true;
    bool condition_b = false;
    bool condition_c = true;

    //a1. check if the key exists
    int key_a = key(node);
    int key_b = key(prev);
    if(key_a == -1 || key_b == -1)
        condition_a = false;

    //a2. since it is prepared for backward growing, the volume could be made smaller
    float frac = 0.85f, top_frac = 0.95f;
    if(!(node.x()>=_left*frac && node.x()<=_right*frac && node.y()>=_out*frac && node.y()<=_in*frac && node.z()>=_bottom*frac && node.z()<=_top*top_frac))
        condition_a = false;

    if(key_b != -1) //even if condition_a is false, we also want to find the hit point
    {
        //b. check if projected node is inside any one of the view
        for(unsigned int i=0; i<_segmentations.size(); i++)
        {
            osg::Vec2 p = _projector.project(node + _sp_cg, i, false, true);
            if(is_inside_mask(i, p.x(), p.y()))
            {
                condition_b = true;
                break;
            }
        }

        if(condition_b)
        {
            //c. check if the ray from node to prev crosses the boundary, (assume prev is inside)
            //NOTE: this is too unstable
            osg::Vec3 ray = prev - node;
            float ray_len = ray.length();
            ray.normalize();
            int steps = ray_len / _ray_trace_step;

            bool ever_hit = false;
            for(int i=0; i<steps; i++)
            {
                osg::Vec3 probe = node + ray * i * _ray_trace_step;//probe does not include prev because prev is inside
                int key_p = key(probe);
                if(key_p != -1 && !_voxel_dict[key_p].empty())
                {
                    condition_c = false;
                    hit = probe;
                    ever_hit = true;
                    if((prev - probe).length() > ray_len * 0.2f)
                        hit_valid = true;
                    break;
                }
            }

            //if it is not ever hit, we must also need to infer one hit point
            if(!ever_hit)
            {
                hit = prev - ray * ray_len * 0.2f; //value is hard-coded only
                hit_valid = false;
            }
        }
        //may also update the hit point by segmentation
        else
        {
        }
    }
    else
    {
        //printf("LaserPruner::is_inside(): _cg is outside the bound error\n");
    }

    //debug
    //printf("condition_a(%d) condition_b(%d) condition_c(%d)\n", condition_a, condition_b, condition_c);

    //return condition_a && condition_b && condition_c;
    return condition_a && condition_b;//condition_c may be too strong
}

bool LaserPruner::is_inside(osg::Vec3 q, osg::Vec3& hit, bool& hit_valid)
{
    return is_inside(q, _cg, hit, hit_valid);
}

bool LaserPruner::is_inside(osg::Vec3 q)
{
    osg::Vec3 hit;
    bool hit_valid;
    return is_inside(q, _cg, hit, hit_valid);
}

int LaserPruner::width(int index)
{
    int ret = -1;
    IplImage *seg = _segmentations[index];

    if(seg)
        ret = seg->width;

    return ret;
}

int LaserPruner::height(int index)
{
    int ret = -1;
    IplImage *seg = _segmentations[index];

    if(seg)
        ret = seg->height;

    return ret;
}

uchar LaserPruner::read(int index, int x, int y)
{
    IplImage *seg = _segmentations[index];
    uchar ret = 0;
    if(seg && x >= 0 && x < width(index) && y >= 0 && y < height(index))
        ret = ((uchar *)(seg->imageData + y*seg->widthStep))[x];
    
    return ret;
}

bool LaserPruner::is_inside_mask(int index, float x, float y, bool lenient)
{
    bool ret = false;

    //use probe to check the value in _segmentation
    //less strict: use k-neighbours to test, instead of one
	int k = 1;

    for(int i=-k; i<=k; i++)
        for(int j=-k; j<=k; j++)
            if(read(index, x+i, y+j) != 0)
			{
                ret = true;
				break;
			}

	if(!ret && lenient)
	{
		int k = 20;
		for(int i=-k; i<=k; i++)
			for(int j=-k; j<=k; j++)
				if(read(index, x+i, y+j) != 0)
					return true;
	}

    return ret;
}

//1. project each raw point to each view to see if it's inside at least one mask
//2. if no, then return it 
//3. refine the points
//4. and save the foliage into a new file
std::vector <osg::Vec3> LaserPruner::segment_initial_branch(std::string foliage_path)
{
    std::vector <osg::Vec3> ret, foliage;
    std::vector <osg::Vec3> ret2, foliage2;//the thresholded points
    std::vector <int> pts_img, pts_img2;
    std::vector <osg::Vec2> pts_uv, pts_uv2;
    std::vector <int> ret_idx;//store the index where the ret is got from
    bool inside = false;

    for(unsigned int i=0; i<_raw_pts.size(); i++)
    {
        osg::Vec3 node = _raw_pts[i];
        inside = false;

        //1. check if projected node is inside any one of the view
        for(unsigned int s=0; s<_segmentations.size(); s++)
        {
            osg::Vec2 p = _projector.project(node + _sp_cg, s, false, true);
            if(is_inside_mask(s, p.x(), p.y()))
            {
                inside = true;
                break;
            }
        }

        //2. save the foliage and its parameters
        if(inside)
        {
            foliage.push_back(node);
            pts_img.push_back(_pts_img[i]);
            pts_uv.push_back(_pts_uv[i]);
        }
        else
        {
            ret.push_back(node);
            ret_idx.push_back(i);
        }
    }

    //3. find the thresholding line between cg and lowest point
    float cg_z = 0.0f, cg_y = 0.0f, lowest = -1.0f, rightest = -1.0f, leftest = -1.0f;
    float threshold_left, threshold_right, threshold;
    for(unsigned int i=0; i<foliage.size(); i++)
    {
        osg::Vec3 pt = foliage[i];
        float y = pt.y(), z = pt.z();
        cg_y += y;
        cg_z += z;

        if(lowest == -1.0f || z<lowest)
            lowest = z;
        if(rightest == -1.0f || y>rightest)
            rightest = y;
        if(leftest == -1.0f || y<leftest)
            leftest = y;
    }
    if(!foliage.empty())
    {
        cg_z /= foliage.size();
        cg_y /= foliage.size();
        threshold_left = (cg_y + leftest) / 2.0f;
        threshold_right = (cg_y + rightest) / 2.0f;
        threshold = (cg_z + lowest) / 2.0f;
        //printf("threshold_left(%f) threshold_right(%f)\n", threshold_left, threshold_right);
    }

    //do the thresholding
    //put points lower than the line threshold and within the left/right bound of cg_y to ret2
    for(unsigned int i=0; i<foliage.size(); i++)
    {
        osg::Vec3 pt = foliage[i];
        if(pt.z() < threshold && pt.y() > threshold_left && pt.y() < threshold_right)
            ret2.push_back(pt);
    }

    //put points higher than the line threshold to foliage2
    for(unsigned int i=0; i<ret.size(); i++)
    {
        osg::Vec3 pt = ret[i];
        if(pt.z() > threshold)
        {
            foliage2.push_back(pt);
            pts_img2.push_back(_pts_img[ret_idx[i]]);
            pts_uv2.push_back(_pts_uv[ret_idx[i]]);
        }
        else if(pt.y() > threshold_left && pt.y() < threshold_right)
            ret2.push_back(pt);
    }

    //4. save the foliage point into a new file
    FILE *out = fopen(foliage_path.c_str(), "w");

    //simple point cg
    fprintf(out, "%f %f %f\n", _sp_cg.x(), _sp_cg.y(), _sp_cg.z());

    //point size
    fprintf(out, "%d\n", int(foliage.size()));

    //foliage points
    for(unsigned int i=0; i<foliage.size(); i++)
    {
        osg::Vec3 pt = foliage[i];
        fprintf(out, "%f %f %f\n", pt.x(), pt.y(), pt.z());

        //debug
        //printf("v %f %f %f\n", pt.x(), pt.y(), pt.z());
    }

    //thresholded foliage points;
    for(unsigned int i=0; i<foliage2.size(); i++)
    {
        osg::Vec3 pt = foliage2[i];
        fprintf(out, "%f %f %f\n", pt.x(), pt.y(), pt.z());

        //debug
        //printf("v %f %f %f\n", pt.x(), pt.y(), pt.z());
    }

    //paras
    for(unsigned int i=0; i<pts_img.size(); i++)
        fprintf(out, "%d %d %d\n", pts_img[i], int(pts_uv[i].x()), int(pts_uv[i].y()));

    //thresholded paras
    for(unsigned int i=0; i<pts_img2.size(); i++)
        fprintf(out, "%d %d %d\n", pts_img2[i], int(pts_uv2[i].x()), int(pts_uv2[i].y()));

    fclose(out);

    //debug
    //for(unsigned int i=0; i<ret2.size(); i++)
    //    printf("v %f %f %f\n", ret2[i].x(), ret2[i].y(), ret2[i].z());

    return ret2;
}

//segment the initial branch by plane sweep
//1. from lowest to highest points, pack different points into diffferent height bins
//2. for each bin, find the x, y min and max ranges, and get its area
//3. find a threshold where there is a sudden increase in the area
//4. segment according to this line
//5. find the center of foliage for extending the end-points of initial skeleton(done in LaserSkeletonGrower)
//6. mirror the foliage over the root point of initial
//7. save the initial
std::vector <osg::Vec3> LaserPruner::segment_initial_branch_plane_sweep(std::vector <osg::Vec3>& foliage, std::vector <int>& pts_img, std::vector <osg::Vec2>& pts_uv, osg::Vec3& foliage_center, float& init_threshold)
{
    std::vector <osg::Vec3> ret;
    foliage.clear();
    pts_img.clear();
    pts_uv.clear();

    //pre-checking
    if(_raw_pts.empty())
    {
        printf("LaserPruner::segment_initial_branch_plane_sweep():_raw_pts empty error\n");
        return ret;
    }

    //1. pack points into different height bins
    //1a. find min and max height
    float min_h = -1.0f, max_h = -1.0f;
    for(unsigned int i=0; i<_raw_pts.size(); i++)
    {
        osg::Vec3 node = _raw_pts[i];
        float z = node.z();

        if(min_h == -1.0f || z < min_h)
            min_h = z;
        if(max_h == -1.0f || z > max_h)
            max_h = z;
    }

    //1b. find the bins by dividing the bins into 20 steps
    float bin_size = (max_h - min_h) / 15.0f;
    std::vector <int> bins(_raw_pts.size(), 0);//the i-th node goes to height_bins[bins[i]]
    for(unsigned int i=0; i<_raw_pts.size(); i++)
    {
        float z = _raw_pts[i].z();
        z -= min_h;
        int step = z / bin_size;
        bins[i] = step;
    }

    //1c. store the bins into a 2D vector, by index
    int max_bin = (max_h-min_h) / bin_size;
    std::vector <std::vector <int> > height_bins(max_bin+1, std::vector <int>());
    for(unsigned int i=0; i<bins.size(); i++)
        height_bins[bins[i]].push_back(i);

    //2. find the 2D bounding box area of each bins
    std::vector <float> areas(height_bins.size(), 0.0f);
    for(unsigned int i=0; i<height_bins.size(); i++)
    {
        std::vector <int> bin_idxs = height_bins[i];
        float minX, maxX, minY, maxY;

        for(unsigned j=0; j<bin_idxs.size(); j++)
        {
            int index = bin_idxs[j];
            osg::Vec3 pt = _raw_pts[index];
            float x = pt.x(), y = pt.y();

            if(j == 0)
            {
                minX = x;
                maxX = minX;
                minY = y;
                maxY = minY;
            }
            else
            {
                if(x < minX)
                    minX = x;
                if(x > maxX)
                    maxX = x;
                if(y < minY)
                    minY = y;
                if(y > maxY)
                    maxY = y;
            }
        }

        //2d area
        float area = (maxX - minX) * (maxY - minY);
        areas[i] = area;
    }

    //debug 2
    for(unsigned int i=0; i<areas.size(); i++)
        printf("%d: height(%f) area(%f)\n", i, min_h+bin_size*(i+0.5f), areas[i]);

    //3. if the area increased to 30 times more than the average, then the threshold is at that bin
    //3a. find the average of the first 4 bins
    float average = 0.0f;
    for(int i=0; i<4; i++)
        average += areas[i];
    average /= 4.0f;
    printf("average(%f)\n", average);

    //3b. check the percentage increase
    float threshold_foliage = -1.0f, threshold_init;
    for(unsigned int i=4; i<areas.size(); i++)
    {
        float a = areas[i];
        if(a > average)
        {
            float times = (a - average) / average;
            printf("%d: times(%f)\n", i, times);
            //if(a < 2.0f)//tree5
            if(a < 0.5f)//tree19
                continue;

            //if(times > 15.0f && times < 30.0f)//tree1
            //if(threshold_foliage == -1.0f && times > 3.0f && times < 30.0f)//tree2
            //{
            //    threshold_foliage = min_h + bin_size * (i+0.5f);
            //    continue;
            //}

            //if(times > 30.0f)//tree1
            //if(times > 5.0f)//tree2
            if(times > 3.0f)//tree4
            {
                //threshold_init = min_h + bin_size * (i+0.5f);//tree6,16
                threshold_init = min_h + bin_size * (i+1.5f);//tree5,8,17
                //threshold_init = min_h + bin_size * (i+2.0f);//tree10
                threshold_foliage = min_h + bin_size * (i-0.5f);//tree5
                break;
            }
        }
    }
    //threshold_init = 10;//for bare tree or very big tree only 
    init_threshold = threshold_init;//return back for extending the end-points

    //debug 3
    //printf("average(%f) threshold_init(%f)\n", average, threshold_init);

    //4. segment according to the 'threshold'
    for(unsigned int i=0; i<_raw_pts.size(); i++)
    {
        osg::Vec3 node = _raw_pts[i];

        if(node.z() > threshold_foliage)
        {
            foliage.push_back(node);
            pts_img.push_back(_pts_img[i]);
            pts_uv.push_back(_pts_uv[i]);
        }

        if(node.z() < threshold_init)
            ret.push_back(node);
    }

    //5. find the center of foliage
    osg::Vec3 fol_c(0.0f, 0.0f, 0.0f);
    for(unsigned int i=0; i<foliage.size(); i++)
    {
        osg::Vec3 node = foliage[i];
        fol_c.x() += node.x();
        fol_c.y() += node.y();
        fol_c.z() += node.z();
    }
    fol_c /= foliage.size();
    foliage_center = fol_c;

    //debug
    //for(unsigned int i=0; i<ret.size(); i++)
    //    printf("v %f %f %f\n", ret2[i].x(), ret2[i].y(), ret2[i].z());

    return ret;
}

void LaserPruner::save_foliage(std::string foliage_path, std::vector <osg::Vec3> pts, std::vector <int> pts_img, std::vector <osg::Vec2> pts_uv)
{
    //1. save the foliage point into a new file
    FILE *out = fopen(foliage_path.c_str(), "w");

    //simple point cg
    fprintf(out, "%f %f %f\n", _sp_cg.x(), _sp_cg.y(), _sp_cg.z());

    //point size
    fprintf(out, "%d\n", int(pts.size()));

    //foliage points
    for(unsigned int i=0; i<pts.size(); i++)
    {
        osg::Vec3 pt = pts[i];
        fprintf(out, "%f %f %f\n", pt.x(), pt.y(), pt.z());

        //debug
        //printf("v %f %f %f\n", pt.x(), pt.y(), pt.z());
    }

    //paras
    for(unsigned int i=0; i<pts_img.size(); i++)
        fprintf(out, "%d %d %d\n", pts_img[i], int(pts_uv[i].x()), int(pts_uv[i].y()));

    fclose(out);
}
