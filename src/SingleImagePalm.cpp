#include "SingleImagePalm.h"
#include "ISPLoader.h"
#include <queue>
#include <set>
#include "Transformer.h"
#include "LineSegmentDetector.h"
#include "GenericLeafGrower.h"
#include "RealisticLeafGrower.h"

const bool ImageNode::operator < (const ImageNode& node) const
{
    return !(_dist < node._dist);
}

SingleImagePalm::SingleImagePalm(std::string isp0, std::string output): _verbose(false), _output_debug_img(false), _data_valid(false), _grow_valid(false), _img2skeleton_scale(250.0f)
{
    ISPLoader loader;
    loader.load(isp0);
    _input_img_path = loader._img_path;
    _input_seg_path = loader._seg_path;
    _output_dir = output;

    _img = QImage(_input_img_path.c_str());
    if(_img.isNull())
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%s) error\n", loader._img_path.c_str());
        return;
    }
    _seg = QImage(_input_seg_path.c_str());
    if(_seg.isNull())
    {
        printf("SingleImagePalm::SingleImagePalm():_seg(%s) error\n", loader._seg_path.c_str());
        return;
    }
    
    //check if both img and seg are of the same size
    int iw = _img.width();
    int ih = _img.height();
    int sw = _seg.width();
    int sh = _seg.height();

    if(iw != sw || ih != sh || iw <= 0 || ih <= 0)
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%d x %d):_seg(%d x %d) does not match error\n", iw, ih, sw, sh);
        return;
    }

    //at this point isp is valid
    _data_valid = true;

    //_img.save(QString("/tmp/test_tiff.png"), "PNG", 80);test if the qt plugin works

    //some initial values
    _isp0 = isp0;
    _debug_img = QImage(iw, ih, QImage::Format_ARGB32);
    _debug_img.fill(0);
    _edge_map = QImage(iw, ih, QImage::Format_ARGB32);
    _edge_map.fill(0);
    _edge_field = QImage(iw, ih, QImage::Format_ARGB32);
    _edge_field.fill(0);
    _w = iw;
    _h = ih;
    _max_dist = -1.0f;
    _max_kingdom = -1;
    _raw_skeleton = NULL;
    _skeleton = NULL;
    _blender_skeleton = NULL;
    _lower_foliage_y = -1;
    _higher_foliage_y = -1;
    _first_branching_node_idx = -1;
    _min_potential = -1;
    _max_potential = -1;
    _branch_id = 0;
}

SingleImagePalm::~SingleImagePalm()
{
    if(_output_debug_img)
    {
        std::string path = _output_dir + "/debug.png";
        if(_root.x() != -1 && _root.y() != -1)
        {
            //airbrush(_root.x(), _root.y());
            //printf("root(%d,%d)\n", int(_root.x()), int(_root.y()));
        }
        //visualize_dijkstra();
        //visualize_bin();
        //visualize_king();
        //visualize_linesweep();
        //visualize_branch_search_limit();
        //visualize_kingdom();
        //visualize_kingdom(false);
        visualize_edge(true);
        visualize_skeleton(_skeleton, true, true);

        //visualize_voting_space();
        //visualize_edge(true, &_edge_map);

        if(!_debug_img.isNull())
            _debug_img.save(QString(path.c_str()), "PNG", 70);

        //if(!_edge_map.isNull())
        //    _edge_map.save(QString("/tmp/debug_edge.png"), "PNG", 70);
        //if(!_edge_field.isNull())
        //    _edge_field.save(QString("/tmp/debug_edge_point.png"), "PNG", 70);

        printf("debug image is outputted at '%s'\n", path.c_str());
    }

    //delete all the bdl skeletons
    BDLSkeletonNode::delete_this(_raw_skeleton);
    BDLSkeletonNode::delete_this(_skeleton);
    BDLSkeletonNode::delete_this(_blender_skeleton);
}

void SingleImagePalm::setVerbose(bool debug, bool debug_image)
{
    _verbose = debug;
    _output_debug_img = debug_image;
}

void SingleImagePalm::growSkeleton()
{
    if(_data_valid)
    {
        if(findRoot())
        {
            //does not work
            //inferRawSkeleton();
            //extractMainBranch();
            lineSweep();
            inferBestTerminalNode();
            extractMainBranch2();
            dijkstra();
            assignBin();
            inferKingdom();
            produceLineEdge();
            //inferKingAvg();
            inferKingPotential();
            dqMainbranchKingdom();
            while(extractSingleSubBranch()) ;
            forceGrow();
            convertTo3D();
            _grow_valid = true;
        }
    }
}

void SingleImagePalm::growGenericLeaf(BDLSkeletonNode *root, std::string gleaf, float leaf_scale)
{
    GenericLeafGrower leaf_grower;
    leaf_grower.set_verbose(_verbose);
    leaf_grower.setup(root, gleaf, leaf_scale);
    leaf_grower.grow_palm3();

    std::string out_leaves = _output_dir + "/z_leaves";
    leaf_grower.save(out_leaves);
}

void SingleImagePalm::growRealisticLeaf(BDLSkeletonNode *root, std::string gleaf, float leaf_scale)
{
    GenericLeafGrower leaf_grower;
    leaf_grower.set_verbose(_verbose);
    leaf_grower.setup(root, gleaf, leaf_scale);
    leaf_grower.grow_palm3();

    RealisticLeafGrower real_grower;
    real_grower.set_verbose(_verbose);
    real_grower.setup(root, gleaf, leaf_scale);

    int w = 2048;//hard-code: width and height of giant texture
    real_grower.grow_single_palm(w, w, _img2skeleton_scale, _isp0, leaf_grower._all_v, leaf_grower._all_tex);

    std::string out_leaves = _output_dir + "/z_leaves";
    real_grower.save(out_leaves);//realistic texture path
}

void SingleImagePalm::airbrush(int x, int y, int w, int h, QColor color, QImage *img)
{
    QImage *board = &_debug_img;
    if(img)
        board = img;
    QPainter p;
    p.begin(board);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(color));
    p.drawEllipse(x-w/2, y-h/2, w, h);
    p.end();
}

void SingleImagePalm::drawLine(int x1, int y1, int x2, int y2, QColor color, int size, QImage *img)
{
    QImage *board = &_debug_img;
    if(img)
        board = img;

    QPainter p;
    p.begin(board);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(color, size));
    p.setBrush(Qt::NoBrush);
    p.drawLine(x1, y1, x2, y2);
    p.end();
}


bool SingleImagePalm::isInside(int x, int y)
{
    bool ret = false;
    if(!_seg.isNull() && x >= 0 && x < _w && y >= 0 && y < _h)
    {
        QRgb c = _seg.pixel(x, y);
        if(qRed(c) != 0 || qGreen(c) != 0 || qBlue(c) != 0) // if not fully black
            ret = true;
    }
    return ret;
}

bool SingleImagePalm::isEdge(int x, int y)
{
    bool ret = false;
    if(!_edge_map.isNull() && x >= 0 && x < _w && y >= 0 && y < _h)
    {
        QRgb c = _edge_map.pixel(x, y);
        if(qRed(c) != 0 || qGreen(c) != 0 || qBlue(c) != 0) // if not fully black
            ret = true;
    }
    return ret;
}

QColor SingleImagePalm::mapColor(float scalar)
{
    int h = int(round(scalar * 360)) % 360;
    int s = 255;
    int v = 255;
    return QColor::fromHsv(h, s, v);
}

bool SingleImagePalm::findRoot()
{
    int w = _seg.width(), h = _seg.height();
    for(int y=h-1; y>=0; y--)
		for(int x=0; x<w; x++)
		{
            if(isInside(x, y))
            {
                _root = osg::Vec2(x, y);
                printf("root(%d,%d)\n", x, y);
                return true;
            }
        }
    _root = osg::Vec2(-1, -1);
    return false;
}

void SingleImagePalm::setupChildren()
{
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            int px = int(n._prev.x());
            int py = int(n._prev.y());
            if(n._valid && px >= 0 && py >= 0)
                _nodes[px][py]._children.push_back(&_nodes[x][y]);
        }
}

void SingleImagePalm::dijkstra()
{
    printf("dijkstra...\n");
    std::vector <std::vector <ImageNode> > nodes(_w, std::vector <ImageNode> (_h, ImageNode(0, 0)));
    std::vector <std::vector <bool> > visited(_w, std::vector <bool> (_h, false));

    const float diag_d = 1.4142135623730951f;
    float max_dist = -1.0f;

    std::priority_queue<ImageNode> pq;
    ImageNode root(_first_branching_node.x(), _first_branching_node.y());
    pq.push(root);
    while(!pq.empty())
    {
        ImageNode n = pq.top();
        pq.pop();
        int nx = n._pos.x(), ny = n._pos.y();
        if(nx < 0 || nx >= _w || ny < 0 || ny >= _h || visited[nx][ny] || !isInside(nx, ny))
            continue;

        float d = n._dist;
        visited[nx][ny] = true;
        nodes[nx][ny]._pos = osg::Vec2(nx, ny);
        nodes[nx][ny]._prev = osg::Vec2(n._prev.x(), n._prev.y());
        nodes[nx][ny]._dist = d;
        nodes[nx][ny]._valid = true;
        if(max_dist == -1.0f || d > max_dist)
            max_dist = d;
        //upwards
        pq.push(ImageNode(nx+1, ny-1, nx, ny, d+diag_d));
        pq.push(ImageNode(nx, ny-1, nx, ny, d+1));
        pq.push(ImageNode(nx-1, ny-1, nx, ny, d+diag_d));

        //sideways
        pq.push(ImageNode(nx-1, ny, nx, ny, d+1));
        pq.push(ImageNode(nx+1, ny, nx, ny, d+1));

        //downwards
        pq.push(ImageNode(nx-1, ny+1, nx, ny, d+diag_d));
        pq.push(ImageNode(nx, ny+1, nx, ny, d+1));
        pq.push(ImageNode(nx+1, ny+1, nx, ny, d+diag_d));
    }

    if(_max_dist == -1.0f && max_dist > 0)
        _max_dist = max_dist;

    if(_nodes.empty())
    {
        _nodes = nodes;
        //setupChildren();
    }
}

void SingleImagePalm::assignBin(int divide)
{
    printf("assignBin...\n");
    if(_nodes.empty() || _max_dist <= 0.0f)
        return;

    _max_bin = -1;
    float bin_len = _max_dist / divide;
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
            {
                int bin = floor((n._dist - 0.5 * bin_len) / bin_len) + 1;
                _nodes[x][y]._bin = bin;
                if(_max_bin == -1 || bin > _max_bin)
                    _max_bin = bin;
            }
        }
}

ImageNode * SingleImagePalm::isGoodNeighbor(int x, int y, int bin)
{
    ImageNode *ret = NULL;
    if(x >= 0 && x < _w && y >= 0 && y < _h)
    {
        ImageNode n = _nodes[x][y];
        if(n._bin == bin && n._valid && !n._considered)
            ret = &_nodes[x][y];
    }
    return ret;
}

long long SingleImagePalm::floodFillAt(int x, int y, int label, bool strong)
{
    long long ret = 0;
    if(x < 0 || x >= _w || y < 0 || y >= _h)
        return ret;

    //bfs
    int ref_bin = _nodes[x][y]._bin;
    std::queue <ImageNode *> Queue;
    Queue.push(&_nodes[x][y]);
    while(!Queue.empty())
    {
        ImageNode *n_p = Queue.front();
        Queue.pop();
        if(n_p->_considered)
            continue;

        int nx = n_p->_pos.x(), ny = n_p->_pos.y();
        n_p->_kingdom = label;
        n_p->_considered = true;
        ret++;

        ImageNode *a, *b, *c, *d, *e, *f, *g, *h;
        a = isGoodNeighbor(nx-1, ny, ref_bin);//left
        b = isGoodNeighbor(nx-1, ny+1, ref_bin);//down-left
        c = isGoodNeighbor(nx, ny+1, ref_bin);//down
        d = isGoodNeighbor(nx+1, ny+1, ref_bin);//down-right
        e = isGoodNeighbor(nx+1, ny, ref_bin);//right
        f = isGoodNeighbor(nx+1, ny-1, ref_bin);//up-right
        g = isGoodNeighbor(nx, ny-1, ref_bin);//up
        h = isGoodNeighbor(nx-1, ny-1, ref_bin);//up-left
        if(a)
            Queue.push(a);
        if(c)
            Queue.push(c);
        if(e)
            Queue.push(e);
        if(g)
            Queue.push(g);
        if(strong)
        {
            if(b)
                Queue.push(b);
            if(d)
                Queue.push(d);
            if(f)
                Queue.push(f);
            if(h)
                Queue.push(h);
        }
    }
    //printf("kingdom(%d) population(%lld)\n", label, ret);

    return ret;
}

void SingleImagePalm::inferKingdom()
{
    printf("inferKingdom...\n");
    if(_max_bin < 0)
        return;

    //1. put ImageNode into (_max_bin+1) boxes
    std::vector <std::vector <ImageNode *> > boxes(_max_bin+1, std::vector <ImageNode *>(NULL));
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                boxes[n._bin].push_back(&_nodes[x][y]);
        }

    //2. find connected component(s) for each bin
    _max_kingdom = -1;
    for(int i=0; i<=_max_bin; i++)
    {
        std::vector <ImageNode *> ring = boxes[i];
        for(unsigned int j=0; j<ring.size(); j++)
        {
            //now we are in bin or ring j
            ImageNode n = *ring[j];
            if(n._considered)
                continue;

            //using the bfs tree is incorrect, instead just flood fill the graph to infer the kingdom
            _max_kingdom++;
            long long population = floodFillAt(int(n._pos.x()), int(n._pos.y()), _max_kingdom);
            _population.push_back(population);
        }
    }

    //3. put ImageNode into {kingdom, osg::Vec2} dict
    _citizens = std::vector <std::vector <osg::Vec2> > (_max_kingdom+1, std::vector <osg::Vec2>());
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid && n._kingdom >=0 && n._kingdom <= _max_kingdom)
                _citizens[n._kingdom].push_back(osg::Vec2(x, y));
        }
}

void SingleImagePalm::inferKingAvg()
{
    printf("inferKingAvg...\n");
    if(_max_kingdom <= 0)
        return;
    _kings = std::vector <osg::Vec2>(_max_kingdom+1, osg::Vec2(-1, -1));

    //1. put ImageNode into (_max_kingdom+1) boxes
    std::vector <std::vector <ImageNode *> > boxes(_max_kingdom+1, std::vector <ImageNode *>(NULL));
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                boxes[n._kingdom].push_back(&_nodes[x][y]);
        }

    //2. find king of each kingdom
    for(int i=0; i<=_max_kingdom; i++)
    {
        std::vector <ImageNode *> kingdom = boxes[i];
        if(!kingdom.empty())
        //if(kingdom.size() > 1000)//population constraint
        {
            long long x = 0, y = 0;
            for(unsigned int j=0; j<kingdom.size(); j++)
            {
                ImageNode n = *kingdom[j];
                x += n._pos.x();
                y += n._pos.y();
            }
            int ax = int(x / kingdom.size());
            int ay = int(y / kingdom.size());
            _kings[i] = osg::Vec2(ax, ay);
        }
    }
}

void SingleImagePalm::inferKingPotential()
{
    printf("inferKingPotential...\n");
    if(_citizens.empty() || _max_kingdom <= 0)
        return;
    _kings = std::vector <osg::Vec2>(_max_kingdom+1, osg::Vec2(-1, -1));
    for(int i=0; i<=_max_kingdom; i++)
    {
        std::vector <osg::Vec2> cs = _citizens[i];
        double max_p = -1;
        for(unsigned int j=0; j<cs.size(); j++)
        {
            int x = cs[j].x();
            int y = cs[j].y();
            double p = _voting_space[x][y];
            if(max_p == -1 || p > max_p)
            {
                max_p = p;
                _kings[i] = cs[j];
            }
        }
    }
}

void SingleImagePalm::inferRawSkeleton()
{
    printf("inferring skeleton...\n");
    if(_max_kingdom <= 0)
        return;

    //1. put ImageNode into (_max_kingdom+1) boxes
    std::vector <std::vector <ImageNode *> > boxes(_max_kingdom+1, std::vector <ImageNode *>(NULL));
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                boxes[n._kingdom].push_back(&_nodes[x][y]);
        }

    //2. establish edge info
    std::vector <osg::Vec2> edges;//list of <par,child>
    for(int i=0; i<=_max_kingdom; i++)
    {
        //find the edge info by tranversing down to the boundary
        std::vector <ImageNode *> kingdom = boxes[i];
        if(!kingdom.empty())
        //if(kingdom.size() > 1000)//population constraint
        {
            for(unsigned int j=0; j<kingdom.size(); j++)
            {
                ImageNode n = *kingdom[j];
                ImageNode cur = n;
                ImageNode target = cur;
                bool toBreak = false;
                while(cur._valid)
                {
                    int pre_x = cur._prev.x();
                    int pre_y = cur._prev.y();
                    if(pre_x >= 0 && pre_y >= 0)
                    {
                        ImageNode par = _nodes[pre_x][pre_y];
                        if(par._kingdom == n._kingdom)
                            cur = par;
                        else
                        {
                            edges.push_back(osg::Vec2(par._kingdom, n._kingdom));
                            toBreak = true;
                            break;
                        }
                    }
                    else
                    {
                        //edges.push_back(osg::Vec2(-1, n._kingdom));
                        toBreak = true;
                        break;
                    }
                }
                if(toBreak)
                    break;
            }
        }
    }

    //debug edge info
    //for(unsigned int i=0; i<edges.size(); i++)
    //{
    //    osg::Vec2 e = edges[i];
    //    printf("%d -> %d\n", int(e.x()), int(e.y()));
    //}

    //3. build BDLSkeleton
    std::vector <BDLSkeletonNode *> nodes;

    for(unsigned int i=0; i<_kings.size(); i++)
    {
        //ignore low population kingdom
        if(_kings[i].x() < 0 || _kings[i].y() < 0)
        {
            nodes.push_back(NULL);
            continue;
        }

        //in image space, i.e. origin is left-top corner
        BDLSkeletonNode *node = new BDLSkeletonNode;
        node->_sx = _kings[i].x();
        node->_sy = 0.0f;
        node->_sz = _kings[i].y();

        nodes.push_back(node);
    }

    for(unsigned int i=0; i<edges.size(); i++)
    {
        osg::Vec2 e = edges[i];
        int parent = int(e.x());
        int child = int(e.y());

        if(nodes[parent] && nodes[child])
        {
            nodes[parent]->_children.push_back(nodes[child]);
            nodes[child]->_prev = nodes[parent];
        }
    }

    if(!nodes.empty())
        _raw_skeleton = nodes[0];
}

void SingleImagePalm::extractMainBranch()
{
    printf("extracting main branch...\n");
    if(!_raw_skeleton)
        return;

    //1. bfs to find the first branching node
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_raw_skeleton);

    BDLSkeletonNode *first_branching_node = NULL;
    std::vector <BDLSkeletonNode *> travelled;

    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        travelled.push_back(node);
        if(node->_children.size() > 1)
        {
            first_branching_node = node;
            break;
        }

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    std::vector <BDLSkeletonNode *> nodes;
    std::vector <osg::Vec2> edges;//list of <par,child>
    for(unsigned int i=0; i<travelled.size(); i++)
    {
        //in image space, i.e. origin is left-top corner
        BDLSkeletonNode *node = new BDLSkeletonNode;
        node->_sx = travelled[i]->_sx;
        node->_sy = travelled[i]->_sy;
        node->_sz = travelled[i]->_sz;

        nodes.push_back(node);
        if(i>0)
            edges.push_back(osg::Vec2(i-1, i));
    }

    for(unsigned int i=0; i<edges.size(); i++)
    {
        osg::Vec2 e = edges[i];
        int parent = int(e.x());
        int child = int(e.y());

        if(nodes[parent] && nodes[child])
        {
            nodes[parent]->_children.push_back(nodes[child]);
            nodes[child]->_prev = nodes[parent];
        }
    }

    if(!nodes.empty())
        _skeleton = nodes[0];
}

void SingleImagePalm::lineSweep()
{
    printf("lineSweep...\n");
    int query = _root.x();
    int level = _root.y();
    std::vector <float> xs;
    float std = -1.0f;

    //1. go from root to top
    while(level >= 0 && query >= 0)
    {
        int left = query;
        int right = query;

        while(isInside(left, level))
            left--;
        left++;
        while(isInside(right, level))
            right++;
        right--;

        int mid = (left + right) / 2;
        int diff = abs(mid - query);
        //printf("%d  sd(%f)\n", abs(mid-query), 1.5*std);

        //2. infer the next best position, resort to original position if deviates too much
        if(xs.size() > 500 && std > 0.0f && diff > 10 && (diff > 20 || diff > 1.0f * std))//hard-code: enforce at least 500 records
            mid = query;
        else if(xs.size() > 500 && std == 0.0f)
        {
            float sd_all = Transformer::standard_deviation(xs);
            if(abs(mid - query) > sd_all)
            {
                int sign = mid - query > 0 ? 1 : -1;
                mid = query + sign * sd_all;
            }
        }
        _main_branch_locus.push_back(osg::Vec2(mid, level));
        xs.push_back(mid);
        std = Transformer::standard_deviation(xs, 500);//only consider the 500-sd

        //3. find the first occurence of y, such that its (max_x - min_x) > 0.5 * _w, i.e. the lower horizontal bound of foliage
        //_lower_foliage_y may become 0
        if(_lower_foliage_y == -1)
        {
            int min_x = 0, max_x = _w-1;
            while(!isInside(min_x, level))
                min_x++;
            while(!isInside(max_x, level))
                max_x--;
            if(max_x - min_x > 0.5 * _w)
                _lower_foliage_y = level;
        }

        //4. find the highest point in foliage
        if(_higher_foliage_y == -1)
        {
            for(int y=0; y<_h && _higher_foliage_y==-1; y++)
            {
                for(int x=0; x<_w && _higher_foliage_y==-1; x++)
                    if(isInside(x, y))
                        _higher_foliage_y = y;
            }
        }

        level--;

        if(isInside(mid, level-1))
        {
            query = mid;
            continue;
        }
        else if(isInside(mid-1, level-1))
        {
            query = mid-1;
            continue;
        }
        else if(isInside(mid+1, level-1))
        {
            query = mid+1;
            continue;
        }
        else
            break;
    }

    //debug
    //for(unsigned int i=0; i<_main_branch_locus.size(); i++)
    //    printf("(%d,%d)\n", int(_main_branch_locus[i].x()), int(_main_branch_locus[i].y()));
}

long long SingleImagePalm::detectBranchingConvolution(int x, int y)
{
    long long ret = 0;

    //convolute with a mask like this: -><-
    //inside segmentation gives 1, otherwise 0
    int length = 800;//hard-code

    //note: smaller y is higher in image space
    //y = k (left)
    for(int i=x-length; i<0; i++)
    {
        if(isInside(i, y))
            ret++;
        else
            break;
    }

    //y = k (right)
    for(int i=1; i<=x+length; i++)
    {
        if(isInside(i, y))
            ret++;
        else
            break;
    }

    //y = x (up)
    for(int i=1; i<length; i++)
    {
        if(isInside(x+i, y-i))
            ret++;
        else
            break;
    }

    //y = x (down)
    for(int i=1; i<length; i++)
    {
        if(isInside(x-i, y+i))
            ret++;
        else
            break;
    }

    //y = -x (up)
    for(int i=1; i<length; i++)
    {
        if(isInside(x-i, y-i))
            ret++;
        else
            break;
    }

    //y = -x (down)
    for(int i=1; i<length; i++)
    {
        if(isInside(x-i, y+i))
            ret++;
        else
            break;
    }

    //y = 2.4142135623730945x
    for(int i=1; i<length; i++)
    {
        int fx = y - 2.4142135623730945 * i;
        if(isInside(x+i, fx))
            ret++;
        else
            break;
    }

    //y = -2.4142135623730945x
    for(int i=1; i<length; i++)
    {
        int fx = y - 2.4142135623730945 * i;
        if(isInside(x-i, fx))
            ret++;
        else
            break;
    }

    //y = 0.41421356237309509x
    for(int i=1; i<length; i++)
    {
        int fx = y - 0.41421356237309509 * i;
        if(isInside(x+i, fx))
            ret++;
        else
            break;
    }

    //y = -0.41421356237309509x
    for(int i=1; i<length; i++)
    {
        int fx = y - 0.41421356237309509 * i;
        if(isInside(x-i, fx))
            ret++;
        else
            break;
    }

    return ret;
}

long long SingleImagePalm::detectBranchingBlockFilling(int cx, int cy)
{
    long long ret = 0;

    int max_k = std::min(_w, _h) / 2;
    //construct a (2k+1)x(2K+1) squre, i.e. 3x3, 5x5, 7x7, ...
    for(int k=1; k<max_k; k++)
    {
        int x, y;
        for(int s=-1; s<=1; s+=2)//sign
        {
            //left and right, including corners
            x = s * k;
            for(y=-k; y<=k; y++)
            {
                if(isInside(cx + x, cy + y))
                    ret++;
                else
                    return ret;
            }

            //bottom and top, excluding corners
            y = s * k;
            for(x=-k+1; x<=k-1; x++)
            {
                if(isInside(cx + x, cy + y))
                    ret++;
                else
                    return ret;
            }
        }
    }

    return ret;
}

void SingleImagePalm::closestPoint(int x, int y, int& rx, int& ry)
{
    //1. dijkstra from query until it find a point that is inside the segmentation
    const float diag_d = 1.4142135623730951f;
    std::vector <std::vector <bool> > visited(_w, std::vector <bool> (_h, false));
    ImageNode root(x, y);
    std::priority_queue<ImageNode> pq;
    pq.push(root);

    while(!pq.empty())
    {
        ImageNode n = pq.top();
        pq.pop();
        int nx = n._pos.x(), ny = n._pos.y();
        if(nx < 0 || nx >= _w || ny < 0 || ny >= _h || visited[nx][ny])
            continue;

        if(_nodes[nx][ny]._valid)
        {
            rx = nx;
            ry = ny;
            return;
        }
        visited[nx][ny] = true;
        float d = n._dist;

        //upwards
        pq.push(ImageNode(nx+1, ny-1, nx, ny, d+diag_d));
        pq.push(ImageNode(nx, ny-1, nx, ny, d+1));
        pq.push(ImageNode(nx-1, ny-1, nx, ny, d+diag_d));

        //sideways
        pq.push(ImageNode(nx-1, ny, nx, ny, d+1));
        pq.push(ImageNode(nx+1, ny, nx, ny, d+1));

        //downwards
        pq.push(ImageNode(nx-1, ny+1, nx, ny, d+diag_d));
        pq.push(ImageNode(nx, ny+1, nx, ny, d+1));
        pq.push(ImageNode(nx+1, ny+1, nx, ny, d+diag_d));
    }
}

void SingleImagePalm::voteSingleCharge(int x, int y, float r, bool positive)
{
    int charge = positive ? 1 : -1;
    for(int i=x-r; i<=x+r; i++)
        for(int j=y-r; j<=y+r; j++)
        {
            if(i < 0 || i >= _w || j < 0 || j >= _h || !isInside(i, j))
                continue;

            float d = pow((i-x)*(i-x) + (j-y)*(j-y), 0.5);
            if(d <= r)
            {
                float assign = charge;
                if(d > 1)
                    assign = charge / d;
                _voting_space[i][j] += assign;
            }
        }
}

void SingleImagePalm::produceLineEdge()
{
    printf("produceLineEdge...\n");
    if(_nodes.empty())
        return;

    //1. detect line segments
    LineSegmentDetector lsd = LineSegmentDetector(_img);
    _edges = lsd.run();
    _edge_widths = lsd.getWidths();
    _edge_polarity = std::vector <bool> (_edge_widths.size(), false);
    _voting_space = std::vector <std::vector <double> > (_w, std::vector <double> (_h, 0.0));

    ImageNode n1(0, 0), n2(0, 0);
    for(unsigned int i=0; i<_edges.size(); i++)
    {
        osg::Vec4 v = _edges[i];
        int x1 = round(v.x());
        int y1 = round(v.y());
        int x2 = round(v.z());
        int y2 = round(v.w());

        //2. initialize _edge_map by drawing each edge
        //drawLine(x1, y1, x2, y2, Qt::white, _edge_widths[i], &_edge_map);

        //3. infer polarity of charge at each edge point
        int cx1, cy1, cx2, cy2;
        closestPoint(x1, y1, cx1, cy1);
        closestPoint(x2, y2, cx2, cy2);
        n1 = _nodes[cx1][cy1];
        n2 = _nodes[cx2][cy2];
        float d1 = n1._dist;
        float d2 = n2._dist;
        _edge_polarity[i] = d1 >= d2 ? false : true;

        //4. vote charge
        float ed = pow((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2), 0.5) / 0.5f;
        voteSingleCharge(x1, y1, std::min(ed, 10 * _edge_widths[i]), _edge_polarity[i]);//hard-code: radius of voting
        voteSingleCharge(x2, y2, std::min(ed, 10 * _edge_widths[i]), !_edge_polarity[i]);//hard-code: radius of voting

        //5. visualize edges and edge points
        //airbrush(x1, y1, 1, 1, Qt::white, &_edge_field);
        //airbrush(x2, y2, 1, 1, Qt::white, &_edge_field);
    }

    //6. set global max, min of potential
    for(int i=0; i<_w; i++)
        for(int j=0; j<_h; j++)
        {
            double p = _voting_space[i][j];
            if(_min_potential == -1 || p < _min_potential)
                _min_potential = p;
            if(_max_convolute_score == -1 || p > _max_potential)
                _max_potential = p;
        }
}

//outdated
long long SingleImagePalm::convoluteEdgeMap(int x, int y)
{
    long long ret = 0;
    int len = 50;
    int len2 = len / 2;
    for(int i=x-len2; i<=x+len2; i++)
        for(int j=y-len2; j<=y+len2; j++)
            if(isEdge(i, j))
                ret++;
    return ret;
}

void SingleImagePalm::inferBestTerminalNode()
{
    printf("inferBestTerminalNode...\n");
    if(_main_branch_locus.empty())
        return;
    int min_search_y = (_lower_foliage_y + _higher_foliage_y) / 2;
    _convolute_score.clear();
    _max_convolute_score = -1;
    for(unsigned int i=0; i<_main_branch_locus.size(); i++)
    {
        osg::Vec2 n = _main_branch_locus[i];
        int nx = n.x(), ny = n.y();
        if(ny < min_search_y || ny > _lower_foliage_y)
            _convolute_score.push_back(0);
        else
        {
            long long score_con = detectBranchingConvolution(nx, ny);
            long long score_blk = detectBranchingBlockFilling(nx, ny);
            long long score = score_con + 0.45 * pow(score_blk, 0.5);
            if(_max_convolute_score == -1 || score > _max_convolute_score)
            {
                _max_convolute_score = score;
                _first_branching_node = n;
                _first_branching_node_idx = i;
            }
            _convolute_score.push_back(score);
        }
    }

    //debug log
    //for(unsigned int i=0; i<_convolute_score.size(); i++)
    //    printf("%lld\n", _convolute_score[i]);
}

void SingleImagePalm::extractMainBranch2()
{
    printf("extractMainBranch2...\n");
    if(_main_branch_locus.empty() || _first_branching_node_idx == -1)
        return;

    //1. box height
    int no_box = 10;
    float max_h = (_first_branching_node - _root).length();
    float box_h = max_h / no_box;

    //2. put nodes into different bins
    std::vector <std::vector <osg::Vec2> > boxes(no_box+1, std::vector <osg::Vec2>());
    for(int i=0; i<=_first_branching_node_idx; i++)
    {
        osg::Vec2 v = _main_branch_locus[i];
        float d = (v - _root).length();
        int box_id = floor((d - 0.5 * box_h) / box_h) + 1;

        if(box_id > 0 && box_id < no_box)
            boxes[box_id].push_back(v);
    }
    //terminal nodes
    boxes[0].push_back(_root);
    boxes[no_box].push_back(_first_branching_node);

    //3. find average of each bin
    std::vector <osg::Vec2> kings;
    for(unsigned int i=0; i<boxes.size(); i++)
    {
        std::vector <osg::Vec2> box = boxes[i];
        double ax = 0, ay = 0;
        long long cnt = 0;
        for(unsigned int j=0; j<box.size(); j++)
        {
            ax += box[j].x();
            ay += box[j].y();
            cnt++;
        }
        if(cnt > 0)
        {
            ax /= cnt;
            ay /= cnt;
            kings.push_back(osg::Vec2(ax, ay));
        }
    }

    //4. build BDLSkeleton
    std::vector <BDLSkeletonNode *> nodes;
    std::vector <osg::Vec2> edges;//list of <par,child>

    for(unsigned int i=0; i<kings.size(); i++)
    {
        //in image space, i.e. origin is left-top corner
        BDLSkeletonNode *node = new BDLSkeletonNode;
        node->_sx = kings[i].x();
        node->_sy = 0.0f;
        node->_sz = kings[i].y();

        nodes.push_back(node);
        if(i != kings.size()-1)
            edges.push_back(osg::Vec2(i, i+1));
    }

    for(unsigned int i=0; i<edges.size(); i++)
    {
        osg::Vec2 e = edges[i];
        int parent = int(e.x());
        int child = int(e.y());

        if(nodes[parent] && nodes[child])
        {
            nodes[parent]->_children.push_back(nodes[child]);
            nodes[child]->_prev = nodes[parent];
        }
    }

    if(!nodes.empty())
    {
        _skeleton = nodes[0];
        _branching = nodes[nodes.size()-1];
    }
}

void SingleImagePalm::dqMainbranchKingdom()
{
    printf("dqMainbranchKingdom...\n");
    if(_max_kingdom <= 0)
        return;
    _kingdom_states = std::vector <bool> (_max_kingdom+1, true);

    //1. transverse from root to first branching node
    osg::Vec2 cur = _root;
    while(int(cur.x()) != -1 && int(cur.y()) != -1)
    {
        int nx = int(cur.x());
        int ny = int(cur.y());

        //2. dq the respective kingdom
        int kid = _nodes[nx][ny]._kingdom;
        _kingdom_states[kid] = false;
        _root_kingdom_id = kid;

        //airbrush(nx, ny, 5, 5, Qt::yellow);
        osg::Vec2 par = _nodes[nx][ny]._prev;
        cur = par;
    }
}

std::vector <int> SingleImagePalm::overlappedKingdom(osg::Vec2 query)
{
    std::set <int> travelled;

    //1. transverse from query back to root
    osg::Vec2 cur = query;
    while(int(cur.x()) != -1 && int(cur.y()) != -1)
    {
        int nx = int(round(cur.x()));
        int ny = int(round(cur.y()));

        travelled.insert(_nodes[nx][ny]._kingdom);

        //airbrush(nx, ny, 2, 2, Qt::yellow);
        osg::Vec2 par = _nodes[nx][ny]._prev;
        cur = par;
    }

    std::vector <int> ret;
    std::set <int>::iterator it;
    for(it=travelled.begin(); it!=travelled.end(); it++)
        if(*it != -1)
            ret.push_back(*it);
    sort(ret.begin(), ret.end());

    return ret;
}

std::vector <osg::Vec2> SingleImagePalm::getRetracement(osg::Vec2 leaf, std::vector <float> percents)
{
    std::vector <osg::Vec2> ret(percents.size(), osg::Vec2());
    if(percents.empty())
        return ret;

    //1. find the distance from leaf to root
    int x, y, lx, ly;
    x = leaf.x();
    y = leaf.y();
    lx = x;
    ly = y;
    if(!_nodes[x][y]._valid)
        closestPoint(x, y, lx, ly);
    float dist = _nodes[lx][ly]._dist;
    if(dist <= 0)
        return ret;

    //2. find the target distances
    std::vector <float> targets;
    for(unsigned int i=0; i<percents.size(); i++)
        targets.push_back(dist * percents[i]);
    std::vector <float> min_ds(percents.size(), -1.0f);
    float cd = 0.0f;

    //3. transverse from leaf to root to find the closest one to target
    osg::Vec2 cur = leaf;
    while(int(cur.x()) != -1 && int(cur.y()) != -1)
    {
        int nx = int(round(cur.x()));
        int ny = int(round(cur.y()));
        int nx2 = nx, ny2 = ny;

        if(!_nodes[nx][ny]._valid)
            closestPoint(nx, ny, nx2, ny2);

        float d = _nodes[nx2][ny2]._dist;
        for(unsigned int i=0; i<targets.size(); i++)
        {
            cd = fabs(targets[i] - d);
            if(min_ds[i] == -1.0f || cd < min_ds[i])
            {
                min_ds[i] = cd;
                ret[i] = osg::Vec2(nx2, ny2);
            }
        }

        //airbrush(nx2, ny2, 2, 2, Qt::yellow);
        cur = _nodes[nx2][ny2]._prev;
    }

    return ret;
}

bool SingleImagePalm::isWellOriented(osg::Vec2 leaf, osg::Vec2 query, osg::Vec2 center)
{
    bool ret = false;
    float det = Transformer::orient(leaf, center, query);
    if(leaf.x() <= center.x())
    {
        //if(det > 0)
        if(det > 0.01)//prevent straight line
            ret = true;
    }
    else
    {
        //if(det < 0)
        if(det < -0.01)//prevent straight line
            ret = true;
    }
    return !ret;//because y-axis is pointing downwards
}

float SingleImagePalm::computePathScore(osg::Vec2 a, osg::Vec2 b, osg::Vec2 c, osg::Vec2 d, int k)
{
    float ret = 0.0f;
    //1. interpolate the control points
    std::vector <osg::Vec2> pts = Transformer::interpolate_bezier_3_2d(a, b, c, d, k);
    for(unsigned int i=0; i<pts.size(); i++)
    {
        osg::Vec2 p = pts[i];
        if(p.x() < 0 || p.x() >= _w || p.y() < 0 || p.y() >= _h)
            continue;
        int x = p.x();
        int y = p.y();

        //2. sum up the potential
        ret += _voting_space[x][y];

        //3. penalize any point that is not inside the segmentation
        if(!isInside(x, y))
            ret -= 10;
    }
    return ret;
}

std::vector <osg::Vec2> SingleImagePalm::circularZone(osg::Vec2 center, float r, int step)
{
    std::vector <osg::Vec2> ret;
    if(step <= 0)
        return ret;
    int x = center.x(), y = center.y();
    for(int i=x-r; i<=x+r; i+=step)
        for(int j=y-r; j<=y+r; j+=step)
        {
            //if(i < 0 || i >= _w || j < 0 || j >= _h || !isInside(i, j))//more restricted, must be inside segmentation
            if(i < 0 || i >= _w || j < 0 || j >= _h)
                continue;

            float d = pow((i-x)*(i-x) + (j-y)*(j-y), 0.5);
            if(d <= r)
                ret.push_back(osg::Vec2(i, j));
        }
    return ret;
}

bool SingleImagePalm::extractSingleSubBranch(bool force)
{
    printf("extractSingleSubBranch...\n");
    bool ret = false;
    if(_max_kingdom <= 0 || _kingdom_states.empty() || !_branching)
        return ret;

    //1. pick the un-consumed kingdom which has the highest kingdom id
    int picked = -1;
    for(int i=_kingdom_states.size()-1; i>=0; i--)
        if(_kingdom_states[i])
        {
            if(_population[i] < 2000)//hard-code: ignore all kindoms that have population less than 1000
            {
                _kingdom_states[i] = false;
                continue;
            }
            printf("population(%lld)\n", _population[i]);
            picked = i;
            break;
        }
    if(picked == -1)
        return ret;

    //2. pick the best 4-th control point
    osg::Vec2 four = _kings[picked];

    //3. get a list of kingdom from 1-st and 4-th control points
    std::vector <int> first_round_can = overlappedKingdom(four);
    if(first_round_can.empty())
        return ret;

    //4. get the retracement nodes
    std::vector <float> percents;
    percents.push_back(0.382f);
    percents.push_back(0.618f);
    std::vector <osg::Vec2> retracements = getRetracement(four, percents);
    if(retracements.empty())
        return ret;
    osg::Vec2 second = retracements[0], third = retracements[1];

    //5. wiggle the two middle points to get the best path
    int inter = (four - _first_branching_node).length() / 2;//hard-code: number of interpolation of bezier curve
    std::vector <osg::Vec2> zone2 = circularZone(second, 50);//hard-code: radius and step of wiggling zone
    std::vector <osg::Vec2> zone3 = circularZone(third, 50);//hard-code: radius and step of wiggling zone
    float max_score = -1.0f;
    long long cnt = 0;
    for(unsigned int i=0; i<zone2.size(); i++)
        for(unsigned int j=0; j<zone3.size(); j++)
        {
            osg::Vec2 w2 = zone2[i];
            osg::Vec2 w3 = zone3[j];

            //check orientations
            if(!isWellOriented(w3, w2, _first_branching_node))
                continue;
            if(!isWellOriented(four, w3, w2))
                continue;
            //check distance (sometimes 2nd is further away from 3rd)
            if((w2-_first_branching_node).length() >= (w3-_first_branching_node).length())
                continue;
            //check if all lay on one side (sometimes one of them is mis-regarded as on the other side)
            if((four.x() > _first_branching_node.x() && w2.x() > w3.x()) || (four.x() < _first_branching_node.x() && w2.x() < w3.x()))
                continue;

            cnt++;
            float score = computePathScore(_first_branching_node, w2, w3, four, std::min(500, inter));//hard-code: limit the max ops
            if(max_score == -1.0f || score > max_score)
            {
                max_score = score;
                second = w2;
                third = w3;
            }

            //debug
            //airbrush(w2.x(), w2.y(), 1, 1, Qt::white);
            //airbrush(w3.x(), w3.y(), 1, 1, Qt::white);
            //printf("score(%f)\n", score);
        }

    printf("ops(%lld)\n", cnt * inter);
    printf("max score(%f)\n", max_score);

    //6. check if the score is too low, which indicates a invalid branch
    bool valid = true;
    if(max_score < -500 && _branch_id > 4)//hard-code: lowest acceptable score, and the first four branch must pass the test
        valid = false;
    if(force)
        valid = true;

    //7. visualize the bezier curve
    if(valid)
    {
        std::vector <osg::Vec2> bezier = Transformer::interpolate_bezier_3_2d(_first_branching_node, second, third, four, inter);
        for(unsigned int i=0; i<bezier.size(); i++)
            airbrush(bezier[i].x(), bezier[i].y(), 5, 5, Qt::green);

        //airbrush(second.x(), second.y(), 10, 10, Qt::blue);
        //airbrush(third.x(), third.y(), 10, 10, Qt::red);
        //airbrush(four.x(), four.y(), 10, 10, Qt::magenta);
    }

    //8. add to main skeleton if this inference is valid
    if(valid)
    {
        _branch_id++;
        std::vector <BDLSkeletonNode *> nodes;
        std::vector <osg::Vec2> edges;
        std::vector <osg::Vec2> new_nodes;
        new_nodes.push_back(second);
        new_nodes.push_back(third);
        new_nodes.push_back(four);
        for(unsigned int i=0; i<new_nodes.size(); i++)
        {
            //in image space, i.e. origin is left-top corner
            BDLSkeletonNode *node = new BDLSkeletonNode;
            node->_sx = new_nodes[i].x();
            node->_sy = 0.0f;
            node->_sz = new_nodes[i].y();

            nodes.push_back(node);
            if(i != new_nodes.size()-1)
                edges.push_back(osg::Vec2(i, i+1));
        }

        for(unsigned int i=0; i<edges.size(); i++)
        {
            osg::Vec2 e = edges[i];
            int parent = int(e.x());
            int child = int(e.y());

            if(nodes[parent] && nodes[child])
            {
                nodes[parent]->_children.push_back(nodes[child]);
                nodes[child]->_prev = nodes[parent];
            }
        }

        _branching->_children.push_back(nodes[0]);
        nodes[0]->_prev = _branching;
    }

    //9. consumed the kingdoms of this sub-branch
    for(unsigned int i=0; i<first_round_can.size(); i++)
        _kingdom_states[first_round_can[i]] = false;

    ret = true;
    return ret;
}


void SingleImagePalm::forceGrow()
{
    printf("forceGrow...\n");
    if(!_skeleton)
        return;

    //1. bfs skeleton to see which kingdom are not covered
    std::vector <bool> consumed(_kingdom_states.size(), false);
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_skeleton);
    while(!Queue.empty())
    {
        BDLSkeletonNode *n = Queue.front();
        Queue.pop();

        int x = n->_sx;
        int y = n->_sz;
        int nx, ny;
        closestPoint(x, y, nx, ny);
        consumed[_nodes[nx][ny]._kingdom] = true;

        for(unsigned int i=0; i<n->_children.size(); i++)
            Queue.push(n->_children[i]);
    }

    //2. for each unconvered kingdom, inspect...
    for(int i=consumed.size()-1; i>=0; i--)
    {
        bool state = consumed[i];
        if(state)
            continue;
        if(_population[i] < 2000)//hard-code: populaton constraint
            continue;
        std::vector <int> overlap = overlappedKingdom(_kings[i]);
        if(overlap.size() < 3)//hard-code: skip it
            continue;
        if(_kings[i].y() > _root.y() - abs(_lower_foliage_y-_root.y())*0.8f)
            continue;
        int un_cnt = 0;
        for(unsigned int j=0; j<overlap.size(); j++)
            if(!consumed[overlap[j]])
                un_cnt++;
        if(un_cnt / float(overlap.size()) > 0.5f)
        {

            for(unsigned int j=0; j<overlap.size(); j++)
            {
                _kingdom_states[overlap[j]] = true;
                consumed[overlap[j]] = true;
                if(j!=overlap.size()-1)
                    printf("%d->", overlap[j]);
                else
                    printf("%d\n", overlap[j]);
            }
            extractSingleSubBranch(true);
        }
    }
}

std::vector <float> SingleImagePalm::bounce(std::vector <float> min_angs, std::vector <float> max_angs, std::vector <float> radii, int times)
{
    std::vector <float> ret;
    if(min_angs.empty() || min_angs.size() != max_angs.size() || max_angs.size() != radii.size())
        return ret;

    //1. initially all angles are random
    std::vector <osg::Vec2> positions;//points on unit circle
    for(unsigned int i=0; i<min_angs.size(); i++)
    {
        float ang = (rand() % int(max_angs[i] - min_angs[i])) + min_angs[i];
        float r = radii[i];
        ret.push_back(ang);
        float rad = ang * M_PI / 180.0f;
        positions.push_back(osg::Vec2(r * cos(rad), r * sin(rad)));
    }

    //2. bounce under the effect of repulsive force
    for(int t=0; t<times; t++)
    {
        std::vector <osg::Vec2> tmp_pos = positions;
        for(unsigned int i=0; i<tmp_pos.size(); i++)
        {
            osg::Vec2 c = tmp_pos[i];
            osg::Vec2 uc = c;
            float r = radii[i];
            uc.normalize();
            osg::Vec2 f(0.0f, 0.0f);
            for(unsigned int j=0; j<tmp_pos.size(); j++)
                if(i != j)
                {
                    osg::Vec2 o = tmp_pos[j];
                    osg::Vec2 d = c - o;
                    float r = d.length();
                    if(r != 0.0f)
                    {
                        d.normalize();
                        f += d * (1.0f / (r * r));
                    }
                }
            osg::Vec2 n = c + f - uc * (f * uc);
            n.normalize();
            float arg_n = atan2(n.y(), n.x()) * 180 / M_PI;
            int ret_i = int(ret[i] + 360) % 360;
            int new_i = int(arg_n + 360) % 360;
            int case1 = abs(ret_i - 1 - new_i);
            int case2 = abs(ret_i + 1 - new_i);
            //printf("%d: ang(%.0f) -> ", i, ret[i]);
            if(std::min(case1, 360 - case1) < std::min(case2, 360 - case2))
            {
                if(ret[i]-1 >= min_angs[i])
                {
                    ret[i]--;
                    float rad = ret[i] * M_PI / 180.0f;
                    positions[i] = osg::Vec2(r * cos(rad), r * sin(rad));
                }
            }
            else if(std::min(case1, 360 - case1) > std::min(case2, 360 - case2))
            {
                if(ret[i]+1 <= max_angs[i])
                {
                    ret[i]++;
                    float rad = ret[i] * M_PI / 180.0f;
                    positions[i] = osg::Vec2(r * cos(rad), r * sin(rad));
                }
            }
            //printf("ang(%.0f)\n", ret[i]);
        }
    }

    return ret;
}

void SingleImagePalm::convertTo3D()
{
    printf("convertTo3D...\n");
    float scale = _img2skeleton_scale;//hard-code: to scale down from image to skeleton space

    //1. convert everything from image space to blender space
    _blender_skeleton = BDLSkeletonNode::copy_tree(_skeleton);

    //2. translate to origin and scale it
    BDLSkeletonNode *branching = NULL;
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(_blender_skeleton);
    while(!Queue.empty())
    {
        BDLSkeletonNode *node = Queue.front();
        Queue.pop();

        node->_sx -= _skeleton->_sx;
        node->_sy -= _skeleton->_sy;
        node->_sz -= _skeleton->_sz;

        node->_sx *= 1.0f / scale;
        node->_sy *= 1.0f / scale;
        node->_sz *= -1.0f / scale;

        if(node->_children.size() >= 2)
            branching = node;

        for(unsigned int i=0; i<node->_children.size(); i++)
            Queue.push(node->_children[i]);
    }

    //3. find the maximum displacement in x-axix of each branch
    if(!branching || branching->_children.size() < 1)
    {
        printf("SingleImagePalm::convertTo3D():branching(NULL) error\n");
        return;
    }
    float branching_x = branching->_sx;
    std::vector <float> max_xs;
    float max_depth = -1.0f;//limit of maximum +/- depth
    for(unsigned int i=0; i<branching->_children.size(); i++)
    {
        BDLSkeletonNode *cur = branching->_children[i];
        float h = 0.0f;
        float mx = -1.0f, mx2 = -1.0f;//mx2 stores the sign, too
        while(cur)
        {
            h = cur->_sz;
            float sx = fabs(cur->_sx - branching_x);
            if(max_depth == -1.0f || sx > max_depth)
                max_depth = sx;
            if(mx == -1.0f || sx > mx)
            {
                mx = sx;
                mx2 = cur->_sx;
            }
            if(cur->_children.empty())
                cur = NULL;
            else
                cur = cur->_children[0];
        }
        max_xs.push_back(mx2);
    }
    //printf("max_depth(%f) branching_x(%f)\n", max_depth, branching_x);

    //4. given the maximum depth, infer the range of permitting Θ_i
    std::vector <float> min_angs, max_angs, radii;
    for(unsigned int i=0; i<branching->_children.size(); i++)
    {
        int branch = i;
        float min_range = -M_PI / 4;
        float max_range = M_PI / 4;
        float min_range2 = 3 * M_PI / 4;
        float max_range2 = 5 * M_PI / 4;
        float mx = max_xs[branch];
        if(fabs(mx - branching_x) > max_depth * 0.2f && max_depth > 0.0f)//hard-code: don't rotate too much if it is closed to origin
        {
            min_range = atan2(-max_depth, mx - branching_x);
            max_range = atan2(max_depth, mx - branching_x);
            if(mx - branching_x < 0)
            {
                min_range += 2 * M_PI;
                std::swap(min_range, max_range);
            }
        }
        else
        {
            if(mx - branching_x  < 0)
            {
                min_range = min_range2;
                max_range = max_range2;
            }
        }
        min_angs.push_back(min_range * 180 / M_PI);
        max_angs.push_back(max_range * 180 / M_PI);
        radii.push_back(fabs(mx - branching_x));

        //printf("branch(%d) min(%.02f) max(%.02f)\n", i, min_range * 180/M_PI, max_range * 180/M_PI);
    }

    //5. bounce back and forth to get the best arrangement
    std::vector <float> best_angs;//store all the current best angles
    best_angs = bounce(min_angs, max_angs, radii);//hard-code: to bounce 100 times

    //6. add depth to the branch
    for(unsigned int i=0; i<branching->_children.size(); i++)
    {
        int branch = i;
        float best_ang = best_angs[branch] * M_PI / 180;
        BDLSkeletonNode *cur = branching->_children[branch];
        while(cur)
        {
            cur->_sy = (cur->_sx - branching_x) * tan(best_ang);
            //printf("branch(%d) depth(%f)\n", branch, cur->_sy);
            if(cur->_children.empty())
                cur = NULL;
            else
                cur = cur->_children[0];
        }
    }
}

void SingleImagePalm::save()
{
    if(_data_valid && _grow_valid)
    {
        std::string out_skeleton = _output_dir + "/z_skeleton";
        std::string out_isp0 = _output_dir + "/master.isp0";
        BDLSkeletonNode::save_skeleton(_blender_skeleton, out_skeleton.c_str());

        //file IO
        FILE *out = fopen(out_isp0.c_str(), "w");
        fprintf(out, "%s\n", _input_img_path.c_str());
        fprintf(out, "%s\n", _input_seg_path.c_str());
        fprintf(out, "%d %d\n", int(_root.x()), int(_root.y()));
    }
    else
        printf("SingleImagePalm::save():_data_valid(%d):_grow_valid(%d) error\n", int(_data_valid), int(_grow_valid));
}

void SingleImagePalm::visualize_dijkstra()
{
    printf("visualize_dijkstra...\n");
    if(_max_dist <= 0.0f)
        return;

    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                airbrush(x, y, 1, 1, mapColor(n._dist/_max_dist));
        }
}

void SingleImagePalm::visualize_bin()
{
    printf("visualize_bin...\n");
    if(_max_bin <= 0)
        return;

    float mk = float(_max_bin);
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                airbrush(x, y, 1, 1, mapColor(n._bin/mk));
        }
}

void SingleImagePalm::visualize_kingdom(bool show_all)
{
    printf("visualize_kingdom...\n");
    if(_max_kingdom <= 0 || _kingdom_states.empty())
        return;

    float mk = float(_max_kingdom);
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
                if(show_all || _kingdom_states[n._kingdom])
                    airbrush(x, y, 1, 1, mapColor(n._kingdom/mk));
        }
}

void SingleImagePalm::visualize_king()
{
    for(unsigned int i=0; i<_kings.size(); i++)
    {
        osg::Vec2 king = _kings[i];
        if(king.x() >= 0 && king.y() >= 0)
            airbrush(king.x(), king.y(), 10, 10, Qt::white);
    }
}

void SingleImagePalm::visualize_skeleton(BDLSkeletonNode *root, bool show_node, bool show_edge, QColor node_color, QColor edge_color)
{
    printf("visualize_skeleton...\n");
    if(!root)
        return;

    //bfs to draw all the edges first
    if(show_edge)
    {
        std::queue <BDLSkeletonNode *> Queue;
        Queue.push(root);
        while(!Queue.empty())
        {
            BDLSkeletonNode *node = Queue.front();
            Queue.pop();

            int x1 = node->_sx;
            int y1 = node->_sz;

            for(unsigned int i=0; i<node->_children.size(); i++)
            {
                BDLSkeletonNode *child = node->_children[i];
                int x2 = child->_sx;
                int y2 = child->_sz;
                drawLine(x1, y1, x2, y2, edge_color);

                Queue.push(child);
            }
        }
    }

    //bfs to draw nodes
    if(show_node)
    {
        std::queue <BDLSkeletonNode *> Queue;
        Queue.push(root);
        while(!Queue.empty())
        {
            BDLSkeletonNode *node = Queue.front();
            Queue.pop();

            int x1 = node->_sx;
            int y1 = node->_sz;
            airbrush(x1, y1, 20, 20, node_color);

            for(unsigned int i=0; i<node->_children.size(); i++)
                Queue.push(node->_children[i]);
        }
    }
}

void SingleImagePalm::visualize_linesweep()
{
    printf("visualize_linesweep...\n");
    if(_main_branch_locus.size() == _convolute_score.size())
    {
        for(unsigned int i=0; i<_main_branch_locus.size(); i++)
        {
            osg::Vec2 n = _main_branch_locus[i];
            long long cs = _convolute_score[i];
            if(cs < 0)
                cs = 0;
            int s = 2 + (cs / float(_max_convolute_score)) * 8;
            airbrush(n.x(), n.y(), s, s, Qt::black);
        }

        airbrush(_first_branching_node.x(), _first_branching_node.y());
    }
}

void SingleImagePalm::visualize_edge(bool thin_edge, QImage *img)
{
    printf("visualize_edge...\n");
    QImage *board = &_debug_img;
    if(img)
        board = img;
    if(_edges.size() == _edge_widths.size())
        for(unsigned int i=0; i<_edges.size(); i++)
        {
            osg::Vec4 v = _edges[i];
            if(thin_edge)
                drawLine(v.x(), v.y(), v.z(), v.w(), Qt::black, 1, board);
            else
                drawLine(v.x(), v.y(), v.z(), v.w(), Qt::black, _edge_widths[i], board);
        }
}

void SingleImagePalm::visualize_branch_search_limit()
{
    printf("visualize_branch_search_limit...\n");
    //int min_search_y = (_lower_foliage_y - _higher_foliage_y) * 0.618f + _higher_foliage_y;//golden ratio
    int min_search_y = (_lower_foliage_y + _higher_foliage_y) / 2;
    drawLine(0, _higher_foliage_y, _w-1, _higher_foliage_y, Qt::red);
    drawLine(0, min_search_y, _w-1, min_search_y, Qt::red);
    drawLine(0, _lower_foliage_y, _w-1, _lower_foliage_y, Qt::red);
}

void SingleImagePalm::visualize_polarity()
{
    printf("visualize_polarity...\n");
    if(_edges.size() == _edge_widths.size())
        for(unsigned int i=0; i<_edges.size(); i++)
        {
            osg::Vec4 v = _edges[i];
            if(_edge_polarity[i])
            {
                airbrush(v.x(), v.y(), 5, 5, Qt::red, &_edge_map);//red is positive
                airbrush(v.z(), v.w(), 5, 5, Qt::green, &_edge_map);//green is negative
            }
            else
            {
                airbrush(v.x(), v.y(), 5, 5, Qt::green, &_edge_map);//green is negative
                airbrush(v.z(), v.w(), 5, 5, Qt::red, &_edge_map);//red is positive
            }
        }
}

void SingleImagePalm::visualize_voting_space()
{
    printf("visualize_voting_space...\n");
    double range = _max_potential - _min_potential;
    if(_voting_space.empty() || range == 0.0f)
        return;

    //printf("_max_potential(%lf) _min_potential(%lf)\n", _max_potential, _min_potential);
    for(int i=0; i<_w; i++)
        for(int j=0; j<_h; j++)
            //if(isInside(i, j) && _voting_space[i][j] != 0)
            if(isInside(i, j))
            {
                airbrush(i, j, 1, 1, mapColor((_voting_space[i][j]-_min_potential) / range), &_edge_map);
                //printf("%lf\n", (_voting_space[i][j]-_min_potential) / range);
            }
}
