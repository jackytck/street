#include "SingleImagePalm.h"
#include "ISPLoader.h"
#include <queue>

SingleImagePalm::SingleImagePalm(std::string isp0): _verbose(false), _data_valid(false)
{
    ISPLoader loader;
    loader.load(isp0);

    _img = QImage(loader._img_path.c_str());
    if(_img.isNull())
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%s) error\n", loader._img_path.c_str());
        return;
    }
    _seg = QImage(loader._seg_path.c_str());
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
    _debug_img = QImage(iw, ih, QImage::Format_ARGB32);
    _debug_img.fill(0);
    _w = iw;
    _h = ih;
    _max_dist = -1.0f;
    _max_kingdom = -1;
}

SingleImagePalm::~SingleImagePalm()
{
    if(_verbose)
    {
        const char * path = "/tmp/debug.png";
        if(_root.x() != -1 && _root.y() != -1)
        {
            //airbrush(_root.x(), _root.y());
            //printf("root(%d,%d)\n", int(_root.x()), int(_root.y()));
        }
        //visualize_bfs();
        //visualize_bin();
        visualize_kingdom();
        visualize_king();

        if(!_debug_img.isNull())
            _debug_img.save(QString(path), "PNG", 70);

        printf("debug image is outputted at '%s'\n", path);
    }

    BDLSkeletonNode::delete_this(_skeleton);
}

void SingleImagePalm::setVerbose(bool debug)
{
    _verbose = debug;
}

void SingleImagePalm::grow()
{
    if(_data_valid)
    {
        if(findRoot())
        {
            bfs();
            assignBin();
            inferKingdom();
            inferKing();
            inferSkeleton();
        }
    }
}

void SingleImagePalm::airbrush(int x, int y, int w, int h, QColor color)
{
    QPainter p;
    p.begin(&_debug_img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(color));
    p.drawEllipse(x-w/2, y-h/2, w, h);
    p.end();
}

void SingleImagePalm::drawLine(int x1, int y1, int x2, int y2)
{
    QPainter p;
    p.begin(&_debug_img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(Qt::green, 10));
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

QColor SingleImagePalm::mapColor(float scalar)
{
    int h = int(scalar * 360) % 360;
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

void SingleImagePalm::bfs()
{
    printf("bfs-ing...\n");
    std::vector <std::vector <ImageNode> > nodes(_w, std::vector <ImageNode> (_h, ImageNode(0, 0)));
    std::vector <std::vector <bool> > visited(_w, std::vector <bool> (_h, false));

    const float diag_d = 1.4142135623730951f;
    float max_dist = -1.0f;

    std::queue <ImageNode> Queue;
    ImageNode root(_root.x(), _root.y());
    Queue.push(root);
    while(!Queue.empty())
    {
        ImageNode n = Queue.front();
        Queue.pop();
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
        Queue.push(ImageNode(nx-1, ny, nx, ny, d+1));
        Queue.push(ImageNode(nx-1, ny+1, nx, ny, d+diag_d));
        Queue.push(ImageNode(nx, ny+1, nx, ny, d+1));
        Queue.push(ImageNode(nx+1, ny+1, nx, ny, d+diag_d));
        Queue.push(ImageNode(nx+1, ny, nx, ny, d+1));
        Queue.push(ImageNode(nx+1, ny-1, nx, ny, d+diag_d));
        Queue.push(ImageNode(nx, ny-1, nx, ny, d+1));
        Queue.push(ImageNode(nx-1, ny-1, nx, ny, d+diag_d));
    }

    if(_max_dist == -1.0f && max_dist > 0)
        _max_dist = max_dist;

    if(_nodes.empty())
    {
        _nodes = nodes;
        setupChildren();
    }
}

void SingleImagePalm::assignBin(int divide)
{
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
                int bin = int(n._dist / bin_len);
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

void SingleImagePalm::floodFillAt(int x, int y, int label)
{
    if(x < 0 || x >= _w || y < 0 || y >= _h)
        return;

    //bfs
    int cnt = 0;
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
        cnt++;

        ImageNode *a, *b, *c, *d;
        a = isGoodNeighbor(nx-1, ny, ref_bin);
        b = isGoodNeighbor(nx, ny+1, ref_bin);
        c = isGoodNeighbor(nx+1, ny, ref_bin);
        d = isGoodNeighbor(nx, ny-1, ref_bin);
        if(a)
            Queue.push(a);
        if(b)
            Queue.push(b);
        if(c)
            Queue.push(c);
        if(d)
            Queue.push(d);
    }
    //printf("kingdom(%d) population(%d)\n", label, cnt);
}

void SingleImagePalm::inferKingdom()
{
    printf("infering kingdom...\n");
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
    _max_kingdom = 0;
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
            floodFillAt(int(n._pos.x()), int(n._pos.y()), _max_kingdom);
            _max_kingdom++;
        }
    }
}

void SingleImagePalm::inferKing()
{
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

void SingleImagePalm::inferSkeleton()
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
                        edges.push_back(osg::Vec2(-1, n._kingdom));
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
    for(unsigned int i=0; i<edges.size(); i++)
    {
        osg::Vec2 e = edges[i];
        printf("%d -> %d\n", int(e.x()), int(e.y()));
    }

    //3. build BDLSkeleton
    std::vector <BDLSkeletonNode *> nodes;

    for(unsigned int i=0; i<_kings.size(); i++)
    {
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

        nodes[parent]->_children.push_back(nodes[child]);
        nodes[child]->_prev = nodes[parent];
    }

    if(!nodes.empty())
        _skeleton = nodes[0];
}

void SingleImagePalm::visualize_bfs()
{
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

void SingleImagePalm::visualize_kingdom()
{
    if(_max_kingdom <= 0)
        return;

    float mk = float(_max_kingdom);
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
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
