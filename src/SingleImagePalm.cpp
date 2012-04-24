#include "SingleImagePalm.h"
#include "ISPLoader.h"
#include <queue>
//#include "Transformer.h"

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
}

SingleImagePalm::~SingleImagePalm()
{
    if(_verbose)
    {
        const char * path = "/tmp/debug.png";
        if(_root.x() != -1 && _root.y() != -1)
        {
            airbrush(_root.x(), _root.y());
            //printf("root(%d,%d)\n", int(_root.x()), int(_root.y()));
        }
        //visualize_bfs();
        visualize_kingdom();

        if(!_debug_img.isNull())
            _debug_img.save(QString(path), "PNG", 70);

        printf("debug image is outputted at '%s'\n", path);
    }
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
            assignKingdom();
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
                return true;
            }
            /*
			QRgb c = _seg.pixel(x, y);
			if(qRed(c) != 0 || qGreen(c) != 0 || qBlue(c) != 0) // if not fully black
            {
                _root = osg::Vec2(x, y);
                return true;
            }
            */
        }
    _root = osg::Vec2(-1, -1);
    return false;
}

void SingleImagePalm::bfs()
{
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
        nodes[nx][ny]._valid = true;
        nodes[nx][ny]._dist = d;
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
        _nodes = nodes;
}

void SingleImagePalm::assignKingdom(int divide)
{
    if(_nodes.empty() || _max_dist <= 0.0f)
        return;

    _max_kingdom = -1;
    float bin_len = _max_dist / divide;
    for(int y=_h-1; y>=0; y--)
		for(int x=0; x<_w; x++)
		{
            ImageNode n = _nodes[x][y];
            if(n._valid)
            {
                int kingdom = int(n._dist / bin_len);
                _nodes[x][y]._kingdom = kingdom;
                if(_max_kingdom == -1 || kingdom > _max_kingdom)
                    _max_kingdom = kingdom;
            }
        }
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
