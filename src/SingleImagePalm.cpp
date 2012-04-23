#include "SingleImagePalm.h"
#include "ISPLoader.h"
#include <QPainter>
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

    if(iw != sw || ih != sh)
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%d x %d):_seg(%d x %d) does not match error\n", iw, ih, sw, sh);
        return;
    }

    //at this point isp is valid
    _data_valid = true;

    //_img.save(QString("/tmp/test_tiff.png"), "PNG", 80);test if the qt plugin works
    _debug_img = QImage(iw, ih, QImage::Format_ARGB32);
    _debug_img.fill(0);
}

SingleImagePalm::~SingleImagePalm()
{
    if(_verbose)
    {
        if(_root.x() != -1 && _root.y() != -1)
        {
            airbrush(_root.x(), _root.y());
            printf("_root(%d,%d)\n", int(_root.x()), int(_root.y()));
        }

        if(!_debug_img.isNull())
            _debug_img.save(QString("/tmp/debug.png"), "PNG", 70);
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
        findRoot();
    }
}

void SingleImagePalm::airbrush(int x, int y)
{
    int w = 50, h = 50;
    QPainter p;
    p.begin(&_debug_img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(Qt::red));
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

void SingleImagePalm::findRoot()
{
    printf("SingleImagePalm::findRoot()\n");
    int w = _seg.width(), h = _seg.height();
    for(int y=h-1; y>=0; y--)
		for(int x=0; x<w; x++)
		{
			QRgb c = _seg.pixel(x, y);
			if(qRed(c) != 0 || qGreen(c) != 0 || qBlue(c) != 0) // if not fully transparent
            {
                _root = osg::Vec2(x, y);
                return;
            }
        }
    _root = osg::Vec2(-1, -1);
}
