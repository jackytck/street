#include "ImageContour.h"
#include "Volume.h"

ImageContour::ImageContour(): _valid(false), _volume(NULL)
{
}

ImageContour::~ImageContour()
{
}

void ImageContour::clear()
{
    _segmentation.fill(Qt::white);
}

void ImageContour::set_segmentation(QString path)
{
    //printf("Loading %s\n", path.toStdString().c_str());
    _segmentation = QImage(path);
}

void ImageContour::set_root(int x, int y)
{
    _root = QPoint(x, y);
}

void ImageContour::set_scale(float s)
{
    _scale = s;
    //printf("ImageContour(%p)::set_scale(%f)\n", this, _scale);
}

void ImageContour::set_surface(std::vector <osg::Vec3> surface, osg::Vec3 cg)
{
    if(surface.empty())
        return;

    _surface = surface;
    _cg = cg;

    //set the volume
    SimpleVolume tmp(1000, 1000, 1000, 3, 3, 3);//hard-code the step
    _volume = tmp;

    for(unsigned int i=0; i<_surface.size(); i++)
        //printf("surface(%d): (%f %f %f)\n", i, _surface[i].x(), _surface[i].y(), _surface[i].z());
        _volume.add_point(500+_surface[i].x(), 500+_surface[i].y(), _surface[i].z());

    //printf("%p: size(%d %d %d)\n", &_volume, _volume._sizeX, _volume._sizeY, _volume._sizeZ);
}

bool ImageContour::isInside(float x, float y)
{
    bool ret = false;

    if(_segmentation.isNull())
        return true;

    //use probe to check the value in _segmentation
    //less strict: use 8-neighbours to test, instead of one
    for(int i=-1; i<=1; i++)
        for(int j=-1; j<=1; j++)
        {
            QPoint probe(x+i, y+j);

            if(x+i >= 0 && x+i < 1000 && y+j >= 0 && y+j < 1000)
            {
                QRgb color = _segmentation.pixel(probe);

                //inside is non-zero
                if(qRed(color) != 0 || qGreen(color) != 0 || qBlue(color) != 0)
                    ret = true;
            }
        }

    //debug
    //if(!ret)
    //    printf("ImageContour::isInside(%f %f)\n", x, y);

    return ret;
}

bool ImageContour::isInside(osg::Vec3 q)
{
    //simply discard the y-coords, assume orthogonal projection
    //return isInside(q.x(), q.z());

    //simply alternate orthogonal plane
    //int draw = rand()%10;
    //if(draw < 5)
    //    return isInside(q.x(), q.z());
    //else
    //    return isInside(q.x(), q.y());

    //use revolution to compute the coordinates
    float rotated_x = pow(q.x()*q.x() + q.y()*q.y(), 0.5);

    //root is always (0, 0), not equal to the _root in image space
    //if(q.x() < _root.x())
    if(q.x() < 0.0)
        rotated_x *= -1.0f;

    //in skeleton space
    QPointF q2(rotated_x, q.z());

    //find the image coordinates by inverting the processes
    q2.setY(q2.y() * -1);
    
    if(_scale == 0.0f)
        printf("ImageContour:: hihi, scale == 0.0f\n");

    q2 = q2 / _scale;
    q2 = q2 + _root;

    return isInside(q2.x(), q2.y());
}

bool ImageContour::isInside(BDLSkeletonNode *node)
{
    return isInside(osg::Vec3(node->_sx, node->_sy, node->_sz));
}

bool ImageContour::isInsideOrtho(osg::Vec3 q)
{
    //find the image coordinates by inverting the processes
    QPoint q2(q.x(), q.z());

    q2.setY(q2.y() * -1);
    
    if(_scale == 0.0f)
        printf("ImageContour:: hihi, scale == 0.0f\n");

    q2 = q2 / _scale;
    q2 = q2 + _root;

    return isInside(q2.x(), q2.y());
}

//this is a very restrictsive test,
//because the volume is not empty inside
bool ImageContour::isInsideVolume(osg::Vec3 q)
{
    bool ret = false;
    float min_dist = -1.0f;
    osg::Vec3 min_pt;

    //method 1: use perpendicular distance
    //right is 1, left is -1
    if(false)
    {
        int side = 1;
        if(q.x() < _cg.x())
            side = -1;

        //find the min_pt which is the closest node on the surface to the line formed by cg and q
        for(unsigned int i=0; i<_surface.size(); i++)
        {
            osg::Vec3 pt = _surface[i];
            int pt_side = pt.x() - _cg.x() > 0 ? 1 : -1;

            //only want right-side-right-pt or left-side-left-pt
            if(side * pt_side > 0)
            {
                //find perpendicular distance
                float dist = perp_dist(_cg, pt, q);

                if(min_dist == -1.0f || dist < min_dist)
                {
                    min_dist = dist;
                    min_pt = pt;
                }
            }
        }
    }

    //method 2: use the closest point
    if(false)
    {
        //just find the closest point from surface to query point
        for(unsigned int i=0; i<_surface.size(); i++)
        {
            osg::Vec3 pt = _surface[i];
            float dist = (pt - q).length();

            if(min_dist == -1.0f || dist < min_dist)
            {
                min_dist = dist;
                min_pt = pt;
            }
        }
    }

    //method 3: use volume as a look-up table
    if(true)
    {
        if(!_volume.is_empty(500+q.x(), 500+q.y(), q.z()))
            return true;
        else
            return false;
    }

    //check if cg sees q or min_pt first
    //float cg_q = (_cg - q).length();
    //float cg_min_pt = (_cg - min_pt).length();

    //debug
    //if(cg_q >= cg_min_pt)
        //printf("q(%f %f %f) min_pt(%f %f %f)\n", q.x(), q.y(), q.z(), min_pt.x(), min_pt.y(), min_pt.z());
    //printf("cg_q(%f) cg_min_pt(%f) diff(%f)\n", cg_q, cg_min_pt, cg_q - cg_min_pt);

    //- <- inside -> | undetermined | + <-outside->
    //either use confidence or min_dist

    //float confidence = cg_min_pt - cg_q;

    //if(confidence > 5)
    //{
    //    //printf("confidence(%f) q(%f %f %f)\n", confidence, q.x(), q.y(), q.z());
    //    ret = true;
    //}

    if(min_dist < 5)
        ret = true;

    return ret;
}

void ImageContour::validated()
{
    _valid = true;
}

bool ImageContour::isValid()
{
    return _valid;
}

QPoint ImageContour::obj_to_img_space(osg::Vec3 v)
{
    QPoint ret;

    if(_scale == 0.0f)
    {
        printf("ImageContour:: hihi, scale == 0.0f\n");
        return ret;
    }
    
    //method 1: by revolution
    //drawback: the middle part will be gone
    float rotated_x = pow(v.x()*v.x() + v.y()*v.y(), 0.5);

    //root is always (0, 0), not equal to the _root in image space
    if(v.x() < 0.0)
        rotated_x *= -1.0f;

    //in skeleton space
    QPointF v2(rotated_x, v.z());

    //method 2: by projection
    //drawback: the two-side will have severe aliasing
    if(true)
    {
        v2 = QPointF(v.x(), v.z());

        //try to minimize the drawback, //no significant difference
        //if(v2.y() > 0)
        //    v2.setX(v2.x() * -1);
    }

    //printf("root(%d %d) scale(%f) v2(%f %f)\n", _root.x(), _root.y(), _scale, v2.x(), v2.y());

    //find the image coordinates by inverting the processes
    v2.setY(v2.y() * -1);
    
    v2 = v2 / _scale;
    v2 = v2 + _root;

    ret = QPoint(v2.x(), v2.y());

    //debug
    //printf("ImageContour(%p): img_space v(%f %f %f) ret(%d %d)\n", this, v.x(), v.y(), v.z(), ret.x(), ret.y());

    return ret;
}

float ImageContour::perp_dist(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c)
{
    float ret = -1.0f;

    if(a == b)
        return ret;

    ret = ((c-a) ^ (c-b)).length();
    ret /= (b-a).length();

    return ret;
}
