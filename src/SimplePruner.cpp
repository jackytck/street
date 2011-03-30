#include "SimplePruner.h"
#include <highgui.h>
#include "ISPLoader.h"

SimplePruner::SimplePruner(): _segmentation(NULL), _rootX(-1), _rootY(-1), _scale(-1.0f)
{
}

SimplePruner::~SimplePruner()
{
    clear();
}

void SimplePruner::clear()
{
    //release image
    if(_segmentation)
    {
        cvReleaseImage(&_segmentation);
        _segmentation = NULL;
        _rootX = -1;
        _rootY = -1;
        _scale = -1.0f;
    }
}

void SimplePruner::setup(std::string path, int x, int y, float s)
{
    clear();

    if(set_segmentation(path))
    {
        set_root(x, y);
        set_scale(s);
    }
}

void SimplePruner::setup(std::string isp0, float s)
{
    ISPLoader loader;
    loader.load(isp0);
    setup(loader._seg_path, loader._rootX, loader._rootY, s);
}

bool SimplePruner::set_segmentation(std::string path)
{
    //load the segmentation
    _segmentation = cvLoadImage(path.c_str(), 0);//0 == CV_LOAD_IMAGE_GRAYSCALE
    if(!_segmentation)
        printf("SimplePruner::set_segmentation: Error loading %s.\n", path.c_str());

    return _segmentation ? true : false;
}

void SimplePruner::set_root(int x, int y)
{
    _rootX = x;
    _rootY = y;
}

void SimplePruner::set_scale(float s)
{
    _scale = s;
}

bool SimplePruner::is_loaded()
{
    bool ret = false;

    if(_segmentation && _rootX != -1 && _rootY != -1 && _scale != -1.0f)
        ret = true;

    return ret;
}

bool SimplePruner::is_inside(float x, float y, bool lenient)
{
    bool ret = false;

    //use probe to check the value in _segmentation
    //less strict: use k-neighbours to test, instead of one
	int k = 1;

    for(int i=-k; i<=k; i++)
        for(int j=-k; j<=k; j++)
            if(read(x+i, y+j) != 0)
			{
                ret = true;
				break;
			}

	if(!ret && lenient)
	{
		int k = 20;
		for(int i=-k; i<=k; i++)
			for(int j=-k; j<=k; j++)
				if(read(x+i, y+j) != 0)
					return true;
	}

    return ret;
}

bool SimplePruner::is_inside(BDLSkeletonNode *node, bool lenient)
{
    bool ret = false;
    if(!node || _scale == -1.0f || _scale == 0.0f)
        return ret;

    float x = node->_sx, y = node->_sy, z = node->_sz;
    return is_inside(osg::Vec3(x, y, z), lenient);
}

bool SimplePruner::is_inside(osg::Vec3 node, bool lenient)
{
    float x = node.x(), y = node.y(), z = node.z();

    //use revolution to compute the coordinates
    float rotated_x = pow(x*x + y*y, 0.5);
	
	if(false)
	{
		rotated_x = x;
	}
	else
	{
		//don't want to look like a perfect circle
		if(abs(y) > abs(x))
			//rotated_x *= 1.3f;
			//rotated_x *= 0.85f;
			rotated_x *= 1.20f;

		//root(origin) is always (0, 0), not equal to the _root in image space
		if(x < 0.0)
			rotated_x *= -1.0f;
	}

    //in skeleton space
    float qX = rotated_x, qY = z;

    //find the image coordinates by inverting the processes
    qY *= -1;
    
    qX /= _scale;
    qY /= _scale;
    qX += _rootX;
    qY += _rootY;

    return is_inside(qX, qY, lenient);
}

bool SimplePruner::is_inside_ortho(osg::Vec3 node, bool lenient)
{
    float x = node.x(), z = node.z();

    //use orthogonal projection to compute the coordinates
    //in skeleton space
    float qX = x, qY = z; //ignore y

    //find the image coordinates by inverting the processes
    qY *= -1;
    
    qX /= _scale;
    qY /= _scale;
    qX += _rootX;
    qY += _rootY;

    return is_inside(qX, qY, lenient);
}

int SimplePruner::width()
{
    int ret = -1;

    if(_segmentation)
        ret = _segmentation->width;

    return ret;
}

int SimplePruner::height()
{
    int ret = -1;

    if(_segmentation)
        ret = _segmentation->height;

    return ret;
}

uchar SimplePruner::read(int x, int y)
{
    uchar ret = 0;
    if(_segmentation && x >= 0 && x < width() && y >= 0 && y < height())
        ret = ((uchar *)(_segmentation->imageData + y*_segmentation->widthStep))[x];
    
    return ret;
}
