#include "LaserProjector.h"
#include "LaserCameraInfo.h"
#include "LaserCamera.h"
#include <fstream>

LaserProjector::LaserProjector(): _loaded(false)
{
}

bool LaserProjector::setup(std::string path)
{
    bool ret = false;

    std::ifstream fs(path.c_str());
    std::string s;
    int n;

    //num of view
    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &n);

    for(int i=0; i<n; i++)
    {
        //view id
        int view_id;
        getline(fs, s);
        sscanf(s.c_str(), "%d\n", &view_id);
        _view.push_back(view_id);

        //view image path
        getline(fs, s);
        if(!s.empty() && s[s.length()-1] == '\n')
            s.erase(s.length()-1);
        _view_img.push_back(s);

        //view image segmentation path
        getline(fs, s);
        if(!s.empty() && s[s.length()-1] == '\n')
            s.erase(s.length()-1);
        _view_seg.push_back(s);
    }

    //num of cameras
    getline(fs, s);
    sscanf(s.c_str(), "%d\n", &n);

    for(int i=0; i<n; i++)
    {
        int camera_id;
        float a, b, c, d, e, f, g, h, i, j, k, l;

        //camera id
        getline(fs, s);
        sscanf(s.c_str(), "%d\n", &camera_id);

        //3x4 projection matrix
        getline(fs, s);
        sscanf(s.c_str(), "%f %f %f %f %f %f %f %f %f %f %f %f\n", &a, &b, &c, &d, &e, &f, &g, &h, &i, &j, &k, &l);

        _bdlcameras.push_back(BDLCamera(camera_id, a, b, c, d, e, f, g, h, i, j, k, l));
    }

    //check if each view has 1936 matrix and if no. of image equals no. of segmentation
    if(_bdlcameras.size() == _view.size() * 1936 && _view_img.size() == _view_seg.size())
    {
        ret = true;
        _loaded = true;
    }
    else
        printf("_bdlcameras(%d) != _view(%d) * 1936 error\n", int(_bdlcameras.size()), int(_view.size()));

    return ret;
}

bool LaserProjector::is_loaded()
{
    return _loaded;
}

osg::Vec2 LaserProjector::project(osg::Vec3 P, int view, bool texture, bool fast)
{
    osg::Vec2 ret(-1.0f, -1.0f);
    if(view < 0 || view >= int(_view.size()))
        return ret;

    //1. construct a LaserCamera object for projecting on this view
    LaserCamera lc;
    int c_type = _view[view] % 9;//camera type
    int p_type = c_type == 8 ? 1 : 0;//projection type
    lc.set_id_type(_view[view], p_type);
    lc.set_parameters(c_type);
    lc.set_img_size();

    //2. set projection for each BDLCamera
    unsigned int start = view * 1936;
    unsigned int end = start + 1936;
    osg::Vec2 p;
    float min_diff = -1.0f;
    int min_index = -1;
    if(fast)
    {
        if(true)
            min_index = (start + end) / 2;//too inaccurate
        else
        {
            int big_step = 1936/10-1;
            for(unsigned int i=start; i<end; i+=big_step)
            {
                lc.set_projection(_bdlcameras[i]);

                //3. find the difference between scanline and x-coords of image coords
                p = lc.project(P);
                float diff = abs(p.x() - (i-start));
                
                if(min_diff == -1.0f || diff < min_diff)
                {
                    min_diff = diff;
                    min_index = i;
                }
            }
        }
    }
    else
    {
        for(unsigned int i=start; i<end; i++)
        {
            lc.set_projection(_bdlcameras[i]);

            //3. find the difference between scanline and x-coords of image coords
            p = lc.project(P);
            float diff = abs(p.x() - (i-start));
            
            if(min_diff == -1.0f || diff < min_diff)
            {
                min_diff = diff;
                min_index = i;
            }
        }
    }

    //4. project P by the closest projection matrix to get texture coords
    lc.set_projection(_bdlcameras[min_index]);

    if(texture)
        ret = lc.project2tex(P);
    else
        ret = lc.project(P);

    return ret;
}

std::string LaserProjector::view_name(int view)
{
    std::string ret;
    if(view < 0 || view >= int(_view.size()))
        return ret;

    int k = _view[view];
    char buffer[50];
    sprintf(buffer, "unstitched_%06d_%02d.jpg", k/9, k%9);
    ret = buffer;

    return ret;
}
