#include "VolumeSurface.h"
#include "ISPLoader.h"
#include <fstream>

VolumeSurface::VolumeSurface(): _k_d(-1.0f)
{
}

float VolumeSurface::load(std::string path, float mb_length_d)
{
    clear();

    std::string ext = path.substr(path.find_last_of(".") + 1);
    _save_name = path.substr(0, path.find_last_of("."));

    //load isp0
    if(ext == "isp0")
    {
        ISPLoader loader;
        loader.load(path);
        _surface_pts = loader.surface_points_fast(mb_length_d);
        _k_d = loader._k_d;
		_relative_pts = loader._relative_pts;

        //save(std::string("/Users/jacky/work/resources/branch_learner/surface.points"));
    }
    //load text as defined in VolumeSurface::save()
    else
    {
        std::ifstream fs(path.c_str());
        std::string s;

        //k_d
        getline(fs, s);
        sscanf(s.c_str(), "%f\n", &_k_d);

        //num of surface points
        int n;
        getline(fs, s);
        sscanf(s.c_str(), "%d\n", &n);

        for(int i=0; i<n; i++)
        {
            float x, y, z;
            getline(fs, s);
            sscanf(s.c_str(), "%f %f %f\n", &x, &y, &z);
            _surface_pts.push_back(osg::Vec3(x, y, z));
        }

		//num of relative points
        getline(fs, s);
        sscanf(s.c_str(), "%d\n", &n);

		for(int i=0; i<n; i++)
        {
            float x, y, z;
            getline(fs, s);
            sscanf(s.c_str(), "%f %f %f\n", &x, &y, &z);
            _relative_pts.push_back(osg::Vec3(x, y, z));
        }
    }

    return _k_d;
}

void VolumeSurface::save(std::string path)
{
    if(path == "")
        path = _save_name + ".points";

    int n = _surface_pts.size();
    int rn = _relative_pts.size();

    FILE *out = fopen(path.c_str(), "w");
    if(!out)
    {
        printf("VolumeSurface::save(%s) error\n", path.c_str());
        return;
    }
    
    //first line is k_d
    fprintf(out, "%f\n", _k_d);

    //second line is number of surface points
    fprintf(out, "%d\n", n);

    //all the points follow
    for(int i=0; i<n; i++)
    {
        osg::Vec3 p = _surface_pts[i];
        fprintf(out, "%f %f %f\n", p.x(), p.y(), p.z());
    }

	//third line is number of relative points
	fprintf(out, "%d\n", rn);

    //all the points follow
    for(int i=0; i<rn; i++)
    {
        osg::Vec3 p = _relative_pts[i];
        fprintf(out, "%f %f %f\n", p.x(), p.y(), p.z());
    }

    fclose(out);
}

void VolumeSurface::clear()
{
    _surface_pts.clear();
}

bool VolumeSurface::is_loaded()
{
    return !_surface_pts.empty();
}
