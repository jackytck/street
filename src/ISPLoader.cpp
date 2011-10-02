#include "ISPLoader.h"
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include "Volume.h"
#include "FastCache.h"

ISPLoader::ISPLoader()
{
}

void ISPLoader::load(std::string path)
{
    std::ifstream fs(path.c_str());
    std::string s;

    getline(fs, s);
    if(!s.empty() && s[s.length()-1] == '\n')
        s.erase(s.length()-1);
    _img_path = s;

    getline(fs, s);
    if(!s.empty() && s[s.length()-1] == '\n')
        s.erase(s.length()-1);
    _seg_path = s;

    getline(fs, s);
    sscanf(s.c_str(), "%d %d", &_rootX, &_rootY);
}

std::vector <osg::Vec3> ISPLoader::absolute_contour_pts()
{
    std::vector <osg::Vec3> ret;

    //load the segmentation
    IplImage *cv_segmentation = cvLoadImage(_seg_path.c_str(), 0);//0 == CV_LOAD_IMAGE_GRAYSCALE

    if(!cv_segmentation)
        return ret;

    //convert segmentation to binary 8-bit mask
    //anythink greater than 1 is assign to 255, otherwise 0
    cvThreshold(cv_segmentation, cv_segmentation, 1, 255, CV_THRESH_BINARY);

    if(!cv_segmentation)
        return ret;

    //cvFindContours()
    CvMemStorage *storage = cvCreateMemStorage();
    CvSeq *contour = NULL, *contour_iter = NULL;
    //cvFindContours(cv_segmentation, storage, &first_contour, sizeof(CvContour), CV_RETR_EXTERNAL);
    cvFindContours(cv_segmentation, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //save back all the contour points
    contour_iter = contour;
    for( ; contour_iter != NULL; contour_iter = contour_iter->h_next )
    {
        //printf("total (%d)\n", contour_iter->total);

        for(int k=0; k<contour_iter->total; k++)
        {
            CvPoint *p = CV_GET_SEQ_ELEM(CvPoint, contour_iter, k);
            ret.push_back(osg::Vec3(p->x, p->y, 0));
        }
    }

    //for debugging purposes only, cvDrawContorus
    if(false)
    {
        IplImage *debug_8u3 = cvCreateImage(cvGetSize(cv_segmentation), IPL_DEPTH_8U, 3);

        contour_iter = contour;
        for( ; contour_iter != NULL; contour_iter = contour_iter->h_next )
            cvDrawContours(debug_8u3, contour_iter, CV_RGB(0xff,0x00,0x00), CV_RGB(0x00,0x00,0xff), 0, 2, 8);

        cvNamedWindow("cv_segmentation", CV_WINDOW_AUTOSIZE);
        cvShowImage("cv_segmentation", cv_segmentation);
        cvReleaseImage(&debug_8u3);
    }
    
    //release all cv images
    cvReleaseImage(&cv_segmentation);

    return ret;
}

std::vector <osg::Vec3> ISPLoader::relative_contour_pts(float mb_length_d)
{
    //relative contour
    std::vector <osg::Vec3> ret;

    //root
    osg::Vec3 root(_rootX, _rootY, 0.0f);

    //absolute contour
    std::vector <osg::Vec3> absolute_contour = absolute_contour_pts();

    //find the closest distance from root to the contour
    //call it mb_length
    float mb_length = -1.0f;
	float high_lenght = -1.0f;

	for(unsigned int i=0; i<absolute_contour.size(); i++)
	{
		//float cur = (absolute_contour[i]-root).length();
		osg::Vec3 pt = absolute_contour[i];
		float cur;
		if(abs(pt.x() - _rootX) < 20)
			cur = abs(absolute_contour[i].y() - _rootY);//closest vertical distance
		else
			continue;

		if(mb_length == -1.0f || cur < mb_length)
			mb_length = cur;

		if(high_lenght == -1.0f || cur > high_lenght)
			high_lenght = cur;
	}

    if(mb_length_d <= 0.0f)
        return ret;

    //scaling factor, also used in image_contour
    //float k_d = mb_length_d / mb_length; //exact match at the lowest point
	//float k_d = (mb_length_d*0.85f) / mb_length;
	//float k_d = (mb_length_d*1.02f) / mb_length;
	float k_d = (mb_length_d*1.0f) / mb_length;
    _k_d = k_d;

    //translate, scale and flip y-axis
    for(unsigned int i=0; i<absolute_contour.size(); i++)
    {
        osg::Vec3 absolute = absolute_contour[i];
        osg::Vec3 relative = (absolute - root) * k_d;
        relative.y() *= -1.0f;

        ret.push_back(relative);

		if(false) //set true and use make_initial.blend to make an initial skeleton
			printf("%f %f %f\n", relative.x(), relative.z(), relative.y());
    }

    return ret;
}

std::vector <osg::Vec3> ISPLoader::surface_points(float mb_length_d, int v_step, int v_size)
{
    std::vector <osg::Vec3> ret;

    //relative bound
    std::vector <osg::Vec3> relative_bounds = relative_contour_pts(mb_length_d);
	_relative_pts = relative_bounds;

    //center of mass
    osg::Vec3 cg = center_mass(relative_bounds);
    osg::Vec3 cg3(cg.x(), 0, cg.y());

    if(cg == osg::Vec3(-1.0f, -1.0f, -1.0f))
            return ret;

    if(v_step == -1)
    {
        v_step = mb_length_d / 6.0f;
        if(v_step < 1)
            v_step = 2;
    }

    //find the bound's metrics
    float b_h, w_r, w_l;
    bound_metrics(relative_bounds, cg, b_h, w_r, w_l);

    //default volume for calculating the voxel's index
    //vote this volume, check if the new index is really new by inspecting if the voxel has been voted
    Volume volume(v_size, v_size, v_size, v_step, v_step, v_step);

    //for each point in relative bounds, find the corresponding volume index
    for(unsigned int i=0; i<relative_bounds.size(); i++)
    {
        osg::Vec3 rb = relative_bounds[i];

        //radius
        float r = rb.x() - cg.x();

        //if(r < 0.0f)
        //    continue;

        //fit an ellipse
        osg::Vec3 R(rb.x()-cg.x(), 0.0f, rb.y()-cg.y());

        if(r > 0)
            r = R.length() * w_r/b_h;
        else
            r = R.length() * w_r/b_h;

        //empirical constant
        float rk = 2*w_r > b_h ? 1.2f : 1.5f;

        osg::Vec3 R_perp(0.0f, -r*rk, 0.0f);

        //debug one slice
        unsigned int size = relative_bounds.size();
        if(true || i==size/6 || i==4*size/6)
        {
            //rotate by half-circle
            for(int theta=-90; theta<90; theta++)
            {
                osg::Vec3 rb_t(r*cos(theta*M_PI/180.0), r*sin(theta*M_PI/180.0), rb.y());
                rb_t = cg3 + R_perp * cos((theta+90)*M_PI/180.0) + R * sin((theta+90)*M_PI/180.0);

                //check if it falls inside the segmentation mask
                //now just grow it inside this family of ellipses, pruning is done later
                //we cannot use the image contour since image contour depends on this surface
                if(true)// || _image_contour.isInsideOrtho(rb_t))//if false, then a lot of holes will appear
                {
                    //all index are raw, i.e. positive
                    VolumeIndex vi = volume.voxel_index(rb_t.x(), rb_t.y(), rb_t.z());

                    //check if this is new in volume
                    if(volume.count(vi) == 0)
                    {
                        //ret.push_back(vi);
                        osg::Vec3 p((vi._vx-v_size/2.0f/v_step) * v_step, (vi._vy-v_size/2.0f/v_step) * v_step, vi._vz * v_step);
                        ret.push_back(p);
                        
                        //vote back in the volume
                        volume.add_point(vi);
                    }
                }
            }
        }
    }

    //debug
    //for(unsigned int i=0; i<ret.size(); i++)
    //    printf("VolumeIndex: (%d %d %d)\n", ret[i]._vx-50, ret[i]._vy-50, ret[i]._vz);

    _surface_pts = ret;
    return ret;
}

std::vector <osg::Vec3> ISPLoader::surface_points_fast(float mb_length_d)
{
    std::vector <osg::Vec3> rbounds = relative_contour_pts(mb_length_d);
	_relative_pts = rbounds;

    FastCache fc(rbounds, 20);
    return fc.surface_points();
}

void ISPLoader::construct_min_texture(std::string path, osg::Vec3& cg, osg::Vec2& bottomleft, int& width, int& height)
{
    cg = osg::Vec3(0.0f, 0.0f, 0.0f); //for LeafGrower's which_quadrant

    if(path.empty())
        return;

    std::vector <osg::Vec3> contour = absolute_contour_pts();
    if(contour.empty())
        return;
    
    int most_left = -1, most_right = -1, most_top = -1, most_bottom = -1;

    for(unsigned int i=0; i<contour.size(); i++)
    {
        osg::Vec3 point = contour[i];
        int x = point.x();
        int y = point.y();

        if(most_left == -1 || x < most_left)
            most_left = x;

        if(most_right == -1 || x > most_right)
            most_right = x;

        if(most_top == -1 || y > most_top)
            most_top = y;

        if(most_bottom == -1 || y < most_bottom)
            most_bottom = y;

        cg.x() += x;
        cg.y() += y;
    }

    if(!contour.empty())
        cg = cg / contour.size();

    //top-left, bottom-right
    //QRect bound(QPoint(most_left, most_bottom), QPoint(most_right, most_top));
    bottomleft = osg::Vec2(most_left, most_top);
    width = most_right - most_left + 1;
    height = most_top - most_bottom + 1;

    //clip the image
    QImage img(_img_path.c_str());
    //QImage texture(bound.width(), bound.height(), QImage::Format_ARGB32);
    QImage texture(width, height, QImage::Format_ARGB32);
    texture.fill(0);

    //segmentation
    QImage seg(_seg_path.c_str());

    for(int i=most_left; i<=most_right; i++)
        for(int j=most_bottom; j<=most_top; j++)
        {
            QPoint tex_pt(i-most_left, j-most_bottom);

            QRgb color = img.pixel(QPoint(i, j));
            QRgb seg_color = seg.pixel(QPoint(i, j));

            //set only when it is inside the segmentation mask
            if(qRed(seg_color) != 0 || qGreen(seg_color) != 0 || qBlue(seg_color) != 0)
                texture.setPixel(tex_pt, color);
        }

    //save the texture
    texture.save(QString(path.c_str()), "PNG", 100);
}

osg::Vec3 ISPLoader::center_mass(std::vector <osg::Vec3> bounds)
{
    osg::Vec3 ret(-1.0f, -1.0f, -1.0f);

    osg::Vec3 sum(0.0f, 0.0f, 0.0f); //assume no overflow
    int cnt = 0;

    for(unsigned int i=0; i<bounds.size(); i++)
    {
        sum += bounds[i];
        cnt++;
    }

    if(cnt > 0)
    {
        sum = sum * (1.0f/cnt);
        ret = sum;
    }

    return ret;
}

void ISPLoader::bound_metrics(std::vector <osg::Vec3> rbound, osg::Vec3 cg, float& height, float& width_right, float& width_left)
{
    width_right = -1.0f;
    width_left = -1.0f;
    height = -1.0f;

    for(unsigned int i=0; i<rbound.size(); i++)
    {
        osg::Vec3 cur_p = rbound[i];
        float cur_d = abs(cur_p.x() - cg.x());

        //max width right
        if(cur_p.x() >= cg.x())
        {
            if(width_right == -1.0f || cur_d > width_right)
                width_right = cur_d;
        }
        //max width left
        else
        {
            if(width_left == -1.0f || cur_d > width_left)
                width_left = cur_d;
        }

        //max height
        for(unsigned int j=0; j<rbound.size(); j++)
            if(i != j)
            {
                float cur_h = abs(rbound[i].y() - rbound[j].y());
                if(height == -1.0f || cur_h > height)
                    height = cur_h;
            }
    }
}
