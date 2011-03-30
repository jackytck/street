#include "BillboardTree.h"
#include "PointModel.h"
#include "RectangleModel.h"
#include <osg/Geometry>
#include "RectPlacement.h"
#include "osgModeler.h"
#include "SimpleVolume.h"
#include "AdvancedVolume.h"
#include "Transformer.h"
#include <fstream>

BillboardTree::BillboardTree(std::vector <osg::Vec3> points): _mode(0), _points(points), _gpca_result(NULL), _number_of_points(-1), _number_of_planes(-1)
{
	_verbose = false;
	srand(time(NULL));
}

BillboardTree::BillboardTree(int mode, std::vector <osg::Vec3> points, std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> texs, const char *tex_path): _mode(mode),  _points(points), _gpca_result(NULL), _number_of_points(-1), _number_of_planes(-1), _all_v(all_v), _texs(texs)
{
    _tex_path = std::string(tex_path);

    //check if the size of arrays match
    if(_mode == 0)
        if(all_v.size() != texs.size() || all_v.size() != 4*points.size())
            printf("BillboardTree::BillboardTree():points(%d) all_v(%d) texs(%d) sizes error\n", int(points.size()), int(all_v.size()), int(texs.size()));

	_verbose = false;
}

BillboardTree::BillboardTree(int mode, osg::ref_ptr <osg::Vec3Array> points, osg::ref_ptr <osg::Vec3Array> all_v, osg::ref_ptr <osg::Vec2Array> texs, const char *tex_path): _mode(mode)
{
    std::vector <osg::Vec3> std_points;
    std::vector <osg::Vec3> std_all_v;
    std::vector <osg::Vec2> std_texs;

    for(unsigned int i=0; i<points->size(); i++)
        std_points.push_back((*points)[i]);

    for(unsigned int i=0; i<all_v->size(); i++)
        std_all_v.push_back((*all_v)[i]);

    for(unsigned int i=0; i<texs->size(); i++)
        std_texs.push_back((*texs)[i]);

    //BillboardTree(std_points, std_all_v, std_texs, tex_path); //wrong
    _points = std_points;
    _gpca_result = NULL;
    _number_of_points = -1;
    _number_of_planes = -1;
    _all_v = std_all_v;
    _texs = std_texs;
    _tex_path = std::string(tex_path);

    //check if the size of arrays match
    if(_mode == 0)
        if(_all_v.size() != _texs.size() || _all_v.size() != 4*_points.size())
            printf("BillboardTree::BillboardTree():points(%d) all_v(%d) texs(%d) sizes error\n", int(_points.size()), int(_all_v.size()), int(_texs.size()));

	_verbose = false;
}

BillboardTree::~BillboardTree()
{
    delete _gpca_result;
}

void BillboardTree::do_work(int n)
{
    if(_mode == 0)
    {
        do_GPCA(n);
        create_segmented_groups();
    }
    else if(_mode == 1)
    {
        do_GPCA2(n);
        create_segmented_groups2();
    }
    else if(_mode == 2)
    {
        setup_k_means(1, 50, 10);
    }
    else if(_mode == 3)
    {
        setup_t_planes(n);
    }

}

void BillboardTree::do_GPCA(int n)
{
    if(_points.empty())
    {
        printf("BillboardTree::do_GPCA _points empty error\n");
        return;
    }

    //first convert the points into CvMat *
    int numPoints = _points.size();
    int dim = 3;

    CvMat *CvMat_data = cvCreateMat(numPoints, dim, CV_64FC1);

    //cvmSet(M,i,j,2.0); // Set M(i,j)
    //t = cvmGet(M,i,j); // Get M(i,j)

    for(int i=0; i<numPoints; i++)
        for(int j=0; j<dim; j++)
        {
            cvmSet(CvMat_data, i, j, _points[i][j]);
        }

    //run GPCA
    _gpca_result = Hyperplane_GPCA_Known_Group_Count(CvMat_data, n);

    //set back all the clustering results for this instance
    if(_gpca_result)
    {
        //set the number of points
        _number_of_points = numPoints;

        //set the number of planes
        _number_of_planes = _gpca_result->numGroups;

        //set the assignments
        _assignments.clear();
        _assignments.resize(numPoints);

        for(int i=0; i<numPoints; i++)
        {
            int assignment = _gpca_result->assignments[i];
            _assignments[i] = assignment;
        }

        //set the normals
        _normals.clear();
        _normals.resize(_number_of_planes);
        for(int i=0; i<_number_of_planes; i++)
        {
            //t = cvmGet(M,i,j); // Get M(i,j)
            CvMat *row_vec = _gpca_result->normals[i];
            osg::Vec3 normal(cvmGet(row_vec, 0, 0), cvmGet(row_vec, 0, 1), cvmGet(row_vec, 0, 2));
            _normals[i] = normal;
        }
    }

    //if(_gpca_result)
    //    printf("GPCA(%p) is done\n", _gpca_result);

    //release the matrix
    cvReleaseMat(&CvMat_data);
}

void BillboardTree::do_GPCA2(int n)
{
    if(_all_v.empty())
    {
        printf("BillboardTree::do_GPCA _all_v empty error\n");
        return;
    }

    //first convert the points into CvMat *
    int numPoints = _all_v.size();
    int dim = 3;

    CvMat *CvMat_data = cvCreateMat(numPoints, dim, CV_64FC1);

    for(int i=0; i<numPoints; i++)
        for(int j=0; j<dim; j++)
        {
            cvmSet(CvMat_data, i, j, _all_v[i][j]);
        }

    //run GPCA
    _gpca_result = Hyperplane_GPCA_Known_Group_Count(CvMat_data, n);

    //set back all the clustering results for this instance
    if(_gpca_result)
    {
        //set the number of points
        _number_of_points = numPoints;

        //set the number of planes
        _number_of_planes = _gpca_result->numGroups;

        //set the assignments
        _assignments.clear();
        _assignments.resize(numPoints);

        for(int i=0; i<numPoints; i++)
        {
            int assignment = _gpca_result->assignments[i];
            _assignments[i] = assignment;
        }

        //set the normals
        _normals.clear();
        _normals.resize(_number_of_planes);
        for(int i=0; i<_number_of_planes; i++)
        {
            //t = cvmGet(M,i,j); // Get M(i,j)
            CvMat *row_vec = _gpca_result->normals[i];
            osg::Vec3 normal(cvmGet(row_vec, 0, 0), cvmGet(row_vec, 0, 1), cvmGet(row_vec, 0, 2));
            _normals[i] = normal;
        }
    }

    //release the matrix
    cvReleaseMat(&CvMat_data);
}

Hyperplane_GPCA_Result *BillboardTree::gpca(std::vector <osg::Vec3> v, int n)
{
    Hyperplane_GPCA_Result *ret = NULL;

    if(v.empty() || n <= 0)
    {
        printf("BillboardTree::gpca v empty or index error\n");
        return ret;
    }

    //first convert the points into CvMat *
    int numPoints = v.size();
    int dim = 3;

    CvMat *CvMat_data = cvCreateMat(numPoints, dim, CV_64FC1);

    //cvmSet(M,i,j,2.0); // Set M(i,j)
    //t = cvmGet(M,i,j); // Get M(i,j)

    for(int i=0; i<numPoints; i++)
        for(int j=0; j<dim; j++)
        {
            cvmSet(CvMat_data, i, j, v[i][j]);
        }

    //run GPCA
    ret = Hyperplane_GPCA_Known_Group_Count(CvMat_data, n);

    //release the matrix
    cvReleaseMat(&CvMat_data);

    return ret;
}

void BillboardTree::gpca(std::vector <osg::Vec3> v, std::vector <osg::Vec2> tex, std::vector <std::vector <osg::Vec3> >& ret_v, std::vector <std::vector <osg::Vec2> >& ret_tex, std::vector <osg::Vec3>& ret_n, int n)
{
    //check sizes first
    if(v.empty() || v.size() != tex.size() || n <= 0)
    {
        printf("BillboardTree::gpca: input errors\n");
        return;
    }

    Hyperplane_GPCA_Result *result = gpca(v, n);

    if(result)
    {
        //setup return values
        ret_v.clear();
        ret_tex.clear();
        ret_v.resize(result->numGroups);
        ret_tex.resize(result->numGroups);
        ret_n.clear();
        ret_n.resize(result->numGroups);

        //divide points and texs according the assignments
        for(unsigned int i=0; i<v.size(); i++)
        {
            int assignment = result->assignments[i];
            ret_v[assignment].push_back(v[i]);
            ret_tex[assignment].push_back(tex[i]);
        }

        //store back normals
        for(int i=0; i<result->numGroups; i++)
        {
            CvMat *row_vec = result->normals[i];
            osg::Vec3 normal(cvmGet(row_vec, 0, 0), cvmGet(row_vec, 0, 1), cvmGet(row_vec, 0, 2));
            ret_n[i] = normal;
        }
    }

    delete result;
}

std::vector <int> BillboardTree::k_means(std::vector <osg::Vec3> all_v, int cluster_cnt)
{
    std::vector <int> ret;

    if(cluster_cnt <= 0 || all_v.empty())
        return ret;

    //convert to CvMat
    CvMat *matrix = cvCreateMat(all_v.size(), 3, CV_32FC1);
    for(unsigned int i=0; i<all_v.size(); i++)
    {
        for(int j=0; j<3; j++)
            cvmSet(matrix, i, j, all_v[i][j]);
    }

    //resulting cluster-indices
    CvMat* cluster_index = cvCreateMat(all_v.size(), 1, CV_32SC1);

    //estimate the terminating distance
    float sep = 0.1f;
    if(all_v.size() >= 2)
        sep = (all_v[0] - all_v[1]).length();
    //printf("sep(%f)\n", sep);

    //k-means
    cvKMeans2(matrix, cluster_cnt, cluster_index, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 500, sep), 20);
    //cvKMeans2(matrix, cluster_cnt, cluster_index, cvTermCriteria(CV_TERMCRIT_ITER, 500, 0.01));

    //push back result to ret
    for(unsigned int i=0; i<all_v.size(); i++)
    {
        int index = cluster_index->data.i[i];
        ret.push_back(index);
    }

    //release all cv stuff
    cvReleaseMat(&matrix);
    cvReleaseMat(&cluster_index);

    return ret;
}

void BillboardTree::setup_k_means(int iteration, int k_cluster_size, int num_subspace)
{
    bool debug = false;

    //unused
    iteration = iteration;

    //check size of inputs
    if(_all_v.empty() || _all_v.size() != _texs.size() || _all_v.size() != _texs.size())
        return;

    //first do k-means on _all_v
    std::vector <int> assignments = k_means(_all_v, k_cluster_size);

    //debug k-means assignments
    if(false)
        for(unsigned int i=0; i<assignments.size(); i++)
            printf("assignments[%d] = %d\n", i, assignments[i]);

    //divide and store each group as a std::vector <osg::Vec3>
    std::vector <std::vector <osg::Vec3> > k_groups_v; //points
    std::vector <std::vector <osg::Vec2> > k_groups_tex; //texture coords
    k_groups_v.resize(k_cluster_size);
    k_groups_tex.resize(k_cluster_size);

    //i-th point goes to assignments[i]-th element in k_groups_v
    for(unsigned int i=0; i<assignments.size(); i++)
    {
        int group_id = assignments[i];
        if(group_id >= 0 && group_id < k_cluster_size)
        {
            k_groups_v[group_id].push_back(_all_v[i]);
            k_groups_tex[group_id].push_back(_texs[i]);
        }
        else
            printf("BillboardTree::setup_k_means: group_id(%d) error\n", group_id);
    }

    if(debug)
        for(unsigned int i=0; i<k_groups_v.size(); i++)
            printf("%d k-group: %d\n", i, int(k_groups_v[i].size()));

    //setup the required members of the instance
    //set the number of points
    _number_of_points = _all_v.size();

    //set the number of planes
    _number_of_planes = k_cluster_size * num_subspace;

    //re-set the _all_v and _texs coz the orders has been changed
    _all_v.clear();
    _texs.clear();

    //set the assignments
    _assignments.clear();

    //set the normals
    _normals.clear();
    _normals.resize(_number_of_planes);

    //set the segmentd groups
    _group_ptrs_v.clear();
    _group_ptrs_tex.clear();

    //for each clustered groups, do a GPCA
    for(int i=0; i<k_cluster_size; i++)
    {
        std::vector <std::vector <osg::Vec3> > gpca_groups_v;
        std::vector <std::vector <osg::Vec2> > gpca_groups_tex;
        std::vector <osg::Vec3> gpca_n;

        gpca(k_groups_v[i], k_groups_tex[i], gpca_groups_v, gpca_groups_tex, gpca_n, num_subspace);

        //store the normal
        for(int j=0; j<num_subspace; j++)
            _normals[i*num_subspace+j] = gpca_n[j];

        //store back the points and texs and assignments
        for(int j=0; j<num_subspace; j++)
            for(unsigned int k=0; k<gpca_groups_v[j].size(); k++)
            {
                _all_v.push_back(gpca_groups_v[j][k]);
                _texs.push_back(gpca_groups_tex[j][k]);
                _assignments.push_back(i*num_subspace+j);
            }

        //store back as segmented groups
        for(int j=0; j<num_subspace; j++)
        {
            _group_ptrs_v.push_back(to_osg_Vec3Array(gpca_groups_v[j]));
            _group_ptrs_tex.push_back(to_osg_Vec2Array(gpca_groups_tex[j]));
        }

        if(debug)
            for(int j=0; j<num_subspace; j++)
                printf("%d g-group(%d): size(%d) normal(%f %f %f)\n", i*num_subspace+j, j, int(gpca_groups_v[j].size()), gpca_n[j].x(), gpca_n[j].y(), gpca_n[j].y());
    }

    if(debug)
    {
        printf("_number_of_points(%d) should match _all_v(%d) and _texs(%d) and _assignments(%d)\n", _number_of_points, int(_all_v.size()), int(_texs.size()), int(_assignments.size()));

        for(unsigned int i=0; i<_assignments.size(); i++)
            printf("_assignments[%d] = %d\n", i, _assignments[i]);

        for(unsigned int i=0; i<_all_v.size(); i++)
        {
            osg::Vec3 p = _all_v[i];
            osg::Vec2 tex = _texs[i];
            int assign = _assignments[i];
            osg::Vec3 normal = _normals[assign];

            printf("%d: v(%f %f %f) tex(%f %f) assign(%d) normal(%f %f %f)\n", i, p.x(), p.y(), p.z(), tex.x(), tex.y(), assign, normal.x(), normal.y(), normal.z());
        }
    }
}

std::vector <int> BillboardTree::t_planes_recursive(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step)
{
	std::vector <int> ret;

    //check inputs
    if(all_v.empty() || v_step <= 0 || all_v.size() < 4 || all_v.size() != tex.size())
    {
        printf("BillboardTree::t_planes_recursive():input parameters error\n");
        return ret;
    }

	//1. find cg
	AdvancedVolume volume(v_step, all_v);
	osg::Vec3 cg = volume.cg();

	//2. get back all the non-empty voxels in (world coords)
	std::vector <osg::Vec3> plane_vs = volume.get_non_empty_voxels();//a representative point on each plane

	//3. push normal for each plane
    std::vector <osg::Vec3> normals; //all plane normals
	for(unsigned int i=0; i<plane_vs.size(); i++)
	{
		osg::Vec3 p = plane_vs[i];
		osg::Vec3 n = p - cg;
		n.normalize();
		
		normals.push_back(n);
	}

    //4. find all the points that are closer than a given plane within a threshold
    //   and add it to a clusters
    std::vector <osg::ref_ptr <osg::Vec3Array> > clusters_v; //set of pointers to the grouped points
    std::vector <osg::ref_ptr <osg::Vec2Array> > clusters_tex; //set of pointers to the grouped texture-coords
    std::vector <osg::Vec3> normals2;

    float threshold = ((all_v[0]-all_v[1]).length()+(all_v[2]-all_v[0]).length())/2.0f*0.5f;
	threshold *= 2.0f;//1.5f;//too large is not pretty
	if(_verbose)
		printf("BillboardTree::proximity_constant(%f).\n", threshold);

	//boolean map to store which leaf is done
	std::vector <bool> leaf_done(all_v.size()/4, false);

    //loop through each plane
    for(unsigned int i=0; i<normals.size(); i++)
    {
        //plane
        osg::Vec3 n = normals[i];
        osg::Vec3 np = plane_vs[i];

        //points and texs belong to this plane
        osg::ref_ptr <osg::Vec3Array> grouped_pts = new osg::Vec3Array;
        osg::ref_ptr <osg::Vec2Array> grouped_tex = new osg::Vec2Array;

        //loop each point is wrong, should loop each leaf, i.e. 4 points
        for(unsigned int j=0; j<all_v.size(); j+=4)
        {
            osg::Vec3 a = all_v[j];
            osg::Vec3 b = all_v[j+1];
            osg::Vec3 e = all_v[j+2];
            osg::Vec3 f = all_v[j+3];

            float dist_a = fabs(n * (a-np));
            float dist_b = fabs(n * (b-np));
            float dist_e = fabs(n * (e-np));
            float dist_f = fabs(n * (f-np));

            float cur_dist = (dist_a+dist_b+dist_e+dist_f) / 4;

            if(cur_dist <= threshold)
            {
                //printf("cur_dist(%f)\n", cur_dist);

                grouped_pts->push_back(a);
                grouped_pts->push_back(b);
                grouped_pts->push_back(e);
                grouped_pts->push_back(f);

                grouped_tex->push_back(tex[j]);
                grouped_tex->push_back(tex[j+1]);
                grouped_tex->push_back(tex[j+2]);
                grouped_tex->push_back(tex[j+3]);

				//this leaf is done (at least once)
				leaf_done[j/4] = true;
            }
        }

        //if(!grouped_pts->empty() && grouped_pts->size() > 20)
        if(!grouped_pts->empty())
        {
            clusters_v.push_back(grouped_pts);
            clusters_tex.push_back(grouped_tex);
            normals2.push_back(n);
        }
    }

	//store all the index of un-done leaves
	std::vector <int> leaf_undo;
	for(unsigned int i=0; i<leaf_done.size(); i++)
		if(!leaf_done[i])
		{
			leaf_undo.push_back(i);
			//printf("v %f %f %f\n", all_v[i].x(), all_v[i].y(), all_v[i].z());
		}

    //5. return the results BY APPENDING
	for(unsigned int i=0; i<clusters_v.size(); i++)
		ret_v.push_back(clusters_v[i]);
	for(unsigned int i=0; i<clusters_tex.size(); i++)
		ret_tex.push_back(clusters_tex[i]);
	for(unsigned int i=0; i<normals2.size(); i++)
		ret_n.push_back(normals2[i]);

	//return the undone index
	return leaf_undo;
}

void BillboardTree::t_planes2(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step)
{
    //check inputs
    if(all_v.empty() || v_step <= 0 || all_v.size() < 4 || all_v.size() != tex.size())
    {
        printf("BillboardTree::t_planes input parameters error\n");
        return;
    }

	//results of all recursive steps
    std::vector <osg::ref_ptr <osg::Vec3Array> > clusters_v; //set of pointers to the grouped points
    std::vector <osg::ref_ptr <osg::Vec2Array> > clusters_tex; //set of pointers to the grouped texture-coords
    std::vector <osg::Vec3> normals;

	//un-done index
	std::vector <int> undone;
	for(unsigned int i=0; i<all_v.size(); i++)
		undone.push_back(i);//all are un-done initially

	//pass this smaller and smaller set to the recursive step
	std::vector <osg::Vec3> pass_all_v = all_v;
	std::vector <osg::Vec2> pass_tex = tex;
	int pass_step = v_step;

	//debug
	int recursived = 0;

	while(undone.size() > 15)
	{
		undone = t_planes_recursive(pass_all_v, pass_tex, clusters_v, clusters_tex, normals, pass_step);

		//reset pass_* for the next loop
		std::vector <osg::Vec3> next_all_v;
		std::vector <osg::Vec2> next_tex;
		int next_step = std::max(3, pass_step/2);

		for(unsigned int i=0; i<undone.size(); i++)
		{
			next_all_v.push_back(pass_all_v[undone[i]]);
			next_tex.push_back(pass_tex[undone[i]]);
		}

		//debug
		recursived++;
		//printf("%d: undone(%d) v_step(%d)\n", recursived, int(undone.size()), pass_step);

		pass_all_v = next_all_v;
		pass_tex = next_tex;
		pass_step = next_step;
	}

    //return the results
    ret_v = clusters_v;
    ret_tex = clusters_tex;
    ret_n = normals;
}

/* legacy code, only for fallback only
 */
void BillboardTree::t_planes(std::vector <osg::Vec3> all_v, std::vector <osg::Vec2> tex, std::vector <osg::ref_ptr <osg::Vec3Array> >& ret_v, std::vector <osg::ref_ptr <osg::Vec2Array> >& ret_tex, std::vector <osg::Vec3>& ret_n, int v_step)
{
    bool debug = false;

    //check inputs
    if(all_v.empty() || v_step <= 0 || all_v.size() < 4 || all_v.size() != tex.size())
    {
        printf("BillboardTree::t_planes input parameters error\n");
        return;
    }

	//Old steps 1, 2, 3 are too memory intensive and not elegant
    ////1. find cg first
    //osg::Vec3 cg(0.0f, 0.0f, 0.0f);
    //for(unsigned int i=0; i<all_v.size(); i++)
    //    cg = cg + all_v[i];
    //cg = cg * 1.0f / all_v.size();

    ////2. construct a coarse volume grid for constructing unique planes
    //SimpleVolume volume(1000, 1000, 1000, v_step, v_step, v_step);
    ////SimpleVolume volume(100, 100, 100, v_step, v_step, v_step);
    //std::vector <SimpleVolumeIndex> potential_index;
    //for(unsigned int i=0; i<all_v.size(); i++)
    //    if(volume.is_empty(500+all_v[i].x(), 500+all_v[i].y(), all_v[i].z()))
    //        potential_index.push_back(volume.add_point(500+all_v[i].x(), 500+all_v[i].y(), all_v[i].z()));

    ////3. compute normal of each plane
    //std::vector <osg::Vec3> normals; //all plane normals
    //std::vector <osg::Vec3> plane_vs; //a point on each normal
    //for(unsigned int i=0; i<potential_index.size(); i++)
    //{
    //    SimpleVolumeIndex index = potential_index[i];
    //    osg::Vec3 p(index._vx*v_step-500, index._vy*v_step-500, index._vz*v_step);
    //    osg::Vec3 n = p - cg;
    //    n.normalize();
    //    
    //    normals.push_back(n);
    //    plane_vs.push_back(p);
    //}

	//New steps 1, 2, 3 use AdvancedVolume
	//1. find cg
	AdvancedVolume volume(v_step, all_v);
	osg::Vec3 cg = volume.cg();

	//2. get back all the non-empty voxels in (world coords)
	std::vector <osg::Vec3> plane_vs = volume.get_non_empty_voxels();//a representative point on each plane

	//3. push normal for each plane
    std::vector <osg::Vec3> normals; //all plane normals
	for(unsigned int i=0; i<plane_vs.size(); i++)
	{
		osg::Vec3 p = plane_vs[i];
		osg::Vec3 n = p - cg;
		n.normalize();
		
		normals.push_back(n);
	}

    //4. find all the points that are closer than a given plane within a threshold
    //   and add it to a clusters
    std::vector <osg::ref_ptr <osg::Vec3Array> > clusters_v; //set of pointers to the grouped points
    std::vector <osg::ref_ptr <osg::Vec2Array> > clusters_tex; //set of pointers to the grouped texture-coords
    std::vector <osg::Vec3> normals2;

    float threshold = ((all_v[0]-all_v[1]).length()+(all_v[2]-all_v[0]).length())/2.0f*0.5f;
	threshold *= 1.5f;//too large is not pretty
	//threshold = 3.0f;
    //float threshold = 2.8f; //better to be hard-coded
    //float threshold = 0.8f; //better to be hard-coded
	if(_verbose)
		printf("BillboardTree::proximity_constant(%f).\n", threshold);

	//boolean map to store which leaf is done
	std::vector <bool> leaf_done(all_v.size()/4, false);

    //loop through each plane
    for(unsigned int i=0; i<normals.size(); i++)
    {
        //plane
        osg::Vec3 n = normals[i];
        osg::Vec3 np = plane_vs[i];

        //points and texs belong to this plane
        osg::ref_ptr <osg::Vec3Array> grouped_pts = new osg::Vec3Array;
        osg::ref_ptr <osg::Vec2Array> grouped_tex = new osg::Vec2Array;

        //loop each point is wrong, should loop each leaf, i.e. 4 points
        for(unsigned int j=0; j<all_v.size(); j+=4)
        {
            osg::Vec3 a = all_v[j];
            osg::Vec3 b = all_v[j+1];
            osg::Vec3 e = all_v[j+2];
            osg::Vec3 f = all_v[j+3];

            float dist_a = fabs(n * (a-np));
            float dist_b = fabs(n * (b-np));
            float dist_e = fabs(n * (e-np));
            float dist_f = fabs(n * (f-np));

            float cur_dist = (dist_a+dist_b+dist_e+dist_f) / 4;

            if(cur_dist <= threshold)
            {
                grouped_pts->push_back(a);
                grouped_pts->push_back(b);
                grouped_pts->push_back(e);
                grouped_pts->push_back(f);

                grouped_tex->push_back(tex[j]);
                grouped_tex->push_back(tex[j+1]);
                grouped_tex->push_back(tex[j+2]);
                grouped_tex->push_back(tex[j+3]);

                //printf("cur_dist(%f)\n", cur_dist);

				//this leaf is done (at least once)
				leaf_done[j/4] = true;
            }
        }

        //if(!grouped_pts->empty() && grouped_pts->size() > 20)
        if(!grouped_pts->empty())
        {
            clusters_v.push_back(grouped_pts);
            clusters_tex.push_back(grouped_tex);
            normals2.push_back(n);
        }

        if(debug && normals2.size() > 25)
            break;
    }

	//store all the index of un-done leaves
	std::vector <int> leaf_undo;
	for(unsigned int i=0; i<leaf_done.size(); i++)
		if(!leaf_done[i])
		{
			leaf_undo.push_back(i);
			//printf("v %f %f %f\n", all_v[i].x(), all_v[i].y(), all_v[i].z());
		}

	printf("%d leaf undo(%f%%)\n", int(leaf_undo.size()), float(leaf_undo.size())/leaf_done.size()*100);

    if(debug)
    {
        for(unsigned int i=0; i<clusters_v.size(); i++)
        {
            printf("%d: clusters(%d)\n", i, int(clusters_v[i]->size()));
            //for(unsigned int j=0; j<clusters_v[i]->size(); j++)
            //    printf("%d: points(%f %f %f)\n", j, (*clusters_v[i])[j].x(), (*clusters_v[i])[j].y(), (*clusters_v[i])[j].z());
        }
        printf("cg(%f %f %f)\n", cg.x(), cg.y(), cg.z());
        for(unsigned int j=0; j<normals2.size(); j++)
            printf("%d: normal(%f %f %f)\n", j, normals2[j].x(), normals2[j].y(), normals2[j].z());
        //printf("BillboardTree::all_v(%d) t_planes: normals(%d) threshold(%f)\n", int(all_v.size()), int(normals.size()), threshold);
    }

    //5. return the results
    ret_v = clusters_v;
    ret_tex = clusters_tex;
    ret_n = normals2;
}

void BillboardTree::setup_t_planes(int v_step)
{
    bool debug = false;

    //check size of inputs and step
    if(_all_v.empty() || _all_v.size() != _texs.size() || _all_v.size() != _texs.size() || v_step <= 0)
        return;

    //1. do t_planes to get all clusters
    std::vector <osg::ref_ptr <osg::Vec3Array> > clusters_v;
    std::vector <osg::ref_ptr <osg::Vec2Array> > clusters_tex;
    std::vector <osg::Vec3> clusters_n;
    t_planes2(_all_v, _texs, clusters_v, clusters_tex, clusters_n, v_step);

    if(debug)
    {
        printf("clusters_v(%d)\n", int(clusters_v.size()));
        for(unsigned int i=0; i<clusters_v.size(); i++)
            printf("%d: size(%d)\n", i, int(clusters_v[i]->size()));
    }

    //2. setup the instance
    //re-set original input v and tex
    _all_v = to_std_vector(clusters_v);
    _texs = to_std_vector(clusters_tex);

    //set the number of points
    int numPoints = 0;
    for(unsigned int i=0; i<clusters_v.size(); i++)
        numPoints += clusters_v[i]->size();
    _number_of_points = numPoints;

    //set the number of planes
    _number_of_planes = clusters_v.size();

    //set the assignments
    _assignments.clear();
    _assignments.resize(numPoints);

    int idx = 0, assignment = 0;
    for(unsigned int i=0; i<clusters_v.size(); i++)
    {
        for(unsigned int j=0; j<clusters_v[i]->size(); j++)
        {
            _assignments[idx] = assignment;
            idx++;
        }
        assignment++;
    }

    //set the normals
    _normals.clear();
    _normals.resize(_number_of_planes);
    for(int i=0; i<_number_of_planes; i++)
    {
        _normals[i] = clusters_n[i];
    }

    //set the segmented groups
    _group_ptrs_v = clusters_v;
    _group_ptrs_tex = clusters_tex;

    if(debug)
        printf("_all_v(%d) _texs(%d) _number_of_points(%d) _number_of_planes(%d) _normals(%d) _group_ptrs_v(%d) _group_ptrs_tex(%d)\n", int(_all_v.size()), int(_texs.size()), _number_of_points, _number_of_planes, int(_normals.size()), int(_group_ptrs_v.size()), int(_group_ptrs_tex.size()));
}

int BillboardTree::number_of_subspace()
{
    return _number_of_planes;
}

int BillboardTree::which_assignment(int pt_idx)
{
    int ret = -1;

    //int max_index = (_mode == 0) ? int(_points.size()) : int(_all_v.size());
    int max_index = _number_of_points;

    if(pt_idx < 0 || pt_idx >= max_index)
    {
        printf("BillboardTree:: pt_idx(%d) exceeds number_of_subspace(%d) error\n", pt_idx, max_index);
        return ret;
    }
    
    ret = _assignments[pt_idx];

    return ret;
}

//should use indirection, coz not all modes use GPCA
osg::Vec3 BillboardTree::which_normal(int group_id)
{
    osg::Vec3 ret;

    int group_size = number_of_subspace();
    if(group_id < 0 || group_id >= group_size)
        return ret;

    ret = _normals[group_id];

    return ret;
}

void BillboardTree::create_segmented_groups()
{
    int num_of_subspace = number_of_subspace();

    if(num_of_subspace == -1)
    {
        printf("BillboardTree::create_segmented_groups:num_of_subspace(-1) error\n");
        return;
    }

    bool to_separate_leaf = _all_v.size() == 4 * _points.size() ? true : false;

    //initialize each vec3Array for storing all the points that belong to the same subspace
    std::vector <osg::ref_ptr <osg::Vec3Array> > group_ptrs;
    std::vector <osg::ref_ptr <osg::Vec3Array> > group_ptrs_v;
    std::vector <osg::ref_ptr <osg::Vec2Array> > group_ptrs_tex;

    for(int i=0; i<num_of_subspace; i++)
    {
        osg::ref_ptr <osg::Vec3Array> vertices = new osg::Vec3Array;
        group_ptrs.push_back(vertices);

        if(to_separate_leaf)
        {
            osg::ref_ptr <osg::Vec3Array> v_array = new osg::Vec3Array;
            group_ptrs_v.push_back(v_array);

            osg::ref_ptr <osg::Vec2Array> tex_coords = new osg::Vec2Array;
            group_ptrs_tex.push_back(tex_coords);
        }
    }

    //for each point, put into different group_ptrs
    for(unsigned int i=0; i<_points.size(); i++)
    {
        osg::Vec3 p = _points[i];
        int group_id = which_assignment(i);

        if(group_id == -1)
            printf("BillboardTree::visualize_segmentation: group_id(-1) error\n");
        else
        {
            group_ptrs[group_id]->push_back(p);

            if(to_separate_leaf)
            {
                //for each pos, i got 4 v and 4 tex
                for(int j=0; j<4; j++)
                {
                    group_ptrs_v[group_id]->push_back(_all_v[4*i + j]);
                    group_ptrs_tex[group_id]->push_back(_texs[4*i + j]);
                }
            }
        }
    }

    _group_ptrs = group_ptrs;
    _group_ptrs_v = group_ptrs_v;
    _group_ptrs_tex = group_ptrs_tex;
}

//only set _group_ptrs_v and _group_ptrs_tex,
//_group_ptrs is ignored
void BillboardTree::create_segmented_groups2()
{
    int num_of_subspace = number_of_subspace();

    //errors checking
    if(num_of_subspace == -1)
    {
        printf("BillboardTree::create_segmented_groups2:num_of_subspace(-1) error\n");
        return;
    }
    if(_all_v.empty())
    {
        printf("BillboardTree::create_segmented_groups2:_all_v empty error\n");
        return;
    }
    if(_all_v.size() != _texs.size())
    {
        printf("BillboardTree::create_segmented_groups2:_all_v(%d) _texs(%d) sizes not equal error\n", int(_all_v.size()), int(_texs.size()));
        return;
    }

    //initialize each vec3Array for storing all the points that belong to the same subspace
    //note that if a leaf belongs to different subspaces, that leaf will we put into those different subspaces
    std::vector <osg::ref_ptr <osg::Vec3Array> > group_ptrs_v;
    std::vector <osg::ref_ptr <osg::Vec2Array> > group_ptrs_tex;

    for(int i=0; i<num_of_subspace; i++)
    {
        osg::ref_ptr <osg::Vec3Array> v_array = new osg::Vec3Array;
        group_ptrs_v.push_back(v_array);

        osg::ref_ptr <osg::Vec2Array> tex_coords = new osg::Vec2Array;
        group_ptrs_tex.push_back(tex_coords);
    }

    //for each leaf, put into the proper subspace(s)
    for(unsigned int i=0; i<_all_v.size(); i+=4)
    {
        //assume each leaf has 4 vertices
        osg::Vec3 a = _all_v[i];
        osg::Vec3 b = _all_v[i+1];
        osg::Vec3 e = _all_v[i+2];
        osg::Vec3 f = _all_v[i+3];

        //the associated tex-coords
        osg::Vec2 ta = _texs[i];
        osg::Vec2 tb = _texs[i+1];
        osg::Vec2 te = _texs[i+2];
        osg::Vec2 tf = _texs[i+3];

        int group_id_a = which_assignment(i);
        int group_id_b = which_assignment(i+1);
        int group_id_e = which_assignment(i+2);
        int group_id_f = which_assignment(i+3);

        if(group_id_a == -1 || group_id_b == -1 || group_id_e == -1 || group_id_f == -1)
            printf("BillboardTree::visualize_segmentation: group_id(-1) error\n");
        else
        {
            std::vector <int> unique_group_id = unique_set(group_id_a, group_id_b, group_id_e, group_id_f);

            for(unsigned int j=0; j<unique_group_id.size(); j++)
            {
                int group_id = unique_group_id[j];

                group_ptrs_v[group_id]->push_back(a);
                group_ptrs_v[group_id]->push_back(b);
                group_ptrs_v[group_id]->push_back(e);
                group_ptrs_v[group_id]->push_back(f);

                group_ptrs_tex[group_id]->push_back(ta);
                group_ptrs_tex[group_id]->push_back(tb);
                group_ptrs_tex[group_id]->push_back(te);
                group_ptrs_tex[group_id]->push_back(tf);
            }
        }
    }

    //_group_ptrs = group_ptrs;
    _group_ptrs_v = group_ptrs_v;
    _group_ptrs_tex = group_ptrs_tex;
}

osg::ref_ptr <osg::Group> BillboardTree::visualize_segmentation()
{
    bool debug = false;

    osg::ref_ptr <osg::Group> ret = new osg::Group;

    std::vector <osg::ref_ptr <osg::Vec3Array> > point_ptrs = (_mode == 0) ? _group_ptrs : _group_ptrs_v;

    //for each group, use a PointModel with different color to visualize it
    //and construct a bounding-plane to bound it
    for(unsigned int i=0; i<point_ptrs.size(); i++)
    {
        //random color
        float r = (rand()%200 + 55.0f) / 255.0f;
        float g = (rand()%200 + 55.0f) / 255.0f;
        float b = (rand()%200 + 55.0f) / 255.0f;
        //osg::Vec3 rand_n(rand()%10, rand()%10, rand()%10);
        //rand_n.normalize();
        //float r = rand_n.x();
        //float g = rand_n.y();
        //float b = rand_n.z();

        if(debug && int(i) > 20)
        {
            r = 0.0f;
            g = 0.0f;
            b = 0.0f;
        }

        //visualize points
        PointModel pm;
        pm.setColor(r, g, b, 1.0f);
        pm.createScene(point_ptrs[i]);
        pm.setSize(2);

        //ret->addChild(pm.get());

        if(debug && int(i) > 20)
            continue;

        //visualize bounding-plane
        std::vector <osg::Vec3> corners = bounding_box(point_ptrs[i], which_normal(i));
        RectangleModel rm(corners);
        rm.setColor(r, g, b, 1.0f);
        rm.createScene();

        ret->addChild(rm.get());

        //visualize corners of bounding-plane
        osg::ref_ptr <osg::Vec3Array> plane_v = new osg::Vec3Array;
        for(unsigned int i=0; i<4; i++)
        {
            plane_v->push_back(corners[i]);
        }

        PointModel pm2;
        pm2.setColor(r, g, b, 1.0f);
        pm2.createScene(plane_v);
        pm2.setSize(3);

        ret->addChild(pm2.get());
    }

    return ret;
}

std::vector <osg::Vec3> BillboardTree::bounding_box(osg::ref_ptr <osg::Vec3Array> points, osg::Vec3 normal)
{
    std::vector <osg::Vec3> ret;

    normal.normalize();
    if(!points || points->empty() || normal.length() == 0.0)
        return ret;

    //corners
    osg::Vec3 topR, topL, bottomR, bottomL;

    //find the cg first
    osg::Vec3 cg(0.0f, 0.0f, 0.0f);
    for(unsigned int i=0; i<points->size(); i++)
    {
        osg::Vec3 p = (*points)[i];
        cg = cg + p;
    }
    cg = cg * 1.0f / points->size();
    //printf("cg(%f %f %f)\n", cg.x(), cg.y(), cg.z());

    //find the two basis on this plane
    osg::Vec3 u(normal.y(), normal.x()*-1.0f, 0);
    u.normalize();
    osg::Vec3 v = normal ^ u;

    //orthogonal projection of each point on the plane to find the coordinates
    float max_coord_x = -1.0f, min_coord_x = -1.0f, max_coord_y = -1.0f, min_coord_y = -1.0f;
    for(unsigned int i=0; i<points->size(); i++)
    {
        osg::Vec3 p = (*points)[i];
        float coord_x = p * u;
        float coord_y = p * v;

        //keep track of the extreme coords
        if(max_coord_x == -1.0f || coord_x > max_coord_x)
            max_coord_x = coord_x;
        if(min_coord_x == -1.0f || coord_x < min_coord_x)
            min_coord_x = coord_x;
        if(max_coord_y == -1.0f || coord_y > max_coord_y)
            max_coord_y = coord_y;
        if(min_coord_y == -1.0f || coord_y < min_coord_y)
            min_coord_y = coord_y;
    }

    //perpendicular vector from plane to cg
    osg::Vec2 cg_coord(cg*u, cg*v);
    osg::Vec3 plane_to_cg = cg - (u*cg_coord.x() + v*cg_coord.y());

    topR = u*max_coord_x + v*max_coord_y;
    topL = u*min_coord_x + v*max_coord_y;
    bottomR = u*max_coord_x + v*min_coord_y;
    bottomL = u*min_coord_x + v*min_coord_y;

    //add back the offset
    topR += plane_to_cg;
    topL += plane_to_cg;
    bottomR += plane_to_cg;
    bottomL += plane_to_cg;

    //push back results
    ret.push_back(topR);
    ret.push_back(topL);
    ret.push_back(bottomL);
    ret.push_back(bottomR);

    //debug
    //printf("cg(%f %f %f) normal(%f %f %f) u(%f %f %f) v(%f %f %f)\n", cg.x(), cg.y(), cg.z(), normal.x(), normal.y(), normal.z(), u.x(), u.y(), u.z(), v.x(), v.y(), v.z());
    //printf("topR(%f %f %f) topL(%f %f %f) botttomR(%f %f %f) bottomL(%f %f %f)\n", topR.x(), topR.y(), topR.z(), topL.x(), topL.y(), topL.z(), bottomR.x(), bottomR.y(), bottomR.z(), bottomL.x(), bottomL.y(), bottomL.z());

    return ret;
}

osg::Vec3 BillboardTree::plane_cg(int index)
{
    osg::Vec3 ret(0.0f, 0.0f, 0.0f);

    std::vector <osg::ref_ptr <osg::Vec3Array> > point_ptrs = (_mode == 0) ? _group_ptrs : _group_ptrs_v;

    if(index < 0 || index >= number_of_subspace())
        return ret;

    for(unsigned int i=0; i<point_ptrs[index]->size(); i++)
        ret = ret + (*point_ptrs[index])[i];

    if(!point_ptrs[index]->empty())
        ret = ret * 1.0f / point_ptrs[index]->size();

    return ret;
}

void BillboardTree::generate_textures(const char *path, float multiplier)
{
	//printf("BillboardTree::generate_textures():starts...\n");
    bool debug = false;

    //check if texture can be loaded
    //IplImage *texture = cvLoadImage(_tex_path.c_str());
    QImage img(_tex_path.c_str());
    if(img.isNull())
    {
        printf("_tex_path(%s) path error\n", _tex_path.c_str());
        return;
    }

    //check if the sizes of grouped vertices and tex-coords match
    if(_mode == 0)
    {
        bool size_match = true;
        if(_group_ptrs.size() != _group_ptrs_v.size() || _group_ptrs.size() != _group_ptrs_tex.size())
            size_match = false;
        if(size_match)
            for(unsigned int i=0; i<_group_ptrs.size(); i++)
            {
                if(_group_ptrs_v[i]->size() != _group_ptrs_tex[i]->size() || _group_ptrs_v[i]->size() != 4*_group_ptrs[i]->size())
                {
                    size_match = false;
                    break;
                }
            }
        if(!size_match)
        {
            printf("BillboardTree::generate_textures():_group_ptrs(%d) _group_ptrs_v(%d) _group_ptrs_tex(%d) sizes not match error\n", int(_group_ptrs.size()), int(_group_ptrs_v.size()), int(_group_ptrs_tex.size()));
            return;
        }
    }
    //else if(_mode == 1)
    else
    {
        bool size_match = true;
        if(_group_ptrs_v.size() != _group_ptrs_tex.size())
            size_match = false;
        if(size_match)
            for(unsigned int i=0; i<_group_ptrs_v.size(); i++)
            {
                if(_group_ptrs_v[i]->size() != _group_ptrs_tex[i]->size())
                {
                    size_match = false;
                    break;
                }
            }
        if(!size_match)
        {
            printf("BillboardTree::generate_textures(): _group_ptrs_v(%d) _group_ptrs_tex(%d) sizes not match error\n", int(_group_ptrs_v.size()), int(_group_ptrs_tex.size()));
            return;
        }
    }

    //points of interest under currentn mode
    std::vector <osg::ref_ptr <osg::Vec3Array> > points_ptrs = (_mode == 0) ? _group_ptrs : _group_ptrs_v;

    //the global ratio
    //float multiplier;
    //multiplier = 0.9;//20
    //multiplier = 1.65;//50
    //multiplier = 2.1;//500
    //multiplier = 1.90;//100
    //multiplier = 1.60;//50

    //const int texture_width = 1024;
    const int texture_width = 2048;
    //const int texture_width = 4096;
    float ratio = texture_scale_hint(points_ptrs, pow(double(texture_width*texture_width/number_of_subspace()), 0.5)*multiplier);

    //store the sequence of [w0][h0][w1][h1][w2][h2]... for all planes
    std::vector <int> rectangle_seq;
    rectangle_seq.resize(points_ptrs.size()*2);

    //re-set the texture
    _textures.clear();
    _textures.resize(points_ptrs.size());

    //for each plane, for each rectangle leaf, orthogonally project the 4 corners to the plane
    for(unsigned int i=0; i<points_ptrs.size(); i++)
    {
        //all vertices and tex of this subspace
        osg::ref_ptr <osg::Vec3Array> all_v = _group_ptrs_v[i];
        osg::ref_ptr <osg::Vec2Array> all_tex = _group_ptrs_tex[i];

        //the plane of this subspace
        osg::Vec3 normal = which_normal(i);
        std::vector <osg::Vec3> corners = bounding_box(points_ptrs[i], normal); //bounding box of the pos, not v

        if(int(corners.size()) != 4)
        {
            printf("%d-th plane: corners size(%d) not equal 4 error\n", i, int(corners.size()));
            continue;
        }

        //infer the width and height of this plane
        float w = (corners[0]-corners[1]).length();
        float h = (corners[2]-corners[1]).length();
        if(w == 0.0f or h == 0.0f)
            continue;
        w *= ratio;
        h *= ratio;

        //store the w,h for later tiling the textures
        //rectangle_seq.push_back(w);
        //rectangle_seq.push_back(h);
        rectangle_seq[2*i] = w;
        rectangle_seq[2*i+1] = h;

        if(debug)
            printf("billboard texture(%f, %f) ratio(%f)\n", w, h, ratio);

        //billboard texture for this plane
        QImage texture(w, h, QImage::Format_ARGB32);
        texture.fill(0);

        //find the two basis {u,v} on this plane
        //osg::Vec3 u(normal.y(), normal.x()*-1.0f, 0);
        //u.normalize();
        //osg::Vec3 v = normal ^ u;

        //should use the bound to form the basis {u,v} of this plane
        osg::Vec3 u = corners[0] - corners[1];
        osg::Vec3 v = corners[2] - corners[1];
        u.normalize();
        v.normalize();

        //the coord of the top-left corner in terms of basis {u,v}
        osg::Vec2 op(corners[1]*u, corners[1]*v);

        if(debug)
        {
            printf("plane %d: v->size(%d)\n", i, int(all_v->size()));
            //printf("op(%f %f)\n", op.x(), op.y());
        }

        //store each vertex and its texture coords for creating texture after the loop
        std::vector <osg::Vec2> bi_vs, bi_texs;


		//random color for this billboard (for illustration in paper only)
		bool visualize = false;
        int color_r = (rand()%210 + 45);
        int color_g = (rand()%210 + 45);
        int color_b = (rand()%210 + 45);

        //orthogonally project each leaf-point on the plane to find its coordinates
        for(unsigned int j=0; j<all_v->size(); j+=4)
        {
            bool debug2 = false;

            //original 3D points
            osg::Vec3 a = (*all_v)[j];
            osg::Vec3 b = (*all_v)[j+1];
            osg::Vec3 e = (*all_v)[j+2];
            osg::Vec3 f = (*all_v)[j+3];

            //original tex-coords(0 to 1) for the 4 corners
            osg::Vec2 ta = (*all_tex)[j];
            osg::Vec2 tb = (*all_tex)[j+1];
            osg::Vec2 te = (*all_tex)[j+2];
            osg::Vec2 tf = (*all_tex)[j+3];

            //this is very difficult to spot, the tx origin is at bottom-left corner, but QImage's origin is at top-left corner
            ta.y() = 1 - ta.y();
            tb.y() = 1 - tb.y();
            te.y() = 1 - te.y();
            tf.y() = 1 - tf.y();

            //convert original tex-coords(0 to 1) to actual u,v coords
            ta = osg::Vec2(ta.x() * img.width(), ta.y() * img.height());
            tb = osg::Vec2(tb.x() * img.width(), tb.y() * img.height());
            te = osg::Vec2(te.x() * img.width(), te.y() * img.height());
            tf = osg::Vec2(tf.x() * img.width(), tf.y() * img.height());

            //projected 2D coordinates in basis {u,v}
            osg::Vec2 ap(a*u, a*v);
            osg::Vec2 bp(b*u, b*v);
            osg::Vec2 ep(e*u, e*v);
            osg::Vec2 fp(f*u, f*v);

            if(debug2)
                printf("raw: ap(%f %f) bp(%f %f) ep(%f %f) fp(%f %f)\n", ap.x(), ap.y(), bp.x(), bp.y(), ep.x(), ep.y(), fp.x(), fp.y());

            //projected 2D points on the plane with respected to the top-left corner
            ap = ap - op;
            bp = bp - op;
            ep = ep - op;
            fp = fp - op;

            if(debug2)
                printf("op translated: ap(%f %f) bp(%f %f) ep(%f %f) fp(%f %f)\n", ap.x(), ap.y(), bp.x(), bp.y(), ep.x(), ep.y(), fp.x(), fp.y());

            //billboard texture coords == the poisition of the points on the plane
            ap = ap * ratio;
            bp = bp * ratio;
            ep = ep * ratio;
            fp = fp * ratio;

            //<ap,ta>, <bp,tb>, <ep,te>, <fp,tf> are the vertex-attribute pair
            //push the 4 corners first
            //bi_vs.push_back(ap);
            //bi_vs.push_back(bp);
            //bi_vs.push_back(ep);
            //bi_vs.push_back(fp);

            //bi_texs.push_back(ta);
            //bi_texs.push_back(tb);
            //bi_texs.push_back(te);
            //bi_texs.push_back(tf);

            //bilinear interpolation
            //bilinear_interpolate(ap, bp, ep, fp, ta, tb, te, tf, bi_vs, bi_texs);
			paste_sticker(ap, bp, ep, fp, ta, tb, te, tf, bi_vs, bi_texs);

			if(false)
			{
				//printf("%d: w(%d) h(%d)\n", i, int(w), int(h));
				for(unsigned int k=0; k<bi_vs.size(); k++)
                {
                    QPoint p(bi_vs[k].x(), bi_vs[k].y());
					//printf("set(%d %d)\n", int(bi_vs[k].x()), int(bi_vs[k].y()));
                    QPoint s(bi_texs[k].x(), bi_texs[k].y());
                    if(in_bound(p, w, h) && in_bound(s, img.width(), img.height()))
                    {
                        //QRgb color = img.pixel(QPoint(bi_texs[k].x(), bi_texs[k].y()));
						QRgb blue = qRgb(0, 0, 255);
						//if(qAlpha(color) != 0) //if not fully transparent
							texture.setPixel(p, blue);
                    }
                }
				bi_vs.clear();
                bi_texs.clear();
				break;
			}

            //debug
            if(debug2)
            {
                printf("v(%d): original tex: ta(%f %f) tb(%f %f) te(%f %f) tf(%f %f)\n", j, ta.x(), ta.y(), tb.x(), tb.y(), te.x(), te.y(), tf.x(), tf.y());
                printf("v(%d): billboard tex: ap(%f %f) bp(%f %f) ep(%f %f) fp(%f %f)\n", j, ap.x(), ap.y(), bp.x(), bp.y(), ep.x(), ep.y(), fp.x(), fp.y());
            }

            //if bi_vs.size() has reached 2000, then write back and clear this array, to prevent overflow
            if(bi_vs.size() >= 2000)
            {
                for(unsigned int k=0; k<bi_vs.size(); k++)
                {
                    QPoint p(bi_vs[k].x(), bi_vs[k].y());
                    QPoint s(bi_texs[k].x(), bi_texs[k].y());
                    if(in_bound(p, w, h) && in_bound(s, img.width(), img.height()))
                    {
                        QRgb color = img.pixel(QPoint(bi_texs[k].x(), bi_texs[k].y()));

						//only assign a non-transparent color to target
						if(qAlpha(color) != 0) //if not fully transparent
						{
							if(false && visualize)
								color = qRgb(color_r, color_g, color_b);
							texture.setPixel(p, color);
						}
                    }
                }
                bi_vs.clear();
                bi_texs.clear();
            }
        }

        //this will overflow the vector max size if no amortization above
        //retrieve and write back all the points on the texture
        for(unsigned int j=0; j<bi_vs.size(); j++)
        {
            QPoint p(bi_vs[j].x(), bi_vs[j].y());
            QPoint s(bi_texs[j].x(), bi_texs[j].y());
            if(in_bound(p, w, h) && in_bound(s, img.width(), img.height()))
            {
                QRgb color = img.pixel(QPoint(bi_texs[j].x(), bi_texs[j].y()));

				//only assign a non-transparent color to target
				if(qAlpha(color) != 0) //if not fully transparent
				{
					if(false && visualize)
						color = qRgb(color_r, color_g, color_b);
					texture.setPixel(p, color);
				}
            }
        }

		//borders
		if(visualize)
		{
			QRgb red = qRgb(255, 0, 0);
			red = qRgb(color_r, color_g, color_b);
			for(int ui=0; ui<int(w); ui++)
			{
				texture.setPixel(ui, 0, red);
				texture.setPixel(ui, int(h)-1, red);
			}
			for(int vi=0; vi<int(h); vi++)
			{
				texture.setPixel(0, vi, red);
				texture.setPixel(int(w)-1, vi, red);
			}
		}

        //save back the texture with proper indexed_name
        //if(!texture.save(QString("/tmp/billboard_%1.png").arg(i), "PNG", 100))
        //    printf("%d-th billboard texture cannot be saved successfully!\n", i);
        //else
            _textures[i] = texture;
    }

    //tile all textures into one big texture
    tile_textures(rectangle_seq, texture_width, path);
}

void BillboardTree::bilinear_interpolate(osg::Vec2 a, osg::Vec2 ta, osg::Vec2 b, osg::Vec2 tb, osg::Vec2 c, osg::Vec2 tc, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts)
{
    //return values, additive
    std::vector <osg::Vec2> ret_v, ret_t;

    //first check if the triangle is degenerated
    osg::Vec2 ab = b - a;
    osg::Vec2 ac = c - a;
    float ab_len = ab.length(), ac_len = ac.length(), bc_len = (b-c).length();

    if(ab_len == 0.0f || ac_len == 0.0f || bc_len == 0.0f)
        return;

    float small_threshold = 3.0f;

    if(ab_len < small_threshold || ac_len < small_threshold || bc_len < small_threshold)
        return;

    if((ta-tb).length() < small_threshold || (tc-tb).length() < small_threshold || (tc-ta).length() < small_threshold)
        return;

    float x1, y1, w1, x2, y2, w2, x3, y3, w3; //given

    //interpolate x-coord first
    x2 = a.x();
    y2 = a.y();
    w2 = ta.x();

    x1 = b.x();
    y1 = b.y();
    w1 = tb.x();

    x3 = c.x();
    y3 = c.y();
    w3 = tc.x();

    float det = x1*y2-x2*y1+x2*y3-x3*y2+x3*y1-x1*y3;
    if(det == 0.0f)
        return;

    if(fabs(det) < 0.01f)
    {
        //printf("det too small\n");
        return;
    }

    float Ax = ((y2-y3)*w1+(y3-y1)*w2+(y1-y2)*w3) / det;
    float Bx = ((x3-x2)*w1+(x1-x3)*w2+(x2-x1)*w3) / det;
    float Cx = ((x2*y3-x3*y2)*w1+(x3*y1-x1*y3)*w2+(x1*y2-x2*y1)*w3) / det;

    //then interpolate y-coord
    w2 = ta.y();
    w1 = tb.y();
    w3 = tc.y();

    float Ay = ((y2-y3)*w1+(y3-y1)*w2+(y1-y2)*w3) / det;
    float By = ((x3-x2)*w1+(x1-x3)*w2+(x2-x1)*w3) / det;
    float Cy = ((x2*y3-x3*y2)*w1+(x3*y1-x1*y3)*w2+(x1*y2-x2*y1)*w3) / det;

    //rasterize all the points inside the triangle

    //the step size
    float u_step = 1.0f / ac_len;
    float v_step = 1.0f / ab_len;

    //test: a smaller step
	if(true)
	{
		u_step *= 0.5;
		v_step *= 0.5;
	}
	if(false)
	{
		u_step *= 0.9;
		v_step *= 0.9;
	}

    int inter_cnt = 0;
    int inter_cnt_limit = 1000;
    //Barycentric coords, does not include end-points and edge-points
    for(float u=u_step; u<1; u+=u_step)
        for(float v=v_step; v<1; v+=v_step)
            if(u+v < 1)
            {
                osg::Vec2 p = a + ac*u + ab*v;
                ret_v.push_back(p);

                if(inter_cnt > inter_cnt_limit)
                {
                    goto Break;
                }

                inter_cnt++;
            }
    Break:

    //bilinear interpolate the texure attribute for each point in ret_v and push the result in ret_t
    /*
       DET	= determinant of the original matrix = x1*y2-x2*y1+x2*y3-x3*y2+x3*y1-x1*y3
       A 	= ((y2-y3)*w1+(y3-y1)*w2+(y1-y2)*w3) / DET
       B 	= ((x3-x2)*w1+(x1-x3)*w2+(x2-x1)*w3) / DET
       C 	= ((x2*y3-x3*y2)*w1+(x3*y1-x1*y3)*w2+(x1*y2-x2*y1)*w3) / DET
       w 	= A*x+B*y+C
     */       
    for(unsigned int i=0; i<ret_v.size(); i++)
    {
        osg::Vec2 p = ret_v[i];
        osg::Vec2 tex;

        float x, y, w;
        x = p.x();
        y = p.y();

        w = Ax*x+Bx*y+Cx;
        tex.x() = w;

        w = Ay*x+By*y+Cy;
        tex.y() = w;

        ret_t.push_back(tex);
    }

    //push back the results to the inputs
    for(unsigned int i=0; i<ret_v.size(); i++)
    {
        bi_vs.push_back(ret_v[i]);
        bi_ts.push_back(ret_t[i]);
    }
}

void BillboardTree::bilinear_interpolate(osg::Vec2 ap, osg::Vec2 bp, osg::Vec2 ep, osg::Vec2 fp, osg::Vec2 ta, osg::Vec2 tb, osg::Vec2 te, osg::Vec2 tf, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts)
{
    //triangle ap,bp,ep
    bilinear_interpolate(ap, ta, bp, tb, ep, te, bi_vs, bi_ts);

    //triangle bp,ep,fp
    bilinear_interpolate(bp, tb, ep, te, fp, tf, bi_vs, bi_ts);
}

void BillboardTree::paste_sticker(osg::Vec2 ap, osg::Vec2 bp, osg::Vec2 ep, osg::Vec2 fp, osg::Vec2 ta, osg::Vec2 tb, osg::Vec2 te, osg::Vec2 tf, std::vector <osg::Vec2>& bi_vs, std::vector <osg::Vec2>& bi_ts)
{
	//1. find the largest side and use it as the starting site
	float side_ab = (bp-ap).length();
	float side_ae = (ep-ap).length();
	float side_bf = (bp-fp).length();
	float side_ef = (fp-ep).length();

	int start_side = 0;
	if(side_ab > side_ae && side_ab > side_bf && side_ab > side_ef)
		start_side = 0;
	else if(side_ae > side_ab && side_ae > side_bf && side_ae > side_ef)
		start_side = 1;
	else if(side_bf > side_ab && side_bf > side_ae && side_bf > side_ef)
		start_side = 2;
	else if(side_ef > side_ab && side_ef > side_ae && side_ef > side_bf)
		start_side = 3;

	//2. from a given side, infer the top-left corner and the two perpendicular vectors that go east and south of the pasted leaf
	osg::Vec2 op, up, vp;//up, vp is the corner points
	osg::Vec2 perp, op1, op2, up1, up2, vp1, vp2, ab, ae, bf, fe;//tmp variables
	float perp_len;
	switch(start_side)
	{
		case 0:
			op = bp;
			vp = ap;
			ab = bp-ap;
			perp = osg::Vec2(ab.y(), -ab.x());
			perp.normalize();
			perp_len = side_ab / 2.0f;
			if(perp_len == 0.0f)
				return;
			up1 = op+perp*perp_len;
		    up2 = op-perp*perp_len;
			if(Transformer::orient(vp, op, up1) < 0)
				up = up1;
			else
				up = up2;
			break;
		case 1:
			ae = ep-ap;
			perp = osg::Vec2(ae.y(), -ae.x());
			perp.normalize();
			perp_len = side_ae * 2.0f;
			op1 = ap+perp*perp_len; 
			op2 = ap-perp*perp_len;
			if(Transformer::orient(ep, ap, op1) < 0)
				op = op1;
			else
				op = op2;
			vp = ap;
			up = op + ae;
			break;
		case 2:
			op = bp;
			up = fp;
			bf = fp-bp;
			perp = osg::Vec2(bf.y(), -bf.x());
			perp.normalize();
			perp_len = side_bf * 2.0f;
			vp1 = bp+perp*perp_len;
		   	vp2 = bp-perp*perp_len;
			if(Transformer::orient(vp1, bp, fp) < 0)
				vp = vp1;
			else
				vp = vp2;
			break;
		case 3:
			up = fp;
			fe = ep-fp;
			perp = osg::Vec2(fe.y(), -fe.x());
			perp.normalize();
			perp_len = side_ef / 2.0f;
			op1 = fp+perp*perp_len; 
			op2 = fp-perp*perp_len;
			if(Transformer::orient(op1, fp, ep) < 0)
				op = op1;
			else
				op = op2;
			vp = op + fe;
			break;
	}

	//debug
	//op = bp;
	//up = op + osg::Vec2(10, 0);
	//vp = op + osg::Vec2(0, 20);

	//3. rasterize from op, up, vp and get back the corresponding texture coords
	// divide u direction in 128 steps, v direction in 256 steps
	osg::Vec2 tu = tf - tb, tv = ta - tb;
	int sw = tu.length(), sh = tv.length();
	if(tu.y() != 0.0f || tv.x() != 0.0f)
	{
		//printf("tu(%f %f) tv(%f %f)\n", tu.x(), tu.y(), tv.x(), tv.y());
		return;
	}
	te = te;//te is unused, assume its a quad, for realistic leaf
	osg::Vec2 u = up - op, v = vp - op;
	int u_step = 128, v_step = 256;

	//skip very small leaf
	//if(u.length() < 3 || v.length() < 3)
	//	return;

	//some tricks
	u_step = int(u.length())*3;
	v_step = int(v.length())*3;
	float scaleup = 1.0f;
	u = u*scaleup;
	v = v*scaleup;

	//debug u, v
	if(false)
	{
		for(int i=0; i<u_step; i++)
		{
			osg::Vec2 vs = op + u*float(i)/u_step;
			osg::Vec2 ts = tb + tu*float(i)/u_step;
			bi_vs.push_back(vs);
			bi_ts.push_back(ts);

			vs = op + u*float(i)/u_step + v;
			ts = tb + tu*float(i)/u_step + tv;
			bi_vs.push_back(vs);
			bi_ts.push_back(ts);
		}
		for(int i=0; i<v_step; i++)
		{
			osg::Vec2 vs = op + v*float(i)/v_step;
			osg::Vec2 ts = tb + tv*float(i)/v_step;
			bi_vs.push_back(vs);
			bi_ts.push_back(ts);

			vs = op + v*float(i)/v_step + u;
			ts = tb + tv*float(i)/v_step + tu;
			bi_vs.push_back(vs);
			bi_ts.push_back(ts);
		}
	}

	for(int i=0; i<u_step; i++)
		for(int j=0; j<v_step; j++)
		{
			osg::Vec2 vs = op + u*float(i)/u_step + v*float(j)/v_step;
			//osg::Vec2 ts = tb + tu*float(i)/u_step + tv*float(j)/v_step;
			osg::Vec2 ts = tb + osg::Vec2(sw, 0)*float(i)/u_step + osg::Vec2(0, sh)*float(j)/v_step;

			bi_vs.push_back(vs);
			bi_ts.push_back(ts);
		}
}

osg::ref_ptr <osg::Geode> BillboardTree::get_output(bool maya, bool double_side)
{
    osg::ref_ptr <osg::Geode> ret = new osg::Geode;

    //construct the billboard's geometry one by one
    osg::ref_ptr <osg::Vec3Array> v = new osg::Vec3Array; //stores billboard geometry serially
    osg::ref_ptr <osg::Vec3Array> n = new osg::Vec3Array; //stores billboard normals serially

    std::vector <osg::ref_ptr <osg::Vec3Array> > points_ptrs = (_mode == 0) ? _group_ptrs : _group_ptrs_v;

    //for each plane, get and push the 4 corners of a billboard
    for(unsigned int i=0; i<points_ptrs.size(); i++)
    {
        osg::Vec3 normal = which_normal(i);
        std::vector <osg::Vec3> corners = bounding_box(points_ptrs[i], normal);

        if(corners.size() != 4)
        {
            //printf("BillboardTree::get_output(): points_ptrs.size(%d) billboard(%d) corners(%d) not equal 4 error\n", int(points_ptrs[i]->size()), i, int(corners.size()));
            //continue;
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
        }

        osg::Vec3 a = corners[2];
        osg::Vec3 b = corners[1];
        osg::Vec3 e = corners[3];
        osg::Vec3 f = corners[0];

        v->push_back(a);
        v->push_back(b);
        v->push_back(e);
        v->push_back(f);

        n->push_back(normal);

        if(false)
        {
            printf("a(%f %f %f)\n", a.x(), a.y(), a.z());
            printf("b(%f %f %f)\n", b.x(), b.y(), b.z());
            printf("e(%f %f %f)\n", e.x(), e.y(), e.z());
            printf("f(%f %f %f)\n", f.x(), f.y(), f.z());
        }
        //break;
    }

    //swap axis for v and n
    if(maya)
    {
        //v
        for(unsigned int i=0; i<v->size(); i++)
        {
            osg::Vec3 p = (*v)[i];
            float tmp = p.y();
            p.y() = p.z();
            p.z() = -tmp;
            (*v)[i] = p;
        }
        //n
        for(unsigned int i=0; i<n->size(); i++)
        {
            osg::Vec3 p = (*n)[i];
            float tmp = p.y();
            p.y() = p.z();
            p.z() = -tmp;
            (*n)[i] = p;
        }
    }

    //texture coords
    osg::ref_ptr <osg::Vec2Array> tex = new osg::Vec2Array;
    for(unsigned int i=0; i<_texture_coords.size(); i++)
        tex->push_back(_texture_coords[i]);

    //double side?
    if(double_side)
    {
        //printf("BillboardTree::get_output:before: v(%d) n(%d) tex(%d)\n", int(v->size()), int(n->size()), int(tex->size()));
        double_side_it(v, n, tex);
        //printf("BillboardTree::get_output:after: v(%d) n(%d) tex(%d)\n", int(v->size()), int(n->size()), int(tex->size()));
    }

    //create a giant Geometry to store all the billboard
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(v.get());

    //bind a transparent color, to counteract the effect of the branch's color
    osg::Vec4Array *colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.8, 0.8, 0.8, 1.0));
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    //bind normals to each leaf
    geom->setNormalArray(n.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE);

    //bind texture array
    geom->setTexCoordArray(0, tex);

    //number of vertex per leaf
    int leaf_size = 4;

    //triangle strip
    for(unsigned int i=0; i<v->size()/leaf_size; i++)
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, leaf_size*i, leaf_size));

    ret->addDrawable(geom.get());

    if(!_tiled_texture_path.empty())
    {
        osg::StateSet *leaf_state = osgModeler::leaf_stateset(_tiled_texture_path.c_str());
        ret->setStateSet(leaf_state);
    }

    //debug
    //printf("BillboardTree::get_output() v(%d)\n", int(v->size()));

    return ret;
}

std::string BillboardTree::save_output(std::string path, bool double_side)
{
	std::string ret("");
    std::vector <osg::Vec3> v;
    std::vector <osg::Vec3> n;

    std::vector <osg::ref_ptr <osg::Vec3Array> > points_ptrs = (_mode == 0) ? _group_ptrs : _group_ptrs_v;

    //for each plane, get and push the 4 corners of a billboard
    for(unsigned int i=0; i<points_ptrs.size(); i++)
    {
        osg::Vec3 normal = which_normal(i);
        std::vector <osg::Vec3> corners = bounding_box(points_ptrs[i], normal);

        if(corners.size() != 4)
        {
            //printf("BillboardTree::get_output(): points_ptrs.size(%d) billboard(%d) corners(%d) not equal 4 error\n", int(points_ptrs[i]->size()), i, int(corners.size()));
            //continue;
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
            corners.push_back(osg::Vec3(0,0,0));
        }

        osg::Vec3 a = corners[2];
        osg::Vec3 b = corners[1];
        osg::Vec3 e = corners[3];
        osg::Vec3 f = corners[0];

        v.push_back(a);
        v.push_back(b);
        v.push_back(e);
        v.push_back(f);

        n.push_back(normal);
    }

    //texture coords, readily available
    std::vector <osg::Vec2> tex = _texture_coords;

    //double side?
    if(double_side)
        double_side_it(v, n, tex);

    //check data before saving
    if(v.size() != tex.size() || _tiled_texture_path.empty())
    {
        printf("BillboardTree::save_output():v.size(%d) tex.size(%d) _tiled_texture_path(%s) error\n", int(v.size()), int(tex.size()), _tiled_texture_path.c_str());
        return ret;
    }

	if(_verbose)
		printf("Saving billboard clouds to %s.\n", path.c_str());

    //SAVING
    FILE *out = fopen(path.c_str(), "w");

    //first line is number of vertice
    fprintf(out, "%d\n", int(v.size()));

    //v.size()'s lines of vertice
    for(unsigned int i=0; i<v.size(); i++)
        fprintf(out, "%f %f %f\n", v[i].x(), v[i].y(), v[i].z());

    //v.size()'s lines of texure coords
    for(unsigned int i=0; i<tex.size(); i++)
        fprintf(out, "%f %f\n", tex[i].x(), tex[i].y());

    //file path of the tiled texture
    std::string::size_type idx = path.rfind('.');
    std::string path_new = path.substr(0, idx) + "_texture.png";
    fprintf(out, "%s\n", path_new.c_str());

    fclose(out);

	if(false)
	{
		//copy image "_tiled_texture_path" to the "path_new"
		FILE *from, *to;
		char ch;
		from = fopen(_tiled_texture_path.c_str(), "rb");
		to = fopen(path_new.c_str(), "wb");

		/* copy the file */
		while(!feof(from)) 
		{
			ch = fgetc(from);
			if(!feof(from)) 
				fputc(ch, to);
		}

		fclose(from);
		fclose(to);

		//remove the file in "_tiled_texture_path"
		remove(_tiled_texture_path.c_str());
	}

	return path_new;
}

float BillboardTree::texture_scale_hint(std::vector <osg::ref_ptr <osg::Vec3Array> > points_ptrs, float max)
{
    float ret = -1.0f;

    if(points_ptrs.empty() || max <= 0)
        return ret;

    //for each plane, find its 4 corners, and find the min ratio
    for(unsigned int i=0; i<points_ptrs.size(); i++)
    {
        //the plane of this subspace
        osg::Vec3 normal = which_normal(i);
        std::vector <osg::Vec3> corners = bounding_box(points_ptrs[i], normal);

        if(int(corners.size()) != 4)
        {
            //printf("%d-th plane: corners size(%d) not equal 4 error\n", i, int(corners.size()));
            continue;
        }

        //infer the width and height of this plane
        float w = (corners[0]-corners[1]).length();
        float h = (corners[2]-corners[1]).length();
        if(w == 0.0f or h == 0.0f)
            continue;

        float ratio = max / std::max(w, h);

        if(ret == -1.0f || ratio < ret)
            ret = ratio;
    }

    return ret;
}

void BillboardTree::tile_textures(std::vector <int> rects, int size, const char *path)
{
    if(size <= 0)
        return;

    bool debug = false;

    //constants
    const int maxTexW = size;
    const int maxTexH = size;

    //convert to CSubRectArray type
    CSubRectArray vecSubRects;

    for(unsigned int i=0; i<rects.size(); i+=2)
    {
        TSubRect rect(rects[i], rects[i+1], i >> 1);
        vecSubRects.push_back(rect);
    }

    //the resultant texture
    CTextureArray vecTextures;

    // Sort the subRects based on dimensions, larger dimension goes first.
    std::sort(vecSubRects.begin(), vecSubRects.end(), CRectPlacement::TRect::Greater);

    // Generate the first texture
    vecTextures.clear();
    vecTextures.push_back(CRectPlacement());

    // Add all subrects
    for (CSubRectArray::iterator it = vecSubRects.begin(); it != vecSubRects.end(); ++it)
    {
        // We make sure we leave one pixel between subrects, so texels don't bleed with bilinear.
        CRectPlacement::TRect r(0, 0, it->w+1, it->h+1);

        // If an existing texture has actual space
        bool bPlaced = false;
        for (unsigned int i = 0; !bPlaced && i < vecTextures.size(); i++)
        {
            bPlaced = vecTextures[i].AddAtEmptySpotAutoGrow(&r, maxTexW, maxTexH);
            if (bPlaced)
                it->nTex = i;
        }

        //not enouth space
        if(!bPlaced)
        {
            printf("BillboardTree::tile_textures() Subrect is too big error!\n");
            return;
        }

        // If correctly placed in a texture, the coords are returned in r.x and r.y
        // Store them.
        if (bPlaced)
        {
            it->x = r.x;
            it->y = r.y;
        }
    }

    //debug
    if(debug)
    {
        for (CSubRectArray::const_iterator it = vecSubRects.begin(); it != vecSubRects.end(); ++it)
        {
            printf("Subrect %d (originally %d), size %dx%d, goes into texture %d at pos %d,%d\n",
                    it - vecSubRects.begin(), it->n, it->w, it->h, it->nTex, it->x, it->y);
        }
    }

    //print out the texture efficiency only
	if(_verbose)
		printf("%d textures are tiled\n", int(vecSubRects.size()));
    for (CTextureArray::const_iterator it = vecTextures.begin(); it != vecTextures.end(); ++it)
    {
		if(_verbose)
			printf("  Texture %d, size %dx%d, Coverage %ld / %ld (%ld%%)\n",
					it - vecTextures.begin(), it->GetW(), it->GetH(),
					it->GetArea(), it->GetTotalArea(), it->GetArea()*100/it->GetTotalArea());
    }

    if(_textures.size() != vecSubRects.size())
    {
        _tiled_texture_path = "";
        printf("_textures(%d) and vecSubRects(%d)\n", int(_textures.size()), int(vecSubRects.size()));
        return;
    }
    else
    {
        _tiled_texture_path = std::string(path);
    }

    //prepare the texture
    QImage texture(maxTexW, maxTexH, QImage::Format_ARGB32);
    texture.fill(0);

    //prepare the texture-coords
    _texture_coords.clear();
    _texture_coords.resize(4 * _textures.size());

    for(CSubRectArray::const_iterator it = vecSubRects.begin(); it != vecSubRects.end(); ++it)
    {
        int index = it->n;
        int w = it->w;
        int h = it->h;

        //treated as top-left corner
        int pos_x = it->x;
        int pos_y = it->y;

        //current texture
        QImage cur = _textures[index];

        for(int i=pos_x; i<pos_x+w; i++)
            for(int j=pos_y; j<pos_y+h; j++)
            {
                QPoint from(i-pos_x, j-pos_y);
                QPoint to(i, j);

                QRgb color = cur.pixel(from);
                texture.setPixel(to, color);
            }

        //store back the texture-coords
        osg::Vec2 bl_tex(pos_x/float(maxTexW), pos_y/float(maxTexH)); //b
        osg::Vec2 br_tex(pos_x/float(maxTexW) + w/float(maxTexW), pos_y/float(maxTexH)); //f
        osg::Vec2 tl_tex(pos_x/float(maxTexW), pos_y/float(maxTexH) + h/float(maxTexH)); //a
        osg::Vec2 tr_tex(pos_x/float(maxTexW) + w/float(maxTexW), pos_y/float(maxTexH) + h/float(maxTexH)); //e

        //tex-coords' origin is at the bottom-left corner
        osg::Vec2 a_tex(tl_tex.x(), 1.0f-tl_tex.y());
        osg::Vec2 b_tex(bl_tex.x(), 1.0f-bl_tex.y());
        osg::Vec2 e_tex(tr_tex.x(), 1.0f-tr_tex.y());
        osg::Vec2 f_tex(br_tex.x(), 1.0f-br_tex.y());

        _texture_coords[index*4] = a_tex;
        _texture_coords[index*4+1] = b_tex;
        _texture_coords[index*4+2] = e_tex;
        _texture_coords[index*4+3] = f_tex;
    }

    //texture.save(QString(path), "PNG", 100);//2048x2048 (16M)
    texture.save(QString(path), "PNG", 50);//middle quality: 2048x2048 (2.3M)

    if(false)
        for(unsigned int i=0; i<_texture_coords.size(); i++)
            printf("%d: _texture_coords(%f %f)\n", i, _texture_coords[i].x(), _texture_coords[i].y());
}

void BillboardTree::double_side_it(osg::ref_ptr <osg::Vec3Array>& v, osg::ref_ptr <osg::Vec3Array>& n, osg::ref_ptr <osg::Vec2Array>& tex)
{
    if(v->size() != n->size()*4 || v->size() != tex->size())
        return;

    //return values = old sides + new sides
    osg::ref_ptr <osg::Vec3Array> v2 = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> n2 = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> tex2 = new osg::Vec2Array;

    //re-order for v2 and tex2
    for(unsigned int i=0; i<v->size(); i+=4)
    {
        osg::Vec3 a = (*v)[i];
        osg::Vec3 b = (*v)[i+1];
        osg::Vec3 e = (*v)[i+2];
        osg::Vec3 f = (*v)[i+3];

        osg::Vec2 ta = (*tex)[i];
        osg::Vec2 tb = (*tex)[i+1];
        osg::Vec2 te = (*tex)[i+2];
        osg::Vec2 tf = (*tex)[i+3];

        //old side: a,b,e,f
        v2->push_back(a);
        v2->push_back(b);
        v2->push_back(e);
        v2->push_back(f);

        tex2->push_back(ta);
        tex2->push_back(tb);
        tex2->push_back(te);
        tex2->push_back(tf);

        //new side: b,a,f,e
        v2->push_back(b);
        v2->push_back(a);
        v2->push_back(f);
        v2->push_back(e);

        tex2->push_back(tb);
        tex2->push_back(ta);
        tex2->push_back(tf);
        tex2->push_back(te);
    }

    //flip normals
    for(unsigned int i=0; i<n->size(); i++)
    {
        osg::Vec3 normal = (*n)[i];

        n2->push_back(normal);
        n2->push_back(normal*-1.0f);
    }

    //set back the return values
    v = v2;
    n = n2;
    tex = tex2;
}

void BillboardTree::double_side_it(std::vector <osg::Vec3>& v, std::vector <osg::Vec3>& n, std::vector <osg::Vec2>& tex)
{
    osg::ref_ptr <osg::Vec3Array> osg_v = to_osg_Vec3Array(v);
    osg::ref_ptr <osg::Vec3Array> osg_n = to_osg_Vec3Array(n);
    osg::ref_ptr <osg::Vec2Array> osg_tex = to_osg_Vec2Array(tex);

    double_side_it(osg_v, osg_n, osg_tex);

    v.clear();
    n.clear();
    tex.clear();

    for(unsigned int i=0; i<osg_v->size(); i++)
        v.push_back((*osg_v)[i]);
    for(unsigned int i=0; i<osg_n->size(); i++)
        n.push_back((*osg_n)[i]);
    for(unsigned int i=0; i<osg_tex->size(); i++)
        tex.push_back((*osg_tex)[i]);
}

bool BillboardTree::in_bound(int x, int y, int w, int h)
{
    bool ret = true;

    if(x < 0 || x >= w || y < 0 || y >= h)
    {
        //printf("BillboardTree::in_bound check (%d %d) in (%d %d)\n", x, y, w, h);
        ret = false;
    }

    return ret;
}

bool BillboardTree::in_bound(osg::Vec2 v, int w, int h)
{
    return in_bound(v.x(), v.y(), w, h);
}

bool BillboardTree::in_bound(QPoint p, int w, int h)
{
    return in_bound(p.x(), p.y(), w, h);
}

QPoint BillboardTree::vec2_to_qpoint(osg::Vec2 v)
{
    QPoint ret(v.x(), v.y());
    return ret;
}

std::vector <int> BillboardTree::unique_set(int a, int b, int c, int d)
{
    std::vector <int> ret;

    ret.push_back(a);

    if(b != a)
        ret.push_back(b);

    bool unseen = true;
    for(unsigned int i=0; i<ret.size(); i++)
        if(c == ret[i])
        {
            unseen = false;
            break;
        }
    if(unseen)
        ret.push_back(c);

    unseen = true;
    for(unsigned int i=0; i<ret.size(); i++)
        if(d == ret[i])
        {
            unseen = false;
            break;
        }
    if(unseen)
        ret.push_back(d);

    return ret;
}

osg::ref_ptr <osg::Vec3Array> BillboardTree::to_osg_Vec3Array(std::vector <osg::Vec3> v)
{
    osg::ref_ptr <osg::Vec3Array> ret = new osg::Vec3Array;

    for(unsigned int i=0; i<v.size(); i++)
        ret->push_back(v[i]);

    return ret;
}

osg::ref_ptr <osg::Vec2Array> BillboardTree::to_osg_Vec2Array(std::vector <osg::Vec2> v)
{
    osg::ref_ptr <osg::Vec2Array> ret = new osg::Vec2Array;

    for(unsigned int i=0; i<v.size(); i++)
        ret->push_back(v[i]);

    return ret;
}

std::vector <osg::Vec3> BillboardTree::to_std_vector(std::vector <osg::ref_ptr <osg::Vec3Array> > pts)
{
    std::vector <osg::Vec3> ret;

    for(unsigned int i=0; i<pts.size(); i++)
        for(unsigned int j=0; j<pts[i]->size(); j++)
            ret.push_back((*pts[i])[j]);

    return ret;
}

std::vector <osg::Vec2> BillboardTree::to_std_vector(std::vector <osg::ref_ptr <osg::Vec2Array> > pts)
{
    std::vector <osg::Vec2> ret;

    for(unsigned int i=0; i<pts.size(); i++)
        for(unsigned int j=0; j<pts[i]->size(); j++)
            ret.push_back((*pts[i])[j]);

    return ret;
}
