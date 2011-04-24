#include "osgModeler.h"
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/LineWidth>

#include <osgModeling/Helix>
#include <osgModeling/Nurbs>
#include <osgModeling/Extrude>
#include <osgModeling/Lathe>

#include <cmath>

#define isnan std::isnan

osgModeler::osgModeler()
{
}

osg::ref_ptr<osgModeling::Curve> osgModeler::createNurbsCircle(float r, int numPath)
{
    // A standard NURBS circle has 9 control points forming a square.
    osg::ref_ptr<osg::Vec3Array> ctrlPts = new osg::Vec3Array;
    ctrlPts->push_back(osg::Vec3( r ,0.0f, 0.0f)); ctrlPts->push_back(osg::Vec3( r , r , 0.0f));
    ctrlPts->push_back(osg::Vec3( 0.0f,r , 0.0f)); ctrlPts->push_back(osg::Vec3(-r , r , 0.0f));
    ctrlPts->push_back(osg::Vec3(-r ,0.0f, 0.0f)); ctrlPts->push_back(osg::Vec3(-r ,-r , 0.0f));
    ctrlPts->push_back(osg::Vec3( 0.0f,-r ,0.0f)); ctrlPts->push_back(osg::Vec3( r ,-r , 0.0f));
    ctrlPts->push_back(osg::Vec3( r ,0.0f, 0.0f));

    // The weights of 4 corners are less than 1.0, so they won't 'drag' the curve too much.
    osg::ref_ptr<osg::DoubleArray> weightPts = new osg::DoubleArray;
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f );

    // Set the knot vector. Vector size equals to number of control points + order(degree+1) of curve.
    osg::ref_ptr<osg::DoubleArray> knotPts = new osg::DoubleArray;
    knotPts->push_back( 0.0f ); knotPts->push_back( 0.0f );
    knotPts->push_back( 0.0f ); knotPts->push_back( 1.0f );
    knotPts->push_back( 1.0f ); knotPts->push_back( 2.0f );
    knotPts->push_back( 2.0f ); knotPts->push_back( 3.0f );
    knotPts->push_back( 3.0f ); knotPts->push_back( 4.0f );
    knotPts->push_back( 4.0f ); knotPts->push_back( 4.0f );

    // The code below shows 2 methods to configure the NURBS curve.
    // We may also use a third constructor, which is like the gluNurbsCurve, to easily read OpenGL curve data.
#if 0
    osg::ref_ptr<osgModeling::NurbsCurve> nurbsCurve = new osgModeling::NurbsCurve;
    nurbsCurve->setDegree( 2 );
    nurbsCurve->setNumPath( 100 );
    nurbsCurve->setKnotVector( knotPts.get() );
    nurbsCurve->setWeights( weightPts.get() );
    nurbsCurve->setCtrlPoints( ctrlPts.get() );
    nurbsCurve->update();
#else
    osg::ref_ptr<osgModeling::NurbsCurve> nurbsCurve =
        new osgModeling::NurbsCurve( ctrlPts.get(), weightPts.get(), knotPts.get(), 2, numPath );
#endif

    // use the loft tool to show the curve generated.
    /*
    osg::ref_ptr<osgModeling::Curve> section = new osgModeling::Curve;
    section->addPathPoint( osg::Vec3(-0.1f,0.0f,0.0f) );
    section->addPathPoint( osg::Vec3(0.1f,0.0f,0.0f) );
    osg::ref_ptr<osgModeling::Loft> circleLoft =
        new osgModeling::Loft( nurbsCurve.get(), section.get() );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( circleLoft.get() );
    return geode;
    */
    return nurbsCurve;
}

void osgModeler::createPartialCircle(float r, int numPath, double t, osgModeling::Curve *left, osgModeling::Curve *right)
{
    osg::ref_ptr<osgModeling::Curve> circle = createNurbsCircle(r, numPath);
    osg::ref_ptr<osg::Vec3Array> path_pts = circle->getPath();

    int start = 0, end = int(numPath * t);

    osg::ref_ptr<osg::Vec3Array> left_list = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> right_list = new osg::Vec3Array();

    for(int i=start; i<end; i++)
        left_list->push_back((*path_pts)[i]);

    for(int i=end; i<numPath; i++)
        right_list->push_back((*path_pts)[i]);

    left_list->push_back((*path_pts)[start]);
    right_list->push_back((*path_pts)[end]);

    if(left)
        left->setPath(left_list.get());
    if(right)
        right->setPath(right_list.get());
}

osg::ref_ptr<osg::Geode> osgModeler::createLoft(osg::Vec3Array *control_pts, unsigned int degree, std::vector <int> cross_r)
{
    //check argument
    int numPath = cross_r.size();

    osg::ref_ptr<osgModeling::Loft> geom = new osgModeling::Loft;
    //osg::ref_ptr<osgModeling::Curve> path = new osgModeling::Curve;
    osg::ref_ptr<osgModeling::BezierCurve> path = new osgModeling::BezierCurve(control_pts, degree, numPath);

    geom->setProfile( path.get() );

    for(unsigned int i=0; i<cross_r.size(); i++)
    {
        //geom->addShape( createNurbsCircle(2.0f).get() );
        if(i < 3)//to 'fix' the internal problem of Loft
            geom->addShape( createNurbsCircle(cross_r[0]).get() );
        if(cross_r.size()-i < 3)
            geom->addShape( createNurbsCircle(cross_r[cross_r.size()-1]).get() );
    }

    geom->update();

    //geom->getOrCreateStateSet()->setTextureAttributeAndModes(
    //    0, new osg::Texture2D(osgDB::readImageFile("/Users/jacky/Documents/dev/ss/osg/osgmodelling/wood_texture_shadow.jpg")) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    //geode->addDrawable( geom1.get() );
    geode->addDrawable( geom.get() );

    return geode;
}

std::vector <osg::ref_ptr <osgModeling::Loft> > osgModeler::segmentedStroke(osg::Vec3Array *control_pts, unsigned int degree, std::vector <int> cross_r, std::vector <double> breaking_t)
{
    //check if all values in breaking_t is witin 0 to 1

    int numPath = cross_r.size();
    int numSegment = breaking_t.size()+1;

    osg::ref_ptr<osgModeling::BezierCurve> path = new osgModeling::BezierCurve(control_pts, degree, numPath);
    osg::ref_ptr<osg::Vec3Array> path_pts = path->getPath();

    std::vector <osg::ref_ptr<osgModeling::Curve> > segmented_curve_list;
    std::vector <std::vector <int> > segmented_cross_list;

    for(int i=0; i<numSegment; i++)
    {
        double start = 0.0, end = 1.0;
        int start_num = 0, end_num = numPath-1;

        if(i>0)
            start = breaking_t[i-1];
        if(i!=numSegment-1)
            end = breaking_t[i];
        
        start_num = int((numPath-1) * start);
        end_num = int((numPath-1) * end);

        osg::ref_ptr<osg::Vec3Array> segmented_curve_i = new osg::Vec3Array();
        std::vector <int> segmented_cross_i;

        for(int j=start_num; j<=end_num; j++)
        {
            //osg::Vec3 pt = (*path_pts)[j];
            //osg::Vec3 offset(10*i, 0*i, -10*i);
            //pt += offset;
            segmented_curve_i->push_back((*path_pts)[j]);
            //segmented_curve_i->push_back(pt);
            segmented_cross_i.push_back(cross_r[j]);
        }
        
        osg::ref_ptr<osgModeling::Curve> segmented_path = new osgModeling::Curve;
        segmented_path->setPath(segmented_curve_i.get());

        segmented_curve_list.push_back(segmented_path);
        segmented_cross_list.push_back(segmented_cross_i);
    }

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    std::vector <osg::ref_ptr <osgModeling::Loft> > ret;

    int buffer = 10;
    double final_t = 0.7;
    std::vector <double> lerp_t;
    for(int i=1; i<=buffer; i++)
    {
        double s = double(i) / buffer;
        double t = s*final_t + (1-s);
        lerp_t.push_back(t);
    }
    int use_lerp_t = 0;

    for(int i=0; i<numSegment; i++)
    {
        osg::ref_ptr<osgModeling::Loft> geom = new osgModeling::Loft;
        geom->setProfile( segmented_curve_list[i].get() );

        for(unsigned int j=0; j<segmented_cross_list[i].size(); j++)
        {
            geom->addShape( createNurbsCircle(segmented_cross_list[i][j]).get() );

            //try avoiding the bug of osgModeler by smoothing the changes of cross-session (failed)
            /*
            if(j > segmented_cross_list[i].size() - buffer)
            {
                osg::ref_ptr<osgModeling::Curve> left = new osgModeling::Curve;
                double t = lerp_t[use_lerp_t];

                createPartialCircle(segmented_cross_list[i][j], 100/t, t, left.get());
                geom->addShape(left.get());

                use_lerp_t++;
            }
            */
        }
        use_lerp_t = 0;

        /*
        osg::ref_ptr<osgModeling::Curve> left = new osgModeling::Curve;
        createPartialCircle(segmented_cross_list[i][segmented_cross_list[i].size()-1]+10, 100, 0.7, left.get());
        geom->addShape(left.get());
        */

        geom->update();
        //geode->addDrawable( geom.get() );
        ret.push_back(geom);
    }

    //return geode;
    return ret;
}

//osg::ref_ptr <osg::Group> osgModeler::createTube(osg::ref_ptr <osg::Vec3Array> controlPoints, std::vector <int> cross_r, std::vector <double> breaking_t)
std::vector <OrientedCircle2> osgModeler::createTube(osg::ref_ptr <osg::Vec3Array> controlPoints, std::vector <int> cross_r, std::vector <double> breaking_t)
{
    std::vector <OrientedCircle2> ret;

    //osg::ref_ptr <osg::Group> group = new osg::Group;
    if(controlPoints->size() < 4 || cross_r.size() < 2)
        return ret;

    std::vector <osg::ref_ptr <osg::Vec3Array> > oc_list;
    std::vector <osg::Vec3> oc_center_list;
    std::vector <OrientedCircle> oc_class_list;

    for(unsigned int i=0; i<cross_r.size(); i++)
    {
        double t = double(i)/(cross_r.size()-1);

        osg::Vec3 center = center_3D(controlPoints, t);
        osg::Vec3 Tx = normal_3D(controlPoints, t);
        osg::Vec3 Tz = tangent_3D(controlPoints, t);
        osg::Vec3 Ty = Tz ^ Tx;

        osg::ref_ptr <osg::Vec3Array> oc = new osg::Vec3Array;
        //oc = orientedCircle(center, Tx*cross_r[i], Ty*cross_r[i], Tz*cross_r[i]);
        oc = orientedCircle(center, Tx*cross_r[0], Ty*cross_r[0], Tz*cross_r[0]);//Todo: cross_r smoothing

        OrientedCircle oc_class(center, Tx, Ty, Tz);
        //oc_class.setRadius(cross_r[i]);
        oc_class.setRadius(cross_r[0]);//Todo: cross_r smoothing

        oc_list.push_back(oc);
        oc_center_list.push_back(center);
        oc_class_list.push_back(oc_class);
    }

    int numSegment = breaking_t.size()+1;
    int numPath = cross_r.size();

    for(int i=0; i<numSegment; i++)
    {
        double start = 0.0, end = 1.0;
        int start_num = 0, end_num = numPath-1;

        if(i>0)
            start = breaking_t[i-1];
        if(i!=numSegment-1)
            end = breaking_t[i];

        start_num = int((numPath-1) * start);
        end_num = int((numPath-1) * end);//continuous
        //create discontinuity for numSegment > 1
        /*
        */
        if(numSegment != 1 && end_num > 0)
            end_num--;

        std::vector <osg::ref_ptr <osg::Vec3Array> > segment_i_oc_list;
        std::vector <osg::Vec3> segment_i_oc_center_list;

        for(int j=start_num; j<=end_num; j++)
        {
            segment_i_oc_list.push_back(oc_list[j]);
            segment_i_oc_center_list.push_back(oc_center_list[j]);
        }
        
        osg::ref_ptr <osg::Geode> geode = new osg::Geode;
        geode = triangulateTubeCurve(segment_i_oc_list, segment_i_oc_center_list);

        //group->addChild(geode);
        OrientedCircle down = oc_class_list[start_num];
        OrientedCircle up = oc_class_list[end_num];
        OrientedCircle2 oc2(up, down, geode);
        
        ret.push_back(oc2);
    }

    //return group;
    return ret;
}

//osg::ref_ptr <osg::Vec3Array> osgModeler::createTube(OrientedCircle *a, OrientedCircle *c)
osg::ref_ptr <osg::Group> osgModeler::createTube(OrientedCircle *a, OrientedCircle *c)
{
    osg::ref_ptr <osg::Group> root = new osg::Group;
    osg::ref_ptr <osg::Vec3Array> debug = new osg::Vec3Array;
    
    if(!a || !c || a->center() == c->center())
        return root;

    std::vector <int> ac_start = repositionOC2(a, c);
    osg::Vec3 track_start = (*c->points())[ac_start[1]];

    //find which way does cn point
    double ac_dist = dist(a->center(), c->center());
    osg::Vec3 cn = c->normal();
    cn.normalize();
    osg::Vec3 ctr_pt2 = c->center() + cn*ac_dist*0.3;
    if(dist(ctr_pt2, a->center()) > dist(c->center(), a->center()))
        cn = -cn;

    //find the angle between c_oc and a_oc
    double angle = acos(a->normal() * c->normal() / a->normal().length() / c->normal().length());
    if(angle > osg::PI/2.0)
        angle = osg::PI - angle;
    angle = 180 / osg::PI * angle;

    //rotate an existing oc
    OrientedCircle track_oc = *c;
    track_oc.makeRotate(track_start, cn ^ (track_start-c->center()), angle/2.0);

    //find orientations of three oc
    int a_orientation = 1, c_orientation = 1;
    if(orient3D((*c->points())[ac_start[1]], (*c->points())[modular(ac_start[1]+c->points()->size()/4, c->points()->size())], c->center(), a->center()) < 0)
        c_orientation = -1;
    if(orient3D((*a->points())[ac_start[0]], a->center(), (*a->points())[modular(ac_start[0]+a->points()->size()/4, a->points()->size())], c->center()) < 0)
        a_orientation = -1;

    std::vector <osg::ref_ptr <osgModeling::BezierCurve> > curve_list;

    for(unsigned int i=0; i<a->points()->size(); i++)
    {
        osg::Vec3 first, second, third;

        first = (*a->points())[modular(ac_start[0] + i*a_orientation, a->points()->size())];
        second = (*track_oc.points())[modular(ac_start[1] + i*c_orientation, track_oc.points()->size())];
        third = (*c->points())[modular(ac_start[1] + i*c_orientation, c->points()->size())];

        double t = dist(first, second) / (dist(first, second) + dist(third, second));
        osg::Vec3 mid_ctr = middle_ctr_pt(first, third, second, t);

        osg::ref_ptr <osg::Vec3Array> ctr_pts = new osg::Vec3Array;
        ctr_pts->push_back(first);
        ctr_pts->push_back(mid_ctr);
        ctr_pts->push_back(third);

        osg::ref_ptr <osgModeling::BezierCurve> curve = new osgModeling::BezierCurve(ctr_pts, 2, 40);
        curve_list.push_back(curve);

        /*
        for(unsigned int j=0; j<curve->getPath()->size(); j++)
            debug->push_back((*curve->getPath())[j]);
        */
    }

    osg::ref_ptr <osg::Geode> tube = new osg::Geode;
    tube = osgModeler::triangulateCurve(curve_list, track_oc.center());

    /*
    debug->push_back(track_start);
    debug->push_back(track_circle_center);
    for(unsigned int i=0; i<track_oc.points()->size(); i++)
        debug->push_back((*track_oc.points())[i]);
    */

    root->addChild(tube);
    root->addChild(visualizeCurve(debug, 1).get());

    return root;
}

osg::ref_ptr <osg::Geode> osgModeler::createTube(std::vector <osg::Vec3> path, std::vector <double> radii, int step, int step_circle)
//osg::ref_ptr <osg::Vec3Array> osgModeler::createTube(std::vector <osg::Vec3> path, std::vector <double> radii, int step)
{
    //printf("createTube starts.. %d\n", int(path.size()));

    osg::ref_ptr <osg::Geode> ret = new osg::Geode;
    //osg::ref_ptr <osg::Vec3Array> ret = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> debug = new osg::Vec3Array;

    //debug radii
    /*
    for(unsigned int i=0; i<radii.size(); i++)
        printf("radii[%d] = %f\n", i, radii[i]);
    */

    //size of path and radii do not match
    if(path.size() != radii.size())
        return ret;

    //only one or no point
    if(path.size() < 2)
        return ret;

    //step is smaller than 2
    if(step < 2)
        return ret;

    //if(path.size() == 3)
    if(path.size() == 2)
    {
        return createLineBox(path[0], path[1], (radii[0]+radii[1])/2.2);
        //return createLineBox(path[0], path[1], std::max(radii[0], radii[1]));
    }

    //return ret;

    //for each point in path, compute its OC, which is perpendicular to the path
    //loop from first to second last nodes, for each node, find the 4 control points of cubic bezier curve
    //that passes through this node and the next node
    
    //the list that store all the OrientedCircle and centers
    std::vector <osg::ref_ptr <osg::Vec3Array> > oc_list;
    std::vector <osg::Vec3> oc_center_list;

    //for c2 continuatiy
    osg::Vec3 second_ctr_pt;

    for(unsigned int i=0; i<path.size()-1; i++)
    {
        //Cubic BezierCurve control points for this segment
        osg::ref_ptr <osg::Vec3Array> ctr_pts = new osg::Vec3Array;

        double separation = dist(path[i], path[i+1]);
        osg::Vec3 third_ctr_pt;

        if(i==0)
        {
            second_ctr_pt = path[i+1] - path[i];
            second_ctr_pt.normalize();
            second_ctr_pt = path[i] + second_ctr_pt * separation * 0.5;
        }
        if(i!=path.size()-2)
        {
            third_ctr_pt = path[i+1] - path[i+2];
            third_ctr_pt.normalize();
            third_ctr_pt = path[i+1] + third_ctr_pt * separation * 0.5;
        }
        if(i==path.size()-2)
        {
            third_ctr_pt = path[i] - path[i+1];
            third_ctr_pt.normalize();
            third_ctr_pt = path[i+1] + third_ctr_pt * separation * 0.5;
        }

        ctr_pts->push_back(path[i]);
        ctr_pts->push_back(second_ctr_pt);
        ctr_pts->push_back(third_ctr_pt);
        ctr_pts->push_back(path[i+1]);

        //for each step, compute its corresponding center, tangent, normal, bi-normal for OC
        
        //for alignment of different OrientedCircles in the same segment
        osg::Vec3 last_Tx;

        //add more step for the last segment
        //if(i == path.size()-2)
        //    step++;

        for(int s=0; s<step; s++)
        {
            double t = s/double(step-1);
            double r = (1-t)*radii[i] + t*radii[i+1];

            //if the radius is zero, then the subsequence segment should also be zero,
            //so no need to push_back anything to oc_list and oc_center_list
            //visually, some leaves will be floating in the air, if not break, there will
            //be a tiny line that supports the leaf
            if(r == 0.0)
                break;

            osg::Vec3 Tc, Tx, Ty, Tz;

            Tc = center_3D(ctr_pts, t);
            Tx = normal_3D(ctr_pts, t);

            //flip the Tx according to the distance to its previous Tx
            if(s != 0)
            {
                osg::Vec3 flipped_Tx = Tx * -1.0;
                if(dist(last_Tx, Tx) > dist(last_Tx, flipped_Tx))
                    Tx = flipped_Tx;
            }
            last_Tx = Tx;

            Tz = tangent_3D(ctr_pts, t);
            Ty = Tz ^ Tx;

            //set the appropriate radius
            Tx = Tx * r;
            Ty = Ty * r;
            Tz = Tz * r;

            //detect nan
            if(isnan(Tx.x()) || isnan(Tx.y()) || isnan(Tx.z()))
                printf("hihi Tx nan\n");
            if(isnan(Ty.x()) || isnan(Ty.y()) || isnan(Ty.z()))
                printf("hihi Ty nan\n");
            if(isnan(Tz.x()) || isnan(Tz.y()) || isnan(Tz.z()))
                printf("hihi Tz nan\n");
            if(isnan(Tc.x()) || isnan(Tc.y()) || isnan(Tc.z()))
                printf("hihi Tc nan\n");

            osg::ref_ptr<osg::Vec3Array> oc = orientedCircle(Tc, Tx, Ty, Tz, step_circle);

            oc_list.push_back(oc);
            oc_center_list.push_back(Tc);

            /*
            for(int circle=0; circle<oc->size(); circle++)
            {
                osg::Vec3 oc_pt = (*oc)[circle];
                debug->push_back(osg::Vec3(oc_pt.x(), oc_pt.y(), oc_pt.z()));
            }
            debug->push_back(Tc);
            */
        }
        //set the second_ctr_pt for next iteration
        second_ctr_pt = path[i+1] + (path[i+1]-third_ctr_pt);
    }

    //triangulate the oc_list
    //ret->addChild(triangulateTubeCurve(oc_list, oc_center_list));
    ret = triangulateTubeCurve(oc_list, oc_center_list);
    //ret = debug;

    //printf("createTube: oc_list size = %d\n", int(oc_list.size()));

    return ret;
}

osg::ref_ptr <osg::Geode> osgModeler::createLineBox(osg::Vec3 a, osg::Vec3 b, double k)
{
    osg::ref_ptr <osg::Geode> ret = new osg::Geode;
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr <osg::Vec3Array> vertex = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> normal = new osg::Vec3Array;

    geom->setVertexArray(vertex.get());
    geom->setNormalArray(normal.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    //find the perpendicular directions first
    osg::Vec3 AB = b-a;
    osg::Vec3 X(AB.y(), -AB.x(), 0.0);
    X.normalize();
    osg::Vec3 Y = AB ^ X;
    Y.normalize();

    //then find the eight corners
    osg::Vec3 a0, a1, a2, a3, b0, b1, b2, b3;
    a0 = a + X*0.5*k + Y*0.5*k;
    a1 = a + X*0.5*k - Y*0.5*k;
    a2 = a - X*0.5*k - Y*0.5*k;
    a3 = a - X*0.5*k + Y*0.5*k;
    b0 = a0 + AB;
    b1 = a1 + AB;
    b2 = a2 + AB;
    b3 = a3 + AB;

    //printf("a0(%lf %lf %lf), a1(%lf %lf %lf) a2(%lf %lf %lf) a3(%lf %lf %lf)\n", a0.x(), a0.y(), a0.z(), a1.x(), a1.y(), a1.z(), a2.x(), a2.y(), a2.z(), a3.x(), a3.y(), a3.z());
    //printf("b0(%lf %lf %lf), b1(%lf %lf %lf) b2(%lf %lf %lf) b3(%lf %lf %lf)\n", b0.x(), b0.y(), b0.z(), b1.x(), b1.y(), b1.z(), b2.x(), b2.y(), b2.z(), b3.x(), b3.y(), b3.z());
    //printf("X(%lf %lf %lf) Y(%lf %lf %lf)\n", X.x(), X.y(), X.z(), Y.x(), Y.y(), Y.z());

    vertex->push_back(a1);
    vertex->push_back(b1);
    vertex->push_back(a0);
    vertex->push_back(b0);
    
    vertex->push_back(a3);
    vertex->push_back(b3);

    vertex->push_back(a2);
    vertex->push_back(b2);
    
    vertex->push_back(a1);
    vertex->push_back(b1);
    
    normal->push_back(X);
    normal->push_back(X);
    normal->push_back(X);
    normal->push_back(X);

    normal->push_back(Y);
    normal->push_back(Y);

    normal->push_back(X*(-1.0));
    normal->push_back(X*(-1.0));

    normal->push_back(Y*(-1.0));
    normal->push_back(Y*(-1.0));

    //printf("ab = %lf\n", dist(a, b));
    //printf("k = %lf\n", k);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 10));
    ret->addDrawable(geom.get());

    return ret;
}

osg::ref_ptr <osg::Geode> osgModeler::createLeaf(osg::Vec3 pos, osg::Vec3 to, double ang_z, double gS)
{
    osg::ref_ptr <osg::Geode> ret = new osg::Geode;
    //double scale = 0.5;
    double scale = (rand()%20+40)/100.0;

    //geomety

    //quad
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr <osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray(v.get());

    //vertices
    //flat
    /*
    v->push_back(pos + osg::Vec3(0.f, -1.f, -1.f)*scale);
    v->push_back(pos + osg::Vec3(0.f, -1.f, 2.11f)*scale);
    v->push_back(pos + osg::Vec3(0.f, 0.52f, -1.f)*scale);
    v->push_back(pos + osg::Vec3(0.f, 0.52f, 2.11f)*scale);
    */
    //un-flat 2-triangle
    /*
    v->push_back(pos + osg::Vec3(0.3f, -1.f, -1.f)*scale);
    v->push_back(pos + osg::Vec3(0.3f, -1.f, 2.11f)*scale);
    v->push_back(pos + osg::Vec3(-0.3f, 0.52f, -1.f)*scale);
    v->push_back(pos + osg::Vec3(0.3f, 0.52f, 2.11f)*scale);
    */
    //fold along the middle axis 4 triangle, pos is the origin
    /*
    int w = 128, h = 256, b = 100;
    v->push_back(pos + osg::Vec3(0.0f, 0.0f, 0.3f*b)*scale);
    v->push_back(pos + osg::Vec3(0.0f, 1.0f*h, 0.2f*b)*scale);
    v->push_back(pos + osg::Vec3(0.539f*w, 0.0449f*h, 0.0f)*scale);
    v->push_back(pos + osg::Vec3(0.3672f*w, 0.99023f*h, 0.0f)*scale);
    v->push_back(pos + osg::Vec3(1.0f*w, 0.0f, 0.3f*b)*scale);
    v->push_back(pos + osg::Vec3(1.0f*w, 1.0f*h, 0.2f*b)*scale);
    */

    //initial vertices values, depends on the texture
    osg::Vec3 a, b, c, d, e, f, from;
    a = osg::Vec3(-0.539f, 0.0f, -0.0449f);
    b = osg::Vec3(-0.539f, 0.0f, 0.9551f);
    c = osg::Vec3(0.0f, 0.0f, 0.0f);
    d = osg::Vec3(-0.1718f, 0.0f, 0.94533f);
    e = osg::Vec3(0.461f, 0.0f, -0.0449f);
    f = osg::Vec3(0.461f, 0.0f, 0.9551f);
    from = d - c;

    //fold the two sides along from by a random angle
    float width = 0.539;
    float fold_angle = (rand()%320+20)/2.0/180.0*M_PI;
    a.x() = b.x() = -width * sin(fold_angle);
    a.y() = b.y() = -width * cos(fold_angle);
    width = 0.461;
    e.x() = f.x() = width * sin(fold_angle);
    e.y() = f.y() = -width * cos(fold_angle);

    osg::Matrixf GS, S, T, R, Rz;
    GS.makeScale(gS, gS, gS);
    S.makeScale(128.f*scale, 100.f*scale, 256.f*scale);
    T.makeTranslate(pos);
    R.makeRotate(from, to);
    Rz.makeRotate(ang_z/180.*M_PI, 0, 0, 1);

    //resultant matrix, osg use post multiplication, not the same as OpenGL
    osg::Matrix M;
    M = S * GS * Rz * R * T;

    v->push_back(a * M);
    v->push_back(b * M);
    v->push_back(c * M);
    v->push_back(d * M);
    v->push_back(e * M);
    v->push_back(f * M);

    //normals
    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    geom->setNormalArray(n.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));

    //triangle strip
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 6));
    ret->addDrawable(geom.get());

    //texture coords
    osg::Vec2Array *texcoords = new osg::Vec2Array();
    texcoords->push_back(osg::Vec2(0.0f, 0.0f));
    texcoords->push_back(osg::Vec2(0.0f, 1.0f));
    texcoords->push_back(osg::Vec2(0.539f, 0.0449f));
    texcoords->push_back(osg::Vec2(0.3672f, 0.99023f));
    texcoords->push_back(osg::Vec2(1.0f, 0.0f));
    texcoords->push_back(osg::Vec2(1.0f, 1.0f));
    geom->setTexCoordArray(0, texcoords);

    //example usage of setting the StateSet
    //osg::StateSet *stateOne = leaf_stateset();
    //ret->setStateSet(stateOne);

    return ret;

    /*
    //texture
    osg::Texture2D *leaf_texture = new osg::Texture2D;
    // protect from being optimized away as static state:
    leaf_texture->setDataVariance(osg::Object::DYNAMIC); 
    // load an image by reading a file: 
    osg::Image* leaf_img = osgDB::readImageFile("/Users/jacky/work/resources/leaf/leaf3.png");
    if (!leaf_img)
        printf("Couldn't find texture!!");

    // Assign the texture to the image we read from file: 
    leaf_texture->setImage(leaf_img);

    // Create a new StateSet with default settings: 
    osg::StateSet* stateOne = new osg::StateSet();

    // Assign texture unit 0 of our new StateSet to the texture 
    // we just created and enable the texture.
    stateOne->setTextureAttributeAndModes(0, leaf_texture, osg::StateAttribute::ON);

    //transparency or alpha texturing
    stateOne->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateOne->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateOne->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

    // Associate this state set with the Geode
    ret->setStateSet(stateOne);

    return ret;
    */
}

void osgModeler::createLeaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::Vec3 pos, osg::Vec3 to, double ang_z, double gS, bool simple, bool constant_scale)
{
    //double scale = constant_scale ? 0.5 : (rand()%20+40)/100.0;
    double scale = constant_scale ? 0.5 : (rand()%40+30)/100.0;

    //initial vertices values, depends on the texture
    osg::Vec3 a, b, c, d, e, f, from;
    a = osg::Vec3(-0.539f, 0.0f, -0.0449f);
    b = osg::Vec3(-0.539f, 0.0f, 0.9551f);
    c = osg::Vec3(0.0f, 0.0f, 0.0f);
    d = osg::Vec3(-0.1718f, 0.0f, 0.94533f);
    e = osg::Vec3(0.461f, 0.0f, -0.0449f);
    f = osg::Vec3(0.461f, 0.0f, 0.9551f);
    from = d - c;

    //fold the two sides along from by a random angle
    if(!simple)
    {
        float width = 0.539;
        float fold_angle = constant_scale ? 0.5*M_PI : (rand()%320+20)/2.0/180.0*M_PI;
        a.x() = b.x() = -width * sin(fold_angle);
        a.y() = b.y() = -width * cos(fold_angle);
        width = 0.461;
        e.x() = f.x() = width * sin(fold_angle);
        e.y() = f.y() = -width * cos(fold_angle);
    }

    osg::Matrixf GS, S, T, R, Rz;
    GS.makeScale(gS, gS, gS);
    S.makeScale(128.f*scale, 100.f*scale, 256.f*scale);
    T.makeTranslate(pos);
    R.makeRotate(from, to);
    Rz.makeRotate(ang_z/180.*M_PI, 0, 0, 1);

    //resultant matrix, osg use post multiplication, not the same as OpenGL
    osg::Matrix M;
    M = S * GS * Rz * R * T;

    //if simple is true, just use a, b, e, f
    v->push_back(a * M);
    v->push_back(b * M);
    if(!simple)
    {
        v->push_back(c * M);
        v->push_back(d * M);
    }
    v->push_back(e * M);
    v->push_back(f * M);

    //texture coords
    tex->push_back(osg::Vec2(0.0f, 0.0f));
    tex->push_back(osg::Vec2(0.0f, 1.0f));
    if(!simple)
    {
        tex->push_back(osg::Vec2(0.539f, 0.0449f));
        tex->push_back(osg::Vec2(0.3672f, 0.99023f));
    }
    tex->push_back(osg::Vec2(1.0f, 0.0f));
    tex->push_back(osg::Vec2(1.0f, 1.0f));
}

void osgModeler::createFloatLeaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::Vec3 pos, osg::Vec3 normal, double scale)
{
    //initial vertices values, depends on the texture
    osg::Vec3 a, b, e, f, from;
    a = osg::Vec3(-0.5f, -0.5f, 0.0f);
    b = osg::Vec3(-0.5f, 0.5f, 0.0f);
    e = osg::Vec3(0.5f, -0.5f, 0.0f);
    f = osg::Vec3(0.5f, 0.5f, 0.0f);
    from = osg::Vec3(0.0f, 0.0f, 1.0f);

    osg::Matrixf S, T, R;
    S.makeScale(scale, scale, scale);
    T.makeTranslate(pos);
    R.makeRotate(from, normal);

    //resultant matrix, osg use post multiplication, not the same as OpenGL
    osg::Matrix M;
    M = S * R * T;

    //if simple is true, just use a, b, e, f
    //note: order is important for the right normal
    v->push_back(b * M);//b
    v->push_back(a * M);//a
    v->push_back(f * M);//f
    v->push_back(e * M);//e

    //texture coords
    tex->push_back(osg::Vec2(0.0f, 1.0f));//b
    tex->push_back(osg::Vec2(0.0f, 0.0f));//a
    tex->push_back(osg::Vec2(1.0f, 1.0f));//f
    tex->push_back(osg::Vec2(1.0f, 0.0f));//e
}

void osgModeler::perpendicularLeaf(osg::ref_ptr <osg::Vec3Array>& v, osg::ref_ptr <osg::Vec2Array>& tex, osg::ref_ptr <osg::Vec3Array>& all_n, osg::Vec3 cg)
{
    if(v->size() != tex->size())
        return;

    //new perp leaves and tex
    osg::ref_ptr <osg::Vec3Array> new_leaves = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> new_tex = new osg::Vec2Array;
    osg::ref_ptr <osg::Vec3Array> new_n = new osg::Vec3Array;

    //return values
    osg::ref_ptr <osg::Vec3Array> ret_v = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec2Array> ret_tex = new osg::Vec2Array;
    osg::ref_ptr <osg::Vec3Array> ret_n = new osg::Vec3Array;

    //assume each leaf has 4 vertices
    for(unsigned int i=0; i<v->size(); i+=4)
    {
        //orginal corners
        osg::Vec3 a = (*v)[i];
        osg::Vec3 b = (*v)[i+1];
        osg::Vec3 e = (*v)[i+2];
        osg::Vec3 f = (*v)[i+3];

        //orginal tex
        osg::Vec2 ta = (*tex)[i];
        osg::Vec2 tb = (*tex)[i+1];
        osg::Vec2 te = (*tex)[i+2];
        osg::Vec2 tf = (*tex)[i+3];

        //orginal normal
        osg::Vec3 or_n = (*all_n)[i/4];

        //middle points
        osg::Vec3 abm = (a + b) / 2.0f;
        osg::Vec3 efm = (e + f) / 2.0f;
        osg::Vec3 bfm = (b + f) / 2.0f;
        osg::Vec3 aem = (a + e) / 2.0f;

        //plane normal
        osg::Vec3 n = (e-a) ^ (b-a);
        n.normalize();

        //determine sign of n
        if((abm + n - cg).length() > (abm - n - cg).length())
            n = n * -1.0f;

        //length of side, assume it is square
        float w = (b-a).length();

        //new inward points
        osg::Vec3 abmt, efmt, bfmt, aemt;

        //new normals
        osg::Vec3 new_n1, new_n2;

        //experiment
        if(false)
        {
            abm = abm - n*w/2;
            efm = efm - n*w/2;
            bfm = bfm - n*w/2;
            aem = aem - n*w/2;
        }
        if(false)
            w *= 0.8f;
        //end of experiment

        //first quad
        abmt = abm + n*w;
        efmt = efm + n*w;
        new_n1 = b - a;//parallel to b-a but don't know the sign
        new_n1.normalize();

        //second quad
        bfmt = bfm + n*w;
        aemt = aem + n*w;
        new_n2 = e -a;//parallel to e-a but don't know the sign
        new_n2.normalize();

        //push back new leaves
        //first
        new_leaves->push_back(abmt);
        new_leaves->push_back(abm);
        new_leaves->push_back(efmt);
        new_leaves->push_back(efm);

        new_tex->push_back(ta);
        new_tex->push_back(tb);
        new_tex->push_back(te);
        new_tex->push_back(tf);

        new_n->push_back(new_n1);

        //second
        new_leaves->push_back(bfmt);
        new_leaves->push_back(bfm);
        new_leaves->push_back(aemt);
        new_leaves->push_back(aem);

        new_tex->push_back(ta);
        new_tex->push_back(tb);
        new_tex->push_back(te);
        new_tex->push_back(tf);

        new_n->push_back(new_n2);

        //push back results
        //original
        if(true)
        {
            ret_v->push_back(a);
            ret_v->push_back(b);
            ret_v->push_back(e);
            ret_v->push_back(f);

            ret_tex->push_back(ta);
            ret_tex->push_back(tb);
            ret_tex->push_back(te);
            ret_tex->push_back(tf);

            ret_n->push_back(or_n);
        }

        //check which plane is the most horizontal, assume the osg convension
        //equivalently finding the largest displacement in the z direction 
        //0: original, 1: first, 2: second
        int most_horizontal = 0;
        float or_dz = fabs(or_n.z());
        float first_dz = fabs(new_n1.z());
        float second_dz = fabs(new_n2.z());
        //printf("or_n(%f,%f,%f) first_dz(%f,%f,%f) second_dz(%f,%f,%f)\n", or_n.x(), or_n.y(), or_n.z(), new_n1.x(), new_n1.y(), new_n1.z(), new_n2.x(), new_n2.y(), new_n2.z());
        if(first_dz > or_dz && first_dz > second_dz)
            most_horizontal = 1;
        if(second_dz > or_dz && second_dz > first_dz)
            most_horizontal = 2;

        //random
        if(most_horizontal == 0)
        {
            //first
            int flip = rand()%10-5;
            if(flip <= 0)
            {
                ret_v->push_back(abmt);
                ret_v->push_back(abm);
                ret_v->push_back(efmt);
                ret_v->push_back(efm);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                //swapped to b,a,f,e
                ret_v->push_back(abm);
                ret_v->push_back(abmt);
                ret_v->push_back(efm);
                ret_v->push_back(efmt);

                //swapped to b,a,f,e
                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n1);

            //second
            flip = rand()%10-5;
            if(flip <= 0)
            {
                ret_v->push_back(bfmt);
                ret_v->push_back(bfm);
                ret_v->push_back(aemt);
                ret_v->push_back(aem);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                ret_v->push_back(bfm);
                ret_v->push_back(bfmt);
                ret_v->push_back(aem);
                ret_v->push_back(aemt);

                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n2);
        }
        else if(most_horizontal == 1)
        {
            //first
            if(new_n1.z() < 0)
            {
                ret_v->push_back(abmt);
                ret_v->push_back(abm);
                ret_v->push_back(efmt);
                ret_v->push_back(efm);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                //swapped to b,a,f,e
                ret_v->push_back(abm);
                ret_v->push_back(abmt);
                ret_v->push_back(efm);
                ret_v->push_back(efmt);

                //swapped to b,a,f,e
                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n1.z() > 0 ? new_n1 : new_n1*-1);

            //second is random
            int flip = rand()%10-5;
            if(flip <= 0)
            {
                ret_v->push_back(bfmt);
                ret_v->push_back(bfm);
                ret_v->push_back(aemt);
                ret_v->push_back(aem);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                ret_v->push_back(bfm);
                ret_v->push_back(bfmt);
                ret_v->push_back(aem);
                ret_v->push_back(aemt);

                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n2);

        }
        else if(most_horizontal == 2)
        {
            //first is random
            int flip = rand()%10-5;
            if(flip <= 0)
            {
                ret_v->push_back(abmt);
                ret_v->push_back(abm);
                ret_v->push_back(efmt);
                ret_v->push_back(efm);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                //swapped to b,a,f,e
                ret_v->push_back(abm);
                ret_v->push_back(abmt);
                ret_v->push_back(efm);
                ret_v->push_back(efmt);

                //swapped to b,a,f,e
                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n1);

            //second
            if(new_n2.z() < 0)
            {
                ret_v->push_back(bfmt);
                ret_v->push_back(bfm);
                ret_v->push_back(aemt);
                ret_v->push_back(aem);

                ret_tex->push_back(ta);
                ret_tex->push_back(tb);
                ret_tex->push_back(te);
                ret_tex->push_back(tf);
            }
            else
            {
                ret_v->push_back(bfm);
                ret_v->push_back(bfmt);
                ret_v->push_back(aem);
                ret_v->push_back(aemt);

                ret_tex->push_back(tb);
                ret_tex->push_back(ta);
                ret_tex->push_back(tf);
                ret_tex->push_back(te);
            }
            ret_n->push_back(new_n2.z() > 0 ? new_n2 : new_n2*-1);
        }
    }

    //re-set smart pointers
    v = ret_v;
    tex = ret_tex;
    all_n = ret_n;
}

osg::ref_ptr <osg::Geode> osgModeler::create_batch_leaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, bool simple)
{
    osg::ref_ptr <osg::Geode> ret = new osg::Geode;

    //create a giant Geometry to store all the leaves
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(v.get());

    //bind a transparent color, to counteract the effect of the branch's color
    osg::Vec4Array *colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.8, 0.8, 0.8, 1.0));
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    //bind a normal to all leaves
    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    n->push_back(osg::Vec3(1.f, 0.f, 0.f));
    geom->setNormalArray(n.get());
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    //texture coords
    geom->setTexCoordArray(0, tex);

    //number of vertex per leaf
    int leaf_size = simple ? 4 : 6;

    //triangle strip
    for(unsigned int i=0; i<v->size()/leaf_size; i++)
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, leaf_size*i, leaf_size));

    ret->addDrawable(geom.get());
    return ret;
}

//normal array is ignored in the obj output, so this is rather useless
osg::ref_ptr <osg::Geode> osgModeler::create_batch_leaf(osg::ref_ptr <osg::Vec3Array> v, osg::ref_ptr <osg::Vec2Array> tex, osg::ref_ptr <osg::Vec3Array> n)
{
    osg::ref_ptr <osg::Geode> ret = new osg::Geode;

    //create a giant Geometry to store all the leaves
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

    //texture coords
    geom->setTexCoordArray(0, tex);

    //number of vertex per leaf
    int leaf_size = 4;

    //triangle strip
    for(unsigned int i=0; i<v->size()/leaf_size; i++)
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, leaf_size*i, leaf_size));

    ret->addDrawable(geom.get());
    return ret;
}

osg::StateSet *osgModeler::leaf_stateset(const char *path)
{   
    // Create a new StateSet with default settings: 
    osg::StateSet *stateOne = new osg::StateSet();

    //texture
    osg::Texture2D *leaf_texture = new osg::Texture2D;
    // protect from being optimized away as static state:
    //leaf_texture->setDataVariance(osg::Object::DYNAMIC); 
    leaf_texture->setDataVariance(osg::Object::STATIC);//seemms that STATIC is also ok
    // load an image by reading a file: 
    //osg::Image* leaf_img = osgDB::readImageFile("/Users/jacky/work/resources/leaf/leaf3.png");
    osg::Image* leaf_img = osgDB::readImageFile(path);
    if (!leaf_img)
        printf("Couldn't find texture!!");

    // Assign the texture to the image we read from file: 
    leaf_texture->setImage(leaf_img);

    // set the filters
    leaf_texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST_MIPMAP_NEAREST);
    leaf_texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);

    // Assign texture unit 0 of our new StateSet to the texture 
    // we just created and enable the texture.
    stateOne->setTextureAttributeAndModes(0, leaf_texture, osg::StateAttribute::ON);

    //transparency or alpha texturing
    stateOne->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateOne->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateOne->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

    return stateOne;
}

osg::ref_ptr<osg::Geometry> osgModeler::createSphere(double r)
{
#if 1
    double knotsU[12]= { 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4 };
    double knotsV[8] = { 0, 0, 0, 1, 1, 2, 2, 2 };
    double ctrlAndWeightPts[9][5][4] = {
        {{0,0,r,1}, { r, 0,r,1}, { r, 0,0,2}, { r, 0,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, { r,-r,r,1}, { r,-r,0,2}, { r,-r,-r,1}, {0,0,-r,1}},
        {{0,0,r,2}, { 0,-r,r,2}, { 0,-r,0,4}, { 0,-r,-r,2}, {0,0,-r,2}},
        {{0,0,r,1}, {-r,-r,r,1}, {-r,-r,0,2}, {-r,-r,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, {-r, 0,r,1}, {-r, 0,0,2}, {-r, 0,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, {-r, r,r,1}, {-r, r,0,2}, {-r, 1,-r,1}, {0,0,-r,1}},
        {{0,0,r,2}, { 0, r,r,2}, { 0, r,0,4}, { 0, r,-r,2}, {0,0,-r,2}},
        {{0,0,r,1}, { r, r,r,1}, { r, r,0,2}, { r, r,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, { r, 0,r,1}, { r, 0,0,2}, { r, 0,-r,1}, {0,0,-r,1}} };
    osg::ref_ptr<osgModeling::NurbsSurface> geom = new osgModeling::NurbsSurface(
        12, &knotsU[0], 8, &knotsV[0], 20, 4, &ctrlAndWeightPts[0][0][0], 3, 3, 16, 16 );
#else
    double cp[5][3] = {
        {0.5f,1.0f,0.0f}, {-0.5f,0.0f,0.0f}, {0.5f,-1.0f,0.0f},
        {1.5f,0.0f,0.0f}, {0.5f,1.0f,0.0f} };
    osg::ref_ptr<osgModeling::Curve> profile = new osgModeling::Curve;
    profile->setPath( 15, &cp[0][0] );
    osg::ref_ptr<osgModeling::Extrude> geom = new osgModeling::Extrude;
    geom->setGenerateParts( osgModeling::Model::ALL_PARTS );
    geom->setExtrudeLength( 2.0f );
    geom->setProfile( profile.get() );
    geom->update();
#endif
    return geom.get();
}

osg::ref_ptr<osg::Group> osgModeler::visualizeCurve(osg::ref_ptr<osg::Vec3Array> pts, double r)
{
    osg::ref_ptr<osg::Group> ret = new osg::Group;

    osg::ref_ptr<osg::Geode> sphere = new osg::Geode;

    if(r == -1.0)
    {
        r = 3;
        if(pts->size() >1)
        {
            osg::Vec3 f = (*pts)[0];
            osg::Vec3 s = (*pts)[1];

            r = pow(f.x()-s.x(), 2) + pow(f.y()-s.y(), 2) + pow(f.z()-s.z(), 2);
            r = pow(r, 0.5) / 2;
            r = fmax(r, 0.5);
        }
    }
        
    sphere->addDrawable(createSphere(r).get());

    for(unsigned int i=0; i<pts->size(); i++)
    {
        osg::Vec3 point = (*pts)[i];

        osg::ref_ptr<osg::PositionAttitudeTransform> posTransform = new osg::PositionAttitudeTransform;
        posTransform->setPosition(point);

        posTransform->addChild(sphere);
        ret->addChild(posTransform);
    }

    return ret;
}

//special notice: createNurbsCircle creates the same point at the front and end
//this method return unique points in size == numPath
osg::ref_ptr<osg::Vec3Array> osgModeler::orientedCircle(osg::Vec3 Tc, osg::Vec3 Tx, osg::Vec3 Ty, osg::Vec3 Tz, int numPath)
{
    osg::ref_ptr<osg::Vec3Array> ret = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> origin = new osg::Vec3Array;

    osg::Matrixd T(Tx.x(), Ty.x(), Tz.x(), Tc.x(),
                   Tx.y(), Ty.y(), Tz.y(), Tc.y(),
                   Tx.z(), Ty.z(), Tz.z(), Tc.z(),
                   0, 0, 0, 1);

    origin = createNurbsCircle(1.0f, numPath+1)->getPath();

    for(unsigned int i=0; i<origin->size()-1; i++)
    {
        osg::Vec3 p = (*origin)[i];
        osg::Vec4 p1(p.x(), p.y(), p.z(), 1);
        osg::Vec4 Tp1 = T * p1;
        osg::Vec3 Tp(Tp1.x()/Tp1.w(), Tp1.y()/Tp1.w(), Tp1.z()/Tp1.w());
        ret->push_back(Tp);

        if(isnan(Tp.x()) || isnan(Tp.y()) || isnan(Tp.z())) 
            printf("hihi Tp nan(numPath(%d) p(%lf,%lf,%lf) Tp1(%lf,%lf,%lf,%lf)): Tc(%lf,%lf,%lf) Tx(%lf,%lf,%lf) Ty(%lf,%lf,%lf) Tz(%lf,%lf,%lf)\n", numPath, p.x(), p.y(), p.z(), Tp1.x(), Tp1.y(), Tp1.z(), Tp1.w(), Tc.x(), Tc.y(), Tc.z(), Tx.x(), Tx.y(), Tx.z(), Ty.x(), Ty.y(), Ty.z(), Tz.x(), Tz.y(), Tz.z());

        //if(i==0 || i==24 || i==49 || i==74)
            //printf("%f %f %f\n", Tp.x(), Tp.y(), Tp.z());
        //printf("OrientedCircle: (%lf,%lf,%lf)\n", Tp.x(), Tp.y(), Tp.z());
    }

    return ret;
}

osg::ref_ptr<osg::Group> osgModeler::branchJoint(osg::ref_ptr<osg::Vec3Array> a, osg::Vec3 ac, osg::Vec3 an, osg::ref_ptr<osg::Vec3Array> b, osg::Vec3 bc, osg::Vec3 bn, osg::ref_ptr<osg::Vec3Array> c, osg::Vec3 cc, osg::Vec3 cn)
{
    osg::ref_ptr<osg::Vec3Array> ret = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> debug = new osg::Vec3Array;

    int a_start = 0, b_start = 0;
    double min_ab = dist(ac, bc);
    for(unsigned int i=0; i<a->size(); i++)
        for(unsigned int j=0; j<b->size(); j++)
        {
            double distance_ab = dist((*a)[i], (*b)[j]);
            if(distance_ab < min_ab)
            {
                a_start = i;
                b_start = j;
                min_ab = distance_ab;
            }
        }

    //a_start, b_start
    /*
    ret->push_back((*a)[a_start]);
    ret->push_back((*b)[b_start]);
    */

    /*
    double abk = an.z()*bn.x() - an.x()*bn.z();
    double abt = 1/abk * (-bn.z()*((*b)[b_start].x()-(*a)[a_start].x()) + bn.x()*((*b)[b_start].z()-(*a)[a_start].z()));
    osg::Vec3 abm = (*a)[a_start] + an * abt;
    */
    osg::Vec3 abm = intersect((*a)[a_start], an, (*b)[b_start], bn);
    //abm.y() = cc.y();//commented because they may not be on a vertical plane

    //abm
    //ret->push_back(abm);

    //a_start, b_start noramls
    /*
    for(int i=-10; i<10; i++)
    {
        ret->push_back((*a)[a_start]+an*i);
        ret->push_back((*b)[b_start]+bn*i);
    }
    */

    //ab-curve
    osg::ref_ptr <osg::Vec3Array> ab_curve_ctr_pts = new osg::Vec3Array;
    ab_curve_ctr_pts->push_back((*a)[a_start]);
    ab_curve_ctr_pts->push_back(abm);
    ab_curve_ctr_pts->push_back((*b)[b_start]);
    osg::ref_ptr<osgModeling::BezierCurve> ab_curve = new osgModeling::BezierCurve(ab_curve_ctr_pts, 2, 20);
    /*
    for(unsigned int i=0; i<ab_curve->getPath()->size(); i++)
        ret->push_back((*ab_curve->getPath())[i]);
    */

    //find ab-curve-start
    osg::Vec3 ab_curve_start = (*a)[a_start];
    double min_ab_curve_start_abm = dist(ab_curve_start, abm);
    double min_ab_curve_start_abm_t = 0.0;
    for(unsigned int i=0; i<ab_curve->getPath()->size(); i++)
    {
        osg::Vec3 on_curve = (*ab_curve->getPath())[i];
        double new_dist = dist(on_curve, abm);
        if(new_dist < min_ab_curve_start_abm)
        {
            min_ab_curve_start_abm = new_dist;
            min_ab_curve_start_abm_t = i / (ab_curve->getPath()->size() - 1);
            ab_curve_start = on_curve;
        }
    }
    //ret->push_back(ab_curve_start);

    //find c_start which is the closest point to ab_curve_start on curve c
    osg::Vec3 ccba = cc + (*b)[b_start] - (*a)[a_start];
    int c_start = 0, c_start_opposite = 0;
    double min_c_start_ab_curve_start = dist((*c)[c_start], ccba);
    for(unsigned int i=0; i<c->size(); i++)
    {
        osg::Vec3 on_curve = (*c)[i];
        double new_dist = dist(on_curve, ccba);
        if(new_dist < min_c_start_ab_curve_start)
        {
            c_start = i;
            min_c_start_ab_curve_start = new_dist;
        }
    }
    //rotate by pi/4 to approximate the perpendicular direction
    c_start = (c_start + c->size()/4) % c->size();
    c_start_opposite = (c_start + c->size()/2) % c->size();
    //debug->push_back((*c)[c_start]);
    //debug->push_back((*c)[c_start_opposite]);

    //calculate t for c_start ... c_start_opposite curve
    double c_start_curve_t = dist((*c)[c_start], ab_curve_start) / (dist((*c)[c_start], ab_curve_start) + dist((*c)[c_start_opposite], ab_curve_start));
    //c_start_curve_t = 0.5;//for simplicity so as for easier implementation
    osg::Vec3 c_start_middle_ctr = middle_ctr_pt((*c)[c_start], (*c)[c_start_opposite], ab_curve_start, c_start_curve_t);

    //middle control point for c_start c_start_opposite curve
    //ret->push_back(c_start_middle_ctr);

    //c_start c_end curve or called c_start_opposite_curve
    osg::ref_ptr <osg::Vec3Array> c_start_opposite_curve_ctr_pts = new osg::Vec3Array;
    c_start_opposite_curve_ctr_pts->push_back((*c)[c_start]);
    //c_start_opposite_curve_ctr_pts->push_back(c_start_middle_ctr);
    c_start_opposite_curve_ctr_pts->push_back((*c)[c_start] + cubic_ctr((*c)[c_start], (*c)[c_start_opposite], ab_curve_start, cn));
    c_start_opposite_curve_ctr_pts->push_back((*c)[c_start_opposite] + cubic_ctr((*c)[c_start], (*c)[c_start_opposite], ab_curve_start, cn)*(1/c_start_curve_t-1));
    c_start_opposite_curve_ctr_pts->push_back((*c)[c_start_opposite]);
    osg::ref_ptr<osgModeling::BezierCurve> c_start_opposite_curve = new osgModeling::BezierCurve(c_start_opposite_curve_ctr_pts, 3, 49);

    //debug the 2nd and 3rd control pts for the cubic Bezier curve
    //ret->push_back((*c)[c_start] + cubic_ctr((*c)[c_start], (*c)[c_start_opposite], ab_curve_start, cn));
    //ret->push_back((*c)[c_start_opposite] + cubic_ctr((*c)[c_start], (*c)[c_start_opposite], ab_curve_start, cn)*(1/c_start_curve_t-1));

    /* c_start_opposite_curve
    for(unsigned int i=0; i<c_start_opposite_curve->getPath()->size(); i++)
        ret->push_back((*c_start_opposite_curve->getPath())[i]);
    */

    //find the orientation of a, b and c
    int a_orientation = 1, b_orientation = 1, c_orientation = 1;
    if(orient3D((*a)[modular(a_start+a->size()/4, a->size())], (*a)[a_start], ac, cc) < 0)
        a_orientation = -1;
    if(orient3D((*b)[modular(b_start+b->size()/4, b->size())], bc, (*b)[b_start], cc) < 0)
        b_orientation = -1;
    if(orient3D((*c)[modular(c_start+c->size()/4, c->size())], (*c)[c_start], cc, abm) < 0)
        c_orientation = -1;

    //find if on the c_start's or c_start_opposite's side
    bool c_start_side = true;
    if(orient3D((*b)[modular(b_start+b->size()/4*b_orientation, b->size())], (*a)[modular(a_start+a->size()/4*a_orientation, a->size())], (*c)[c_start], cc) < 0)
        c_start_side = false;

    //construct a set of L Bezier curves
    int c_start_opposite_curve_middle_number = int(c_start_curve_t * (c_start_opposite_curve->getPath()->size()-1));
    std::vector <osg::ref_ptr <osgModeling::BezierCurve> > L;
    for(unsigned int i=0; i<=a->size()/4; i++)
    {
        int index_a, index_b, index_m;

        index_a = modular(a_start + i*a_orientation, a->size());
        index_b = modular(b_start + i*b_orientation, b->size());
        index_m = c_start_side ? std::max(0, int(c_start_opposite_curve_middle_number - i)) : std::min(int(c_start_opposite_curve->getPath()->size()-1), int(c_start_opposite_curve_middle_number + i));

        osg::Vec3 L_on_a = (*a)[index_a];
        osg::Vec3 L_on_b = (*b)[index_b];
        osg::Vec3 L_on_cso = (*c_start_opposite_curve->getPath())[index_m];
        double t = dist(L_on_a, L_on_cso) / (dist(L_on_a, L_on_cso) + dist(L_on_b, L_on_cso));
        //t = 0.5;
        osg::Vec3 L_mid_ctr = middle_ctr_pt(L_on_a, L_on_b, L_on_cso, t);

        osg::ref_ptr <osg::Vec3Array> L_curve_ctr = new osg::Vec3Array;
        L_curve_ctr->push_back(L_on_a);
        L_curve_ctr->push_back(L_mid_ctr);
        L_curve_ctr->push_back(L_on_b);

        //a better way is to use cubic Bezier instead of quadradic
        osg::ref_ptr<osgModeling::BezierCurve> L_curve = new osgModeling::BezierCurve(L_curve_ctr, 2, 40);
        L.push_back(L_curve);
    }

    //construct a set of R Bezier curves
    std::vector <osg::ref_ptr <osgModeling::BezierCurve> > R;
    for(unsigned int i=0; i<=a->size()/4; i++)
    {
        int index_a, index_b, index_m;

        index_a = modular(a_start - i*a_orientation, a->size());
        index_b = modular(b_start - i*b_orientation, b->size());
        index_m = !c_start_side ? std::max(0, int(c_start_opposite_curve_middle_number - i)) : std::min(int(c_start_opposite_curve->getPath()->size()-1), int(c_start_opposite_curve_middle_number + i));

        osg::Vec3 R_on_a = (*a)[index_a];
        osg::Vec3 R_on_b = (*b)[index_b];
        osg::Vec3 R_on_cso = (*c_start_opposite_curve->getPath())[index_m];
        double t = dist(R_on_a, R_on_cso) / (dist(R_on_a, R_on_cso) + dist(R_on_b, R_on_cso));
        //t = 0.5;
        osg::Vec3 R_mid_ctr = middle_ctr_pt(R_on_a, R_on_b, R_on_cso, t);

        osg::ref_ptr <osg::Vec3Array> R_curve_ctr = new osg::Vec3Array;
        R_curve_ctr->push_back(R_on_a);
        R_curve_ctr->push_back(R_mid_ctr);
        R_curve_ctr->push_back(R_on_b);

        osg::ref_ptr<osgModeling::BezierCurve> R_curve = new osgModeling::BezierCurve(R_curve_ctr, 2, 40);
        R.push_back(R_curve);
    }

    //find ac_curve_start, ac_curve_mid and ac_curve_end
    osg::Vec3 ac_curve_start, ac_curve_mid, ac_curve_end;
    int L_size = L[L.size()-1]->getPath()->size();
    int R_size = R[R.size()-1]->getPath()->size();
    ac_curve_start = (*L[L.size()-1]->getPath())[L_size/8];//assume L_size > 0
    int ac_curve_mid_index_a, ac_curve_mid_index_c;
    ac_curve_mid_index_a = modular(a_start+(a->size()/2)*a_orientation, a->size()); 
    if(c_start_side)
        ac_curve_mid_index_c = modular(c_start-c->size()/4*c_orientation, c->size());
    else
        ac_curve_mid_index_c = modular(c_start+c->size()/2-c->size()/4*c_orientation, c->size());
    ac_curve_mid = intersect((*c)[ac_curve_mid_index_c], cn, (*a)[ac_curve_mid_index_a], an);
    ac_curve_end = (*R[R.size()-1]->getPath())[R_size/8];//assume R_size > 0
    
    //find normals for ac_curve_start and ac_curve_end
    osg::Vec3 ac_curve_start_pre, ac_start_opposite_curve_normal;
    ac_curve_start_pre = (*L[L.size()-2]->getPath())[L_size/8];//assume L_size > 1
    ac_start_opposite_curve_normal = ac_curve_start - ac_curve_start_pre;
    ac_start_opposite_curve_normal.normalize();

    //find 2nd and 3rd control points for ac_start_opposite_curve
    double ac_start_opposite_curve_t = dist(ac_curve_start, ac_curve_mid) / (dist(ac_curve_start, ac_curve_mid) + dist(ac_curve_end, ac_curve_mid));
    osg::Vec3 ac_start_opposite_curve_ctr2 = cubic_ctr(ac_curve_start, ac_curve_end, ac_curve_mid, ac_start_opposite_curve_normal);
    osg::Vec3 ac_start_opposite_curve_ctr3 = ac_start_opposite_curve_ctr2 * (1/ac_start_opposite_curve_t-1);
    ac_start_opposite_curve_ctr2 = ac_curve_start + ac_start_opposite_curve_ctr2;
    ac_start_opposite_curve_ctr3 = ac_curve_end + ac_start_opposite_curve_ctr3;

    //construct ac_start_opposite_curve
    osg::ref_ptr <osg::Vec3Array> ac_start_opposite_curve_ctr_pts = new osg::Vec3Array;
    ac_start_opposite_curve_ctr_pts->push_back(ac_curve_start);
    ac_start_opposite_curve_ctr_pts->push_back(ac_start_opposite_curve_ctr2);
    ac_start_opposite_curve_ctr_pts->push_back(ac_start_opposite_curve_ctr3);
    ac_start_opposite_curve_ctr_pts->push_back(ac_curve_end);
    osg::ref_ptr<osgModeling::BezierCurve> ac_start_opposite_curve = new osgModeling::BezierCurve(ac_start_opposite_curve_ctr_pts, 3, c->size()/2+1);

    /*
    debug->push_back(ac_curve_start);
    debug->push_back(ac_curve_mid);
    debug->push_back(ac_curve_end);
    debug->push_back(ac_curve_start_pre);
    for(int i=0; i<10; i++)
        debug->push_back(ac_curve_start + ac_start_opposite_curve_normal*i);
    debug->push_back(ac_start_opposite_curve_ctr2);
    debug->push_back(ac_start_opposite_curve_ctr3);
    for(unsigned int i=0; i<ac_start_opposite_curve->getPath()->size(); i++)
        debug->push_back((*ac_start_opposite_curve->getPath())[i]);
    */

    //construct a set of Fl + Fr curves
    std::vector <osg::ref_ptr <osgModeling::BezierCurve> > F;
    for(unsigned int i=0; i<=c->size()/2; i++)
    {
        int index_a, index_c;

        index_a = modular(a_start+(a->size()/4+i)*a_orientation, a->size());
        if(c_start_side)
            index_c = modular(c_start-i*c_orientation, c->size());
        else
            index_c = modular(c_start+c->size()/2-i*c_orientation, c->size());
        //osg::Vec3 mid_ctr = intersect((*c)[index_c], cn, (*a)[index_a], an);//replaced since the intersection point dose not always lay between the two curves and it does not fit L and R
        osg::Vec3 mid_ctr = (*ac_start_opposite_curve->getPath())[i];

        osg::ref_ptr <osg::Vec3Array> Fl_curve_ctr = new osg::Vec3Array;

        Fl_curve_ctr->push_back((*c)[index_c]);
        Fl_curve_ctr->push_back(mid_ctr);
        Fl_curve_ctr->push_back((*a)[index_a]);

        osg::ref_ptr<osgModeling::BezierCurve> Fl_curve = new osgModeling::BezierCurve(Fl_curve_ctr, 2, 25);
        F.push_back(Fl_curve);

        //ret->push_back((*c)[index_c]);
    }

    //find bc_curve_start, bc_curve_mid and bc_curve_end
    osg::Vec3 bc_curve_start, bc_curve_mid, bc_curve_end;
    bc_curve_start = (*L[L.size()-1]->getPath())[3*L_size/4];//assume L_size > 0
    int bc_curve_mid_index_b, bc_curve_mid_index_c;
    bc_curve_mid_index_b = modular(b_start+(b->size()/2)*b_orientation, b->size()); 
    if(c_start_side)
        bc_curve_mid_index_c = modular(c_start+c->size()/4*c_orientation, c->size());
    else
        bc_curve_mid_index_c = modular(c_start+c->size()/2+c->size()/4*c_orientation, c->size());
    bc_curve_mid = intersect((*c)[bc_curve_mid_index_c], cn, (*b)[bc_curve_mid_index_b], bn);
    bc_curve_end = (*R[R.size()-1]->getPath())[3*R_size/4];//assume R_size > 0
    
    //find normals for bc_curve_start and bc_curve_end
    osg::Vec3 bc_curve_start_pre, bc_start_opposite_curve_normal;
    bc_curve_start_pre = (*L[L.size()-2]->getPath())[7*L_size/8];//assume L_size > 1
    bc_start_opposite_curve_normal = bc_curve_start - bc_curve_start_pre;
    bc_start_opposite_curve_normal.normalize();

    //find 2nd and 3rd control points for bc_start_opposite_curve
    double bc_start_opposite_curve_t = dist(bc_curve_start, bc_curve_mid) / (dist(bc_curve_start, bc_curve_mid) + dist(bc_curve_end, bc_curve_mid));
    osg::Vec3 bc_start_opposite_curve_ctr2 = cubic_ctr(bc_curve_start, bc_curve_end, bc_curve_mid, bc_start_opposite_curve_normal);
    osg::Vec3 bc_start_opposite_curve_ctr3 = bc_start_opposite_curve_ctr2 * (1/bc_start_opposite_curve_t-1);
    bc_start_opposite_curve_ctr2 = bc_curve_start + bc_start_opposite_curve_ctr2;
    bc_start_opposite_curve_ctr3 = bc_curve_end + bc_start_opposite_curve_ctr3;

    //construct bc_start_opposite_curve
    osg::ref_ptr <osg::Vec3Array> bc_start_opposite_curve_ctr_pts = new osg::Vec3Array;
    bc_start_opposite_curve_ctr_pts->push_back(bc_curve_start);
    bc_start_opposite_curve_ctr_pts->push_back(bc_start_opposite_curve_ctr2);
    bc_start_opposite_curve_ctr_pts->push_back(bc_start_opposite_curve_ctr3);
    bc_start_opposite_curve_ctr_pts->push_back(bc_curve_end);
    osg::ref_ptr<osgModeling::BezierCurve> bc_start_opposite_curve = new osgModeling::BezierCurve(bc_start_opposite_curve_ctr_pts, 3, c->size()/2+1);

    /*
    debug->push_back(bc_curve_start);
    debug->push_back(bc_curve_mid);
    debug->push_back(bc_curve_end);
    debug->push_back(bc_curve_start_pre);
    for(int i=0; i<10; i++)
        debug->push_back(bc_curve_start + bc_start_opposite_curve_normal*i);
    debug->push_back(bc_start_opposite_curve_ctr2);
    debug->push_back(bc_start_opposite_curve_ctr3);
    for(unsigned int i=0; i<bc_start_opposite_curve->getPath()->size(); i++)
        debug->push_back((*bc_start_opposite_curve->getPath())[i]);
    */

    //construct a set of Bl + Br curves
    std::vector <osg::ref_ptr <osgModeling::BezierCurve> > B;
    for(unsigned int i=0; i<=c->size()/2; i++)
    {
        int index_b, index_c;

        index_b = modular(b_start+(b->size()/4+i)*b_orientation, b->size());
        if(c_start_side)
            index_c = modular(c_start+i*c_orientation, c->size());
        else
            index_c = modular(c_start+c->size()/2+i*c_orientation, c->size());
        //osg::Vec3 mid_ctr = intersect((*c)[index_c], cn, (*b)[index_b], bn);//replaced
        osg::Vec3 mid_ctr = (*bc_start_opposite_curve->getPath())[i];

        osg::ref_ptr <osg::Vec3Array> Bl_curve_ctr = new osg::Vec3Array;

        Bl_curve_ctr->push_back((*c)[index_c]);
        Bl_curve_ctr->push_back(mid_ctr);
        Bl_curve_ctr->push_back((*b)[index_b]);

        osg::ref_ptr<osgModeling::BezierCurve> Bl_curve = new osgModeling::BezierCurve(Bl_curve_ctr, 2, 25);
        B.push_back(Bl_curve);

        //ret->push_back((*b)[index_b]);
    }

    //visualize L
    /*
    for(unsigned int i=0; i<L.size(); i++)
    {
        osg::ref_ptr <osgModeling::BezierCurve> curve = L[i];
        for(unsigned int j=0; j<curve->getPath()->size(); j++)
            ret->push_back((*curve->getPath())[j]);
    }
    */

    //visualize R
    /*
    for(unsigned int i=0; i<R.size(); i++)
    {
        osg::ref_ptr <osgModeling::BezierCurve> curve = R[i];
        for(unsigned int j=0; j<curve->getPath()->size(); j++)
            ret->push_back((*curve->getPath())[j]);
    }
    */

    //visualize Fl and Fr
    /*
    for(unsigned int i=0; i<F.size(); i++)
    {
        osg::ref_ptr <osgModeling::BezierCurve> curve = F[i];
        for(unsigned int j=0; j<curve->getPath()->size(); j++)
            ret->push_back((*curve->getPath())[j]);
    }
    */

    //visualize Bl and Br
    /*
    for(unsigned int i=0; i<B.size(); i++)
    {
        osg::ref_ptr <osgModeling::BezierCurve> curve = B[i];
        for(unsigned int j=0; j<curve->getPath()->size(); j++)
            debug->push_back((*curve->getPath())[j]);
    }
    */

    //return ret;

    //construct geometry for all curves
    osg::ref_ptr <osg::Geode> geodeL = triangulateCurve(L, cc);
    osg::ref_ptr <osg::Geode> geodeB = triangulateCurve(B, bc);
    osg::ref_ptr <osg::Geode> geodeR = triangulateCurve(R, cc);
    osg::ref_ptr <osg::Geode> geodeF = triangulateCurve(F, ac);

    osg::ref_ptr <osg::Group> root = new osg::Group;
    root->addChild(geodeL.get());
    root->addChild(geodeB.get());
    root->addChild(geodeR.get());
    root->addChild(geodeF.get());

    //debug
    root->addChild(visualizeCurve(debug, 2).get());

    return root;
}

osg::ref_ptr <osg::Group> osgModeler::branchJoint(OrientedCircle a, OrientedCircle b, OrientedCircle c)
{
    return branchJoint(a.points(), a.center(), a.normal(), b.points(), b.center(), b.normal(), c.points(), c.center(), c.normal());
}

osg::Vec3 osgModeler::center_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t)
{
    osg::Vec3 ret;

    /*
    int x0 = (*controlPoints)[0].x();
    int x1 = (*controlPoints)[1].x();
    int x2 = (*controlPoints)[2].x();
    int x3 = (*controlPoints)[3].x();
    int y0 = (*controlPoints)[0].y();
    int y1 = (*controlPoints)[1].y();
    int y2 = (*controlPoints)[2].y();
    int y3 = (*controlPoints)[3].y();
    int z0 = (*controlPoints)[0].z();
    int z1 = (*controlPoints)[1].z();
    int z2 = (*controlPoints)[2].z();
    int z3 = (*controlPoints)[3].z();
    */

    double x0 = (*controlPoints)[0].x();
    double x1 = (*controlPoints)[1].x();
    double x2 = (*controlPoints)[2].x();
    double x3 = (*controlPoints)[3].x();
    double y0 = (*controlPoints)[0].y();
    double y1 = (*controlPoints)[1].y();
    double y2 = (*controlPoints)[2].y();
    double y3 = (*controlPoints)[3].y();
    double z0 = (*controlPoints)[0].z();
    double z1 = (*controlPoints)[1].z();
    double z2 = (*controlPoints)[2].z();
    double z3 = (*controlPoints)[3].z();

    ret.x() = pow(1-t, 3)*x0 + 3*t*pow(1-t, 2)*x1 + 3*t*t*(1-t)*x2 + pow(t, 3)*x3;
    ret.y() = pow(1-t, 3)*y0 + 3*t*pow(1-t, 2)*y1 + 3*t*t*(1-t)*y2 + pow(t, 3)*y3;
    ret.z() = pow(1-t, 3)*z0 + 3*t*pow(1-t, 2)*z1 + 3*t*t*(1-t)*z2 + pow(t, 3)*z3;

    return ret;
}

osg::Vec3 osgModeler::tangent_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t)
{
    osg::Vec3 ret;

    /*
    int x0 = (*controlPoints)[0].x();
    int x1 = (*controlPoints)[1].x();
    int x2 = (*controlPoints)[2].x();
    int x3 = (*controlPoints)[3].x();
    int y0 = (*controlPoints)[0].y();
    int y1 = (*controlPoints)[1].y();
    int y2 = (*controlPoints)[2].y();
    int y3 = (*controlPoints)[3].y();
    int z0 = (*controlPoints)[0].z();
    int z1 = (*controlPoints)[1].z();
    int z2 = (*controlPoints)[2].z();
    int z3 = (*controlPoints)[3].z();
    */

    double x0 = (*controlPoints)[0].x();
    double x1 = (*controlPoints)[1].x();
    double x2 = (*controlPoints)[2].x();
    double x3 = (*controlPoints)[3].x();
    double y0 = (*controlPoints)[0].y();
    double y1 = (*controlPoints)[1].y();
    double y2 = (*controlPoints)[2].y();
    double y3 = (*controlPoints)[3].y();
    double z0 = (*controlPoints)[0].z();
    double z1 = (*controlPoints)[1].z();
    double z2 = (*controlPoints)[2].z();
    double z3 = (*controlPoints)[3].z();
 
    double rptx = (-3*x0 + 9*x1 - 9*x2 + 3*x3)*t*t + (6*x0 - 12*x1 + 6*x2)*t + (-3*x0 + 3*x1);
    double rpty = (-3*y0 + 9*y1 - 9*y2 + 3*y3)*t*t + (6*y0 - 12*y1 + 6*y2)*t + (-3*y0 + 3*y1);
    double rptz = (-3*z0 + 9*z1 - 9*z2 + 3*z3)*t*t + (6*z0 - 12*z1 + 6*z2)*t + (-3*z0 + 3*z1);

    double length = pow(rptx*rptx + rpty*rpty + rptz*rptz, 0.5);

    ret.x() = rptx / length;
    ret.y() = rpty / length;
    ret.z() = rptz / length;

    if(length == 0.0)
    {
        ret = osg::Vec3(x3-x0, y3-y0, z3-z0);
        ret.normalize();
        //printf("(%f,%f,%f)\n", ret.x(), ret.y(), ret.z());
        printf("(%f,%f,%f) (%f,%f,%f) (%f,%f,%f) (%f,%f,%f)\n", x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3); 
        //ret = osg::Vec3(1, 0, 0);
    }

    return ret;
}
 
osg::Vec3 osgModeler::normal_3D(osg::ref_ptr <osg::Vec3Array> controlPoints, double t)
{
    osg::Vec3 ret;

    /*
    int x0 = (*controlPoints)[0].x();
    int x1 = (*controlPoints)[1].x();
    int x2 = (*controlPoints)[2].x();
    int x3 = (*controlPoints)[3].x();
    int y0 = (*controlPoints)[0].y();
    int y1 = (*controlPoints)[1].y();
    int y2 = (*controlPoints)[2].y();
    int y3 = (*controlPoints)[3].y();
    int z0 = (*controlPoints)[0].z();
    int z1 = (*controlPoints)[1].z();
    int z2 = (*controlPoints)[2].z();
    int z3 = (*controlPoints)[3].z();
    */

    double x0 = (*controlPoints)[0].x();
    double x1 = (*controlPoints)[1].x();
    double x2 = (*controlPoints)[2].x();
    double x3 = (*controlPoints)[3].x();
    double y0 = (*controlPoints)[0].y();
    double y1 = (*controlPoints)[1].y();
    double y2 = (*controlPoints)[2].y();
    double y3 = (*controlPoints)[3].y();
    double z0 = (*controlPoints)[0].z();
    double z1 = (*controlPoints)[1].z();
    double z2 = (*controlPoints)[2].z();
    double z3 = (*controlPoints)[3].z();
 
    double rptx = (-3*x0 + 9*x1 - 9*x2 + 3*x3)*t*t + (6*x0 - 12*x1 + 6*x2)*t + (-3*x0 + 3*x1);
    double rpty = (-3*y0 + 9*y1 - 9*y2 + 3*y3)*t*t + (6*y0 - 12*y1 + 6*y2)*t + (-3*y0 + 3*y1);
    double rptz = (-3*z0 + 9*z1 - 9*z2 + 3*z3)*t*t + (6*z0 - 12*z1 + 6*z2)*t + (-3*z0 + 3*z1);

    double et = rptx*rptx + rpty*rpty + rptz*rptz;
    double rpptx = (-6*x0 + 18*x1 - 18*x2 + 6*x3)*t + (6*x0 - 12*x1 + 6*x2);
    double rppty = (-6*y0 + 18*y1 - 18*y2 + 6*y3)*t + (6*y0 - 12*y1 + 6*y2);
    double rpptz = (-6*z0 + 18*z1 - 18*z2 + 6*z3)*t + (6*z0 - 12*z1 + 6*z2);
    double ept = 2*rptx*rpptx + 2*rpty*rppty + 2*rptz*rpptz;
    double Tptx = -0.5*pow(et, -1.5)*ept*rptx + pow(et, -0.5)*rpptx;
    double Tpty = -0.5*pow(et, -1.5)*ept*rpty + pow(et, -0.5)*rppty;
    double Tptz = -0.5*pow(et, -1.5)*ept*rptz + pow(et, -0.5)*rpptz;
    double norm = pow(Tptx*Tptx + Tpty*Tpty + Tptz*Tptz, 0.5);

    ret.x() = Tptx/norm;
    ret.y() = Tpty/norm;
    ret.z() = Tptz/norm;

    if(isnan(ret.x()))
    {
        //ret = osg::Vec3(0, 1, 0);
        ret = tangent_3D(controlPoints, t);
        ret = osg::Vec3(ret.y(), ret.x()*-1.0, 0);
        ret.normalize();
    }

    return ret;
}

osg::ref_ptr <osg::Geode> osgModeler::branchJointTriangle()
{
    osg::ref_ptr <osg::Geode> geode = new osg::Geode;
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    
    osg::ref_ptr <osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray(v.get());
    v->push_back(osg::Vec3(-1.f, 0.f, -1.f));
    v->push_back(osg::Vec3(1.f, 0.f, -1.f));
    v->push_back(osg::Vec3(1.f, 0.f, 1.f));
    v->push_back(osg::Vec3(-1.f, -1.f, 1.f));

    osg::ref_ptr<osg::Vec3Array> n = new osg::Vec3Array;
    geom->setNormalArray(n.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    n->push_back(osg::Vec3(0.f, -1.f, 0.f));
    n->push_back(osg::Vec3(0.f, -1.f, 0.f));
    n->push_back(osg::Vec3(0.f, -1.f, 0.f));
    n->push_back(osg::Vec3(0.f, 1.f, 0.f));

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 0, 4));
    geode->addDrawable(geom.get());

    return geode;
}

osg::ref_ptr<osg::Group> osgModeler::test()
{
    osg::ref_ptr<osgModeling::Curve> path = new osgModeling::Curve;

    path->addPathPoint( osg::Vec3(1.0f,0.0f,-3.0f) );
    path->addPathPoint( osg::Vec3(0.0f,0.0f,-2.0f) );
    path->addPathPoint( osg::Vec3(0.0f,0.0f,0.0f) );
    path->addPathPoint( osg::Vec3(-1.0f,0.0f,2.0f) );

    osg::ref_ptr<osgModeling::Loft> geom = new osgModeling::Loft;
    geom->setProfile( path.get() );

    geom->addShape( createNurbsCircle(1.5f).get() );
    geom->addShape( createNurbsCircle(2.0f).get() );
    geom->addShape( createNurbsCircle(2.0f).get() );
    geom->addShape( createNurbsCircle(2.0f).get() );

    geom->update();

    osg::ref_ptr<osgModeling::Curve> path2 = new osgModeling::Curve;

    path2->addPathPoint( osg::Vec3(-0.5f,0.0f,-0.5f) );
    path2->addPathPoint( osg::Vec3(1.0f,0.0f,1.0f) );
    path2->addPathPoint( osg::Vec3(1.5f,0.0f,2.0f) );

    osg::ref_ptr<osgModeling::Loft> geom2 = new osgModeling::Loft;
    geom2->setProfile( path2.get() );

    geom2->addShape( createNurbsCircle(1.0f).get() );
    geom2->addShape( createNurbsCircle(1.0f).get() );
    geom2->addShape( createNurbsCircle(1.0f).get() );

    geom2->update();

    osg::ref_ptr<osgModeling::Curve> path3 = new osgModeling::Curve;

    path3->addPathPoint( osg::Vec3(0.0f,0.0f,-3.0f) );
    path3->addPathPoint( osg::Vec3(0.0f,0.0f,-2.0f) );
    path3->addPathPoint( osg::Vec3(0.0f,0.0f,0.0f) );

    osg::ref_ptr<osgModeling::Loft> geom3 = new osgModeling::Loft;
    geom3->setProfile( path3.get() );

    geom3->addShape( createNurbsCircle(1.3f).get() );
    geom3->addShape( createNurbsCircle(1.3f).get() );
    geom3->addShape( createNurbsCircle(1.3f).get() );

    geom3->update();


    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom.get() );
    osg::ref_ptr<osg::Geode> geode2 = new osg::Geode;
    geode2->addDrawable( geom2.get() );
    osg::ref_ptr<osg::Geode> geode3 = new osg::Geode;
    geode3->addDrawable( geom3.get() );

    osg::ref_ptr<osg::Group> group = new osg::Group;
    group->addChild(geode.get());
    group->addChild(geode2.get());
    //group->addChild(geode3.get());

    return group;
}

double osgModeler::dist(osg::Vec3 a, osg::Vec3 b)
{
    return pow(pow(a.x()-b.x(), 2) + pow(a.y()-b.y(), 2) + pow(a.z()-b.z(), 2), 0.5);
}

osg::Vec3 osgModeler::middle_ctr_pt(osg::Vec3 start, osg::Vec3 end, osg::Vec3 on_curve, double t)
{
    osg::Vec3 ret = start;

    double k = 2*t*(1-t);
    if(k != 0)
    {
        ret = on_curve - start*(1-t)*(1-t) - end*(t*t);
        ret = ret * (1/k);
    }
    /*
    printf("start=(%f,%f,%f)\n", start.x(), start.y(), start.z());
    printf("end=(%f,%f,%f)\n", end.x(), end.y(), end.z());
    printf("middle=(%f,%f,%f)\n", ret.x(), ret.y(), ret.z());
    */

    return ret;
}

double osgModeler::orient3D(osg::Vec3 av, osg::Vec3 bv, osg::Vec3 cv, osg::Vec3 dv)
{
    double a, b, c, d, e, f, g, h, i;

    a = av.x()-dv.x();
    d = bv.x()-dv.x();
    g = cv.x()-dv.x();

    b = av.y()-dv.y();
    e = bv.y()-dv.y();
    h = cv.y()-dv.y();

    c = av.z()-dv.z();
    f = bv.z()-dv.z();
    i = cv.z()-dv.z();

    return a*(e*i-h*f) - d*(b*i-h*c) + g*(b*f-e*c);
}

osg::Vec3 osgModeler::intersect(osg::Vec3 a, osg::Vec3 an, osg::Vec3 b, osg::Vec3 bn)
{
    osg::Vec3 head_y = proj(a, a+an, b, b+bn);
    bn = head_y-b;

    double k = an.y()*bn.x() - an.x()*bn.y();
    double t = (1/k) * (-bn.y()*(b.x()-a.x()) + bn.x()*(b.y()-a.y()));
    return a + an*t;
}

osg::Vec3 osgModeler::proj(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 p)
{
    osg::Vec3 ba = b - a;
    osg::Vec3 bc = b - c;
    osg::Vec3 n = bc ^ ba;
    n.normalize();
    ba.normalize();
    osg::Vec3 u1 = ba;
    osg::Vec3 u2 = u1 ^ n;

    osg::Vec3 bp = p - b;
    osg::Vec3 head_y = u1 * (u1 * bp) + u2 * ( u2 * bp);
    
    return b + head_y;
}

int osgModeler::modular(int x, int size)
{
    return (x % size + size) % size;
}

osg::Vec3 osgModeler::cubic_ctr(osg::Vec3 start, osg::Vec3 opposite, osg::Vec3 on_curve, osg::Vec3 normal)
{
    osg::Vec3 ret;

    double a = dist(start, on_curve);
    double b = dist(opposite, on_curve);
    double t = a / (a + b);

    osg::Vec3 proj_normal = proj(start, on_curve, opposite, start+normal);
    proj_normal = proj_normal - start;
    proj_normal.normalize();

    double D = 3*t*(1-t)*(1-t+t*b/a);
    osg::Vec3 T = (on_curve - start*(2*t*t*t-3*t*t+1) - opposite*(3*t*t-2*t*t*t)) * 1/D;
    
    //don't know why the 3 k are not equal
    double k = T.x() / proj_normal.x();
    //printf("cubic_ctr: k(%f)\n", k);
    //double k = T.y() / proj_normal.y();
    //double k = T.z() / proj_normal.z();
    //printf("proj_normal: (%f %f %f) k: (%f %f %f)\n", proj_normal.x(), proj_normal.y(), proj_normal.z(), T.x()/proj_normal.x(), T.y()/proj_normal.y(), T.z()/proj_normal.z());
    //printf("T: (%f %f %f)\n", T.x(), T.y(), T.z());
    ret = proj_normal * k;

    return ret;
}

osg::ref_ptr <osg::Geode> osgModeler::triangulateCurve(std::vector <osg::ref_ptr <osgModeling::BezierCurve> > a, osg::Vec3 cc)
{
    osg::ref_ptr <osg::Geode> geode = new osg::Geode;
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr <osg::Vec3Array> vertex = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> normal_list = new osg::Vec3Array;

    geom->setVertexArray(vertex.get());
    geom->setNormalArray(normal_list.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    /*this normal test may not apply to all points
    bool normal_flip = false;
    osg::Vec3 flip_test_a = (*a[0]->getPath())[0];
    osg::Vec3 flip_test_b = (*a[1]->getPath())[0];
    osg::Vec3 flip_test_c = (*a[0]->getPath())[1];
    if(orient3D(flip_test_a, flip_test_b, flip_test_c, cc) < 0)
        normal_flip = true;
    */

    for(unsigned int i=0; i<a.size()-1; i++)
    {
        osg::ref_ptr <osgModeling::BezierCurve> curveU = a[i];
        osg::ref_ptr <osgModeling::BezierCurve> curveD = a[i+1];

        int pointSize = curveU->getPath()->size();
        for(int j=0; j<pointSize; j++)
        {
            osg::Vec3 up = (*curveU->getPath())[j];
            osg::Vec3 down = (*curveD->getPath())[j];

            if(j!=pointSize-1)
            {
                osg::Vec3 up_next = (*curveU->getPath())[j+1];
                osg::Vec3 ac = up_next - up;
                osg::Vec3 ab = down - up;
                osg::Vec3 normal = ab ^ ac;
                normal.normalize();
                //if(normal_flip)
                    //normal = normal * -1;
                if(orient3D(down, up_next, up, cc) < 0)
                {
                    normal = -normal;
                    vertex->push_back(down);
                    vertex->push_back(up);
                }
                else
                {
                    vertex->push_back(up);
                    vertex->push_back(down);
                }

                normal_list->push_back(normal);
                normal_list->push_back(normal);
            }
            else
            {
                osg::Vec3 down_pre = (*curveD->getPath())[j-1];
                osg::Vec3 ca = down_pre - down;
                osg::Vec3 cb = up - down;
                osg::Vec3 normal = cb ^ ca;
                normal.normalize();
                //if(normal_flip)
                    //normal = normal * -1;
                if(orient3D(down_pre, down, up, cc) < 0)
                {
                    normal = -normal;
                    vertex->push_back(down);
                    vertex->push_back(up);
                }
                else
                {
                    vertex->push_back(up);
                    vertex->push_back(down);
                }

                normal_list->push_back(normal);
                normal_list->push_back(normal);
            }
        }

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 2*i*pointSize, 2*pointSize));
    }

    geode->addDrawable(geom.get());

    return geode;
}

osg::ref_ptr <osg::Geode> osgModeler::triangulateTubeCurve(std::vector <osg::ref_ptr <osg::Vec3Array> > oc_list, std::vector <osg::Vec3> oc_center_list)
{
    osg::ref_ptr <osg::Geode> geode = new osg::Geode;
    osg::ref_ptr <osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr <osg::Vec3Array> vertex = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> normal_list = new osg::Vec3Array;

    geom->setVertexArray(vertex.get());
    geom->setNormalArray(normal_list.get());
    geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    //try adding color
    //does not work alone, coz it will change the color of leaves too!!
    osg::Vec4Array *colors = new osg::Vec4Array;
    //colors->push_back(osg::Vec4(0.565, 0.384, 0.2, 1.0));//lighter
    //colors->push_back(osg::Vec4(0.392, 0.216, 0.35, 1.0));//darker
    //colors->push_back(osg::Vec4(0.2, 0.1, 0.0, 1.0));//flo
    colors->push_back(osg::Vec4(0.25, 0.15, 0.0, 1.0));//mine

    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    //remove all duplicate such that no adjacent oc_circles are exactly the same
    std::vector <osg::ref_ptr <osg::Vec3Array> > oc_list_stripped; 
    std::vector <osg::Vec3> oc_center_list_stripped;

    for(unsigned int i=0; i<oc_center_list.size(); i++)
    {
        //push all new items into stripped
        if(oc_center_list_stripped.empty())
        {
            oc_list_stripped.push_back(oc_list[i]);
            oc_center_list_stripped.push_back(oc_center_list[i]);
        }
        else
        {
            osg::Vec3 cur = oc_center_list[i];
            if(cur != oc_center_list_stripped[oc_center_list_stripped.size()-1])
            {
                oc_list_stripped.push_back(oc_list[i]);
                oc_center_list_stripped.push_back(oc_center_list[i]);
            }
        }
    }
    //debug duplicate
    //for(unsigned int i=0; i<oc_center_list_stripped.size(); i++)
    //    printf("oc_center[%d] = (%lf,%lf,%lf)\n", i, oc_center_list_stripped[i].x(), oc_center_list_stripped[i].y(), oc_center_list_stripped[i].z());
    //reassign back to original, then everything is not glued together
    oc_list = oc_list_stripped;
    oc_center_list = oc_center_list_stripped;

    //oc_list.size has to be at least 2, for the triangulation to work
    if(oc_list.size() < 2)
        return geode;

    //solve the misalignment problem of adjacent oc_circles in oc_list
    for(unsigned int i=0; i<oc_list.size()-1; i++)
    {
        osg::ref_ptr <osg::Vec3Array> curveU = oc_list[i];
        osg::ref_ptr <osg::Vec3Array> curveD = oc_list[i+1];

        int pointSize = curveU->size();

        if(pointSize < 2)
            break;

        double min_dist = -1.0;
        int min_offset_j = 0;
        int min_offset_k = 0;

        //find the closest-pair as the correspondance
        for(int j=0; j<pointSize; j++)
        {
            osg::Vec3 up = (*curveU)[j];

            //find the closest point in curveD to up
            for(int k=0; k<pointSize; k++)
            {
                osg::Vec3 down = (*curveD)[k];

                double cur_dist = dist(up, down);
                if(min_dist == -1.0 || cur_dist < min_dist)
                {
                    min_dist = cur_dist;
                    min_offset_j = j;
                    min_offset_k = k;
                }
            }
        }

        //update oc_list[i+1] by the min_offset_j and min_offset_k
        osg::ref_ptr <osg::Vec3Array> curveD_offseted = new osg::Vec3Array;

        //check which side should go first
        osg::Vec3 down_next = (*curveD)[(min_offset_k+1)%pointSize];
        osg::Vec3 down_next2 = (*curveD)[(pointSize+min_offset_k-1)%pointSize];
        osg::Vec3 up_next = (*curveU)[(min_offset_j+1)%pointSize];

        bool forward = true;
        if(dist(down_next2, up_next) < dist(down_next, up_next))
            forward = false;

        //relative offset
        int min_offset = (pointSize + min_offset_k - min_offset_j) % pointSize;
        //printf("min_offset(%d) min_offset_j(%d) min_offset_k(%d)\n", min_offset, min_offset_j, min_offset_k);

        //assume always go forward, because it is always formed by createNurbsCircle //not sure
        if(true)
        {
            for(int j=min_offset; j<pointSize; j++)
                curveD_offseted->push_back((*curveD)[j]);

            for(int j=0; j<min_offset; j++)
                curveD_offseted->push_back((*curveD)[j]);
        }
        else
        {
            for(int j=min_offset; j>=0; j--)
                curveD_offseted->push_back((*curveD)[j]);
            
            for(int j=pointSize-1; j>min_offset; j--)
                curveD_offseted->push_back((*curveD)[j]);

            printf("not forward\n");
        }

        //printf("%d: min_offset = %d pointSize = %d\n", i, min_offset, pointSize);

        //final update
        oc_list[i+1] = curveD_offseted;

        //debug points in one circle
        //for(unsigned int j=0; j<curveD_offseted->size(); j++)
        //{
        //    osg::Vec3 d_p = (*curveD_offseted)[j];
        //    printf("%d: (%lf,%lf,%lf)\n", i, d_p.x(), d_p.y(), d_p.z());
        //}

        //printf("\n");

        //printf("pointSize(%d) new_pointSize(%d)\n", pointSize, curveD_offseted->size());
    }

    //special note: now the first and the last points are not equal, 
    //so need to re-use the first point to complete the geometry, for each oc in oc_list
    for(unsigned int i=0; i<oc_list.size(); i++)
    {
        osg::ref_ptr <osg::Vec3Array> curve = oc_list[i];
        if(!curve->empty())
            curve->push_back((*curve)[0]);
    }

    //debug
    //for(unsigned int i=0; i<oc_list.size(); i++)
    //{
    //    osg::ref_ptr <osg::Vec3Array> curveU = oc_list[i];
    //    int pointSize = curveU->size();
    //    for(int j=0; j<pointSize; j++)
    //    {
    //        osg::Vec3 d_p = (*curveU)[j];
    //        printf("%d: (%lf,%lf,%lf)\n", i, d_p.x(), d_p.y(), d_p.z());
    //    }
    //    printf("\n");
    //}

    for(unsigned int i=0; i<oc_list.size()-1; i++)//assume the 2nd and 3rd are not glued together
    //for(unsigned int i=0; i<oc_list.size()-1; i+=2)//assume the 2nd and 3rd are glued together, does not work for step != 2
    {
        osg::ref_ptr <osg::Vec3Array> curveU = oc_list[i];
        osg::ref_ptr <osg::Vec3Array> curveD = oc_list[i+1];
        osg::Vec3 centerU = oc_center_list[i];
        osg::Vec3 centerD = oc_center_list[i+1];

        int pointSize = curveU->size();
        for(int j=0; j<pointSize; j++)
        {
            osg::Vec3 up = (*curveU)[j];
            osg::Vec3 down = (*curveD)[j];

            //detect nan
            if(isnan(up.x()))
                printf("hihi up nan\n");
            if(isnan(down.x()))
                printf("hihi down nan\n");

            //when it's exported into obj format, the normal depends on the vertex order, not the given normal
            //vertex->push_back(up);
            vertex->push_back(down);
            vertex->push_back(up);

            osg::Vec3 nU = up - centerU;
            osg::Vec3 nD = down - centerD;
            nU.normalize();
            nD.normalize();

            normal_list->push_back(nD);
            normal_list->push_back(nU);
            /*
            normal_list->push_back(nU*-1.0);
            normal_list->push_back(nD*-1.0);
            */
        }

        //debug triangle count
        //printf("triangulateCurve: oc_list.size(%d) pointSize(%d)\n", int(oc_list.size()), pointSize);

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, 2*i*pointSize, 2*pointSize));
        //geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP, i*pointSize, 2*pointSize));//for i taking two steps in each loop
    }

    //really don't know why if oc_list == 2, no visual output by osg::DrawArrays
    /*
    */
    if(int(oc_list.size()) > 2)
        geode->addDrawable(geom.get());
    //if oc_list size == 2, draw a line
    else
    {
        /*
        osg::ref_ptr <osg::Geometry> geom2 = new osg::Geometry;
        osg::ref_ptr <osg::Vec3Array> vertex2 = new osg::Vec3Array;
        for(unsigned int i=0; i<oc_center_list.size(); i++)
            vertex2->push_back(oc_center_list[i]);
        geom2->setVertexArray(vertex2.get());
        geom2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, oc_center_list.size()));

        osg::StateSet* stateset = new osg::StateSet;
        osg::LineWidth* linewidth = new osg::LineWidth();

        linewidth->setWidth(1.0f);
        stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
        geom2->setStateSet(stateset);
        
        geode->addDrawable(geom2.get());
        */

        /*
        osg::Vec3 a = oc_center_list[0], b = oc_center_list[1];
        if((a-b).length() > 1.0)
            //printf("%lf (%lf %lf %lf) (%lf %lf %lf)\n", (a-b).length(), a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
            geode = createLineBox(a, b, 1.0);
        */
    }

    return geode;
}

bool osgModeler::isGoodOC3(OrientedCircle a, OrientedCircle b, OrientedCircle c)
//osg::ref_ptr <osg::Vec3Array> osgModeler::isGoodOC3(OrientedCircle a, OrientedCircle b, OrientedCircle c)
{
    bool ab = false, ac = false, bc = false;

    osg::ref_ptr <osg::Vec3Array>  a_points = a.points();
    osg::ref_ptr <osg::Vec3Array>  b_points = b.points();
    osg::ref_ptr <osg::Vec3Array>  c_points = c.points();

    int a_start = 0, b_start = 0;
    double min_ab = dist(a.center(), b.center());
    for(unsigned int i=0; i<a_points->size(); i++)
        for(unsigned int j=0; j<b_points->size(); j++)
        {
            double distance_ab = dist((*a_points)[i], (*b_points)[j]);
            if(distance_ab < min_ab)
            {
                a_start = i;
                b_start = j;
                min_ab = distance_ab;
            }
        }
    int a_start_opposite = modular(a_start+a_points->size()/2, a_points->size());
    int b_start_opposite = modular(b_start+b_points->size()/2, b_points->size());

    osg::Vec3 abm = intersect((*a_points)[a_start], a.normal(), (*b_points)[b_start], b.normal());
    osg::Vec3 abm2 = intersect((*a_points)[a_start_opposite], a.normal(), (*b_points)[b_start_opposite], b.normal());

    bool p = a.whichSide(abm);
    bool q = a.whichSide(abm2);
    bool r = b.whichSide(abm);
    bool s = b.whichSide(abm2);

    //printf("p(%d) q(%d) r(%d) s(%d)\n", p, q, r, s);

    if( ((p && q) || (!p && !q)) && ((r && s) || (!r && !s)) )
        ab = true;
    if(!ab)
        return false;
    
    int c_start = 0;
    double min_ac = dist(a.center(), c.center());
    for(unsigned int i=0; i<b_points->size(); i++)
    {
        double distance_ac = dist((*a_points)[a_start_opposite], (*c_points)[i]);
        if(distance_ac < min_ac)
        {
            c_start = i;
            min_ac = distance_ac;
        }
    }

    osg::Vec3 acm = intersect((*a_points)[a_start_opposite], a.normal(), (*c_points)[c_start], c.normal());

    q = a.whichSide(acm);
    r = c.whichSide(abm);
    s = c.whichSide(acm);

    if( ((p && q) || (!p && !q)) && ((r && s) || (!r && !s)) )
        ac = true;

    int c_start_opposite = modular(c_start+c_points->size()/2, c_points->size());
    osg::Vec3 bcm = intersect((*b_points)[b_start_opposite], b.normal(), (*c_points)[c_start_opposite], c.normal());

    p = b.whichSide(abm);
    q = b.whichSide(bcm);
    r = c.whichSide(abm);
    s = c.whichSide(bcm);

    if( ((p && q) || (!p && !q)) && ((r && s) || (!r && !s)) )
        bc = true;

    return ac && bc;
}

//osg::ref_ptr <osg::Vec3Array> osgModeler::repositionOC3(OrientedCircle *a, OrientedCircle *b, OrientedCircle *c)
osg::ref_ptr <osg::Group> osgModeler::repositionOC3(OrientedCircle *a, OrientedCircle *b, OrientedCircle *c, OrientedCircle *c_down)
{
    osg::ref_ptr <osg::Group> root = new osg::Group;
    osg::ref_ptr <osg::Vec3Array> debug = new osg::Vec3Array;

    osg::Vec3 ab = b->center() - a->center();
    osg::Vec3 ac = c->center() - a->center();
    ab.normalize();
    ac.normalize();

    //ab = ab * (1.25 * a->trueRadius() + 1.25 * b->trueRadius());
    ab = ab * (1.35 * a->trueRadius() + 1.35 * b->trueRadius());
    ab = ab + ac * b->trueRadius() * 2.0;

    b->makeTranslate(ab);

    //place c such that Triangle abc is equilateral
    osg::Vec3 u = b->center() - a->center();
    osg::Vec3 v = c->center() - a->center();
    double k = u.length();
    u.normalize();
    v.normalize();

    osg::Vec3 v2 = v - u * (u*v);
    osg::Vec3 move_c = a->center() - c->center() + u*k/2 + v2*(pow(3, 0.5)/2*k);
    move_c = move_c * 0.55;

    c->makeTranslate(move_c);
    c->makeScale((a->trueRadius() + b->trueRadius()) * 0.75);//duno why it will appear darker when scale is applied
    //c->makeScale(a->trueRadius()+20);

    //transform c_down too
    c_down->makeTranslate(move_c);
    c_down->makeScale(c->center(), (a->trueRadius() + b->trueRadius()) * 0.75);

    //u, v is the orthnormal of the plane spanned by the three centers
    v2.normalize();
    v = v2;

    //project the a b centers on the plane that passes through c's center and has the same normal to c
    /* this is wrong: a_p0 and b_p0 may not be on the plane spanned by the three centers
    osg::Vec3 proj_ac = proj(c->center()+c->Tx(), c->center(), c->center()+c->Ty(), a->center());
    osg::Vec3 proj_bc = proj(c->center()+c->Tx(), c->center(), c->center()+c->Ty(), b->center());

    osg::Vec3 a_p0_dir = proj_ac - proj_bc;
    a_p0_dir.normalize();

    osg::Vec3 a_p0 = c->center() + a_p0_dir*c->trueRadius();
    osg::Vec3 b_p0 = c->center() - a_p0_dir*c->trueRadius();
    */

    //find the direction of line of the intersection of two planes
    osg::Vec3 cn = c->normal();
    cn.normalize();
    osg::Vec3 inter_dir = cn ^ (v ^ u);
    osg::Vec3 a_p0 = c->center() + inter_dir * c->trueRadius();
    osg::Vec3 b_p0 = c->center() - inter_dir * c->trueRadius();
    if(dist(a_p0, a->center()) > dist(b_p0, a->center()))
    {
        osg::Vec3 temp = b_p0;
        b_p0 = a_p0;
        a_p0 = temp;
    }

    //ensure a_p0 and b_p0 are not inside the cicles
    if(dist(a_p0, a->center()) < a->trueRadius())
    {
        osg::Vec3 move_a_further = a->center()-a_p0;
        double diff = 2*a->trueRadius() - move_a_further.length();
        move_a_further.normalize();
        a->makeTranslate(move_a_further*(diff+1));
        printf("hihi move a_p0 out move_a_further(%f %f %f) diff=%f\n", move_a_further.x(), move_a_further.y(), move_a_further.z(), diff);
    }
    if(dist(b_p0, b->center()) < b->trueRadius())
    {
        osg::Vec3 move_b_further = b->center()-b_p0;
        double diff = 2*b->trueRadius() - move_b_further.length();
        move_b_further.normalize();
        b->makeTranslate(move_b_further*(diff+1));
        printf("hihi move b_p0 out\n");
    }

    //treat a as a sphere, find point:p where line:p a_0p tangents the sphere at p
    osg::Vec3 w = a_p0 - a->center();
    double c1 = u * w, c2 = v * w, r = a->trueRadius();
    double A, B, C;
    A = c1*c1 + c2*c2;
    B = -2 * c1 * r * r;
    C = r*r*r*r - c2*c2*r*r;
    double det = B*B - 4*A*C;
    osg::Vec3 p_a;

    if(det < 0)
        printf("hihi at osgModeler: 1436 det=%f < 0 c1=%f c2=%f r=%f\n", det, c1, c2, r);
    else
    {
        double det_sr = pow(det, 0.5);
        double s1, s2, t1, t2;
        s1 = (-B + det_sr) / (2*A);
        s2 = (-B - det_sr) / (2*A);
        t1 = pow(r*r - s1*s1, 0.5);
        t2 = pow(r*r - s2*s2, 0.5);
        osg::Vec3 p_s1 = a->center() + u*s1 + v*t1;
        osg::Vec3 p_s2 = a->center() + u*s2 + v*t2;
        if((p_s1-c->center()).length() > (p_s2-c->center()).length())
            p_a = p_s1;
        else
            p_a = p_s2;
    }

    //treat b as a sphere, find point:p where line:p b_0p tangents the sphere at p
    w = b_p0 - b->center();
    c1 = u * w, c2 = v * w, r = b->trueRadius();
    A = c1*c1 + c2*c2;
    B = -2 * c1 * r * r;
    C = r*r*r*r - c2*c2*r*r;
    det = B*B - 4*A*C;
    osg::Vec3 p_b;

    if(det < 0)
        printf("hihi at osgModeler: 1249 det=%f < 0 c1=%f c2=%f r=%f\n", det, c1, c2, r);
    else
    {
        double det_sr = pow(det, 0.5);
        double s1, s2, t1, t2;
        s1 = (-B + det_sr) / (2*A);
        s2 = (-B - det_sr) / (2*A);
        t1 = pow(r*r - s1*s1, 0.5);
        t2 = pow(r*r - s2*s2, 0.5);
        osg::Vec3 p_s1 = b->center() + u*s1 + v*t1;
        osg::Vec3 p_s2 = b->center() + u*s2 + v*t2;
        if((p_s1-c->center()).length() > (p_s2-c->center()).length())
            p_b = p_s1;
        else
            p_b = p_s2;
    }

    //plane normal and normals of limiting oc
    osg::Vec3 plane_n = v ^ u;
    osg::Vec3 a_Tx_limit, a_n_limit, a_Ty_limit;

    a_Tx_limit = p_a - a->center();
    a_Tx_limit.normalize();
    a_n_limit = plane_n ^ a_Tx_limit;
    a_Ty_limit = a_Tx_limit ^ a_n_limit;

    osg::Vec3 b_Tx_limit, b_n_limit, b_Ty_limit;

    b_Tx_limit = p_b - b->center();
    b_Tx_limit.normalize();
    b_n_limit = b_Tx_limit ^ plane_n;
    b_Ty_limit = b_n_limit ^ b_Tx_limit;

    OrientedCircle a_oc_limit(a->center(), a_Tx_limit, a_Ty_limit, a_n_limit);
    a_oc_limit.setRadius(a->trueRadius());

    OrientedCircle b_oc_limit(b->center(), b_Tx_limit, b_Ty_limit, b_n_limit);
    b_oc_limit.setRadius(b->trueRadius());

    //maximum degree a_md that I can rotate
    //osg::Vec3 cn = c->normal();
    cn.normalize();

    double cos_a_md = a_Tx_limit * cn;
    double a_md = acos(cos_a_md);
    if(a_md > osg::PI/2.0)
        a_md = osg::PI - a_md;
    //int a_max_degree = 180/osg::PI * a_md;//unused variable

    double cos_b_md = b_Tx_limit * cn;
    double b_md = acos(cos_b_md);
    if(b_md > osg::PI/2.0)
        b_md = osg::PI - b_md;
    //int b_max_degree = 180/osg::PI * b_md;//unused variable

    //rotate to a good position
    a_oc_limit.makeRotate(a->center(), plane_n, 5);
    //a_oc_limit.makeRotate(a->center(), plane_n, 20);
    //a_oc_limit.makeRotate(a->center(), plane_n, a_max_degree-25);
    b_oc_limit.makeRotate(b->center(), -plane_n, 5);
    //b_oc_limit.makeRotate(b->center(), -plane_n, b_max_degree);
    if(isGoodOC3(a_oc_limit, b_oc_limit, *c))
        root = branchJoint(a_oc_limit, b_oc_limit, *c);
    else
    {
        printf("hihi in repositionOC3: no a good OC3\n");
        root = branchJoint(a_oc_limit, b_oc_limit, *c);
    }

    //translate to a non-overlapping centers
    osg::Vec3 move_a = a->center() - c->center();
    move_a.normalize();
    a->makeTranslate(move_a);
    osg::Vec3 move_b = b->center() - c->center();
    move_b.normalize();
    b->makeTranslate(move_b);

    //repositionOC2(a, &a_oc_limit);
    //repositionOC2(b, &b_oc_limit);

    root->addChild(createTube(a, &a_oc_limit));
    root->addChild(createTube(b, &b_oc_limit));

    /*
    for(int i=0; i<20; i++)
        debug->push_back(p_a+a_n_limit*i);
    return debug;
    */
    root->addChild(osgModeler::visualizeCurve(debug, 2).get());

    return root;
}

std::vector <int> osgModeler::repositionOC2(OrientedCircle *a, OrientedCircle *c)
{
    std::vector <int> ret;
    ret.push_back(0);
    ret.push_back(0);

    if(!a || !c || a->center() == c->center())
        return ret;

    //make sure a and c are disjoint only, not well-separated
    osg::Vec3 move_a = a->center() - c->center();
    move_a.normalize();
    while(a->isOverlapped(c))
        a->makeTranslate(move_a);
    //a->makeTranslate(move_a*5);

    //project a's center
    osg::Vec3 a_center_proj = proj(c->center(), c->center()+c->normal(), c->center()+c->normal()+a->normal(), a->center());

    move_a = a_center_proj - a->center();
    a->makeTranslate(move_a);

    //connect the two
    osg::ref_ptr <osg::Vec3Array> a_pts = new osg::Vec3Array;
    osg::ref_ptr <osg::Vec3Array> c_pts = new osg::Vec3Array;
    a_pts = a->points();
    c_pts = c->points();
    double minDist = dist((*a_pts)[0], (*c_pts)[0]);
    for(unsigned int i=0; i<a_pts->size(); i++)
        for(unsigned int j=0; j<c_pts->size(); j++)
        {
            double newDist = dist((*a_pts)[i], (*c_pts)[j]);
            if(newDist < minDist)
            {
                minDist = newDist;
                ret[0] = i;
                ret[1] = j;
                move_a = (*c_pts)[j] - (*a_pts)[i];
            }
        }
    a->makeTranslate(move_a);

    return ret;
}
