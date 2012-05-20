#include "BDLSG2Bezier.h"
#include "Transformer.h"
#include <queue>
#include <algorithm>

BDLSG2Bezier::BDLSG2Bezier(std::string path): _output_path(path), _xiao_id(1)
{
    //file handler
    _out = fopen(_output_path.c_str(), "w");
    if(!_out)
    {
        printf("BDLSG2Bezier::BDLSG2Bezier(%s) error.\n", _output_path.c_str());
        return;
    }
}

BDLSG2Bezier::~BDLSG2Bezier()
{
    fclose(_out);
}

//void BDLSG2Bezier::output(BDLSkeletonNode *root, osg::Vec3 origin, double height, int angle, bool maya, double simplication)
void BDLSG2Bezier::output(BDLSkeletonNode *root)
{
    if(!root)
        return;

    //to prevent modifying the original tree
    //root = BDLSkeletonNode::copy_tree(root);

    //scale up root's radius because the scale in Blender is stranglely scaled down
    //root->_radius *= 9.5f;
    
    //translate and scale the tree, find height and radius of each node, get the scale
    //the returned scale is used to add an ugly node below the root and for scaling the leaf
    //double scale = BDLSkeletonNode::rectify_skeleton_tree(root, origin, height, angle, maya);

    //scale += (1.0-simplication) * scale * 3.2;

    //remove trivial nodes
    //BDLSkeletonNode::compress_skeleton_tree(root);

    //simplify the trees by collasping some edges
    //if(true)
    //{
    //    if(simplication != 1.0)
    //    {
    //        int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(root);
    //        for(int i=0; i<remaining_edges*(1-simplication); i++)
    //            BDLSkeletonNode::collaspe_skeleton_tree(root);
    //    }
    //}

    //construct the path from longest to shortest
    //stores all the leaf skeleton nodes
    std::vector <BDLSkeletonNode *> leafs;
    //distance between leafs and root
    std::vector <double> dists;
    //sorted dists
    std::vector <double> sorted_dists;

    //BFS all skeleton nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();
        
        if(front->_children.size() == 0)
        {
            leafs.push_back(front);
            dists.push_back(front->_height);
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue.push(child);
        }
    }

    //sort the dists
    sorted_dists = dists;
    sort(sorted_dists.begin(), sorted_dists.end());
    reverse(sorted_dists.begin(), sorted_dists.end());

    //print the total number of branches
    fprintf(_out, "%d\n", int(sorted_dists.size()));

    for(unsigned int branch=0; branch<sorted_dists.size(); branch++)
    //for(unsigned int branch=0; branch<1; branch++)
    {
        double cur_dist = sorted_dists[branch];
        int cur_index = 0;

        //find the corresponding skeleton leaf
        for(unsigned int l=0; l<dists.size(); l++)
            if(dists[l] == cur_dist && !leafs[l]->_mesh_done)
            {
                cur_index = l;
                break;
            }

        BDLSkeletonNode *cur_leaf = leafs[cur_index];

        std::vector <osg::Vec3> path;
        std::vector <double> radii;

        BDLSkeletonNode *node = cur_leaf;

        while(node)
        {
            path.push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));
            radii.push_back(node->_radius);

            if(node->_mesh_done)
                break;

            node->_mesh_done = true;
            node = node->_prev;
        }

        //provide hint on establishing the second control point
        osg::Vec3 hint(0.0f, 0.0f, 0.0f);
        if(node && node->_prev && path.size() >= 2)
        {
            //osg::Vec3 v_node(node->_sx, node->_sy, node->_sz);
            osg::Vec3 v_node = path[path.size()-2];
            osg::Vec3 v_node_p(node->_prev->_sx, node->_prev->_sy, node->_prev->_sz);
            hint = v_node - v_node_p;
            hint.normalize();
        }

        //shorten the tip and end of the branch for all path, except the root
        //doesn't matter too much, since they will be smoothed in afterwards
        if(!radii.empty())
        {
            radii[0] *= 0.25; //tip
            if(branch != 0 && path.size() != 2)
                radii[radii.size()-1] *= 0.60; //end
            if(branch != 0 && path.size() == 2)
                radii[radii.size()-1] *= 0.45; //end
        }

        //move away the end node for blender to glue it
        if(branch != 0 && path.size() >= 2 && radii.size() >= 2)
        {
            osg::Vec3 end = path[path.size()-1];
            osg::Vec3 end2 = path[path.size()-2];
            osg::Vec3 translate = end2 - end;
			translate.normalize();
            //translate = translate * radii[radii.size()-1] / 3.5f;
            translate = translate * radii[radii.size()-1] / 4.8f;
            //translate = translate * radii[radii.size()-1] / 5.5f;

            path[path.size()-1] = end + translate;
        }

        //real work
        //if(int(path.size()) == 1)
        //{
        //    osg::Vec3 p = path[0];
        //    printf("%f %f %f\n", p.x(), p.y(), p.z());
        //    //printf("doing %d: path(%d) radii(%d)\n", branch, int(path.size()), int(radii.size()));
        //}
        output_tube(path, radii, hint);
    }

    //BDLSkeletonNode::delete_this(root);
}

void BDLSG2Bezier::blender_test()
{
    osg::Vec3 a(-1.236f, 0.775f, 9.585f);
    osg::Vec3 b(-1.811f, 1.405f, 11.045f);
    osg::Vec3 c(-3.166f, 3.035f, 12.412f);
    osg::Vec3 d(-4.686f, 3.875f, 12.137f);

    std::vector <osg::Vec3> on_curve = Transformer::interpolate_bezier_3(a, b, c, d);
    for(unsigned int i=0; i<on_curve.size(); i++)
        printf("v %f %f %f\n", on_curve[i].x(), on_curve[i].y(), on_curve[i].z());
    printf("hihi\n");

    std::vector <osg::Vec3> ctrs;
    ctrs.push_back(a);
    ctrs.push_back(b);
    ctrs.push_back(c);
    ctrs.push_back(d);

    ctrs = seg2BezTriple(ctrs);
    if(!ctrs.empty())
    {
        printf("%d\n", int(ctrs.size()) / 3);
        for(unsigned int j=0; j<ctrs.size(); j+=3)
        {
            osg::Vec3 a, b, c;
            a = ctrs[j];
            b = ctrs[j+1];
            c = ctrs[j+2];

            printf("%f %f %f %f %f %f %f %f %f\n", a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z());
        }
    }
}

void BDLSG2Bezier::output_palm(BDLSkeletonNode *root, std::vector <float> in_radii)
{
    if(!root)
        return;

    //a. check if the palm data structure is valid
    //assume the palm has at least 2 long branching leaves
    //                ____
    //               /
    // root ------ ter -----
    //              \____
    BDLSkeletonNode *terminal = root;
    while(!terminal->_children.empty())
    {
        if(terminal->_children.size() > 1)
            break;
        terminal = terminal->_children[0];
    }
    if(terminal == root || terminal->_children.empty())
    {
        printf("BDLSG2Bezier::output_palm():incorrect input skeleton\n");
        return;
    }

    //b. book keeping the nodes on the main branch
    std::vector <osg::Vec3> main_branch;
    BDLSkeletonNode *node = root;
    while(node != terminal)
    {
        main_branch.push_back(Transformer::toVec3(node));
        node = node->_children[0];

        if(node == terminal)
        {
            osg::Vec3 N = Transformer::toVec3(node);

            //extend a little bit for gluing
            if(!main_branch.empty())
            {
                osg::Vec3 last = main_branch[main_branch.size()-1];
                osg::Vec3 translate = N - last;
                float extend = translate.length() * 0.05f;
                translate.normalize();
                N += translate * extend;
            }
                
            main_branch.push_back(N);
        }
    }

    //c. store the sets of 4 Bezier control points
    std::vector <osg::Vec3> leaf_branch;
    osg::Vec3 ter = Transformer::toVec3(terminal);
    std::vector <osg::Vec3> sec_pts, third_pts, forth_pts;

    //dfs
    std::vector <osg::Vec3> tmp_list;
    std::stack <BDLSkeletonNode *> Stack;
    Stack.push(terminal);
    while(!Stack.empty())
    {
        BDLSkeletonNode *top = Stack.top();
        Stack.pop();

        //assume the structure is fixed and defined as:
        //       __
        //      /
        // ter -----
        //      \__
        if(top != terminal)
            tmp_list.push_back(Transformer::toVec3(top));

        //one way road, end if we reach the leaf
        if(top->_children.empty())
        {
            //ignore the branch if it does not have 4 control points
            if(tmp_list.size() >= 3)
            {
                sec_pts.push_back(tmp_list[0]);
                third_pts.push_back(tmp_list[tmp_list.size()-2]);
                forth_pts.push_back(tmp_list[tmp_list.size()-1]);
                tmp_list.clear();
            }
        }

        for(unsigned int i=0; i<top->_children.size(); i++)
            Stack.push(top->_children[i]);
    }

    if(sec_pts.empty())
    {
        printf("BDLSG2Bezier::output_palm():incorrect input skeleton\n");
        return;
    }

    //d. infer palm-specific radii
    //each node of a leaf-branch has the same radius,
    //which is equal to its total length times a constant k
    std::vector <double> radii;//radius for each branch
    float base_radius;
    if(in_radii.empty())//fallback: base radius is not given
    {
        //radius of main branch is equal to the (max of all the leaf branches' radii) * k2
        const double MassThicknessRatio = 0.003;//hard-code: radius : length
        double main_branch_radius = 0.0;
        for(unsigned int i=0; i<sec_pts.size(); i++)
        {
            osg::Vec3 ctr_pt1 = ter;
            osg::Vec3 ctr_pt2 = sec_pts[i];
            osg::Vec3 ctr_pt3 = third_pts[i];
            osg::Vec3 ctr_pt4 = forth_pts[i];

            float length = (ctr_pt2-ctr_pt1).length() + (ctr_pt3-ctr_pt2).length() + (ctr_pt4-ctr_pt3).length();
            double radius = length * MassThicknessRatio;
            radii.push_back(radius);
            main_branch_radius = std::max(main_branch_radius, radius);
        }
        main_branch_radius *= 8.4253;//hard-code: base_radius : largest sub-branch
        base_radius = main_branch_radius;
    }
    else
    {
        std::vector <float> in_radii_sorted = in_radii;
        sort(in_radii_sorted.begin(), in_radii_sorted.end());
        base_radius = in_radii_sorted[int(in_radii_sorted.size()/3)];
        //sum of the cross-sectional areas of sub-branch is equal to that of the base's area
        std::vector <double> lengths;
        double sum_l2 = 0;
        for(unsigned int i=0; i<sec_pts.size(); i++)
        {
            osg::Vec3 ctr_pt1 = ter;
            osg::Vec3 ctr_pt2 = sec_pts[i];
            osg::Vec3 ctr_pt3 = third_pts[i];
            osg::Vec3 ctr_pt4 = forth_pts[i];
            float length = (ctr_pt2-ctr_pt1).length() + (ctr_pt3-ctr_pt2).length() + (ctr_pt4-ctr_pt3).length();
            sum_l2 += length * length;
            lengths.push_back(length);
        }
        if(sum_l2 > 0)
        {
            double k = base_radius / pow(sum_l2, 0.5);
            for(unsigned int i=0; i<lengths.size(); i++)
                radii.push_back(lengths[i] * k * 0.3618);//hard-code: area conservation seems too large, so times 0.3618
        }
        else
        {
            printf("BDLSG2Bezier::output_palm():sum_l2 evaluates to zero\n");
            return;
        }
    }

    //e. write results, total number of branches first
    fprintf(_out, "%d\n", 1 + int(sec_pts.size()));

    //f. output main branch
    std::vector <osg::Vec3> main_branch_ctrs = segment_controls(main_branch, osg::Vec3(0.0f, 0.0f, 0.0f));
    main_branch_ctrs = seg2BezTriple(main_branch_ctrs);
    if(!main_branch_ctrs.empty())
    {
        fprintf(_out, "%d\n", int(main_branch_ctrs.size()) / 3);
        for(unsigned int i=0; i<main_branch_ctrs.size(); i+=3)
        {
            osg::Vec3 a, b, c;
            a = main_branch_ctrs[i];
            b = main_branch_ctrs[i+1];
            c = main_branch_ctrs[i+2];

            fprintf(_out, "%f %f %f %f %f %f %f %f %f\n", a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z());
        }
        if(main_branch_ctrs.size()/3 == in_radii.size())
        {
            for(unsigned int i=0; i<main_branch_ctrs.size()/3; i++)
            {
                fprintf(_out, "%f\n", in_radii[i]);
            }
        }
        else//size mis-match
        {
            for(unsigned int i=0; i<main_branch_ctrs.size()/3; i++)
            {
                fprintf(_out, "%f\n", base_radius);
            }
        }
    }

    //g. output leaf branch
    for(unsigned int i=0; i<sec_pts.size(); i++)
    {
        osg::Vec3 ctr_pt1 = ter;
        osg::Vec3 ctr_pt2 = sec_pts[i];
        osg::Vec3 ctr_pt3 = third_pts[i];
        osg::Vec3 ctr_pt4 = forth_pts[i];

        //cannot arbitrarily move anything here because it will misaligned with the leaves
        //move away the first node for blender to glue it on tail of main branch
        //osg::Vec3 translate = ctr_pt2 - ctr_pt1;
        //float translate_len = translate.length() * 0.05f;
        //translate.normalize();
        //ctr_pt1 += translate * translate_len;

        std::vector <osg::Vec3> path;
        path.push_back(ctr_pt1);
        path.push_back(ctr_pt2);
        path.push_back(ctr_pt3);
        path.push_back(ctr_pt4);

        path = seg2BezTriple(path);
        if(!path.empty())
        {
            fprintf(_out, "%d\n", int(path.size()) / 3);
            for(unsigned int j=0; j<path.size(); j+=3)
            {
                osg::Vec3 a, b, c;
                a = path[j];
                b = path[j+1];
                c = path[j+2];

                fprintf(_out, "%f %f %f %f %f %f %f %f %f\n", a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z());
            }
            for(unsigned int j=0; j<path.size()/3; j++)
            {
                fprintf(_out, "%f\n", radii[i]);
            }
        }
    }
}

void BDLSG2Bezier::output_xiao(BDLSkeletonNode *root)
{
    if(!root)
        return;

    //to prevent modifying the original tree
    //root = BDLSkeletonNode::copy_tree(root);

    //scale up root's radius because the scale in Blender is stranglely scaled down
    //root->_radius *= 9.5f;
    
    //translate and scale the tree, find height and radius of each node, get the scale
    //the returned scale is used to add an ugly node below the root and for scaling the leaf
    //double scale = BDLSkeletonNode::rectify_skeleton_tree(root, origin, height, angle, maya);

    //scale += (1.0-simplication) * scale * 3.2;

    //remove trivial nodes
    //BDLSkeletonNode::compress_skeleton_tree(root);

    //simplify the trees by collasping some edges
    //if(true)
    //{
    //    if(simplication != 1.0)
    //    {
    //        int remaining_edges = BDLSkeletonNode::collaspe_skeleton_tree(root);
    //        for(int i=0; i<remaining_edges*(1-simplication); i++)
    //            BDLSkeletonNode::collaspe_skeleton_tree(root);
    //    }
    //}

    //construct the path from longest to shortest
    //stores all the leaf skeleton nodes
    std::vector <BDLSkeletonNode *> leafs;
    //distance between leafs and root
    std::vector <double> dists;
    //sorted dists
    std::vector <double> sorted_dists;

    //BFS all skeleton nodes
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();
        
        if(front->_children.size() == 0)
        {
            leafs.push_back(front);
            dists.push_back(front->_height);
        }

        for(unsigned int i=0; i<front->_children.size(); i++)
        {
            BDLSkeletonNode *child = front->_children[i];
            Queue.push(child);
        }
    }

    //sort the dists
    sorted_dists = dists;
    sort(sorted_dists.begin(), sorted_dists.end());
    reverse(sorted_dists.begin(), sorted_dists.end());

    //print the total number of branches
    //fprintf(_out, "%d\n", int(sorted_dists.size()));

    for(unsigned int branch=0; branch<sorted_dists.size(); branch++)
    //for(unsigned int branch=0; branch<1; branch++)
    {
        double cur_dist = sorted_dists[branch];
        int cur_index = 0;

        //find the corresponding skeleton leaf
        for(unsigned int l=0; l<dists.size(); l++)
            if(dists[l] == cur_dist && !leafs[l]->_mesh_done)
            {
                cur_index = l;
                break;
            }

        BDLSkeletonNode *cur_leaf = leafs[cur_index];

        std::vector <osg::Vec3> path;
        std::vector <double> radii;

        BDLSkeletonNode *node = cur_leaf;
        int generation = 1;

        while(node)
        {
            path.push_back(osg::Vec3(node->_sx, node->_sy, node->_sz));
            radii.push_back(node->_radius);

            if(node->_mesh_done)
            {
                generation = node->_generation + 1;
                break;
            }

            node->_mesh_done = true;
            node = node->_prev;
        }

        //assign generation
        BDLSkeletonNode *for_gen = cur_leaf;
        while(for_gen != node)
        {
            for_gen->_generation = generation;
            for_gen = for_gen->_prev;
        }

        //provide hint on establishing the second control point
        osg::Vec3 hint(0.0f, 0.0f, 0.0f);//not used
        if(node && node->_prev && path.size() >= 2)
        {
            //osg::Vec3 v_node(node->_sx, node->_sy, node->_sz);
            osg::Vec3 v_node = path[path.size()-2];
            osg::Vec3 v_node_p(node->_prev->_sx, node->_prev->_sy, node->_prev->_sz);
            hint = v_node - v_node_p;
            hint.normalize();
        }

        //shorten the tip and end of the branch for all path, except the root
        //doesn't matter too much, since they will be smoothed in afterwards
        if(!radii.empty())
        {
            radii[0] *= 0.25; //tip
            if(branch != 0 && path.size() != 2)
                radii[radii.size()-1] *= 0.60; //end
            if(branch != 0 && path.size() == 2)
                radii[radii.size()-1] *= 0.45; //end
        }

        //move away the end node for blender to glue it
        //if(branch != 0 && path.size() >= 2 && radii.size() >= 2)
        //{
        //    osg::Vec3 end = path[path.size()-1];
        //    osg::Vec3 end2 = path[path.size()-2];
        //    osg::Vec3 translate = end2 - end;
		//	translate.normalize();
        //    translate = translate * radii[radii.size()-1] / 3.5f;

        //    path[path.size()-1] = end + translate;
        //}

        //real work
        if(path.size() == radii.size() && path.size() > 1)
        {
            fprintf(_out, "g Branch %d\n", branch+1);
            output_tube_xiao(path, radii, hint, generation);
            if(branch != sorted_dists.size()-1)
                fprintf(_out, "\n");
        }
    }

    //BDLSkeletonNode::delete_this(root);
}

std::vector <osg::Vec3> BDLSG2Bezier::segment_controls(std::vector <osg::Vec3> path, osg::Vec3 hint)
{
    std::vector <osg::Vec3> ret;

    //size of path and radii do not match
    /*
    if(path.size() != radii.size())
        return ret;
     */

    //only one or no point
    if(path.size() < 2)
        return ret;

    //for c2 continuatiy
    osg::Vec3 second_ctr_pt;

    //find Cubic BezierCurve control points for each segment
    for(unsigned int i=0; i<path.size()-1; i++)
    {
        double separation = (path[i]-path[i+1]).length();
        osg::Vec3 third_ctr_pt;

        //first node
        if(i == 0)
        {
            second_ctr_pt = path[i+1] - path[i];
            second_ctr_pt.normalize();
            second_ctr_pt = path[i] + second_ctr_pt * separation * 0.5;
        }
        //intermediate nodes, (also first node if len(path) != 2)
        if(i != path.size()-2)
        {
            third_ctr_pt = path[i+1] - path[i+2];
            third_ctr_pt.normalize();
            third_ctr_pt = path[i+1] + third_ctr_pt * separation * 0.5;
        }
        //last node, (equal first node if len(path) == 2)
        if(i == path.size()-2)
        {
            if(hint != osg::Vec3(0.0f, 0.0f, 0.0f))
                third_ctr_pt = hint;
            else
            {
                third_ctr_pt = path[i] - path[i+1];
                third_ctr_pt.normalize();
            }
            third_ctr_pt = path[i+1] + third_ctr_pt * separation * 0.5;
        }

        ret.push_back(path[i]);
        ret.push_back(second_ctr_pt);
        ret.push_back(third_ctr_pt);
        ret.push_back(path[i+1]);

        //set the second_ctr_pt for next iteration
        second_ctr_pt = path[i+1] + (path[i+1]-third_ctr_pt);
    }

    return ret;
}

std::vector <osg::Vec3> BDLSG2Bezier::seg2BezTriple(std::vector <osg::Vec3> seg_ctrs)
{
    std::vector <osg::Vec3> ret;
    if(seg_ctrs.empty() || seg_ctrs.size()%4 != 0)
        return ret;

    //for each segment, push the first control points
    for(unsigned int i=0; i<seg_ctrs.size(); i+=4)
    {
        osg::Vec3 first, second, third;
        second = seg_ctrs[i];
        third = seg_ctrs[i+1];
        first = second - third + second;

        ret.push_back(first);
        ret.push_back(second);
        ret.push_back(third);

        //also push the last control points
        if(i == seg_ctrs.size()-4)
        {
            second = seg_ctrs[i+3];
            first = seg_ctrs[i+2];
            third = second - first + second;

            ret.push_back(first);
            ret.push_back(second);
            ret.push_back(third);
        }
    }

    return ret;
}

void BDLSG2Bezier::output_tube(std::vector <osg::Vec3> path, std::vector <double> radii, osg::Vec3 hint)
{
    //check if each point has nine coords + one radius
    if(path.size() != radii.size())
    {
        printf("BDLSG2Bezier::output_tube(): path.size(%d), radii.size(%d) not equal error.\n", int(path.size()), int(radii.size()));
        return;
    }

    //convert
    path = segment_controls(path, hint);
    path = seg2BezTriple(path);

    if(path.size() != radii.size() * 3)
    {
        printf("BDLSG2Bezier::output_tube(): path.size(%d), radii.size(%d) error.\n", int(path.size()), int(radii.size()));
        return;
    }

    //print number of nodes
    fprintf(_out, "%d\n", int(radii.size()));

    //n lines follow, i.e. the n nodes, with its 9 coords
    for(unsigned int i=0; i<path.size(); i+=3)
    {
        osg::Vec3 a, b, c;
        a = path[i];
        b = path[i+1];
        c = path[i+2];

        fprintf(_out, "%f %f %f %f %f %f %f %f %f\n", a.x(), a.y(), a.z(), b.x(), b.y(), b.z(), c.x(), c.y(), c.z());
    }

    //another n lines follow, i.e. the n radii
    for(unsigned int i=0; i<radii.size(); i++)
        fprintf(_out, "%f\n", radii[i]);
}

std::vector <osg::Vec3> BDLSG2Bezier::interpolate_segment_controls(osg::Vec3 a, osg::Vec3 b, osg::Vec3 c, osg::Vec3 d, int time)
{
    std::vector <osg::Vec3> ret;
    for(int i=0; i<time; i++)
    {
        float t = float(i)/time;
        osg::Vec3 cur = a*(1-t)*(1-t)*(1-t) + b*3*(1-t)*(1-t)*t + c*3*(1-t)*t*t + d*t*t*t;
        ret.push_back(cur);
    }
    return ret;
}

void BDLSG2Bezier::smooth_radius(std::vector <double>& radii)
{
    /* python code to smooth
        rlen = len(radii)
        if(rlen < 2):
            return None
        start = radii[0]
        end = radii[-1]

        i = 1
        while i<rlen:
            fac = float(1+i) / rlen
            radii[i] = start*(1.0-fac) + end*fac
            i += 1
    */
    int rlen = int(radii.size());
    if(rlen < 2)
        return;
    float start = radii[0];
    float end = radii[rlen-1];
    for(int i=1; i<rlen; i++)
    {
        float fac = (1.0f + i) / rlen;
        radii[i] = start*(1.0f-fac) + end*fac;
    }
}

void BDLSG2Bezier::output_tube_xiao(std::vector <osg::Vec3> path, std::vector <double> radii, osg::Vec3 hint, int gen)
{
    //check if each point has nine coords + one radius
    if(path.size() != radii.size())
    {
        printf("BDLSG2Bezier::output_tube_xiao(): path.size(%d), radii.size(%d) not equal error.\n", int(path.size()), int(radii.size()));
        return;
    }

    smooth_radius(radii);
    std::vector <osg::Vec3> interpolated;
    std::vector <double> interpolated_radii;

    //printf("in: path(%d) radii(%d)\n", int(path.size()), int(radii.size()));

    std::vector <osg::Vec3> controls = segment_controls(path, hint);
    for(unsigned int i=0; i<controls.size(); i+=4)
    {
        osg::Vec3 a = controls[i];
        osg::Vec3 b = controls[i+1];
        osg::Vec3 c = controls[i+2];
        osg::Vec3 d = controls[i+3];

        std::vector <osg::Vec3> tmp = interpolate_segment_controls(a, b, c, d);
        for(unsigned int j=0; j<tmp.size(); j++)
            interpolated.push_back(tmp[j]);

        std::vector <double> tmp2(tmp.size(), radii[i/4]);
        tmp2[tmp.size()-1] = radii[i/4+1];

        if(i == controls.size()-4)
        {
            interpolated.push_back(d);
            tmp2.push_back(radii[i/4+1]);
        }

        smooth_radius(tmp2);
        for(unsigned int j=0; j<tmp2.size(); j++)
            interpolated_radii.push_back(tmp2[j]);
    }

    //printf("out: path(%d) radii(%d)\n", int(interpolated.size()), int(interpolated_radii.size()));

    fprintf(_out, "p Branch 1 1 %d %d\n", int(interpolated.size()), gen);
    fprintf(_out, "usemtl Branch\n");

    //for(unsigned int i=0; i<path.size(); i++)
    //    fprintf(_out, "#v %f %f %f %f\n", path[i].x(), path[i].y(), path[i].z(), radii[i]/10.0f);

    //for(unsigned int i=0; i<interpolated.size(); i++)
    for(int i=interpolated.size()-1; i>=0; i--)
        fprintf(_out, "v %f %f %f %f\n", interpolated[i].x(), interpolated[i].y(), interpolated[i].z(), interpolated_radii[i]/10.0f);

    for(unsigned int i=0; i<interpolated.size()-1; i++)
        fprintf(_out, "f %d// %d// %d//\n", _xiao_id+i, _xiao_id+i, _xiao_id+i+1);

    _xiao_id += interpolated.size();
}
