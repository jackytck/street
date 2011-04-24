#include "ThresholdGraph.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

ThresholdGraph::ThresholdGraph(float threshold, std::vector <osg::Vec3> pts): _threshold(threshold), _points(pts)
{
    _threshold2 = _threshold * _threshold;
}

void ThresholdGraph::clear()
{
    _threshold = -1.0f;
    _points.clear();
}

//overview: O(n^2) to loop through each pair and construct the association
std::vector <std::vector <int> > ThresholdGraph::construct_graph()
{
    std::vector <std::vector <int> > ret;

    //1. pre-check and setup
    if(_threshold <= 0.0f || _points.empty())
        return ret;
    ret = std::vector <std::vector <int> > (_points.size(), std::vector <int>());

    //2. loop through each pair, add relationship if they are within the threshold
    for(unsigned int i=0; i<_points.size(); i++)
    {
        for(unsigned int j=i+1; j<_points.size(); j++)
        {
            float dist = (_points[i]-_points[j]).length2();

            if(dist <= _threshold2)
            {
                ret[i].push_back(j);
                ret[j].push_back(i);
            }
        }
    }

    return ret;
}

//for testing purposes only
std::vector <osg::Vec3> ThresholdGraph::test_points(int n)
{
    std::vector <osg::Vec3> ret;
    
    if(n<=0)
        return ret;

    srand(time(NULL));
    for(int i=0; i<n; i++)
    {
        float x = rand()%10;
        float y = rand()%10;
        float z = rand()%10;

        ret.push_back(osg::Vec3(x, y, z));
    }

    return ret;
}

void ThresholdGraph::test_logs(float threshold)
{
    //setup instance
    _points = test_points();
    _threshold = threshold;
    _threshold2 = threshold * threshold;

    //work
    std::vector <std::vector <int> > links = construct_graph();

    //print logs
    for(unsigned int i=0; i<links.size(); i++)
    {
        //points
        printf("%d: (%f %f %f)\n", i, _points[i].x(), _points[i].y(), _points[i].z());

        //links
        for(unsigned int j=0; j<links[i].size(); j++)
        {
            int to = links[i][j];
            osg::Vec3 pt = _points[to];
            printf("%d (%f %f %f) %f\n", to, pt.x(), pt.y(), pt.z(), (_points[i]-pt).length());
        }
    }
}
