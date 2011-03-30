#ifndef __THRESHOLDGraph__H
#define __THRESHOLDGraph__H

#include <osg/Vec3>
#include <vector>

/** A class for constructing a graph from a set of points
  * by making a link between two points if their
  * Euclidean distance is within the given threshold.
  * Usage:
  * 1. ThresholdGraph(threshold, points)
  * 2. construct_graph()
  */
class ThresholdGraph
{
    public:
        /* setup the instance by 'threshold' and 'pts' 
         */
        ThresholdGraph(float threshold, std::vector <osg::Vec3> pts);

        /* clear this instance
         */
        void clear();

        /* construct threshold graph
         * return the associations of each point, in the same order of _points
         * e.g. the i-th point has links to ret[i] links
         */
        std::vector <std::vector <int> > construct_graph();

        /* test the class via test_points() and construct_graph() by printing logs
         * threshold: the threshold
         */
        void test_logs(float threshold = 3.0f);

    protected:
        /* generate random test points
         * within (0-10, 0-10, 0-10)
         * n: number of points
         */
        std::vector <osg::Vec3> test_points(int n = 100);

    private:
        float _threshold;//input threshold
        float _threshold2;//input threshold squared
        std::vector <osg::Vec3> _points;//input points
};

#endif
