#ifndef __TOPOLOGICALNODE__H
#define __TOPOLOGICALNODE__H

#include <vector>

class TopologicalNode
{
    public:
        TopologicalNode(int);
        TopologicalNode(int, float, float, float);

        /**setter*/
        void setPosition(float, float, float);
        void setUV(int, int);
        void addChild(TopologicalNode *);
        void addParent(TopologicalNode *);
        void clearChilden();/**clear the std::vector, does not delete the pointers*/
        bool deleteChild(TopologicalNode *);
        bool deleteEdge(TopologicalNode *);
        void setDepth(int);

        /**getter*/
        int childenSize();
        float x();
        float y();
        float z();
        int id();
        int depth();
        int u();
        int v();
        TopologicalNode *child(int);
        TopologicalNode *parent();/**assume at most one parent*/
        bool isEqual(TopologicalNode *);

        /**debugging*/
        void printSelf();

    private:
        int _id;
        float _x, _y, _z;
        int _u, _v;
        int _depth;
        TopologicalNode *_parent;
        std::vector <TopologicalNode *> _childen;
};

#endif
