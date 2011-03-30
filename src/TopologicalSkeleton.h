#ifndef __TOPOLOGICALSKELETON__H
#define __TOPOLOGICALSKELETON__H

#include <osg/Geometry>
#include <utility>
#include <stack>
#include "TopologicalNode.h"

class TopologicalSkeleton
{
    public:
        TopologicalSkeleton();
        ~TopologicalSkeleton();

        /**setter*/
        void addNode(TopologicalNode *);
        void pushNode(TopologicalNode *);
        void clear();/**clear and delete*/
        void setCurrentNode(TopologicalNode *);
        void deleteNode(TopologicalNode *);
        void setUVHeight(int h);/**for converting between world.z and uv.v*/

        /**getter*/
        TopologicalNode *root();
        int size();
        int lineSize();
        std::vector <TopologicalNode *> serializedNodes();
        std::vector <std::pair<TopologicalNode *, TopologicalNode *> > serializedEdges();

        /**construction operations*/
        TopologicalNode *popNode();
        void pushCurrentNode();
        void pushNextNode();

        /**main operations*/
        //TopologicalSkeleton *projectOnPlane();
        void projectOnPlane(int plane, int width, int height);
        std::vector<TopologicalNode *> intensifyEdge(TopologicalNode *start, TopologicalNode *end, int k);

        /**pass to model objects for visualization purposes*/
        osg::ref_ptr<osg::Vec3Array> vertices();
        osg::ref_ptr<osg::Vec3Array> lineVertices();

        /**debugging*/
        void printSelf();
        //int pushcnt, popcnt, addcnt, hihijor;

    private:
        TopologicalNode *_root, *_current;
        int _size;
        bool _pushNodeFlag;
        int _pushCnt;
        float _minX, _maxX, _minY, _maxY, _minZ, _maxZ;
        int _uvHeight;
        osg::ref_ptr<osg::Vec3Array> _vertices;/**pass it to Point Model to get it visualized*/
        std::stack <TopologicalNode *> _stack;
        void clear(TopologicalNode *);/**for clear() recursively*/
        void printSelf(TopologicalNode *);/**for printSelf() recursively*/
        void lineSize(TopologicalNode *, int&);/**return number of lines*/
        void checkNewNode(TopologicalNode *newNode, TopologicalNode *node, bool& reallyNew);/**check if it is a new node, return true if it is*/
        void projectNodes(TopologicalNode *, int oldW, int oldH, int newW, int newH);
        void updateOSGVertices();/**need update because nodes may be deleted*/
};

#endif
