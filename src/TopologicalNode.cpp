#include "TopologicalNode.h"
#include <stdio.h>

TopologicalNode::TopologicalNode(int id): _id(id), _depth(0), _parent(NULL)
{
    _childen.clear();
}

TopologicalNode::TopologicalNode(int id, float x, float y, float z): _id(id), _x(x), _y(y), _z(z), _depth(0), _parent(NULL)
{
    _childen.clear();
}

void TopologicalNode::setPosition(float x, float y, float z)
{
    _x = x;
    _y = y;
    _z = z;
}

void TopologicalNode::setUV(int u, int v)
{
    _u = u;
    _v = v;
}

void TopologicalNode::addChild(TopologicalNode *child)
{
    _childen.push_back(child);
}

void TopologicalNode::addParent(TopologicalNode *parent)
{
    _parent = parent;
}

bool TopologicalNode::isEqual(TopologicalNode *node)
{
    bool equal = false;
    if(node && node->x() == x() && node->y() == y() && node->z() == z())
        equal = true;
    /*
    if(!equal)
        printf("isEqual:(%f,%f,%f),(%f,%f,%f)\n", node->x(), node->y(), node->z(), x(), y(), z());
    */
    return equal;
}

void TopologicalNode::clearChilden()
{
    _childen.clear();
}

/**act as parent, delete the node if exist*/
bool TopologicalNode::deleteChild(TopologicalNode *node)
{
    if(!node)
        return false;
    int victim = -1;
    for(int i=0; i<childenSize(); i++)
        if(child(i) == node)
            victim = i;
    if(victim == -1)
        return false;//no such child
    std::vector <TopologicalNode *> childen;
    for(int i=0; i<childenSize(); i++)
        if(i != victim)
            childen.push_back(child(i));
    for(int i=0; i<node->childenSize(); i++)
    {
        childen.push_back(node->child(i));
        node->child(i)->addParent(this);
    }
    _childen = childen;
    delete node;
    return true;
}

/**act as parent, delete the edge from this to node if exist
 this is used by intensifyEdge for deleting one edge and add an
 intensified edge to it*/
bool TopologicalNode::deleteEdge(TopologicalNode *node)
{
    if(!node)
        return false;
    int victim = -1;
    for(int i=0; i<childenSize(); i++)
        if(child(i) == node)
            victim = i;
    if(victim == -1)
        return false;
    std::vector <TopologicalNode *> childen;
    for(int i=0; i<childenSize(); i++)
        if(i != victim)
            childen.push_back(child(i));
    _childen = childen;
    return true;
}

void TopologicalNode::setDepth(int d)
{
    _depth = d;
}

int TopologicalNode::childenSize()
{
    return _childen.size();
}

float TopologicalNode::x()
{
    return _x;
}

float TopologicalNode::y()
{
    return _y;
}

float TopologicalNode::z()
{
    return _z;
}

int TopologicalNode::id()
{
    return _id;
}

int TopologicalNode::depth()
{
    return _depth;
}

int TopologicalNode::u()
{
    return _u;
}

int TopologicalNode::v()
{
    return _v;
}

TopologicalNode *TopologicalNode::child(int index)
{
    return _childen[index];
}

TopologicalNode *TopologicalNode::parent()
{
    return _parent;
}

void TopologicalNode::printSelf()
{
    if(_parent)
        printf("%d--->", _parent->id());
    else
        printf("NULL--->");
    printf("[%d]--->{", id());
    for(int i=0; i<childenSize(); i++)
    {
        TopologicalNode *myChild = child(i);
        printf("%d", myChild->id());
        if(i!=childenSize()-1)
            printf(",");
    }
    printf("}\n");
}
