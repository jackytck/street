#include "TopologicalSkeleton.h"

TopologicalSkeleton::TopologicalSkeleton(): _root(NULL), _current(NULL), _size(0), _pushNodeFlag(false), _pushCnt(0), _minX(0.0), _maxX(0.0), _minY(0.0), _maxY(0.0), _minZ(0.0), _maxZ(0.0), _uvHeight(-1)
{
    _vertices = new osg::Vec3Array();
}

TopologicalSkeleton::~TopologicalSkeleton()
{
   clear();
}

void TopologicalSkeleton::addNode(TopologicalNode *newNode)
{
    bool reallyNew = true;
    checkNewNode(newNode, _root, reallyNew);
    if(reallyNew)
    {
        //printf("\nadd(%d):(%f,%f,%f)\n", _size, newNode->x(), newNode->y(), newNode->z());
        if(_size == 0)
            _root = newNode;
        else
        {
            _current->addChild(newNode);
            newNode->addParent(_current);
        }
        _current = newNode;
        /*
        if(_pushNodeFlag)
        {
            pushCurrentNode();
            _pushNodeFlag = false;
        }
        */
        _size++;
        _vertices->push_back(osg::Vec3(newNode->x(), newNode->y(), newNode->z()));
    }
    if(_pushNodeFlag)
    {
        for(int i=0; i<_pushCnt; i++)
            pushCurrentNode();
        _pushNodeFlag = false;
        _pushCnt = 0;
    }
    //addcnt++;
}

void TopologicalSkeleton::pushNode(TopologicalNode *node)
{
    //pushcnt++;
    _stack.push(node);
}

void TopologicalSkeleton::pushCurrentNode()
{
    _stack.push(_current);
}

void TopologicalSkeleton::setCurrentNode(TopologicalNode *node)
{
    _current = node;
}

/**does not check if this contains node,
  just check if it's not NULL and not root,
  then ask its parent to delete it, all other things will be done by parent
  if success, decrement size, reset current and update _vertices
 */
void TopologicalSkeleton::deleteNode(TopologicalNode *node)
{
    if(node && node->parent())
    {
        TopologicalNode *potential = node->parent();
        if(node->parent()->deleteChild(node))
        {
            _size--;
            _current = potential;
            //updateOSGVertices();
        }
    }
}

void TopologicalSkeleton::setUVHeight(int h)
{
    if(h > 0)
        _uvHeight = h;
}

void TopologicalSkeleton::pushNextNode()
{
    //pushcnt++;
    //if(_pushNodeFlag)
     //   _pushCnt++;
    _pushCnt++;
    _pushNodeFlag = true;
}

void TopologicalSkeleton::projectOnPlane(int plane, int width, int height)
{
    /**plane: 0 = x-plane, 1 = y-plane, 2 = z-plane*/
    switch(plane)
    {
        case 0:
            break;
        case 1:
            for(unsigned int i=0; i<_vertices->size(); i++)
            {
                if(i==0)
                {
                    _minX = _maxX = (*_vertices)[0][0];
                    _minZ = _maxZ = (*_vertices)[0][2];
                }
                else
                {
                    float x = (*_vertices)[i][0];
                    float z = (*_vertices)[i][2];
                    if(x < _minX)
                        _minX = x;
                    else if(x > _maxX)
                        _maxX = x;
                    if(z < _minZ)
                        _minZ = z;
                    else if(z > _maxZ)
                        _maxZ = z;
                }
            }
            //printf("minX=%f maxX=%f minZ=%f maxZ=%f\n", _minX, _maxX, _minZ, _maxZ);
            if(_root)
                projectNodes(_root, _maxX-_minX, _maxZ-_minZ, width, height);
            break;
        case 2:
            break;
    }
}

std::vector<TopologicalNode *> TopologicalSkeleton::intensifyEdge(TopologicalNode *start, TopologicalNode *end, int k)
{
    int x1 = start->x(), y1 = start->z();
    int x2 = end->x(), y2 = end->z();
    float d = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    float stepsF = d/float(k);
    int steps = stepsF-int(stepsF) == 0.0 ? int(stepsF)-1 : int(stepsF);
    float Dx = x2-x1, Dy = y2-y1;
    float norm = sqrt(Dx*Dx+Dy*Dy);
    Dx /= norm;
    Dy /= norm;
    std::vector<TopologicalNode *> newNodes;
    for(int i=1; i<=steps; i++)
    {
        int newX = x1+i*k*Dx;
        int newY = y1+i*k*Dy;
        TopologicalNode *node = new TopologicalNode(i, newX, 0.0, newY);
        if(_uvHeight != -1)
            node->setUV(newX, _uvHeight-newY);
        newNodes.push_back(node);
    }
    for(unsigned int i=0; i<newNodes.size(); i++)
    {
        if(i!=newNodes.size()-1)
            newNodes[i]->addChild(newNodes[i+1]);
        if(i!=0)
            newNodes[i]->addParent(newNodes[i-1]);
    }
    if(newNodes.size() == 0)
        return newNodes;
    if(start->deleteEdge(end))
    {
        start->addChild(newNodes[0]);
        newNodes[newNodes.size()-1]->addChild(end);
        newNodes[0]->addParent(start);
        end->addParent(newNodes[newNodes.size()-1]);
    }
    return newNodes;
}

void TopologicalSkeleton::clear()
{
    if(_root)
        clear(_root);
    while(!_stack.empty())
        _stack.pop();
    _root = NULL;
    _current = NULL;
    _size = 0;
}

void TopologicalSkeleton::clear(TopologicalNode *node)
{
    for(int i=0; i<node->childenSize(); i++)
        clear(node->child(i));
    delete node;
}

TopologicalNode *TopologicalSkeleton::popNode()
{
    if(_pushCnt > 0)
    {
        _pushCnt--;
        return _current;
    }
    TopologicalNode *top = _stack.top();
    _current = top;
    _stack.pop();
    return top;
}

TopologicalNode *TopologicalSkeleton::root()
{
    return _root;
}

int TopologicalSkeleton::size()
{
    return _size;
}

int TopologicalSkeleton::lineSize()
{
    int size = 0;
    if(_root)
        this->lineSize(_root, size);
    return size;
}

std::vector <TopologicalNode *> TopologicalSkeleton::serializedNodes()
{
    std::vector <TopologicalNode *> ret;
    if(!_root)
        return ret;
    std::stack <TopologicalNode *> Stack;
    Stack.push(_root);
    while(!Stack.empty())
    {
        TopologicalNode *node = Stack.top();
        Stack.pop();
        ret.push_back(node);
        for(int i=0; i<node->childenSize(); i++)
            Stack.push(node->child(i));
    }
    return ret;
}

std::vector <std::pair<TopologicalNode *, TopologicalNode *> > TopologicalSkeleton::serializedEdges()
{
    std::vector <std::pair<TopologicalNode *, TopologicalNode *> > ret;
    if(!_root)
        return ret;
    std::stack <TopologicalNode *> Stack;
    Stack.push(_root);
    while(!Stack.empty())
    {
        TopologicalNode *node = Stack.top();
        Stack.pop();
        for(int i=0; i<node->childenSize(); i++)
        {
            std::pair<TopologicalNode *, TopologicalNode *> edge(node, node->child(i));
            ret.push_back(edge);
            Stack.push(node->child(i));
        }
    }
    return ret;
}

osg::ref_ptr<osg::Vec3Array> TopologicalSkeleton::vertices()
{
    updateOSGVertices();
    return _vertices;
}

osg::ref_ptr<osg::Vec3Array> TopologicalSkeleton::lineVertices()
{
    osg::ref_ptr<osg::Vec3Array> lv = NULL;
    if(_root)
    {
        lv = new osg::Vec3Array(this->lineSize()*2);
        int cnt = 0;
        /**run DFS non-recursively*/
        std::stack <TopologicalNode *> Stack;
        Stack.push(_root);
        while(!Stack.empty())
        {
            TopologicalNode *top = Stack.top();
            Stack.pop();
            for(int i=0; i<top->childenSize(); i++)
            {
                TopologicalNode *child = top->child(i);
                (*lv)[cnt].set(top->x(), top->y(), top->z());
                (*lv)[cnt+1].set(child->x(), child->y(), child->z());
                cnt += 2;
                Stack.push(child);
            }
        }
    }
    return lv;
}

void TopologicalSkeleton::printSelf()
{
    if(_root)
        printSelf(_root);
}

void TopologicalSkeleton::printSelf(TopologicalNode *node)
{
    node->printSelf();
    for(int i=0; i<node->childenSize(); i++)
        printSelf(node->child(i));
}

void TopologicalSkeleton::lineSize(TopologicalNode *node, int& cnt)
{
    if(node && node->childenSize() > 0)
    {
        cnt += node->childenSize();
        for(int i=0; i<node->childenSize(); i++)
            lineSize(node->child(i), cnt);
    }
}

void TopologicalSkeleton::checkNewNode(TopologicalNode *newNode, TopologicalNode *node, bool& reallyNew)
{
    if(!reallyNew || node == NULL)
        return;
    if(node->isEqual(newNode))
        reallyNew = false;
    else
        for(int i=0; i<node->childenSize(); i++)
            checkNewNode(newNode, node->child(i), reallyNew);
}

void TopologicalSkeleton::projectNodes(TopologicalNode *node, int oldW, int oldH, int newW, int newH)
{
    float x = node->x();
    float z = node->z();
    node->setUV((x-_minX)*newW/oldW, newH-(z-_minZ)*newH/oldH);
    for(int i=0; i<node->childenSize(); i++)
        projectNodes(node->child(i), oldW, oldH, newW, newH);
}

void TopologicalSkeleton::updateOSGVertices()
{
    if(_root)
    {
        _vertices = new osg::Vec3Array();
        std::stack <TopologicalNode *> Stack;
        Stack.push(_root);
        while(!Stack.empty())
        {
            TopologicalNode *node = Stack.top();
            Stack.pop();
            _vertices->push_back(osg::Vec3(node->x(), node->y(), node->z()));
            for(int i=0; i<node->childenSize(); i++)
                Stack.push(node->child(i));
        }
    }
}
