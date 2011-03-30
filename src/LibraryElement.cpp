#include "LibraryElement.h"
#include <map>
#include <queue>

LibraryElement::LibraryElement(BDLSkeletonNode *root, SLink support): _root(root), _support_branch(support), _is_added(false)
{
}

LibraryElement::LibraryElement(const LibraryElement& element)
{
    //printf("LibraryElement::copy constructor starts(%p)\n", &element);
    _root = BDLSkeletonNode::copy_tree(element._root);

    if(!_root || !element._root)
        return;

    std::map <BDLSkeletonNode *, int> dict;
    int id = 0;
    
    //bfs on source
    std::queue <BDLSkeletonNode *> Queue;
    Queue.push(element._root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        dict[front] = id;

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);

        id++;
    }

    int par_id = dict[element._support_branch._par];
    int child_id = dict[element._support_branch._child];
    id = 0;
    //printf("in.par(%p):par_id(%d) in.child(%p):child_id(%d)\n", element._support_branch._par, par_id, element._support_branch._child, child_id);

    //bfs on target
    Queue.push(_root);

    while(!Queue.empty())
    {
        BDLSkeletonNode *front = Queue.front();
        Queue.pop();

        if(id == par_id)
            _support_branch._par = front;
        else if(id == child_id)
            _support_branch._child = front;

        for(unsigned int i=0; i<front->_children.size(); i++)
            Queue.push(front->_children[i]);

        id++;
    }
    //printf("LibraryElement::copy constructor ends(%p) root(%p)\n", this, _root);
    if(!isValid())
    {
        //printf("in: %p\tout: %p\n", element._support_branch._child, _support_branch._child);
        //printf("LibraryElement::copy constructor: in.root(%p) out.root(%p) in.child(%p) out.child(%p)\n", element._root, _root, element._support_branch._child, _support_branch._child);
    }
}

LibraryElement::~LibraryElement()
{
    if(!_is_added)
        BDLSkeletonNode::delete_this(_root);
}

bool LibraryElement::isValid()
{
    bool ret = false;
    if(_root && _support_branch._par && _support_branch._child)
        ret = true;
    return ret;
}

float LibraryElement::support_branch_length()
{
    if(!isValid())
        return -1.0f;
    return BDLSkeletonNode::dist(_support_branch._par, _support_branch._child);
}
