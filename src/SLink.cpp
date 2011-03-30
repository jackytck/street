#include "SLink.h"

SLink::SLink(): _par(NULL), _child(NULL)
{
}

SLink::SLink(BDLSkeletonNode *par, BDLSkeletonNode *child): _par(par), _child(child)
{
}
