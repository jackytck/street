#include "Library.h"
#include "BDLSkeletonElementLoader.h"
#include <fstream>

Library::Library()
{
}

Library::~Library()
{
    //element will be deleted by its own destructor
    //delete all default skeletons
    for(unsigned int i=0; i<_library_skeleton.size(); i++)
        BDLSkeletonNode::delete_this(_library_skeleton[i]);
}

void Library::load_default_elements(std::string path)
{
	//printf("loading '%s' ...\n", path.c_str());
    std::ifstream fs(path.c_str());
    std::string s;
    BDLSkeletonElementLoader loader;

    while(getline(fs, s))
    {
        if(!s.empty() && s[s.length()-1] == '\n')
            s.erase(s.length()-1);

        loader.load_file(QString(s.c_str())); //malloc_error if path is wrong
        BDLSkeletonNode *root = loader.construct_bdl_skeleton_tree();//memory allocations
        SLink support_branch = loader.support_branch();
        //printf("push: %p\n", support_branch._child);

        LibraryElement element(root, support_branch);
        //printf("push element(%p) root(%p)\n", &element, element._root);
        _library_element.push_back(element);
    }

	fs.close();
}

void Library::load_default_skeleton(std::string path)
{
	//printf("loading '%s' ...\n", path.c_str());
    std::ifstream fs(path.c_str());
    std::string s;
    BDLSkeletonElementLoader loader;

    while(getline(fs, s))
    {
        if(!s.empty() && s[s.length()-1] == '\n')
            s.erase(s.length()-1);

        loader.load_file(QString(s.c_str()));
        BDLSkeletonNode *root = loader.construct_bdl_skeleton_tree();//memory allocations
        _library_skeleton.push_back(root);
    }

	fs.close();
}
