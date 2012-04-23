#include "SingleImagePalm.h"
#include "ISPLoader.h"
//#include "Transformer.h"

SingleImagePalm::SingleImagePalm(std::string isp0): _data_valid(false)
{
    ISPLoader loader;
    loader.load(isp0);

    _img = QImage(loader._img_path.c_str());
    if(_img.isNull())
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%s) error\n", loader._img_path.c_str());
        return;
    }
    _seg = QImage(loader._seg_path.c_str());
    if(_seg.isNull())
    {
        printf("SingleImagePalm::SingleImagePalm():_seg(%s) error\n", loader._seg_path.c_str());
        return;
    }
    
    //check if both img and seg are of the same size
    int iw = _img.width();
    int ih = _img.height();
    int sw = _seg.width();
    int sh = _seg.height();

    if(iw != sw || ih != sh)
    {
        printf("SingleImagePalm::SingleImagePalm():_img(%d x %d):_seg(%d x %d) does not match error\n", iw, ih, sw, sh);
        return;
    }

    //at this point isp is valid
    _data_valid = true;

    //_img.save(QString("/tmp/test_tiff.png"), "PNG", 80);test if the qt plugin works
}

SingleImagePalm::~SingleImagePalm()
{
}

void SingleImagePalm::grow()
{
    if(_data_valid)
    {
        findRoot();
    }
}

void SingleImagePalm::findRoot()
{
    printf("SingleImagePalm::findRoot()\n");
}
