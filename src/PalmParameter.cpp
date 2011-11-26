#include "PalmParameter.h"
#include "Transformer.h"

PalmParameter::PalmParameter()
{
    //generated from palm_parser.py
    _keys = std::vector <osg::Vec3> (12, osg::Vec3(0, 0, 0));
    _keys[0] = osg::Vec3(0.883929, 0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948742, 0.000000, 1.697612);
    _keys[2] = osg::Vec3(2.968704, 0.000001, 2.447819);
    _keys[3] = osg::Vec3(4.148565, 0.000002, 3.235170);
    _keys[4] = osg::Vec3(5.651832, 0.000001, 3.992211);
    _keys[5] = osg::Vec3(6.700876, -0.000000, 4.478881);
    _keys[6] = osg::Vec3(7.801173, -0.000002, 4.826838);
    _keys[7] = osg::Vec3(9.117766, -0.000003, 4.958496);
    _keys[8] = osg::Vec3(10.208656, -0.000004, 4.986708);
    _keys[9] = osg::Vec3(11.261930, -0.000006, 4.883260);
    _keys[10] = osg::Vec3(12.390439, -0.000009, 4.610534);
    _keys[11] = osg::Vec3(13.443712, -0.000008, 4.234362);
    
    _key_frames = std::vector <osg::Vec3> (12, osg::Vec3(0, 0, 0));
    _key_frames[0] = osg::Vec3(-0.129694, 4.461599, 5.003311);
    _key_frames[1] = osg::Vec3(1.021078, 4.091555, 5.725845);
    _key_frames[2] = osg::Vec3(2.512845, 3.604271, 5.699498);
    _key_frames[3] = osg::Vec3(3.012824, 3.649770, 5.165837);
    _key_frames[4] = osg::Vec3(3.309356, 3.356046, 4.386393);
    _key_frames[5] = osg::Vec3(4.695017, 2.562683, 3.945745);
    _key_frames[6] = osg::Vec3(4.477271, 2.401681, 2.889978);
    _key_frames[7] = osg::Vec3(4.187011, 2.041749, 1.918342);
    _key_frames[8] = osg::Vec3(3.724213, 1.098363, 1.421973);
    _key_frames[9] = osg::Vec3(3.221320, 0.810498, 0.895320);
    _key_frames[10] = osg::Vec3(2.265140, 0.596390, 0.539247);
    _key_frames[11] = osg::Vec3(1.025884, 0.008749, -0.000005);

    setupKeyFrames();
}

PalmParameter::~PalmParameter()
{
}

void PalmParameter::setupKeyFrames()
{
    //a. calculate culmulative distance from start to end nodes
    _keys_dist = 0.0f;
    for(unsigned int i=0; i<_keys.size()-1; i++)
    {
        osg::Vec3 a = _keys[i];
        osg::Vec3 b = _keys[i+1];
        _keys_dist += (a - b).length();
    }
}

osg::Vec3 PalmParameter::getTangent(int k)
{
    osg::Vec3 ret;
    int n = onCurveSize();
    if(k >= n || n < 2)
    {
        printf("PalmParameter::getTangent(%d) error\n", k);
        return ret;
    }

    if(k == 0)
        ret = _on_curve[1] - _on_curve[0];
    else if(k == n-1)
        ret = _on_curve[n-1] - _on_curve[n-2];
    else
        ret = (_on_curve[k] - _on_curve[k-1]) + (_on_curve[k+1] - _on_curve[k]);

    ret.normalize();
    return ret;
}

void PalmParameter::setBezier(osg::Vec3 ctr1, osg::Vec3 ctr2, osg::Vec3 ctr3, int steps)
{
    _on_curve = Transformer::interpolate_bezier_2(ctr1, ctr2, ctr3, steps);

    _target_dist = 0.0f;
    for(unsigned int i=0; i<_on_curve.size()-1; i++)
    {
        osg::Vec3 a = _on_curve[i];
        osg::Vec3 b = _on_curve[i+1];
        _target_dist += (a - b).length();
    }

    if(_keys_dist != 0.0f && _target_dist != 0.0f)
        _scale = _target_dist / _keys_dist;
}

unsigned int PalmParameter::onCurveSize()
{
    return _on_curve.size();
}

osg::Vec3 PalmParameter::getOnCurve(int k)
{
    osg::Vec3 ret;
    if(k < 0 || k >= int(_on_curve.size()))
        return ret;

    return _on_curve[k];
}

osg::Vec3 PalmParameter::getFrameAt(int k, bool mirror)
{
    osg::Vec3 ret;
    if(_on_curve.empty() || k < 0 || k >= int(_on_curve.size()) || _scale == 0.0f)
    {
        printf("PalmParameter::getFrameAt(%d) error\n", k);
        return ret;
    }

    //a. find the two closest key frames
    float t = float(k) / _on_curve.size() * (_keys.size() - 1);
    int low = floor(t);
    int high = low + 1;
    osg::Vec3 low_frame = _key_frames[low];
    osg::Vec3 high_frame = _key_frames[high];

    //b. interpolate the two coordinates
    osg::Vec3 interpolated = low_frame * (high - t) + high_frame * (t - low);

    //c. find out the basis at k-th key
    osg::Vec3 tangent = getTangent(k);
    osg::Vec3 up(0, 0, 1);
    osg::Vec3 side = up ^ tangent;
    osg::Vec3 cur = _on_curve[k];

    //d. scale to get final result
    if(mirror)
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() - side * interpolated.z()) * _scale;
    else
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() + side * interpolated.z()) * _scale;

    return ret;
}

std::vector <osg::Vec3> PalmParameter::getFrames()
{
    std::vector <osg::Vec3> ret;

    for(unsigned int i=0; i<_on_curve.size(); i++)
    {
        osg::Vec3 tail1 = getFrameAt(i);
        osg::Vec3 tail2 = getFrameAt(i, true);
        osg::Vec3 cur = _on_curve[i];

        ret.push_back(cur);
        ret.push_back(tail1);
        ret.push_back(cur);
        ret.push_back(tail2);
    }

    return ret;
}
