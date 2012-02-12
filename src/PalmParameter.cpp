#include "PalmParameter.h"
#include "Transformer.h"

PalmParameter::PalmParameter(float noise): _noise(noise)
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
    srand(time(NULL));
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

void PalmParameter::setBezierQuadratic(osg::Vec3 ctr1, osg::Vec3 ctr2, osg::Vec3 ctr3)
{
    int steps = (ctr3 - ctr1).length() * 3;
    _on_curve = Transformer::interpolate_bezier_2(ctr1, ctr2, ctr3, steps);

    _target_dist = 0.0f;
    for(unsigned int i=0; i<_on_curve.size()-1; i++)
    {
        osg::Vec3 a = _on_curve[i];
        osg::Vec3 b = _on_curve[i+1];
        _target_dist += (a - b).length();
    }

    if(_keys_dist != 0.0f && _target_dist != 0.0f)
        _scale = _target_dist / _keys_dist * 0.65f;
}

void PalmParameter::setBezierCubic(osg::Vec3 ctr1, osg::Vec3 ctr2, osg::Vec3 ctr3, osg::Vec3 ctr4)
{
    int steps = ((ctr2-ctr1).length() + (ctr3-ctr2).length() + (ctr4-ctr3).length()) * 4;
    float slop = -1.0f;
    if(ctr4.x()!=ctr1.x())
        slop = fabs((ctr4.z()-ctr1.z()) / (ctr4.x()-ctr1.x()));
    if(slop > 2)
        steps -= 5;
    else if(slop < 1)//increase interpolation if it is flatter
        steps += 15;

    _on_curve = Transformer::interpolate_bezier_3(ctr1, ctr2, ctr3, ctr4, steps);

    _target_dist = 0.0f;
    for(unsigned int i=0; i<_on_curve.size()-1; i++)
    {
        osg::Vec3 a = _on_curve[i];
        osg::Vec3 b = _on_curve[i+1];
        _target_dist += (a - b).length();
    }

    if(_keys_dist != 0.0f && _target_dist != 0.0f)
        _scale = _target_dist / _keys_dist * 0.65f;
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

    //c. add noise to frames
    float max_displace = (low_frame - _keys[low]).length() * _scale * 0.25f * _noise;
    float lat = rand() % 180 - 90.0f;
    float lon = rand() % 360 - 180.0f;
    osg::Vec3 displace = Transformer::spherical_to_cartesian(max_displace, lat, lon);
    displace.x() = 0.0f;//do not displace along the tangent
    interpolated += displace;

    //d. find out the basis at k-th key
    osg::Vec3 tangent = getTangent(k);
    osg::Vec3 up(0, 0, 1);
    osg::Vec3 side = up ^ tangent;
    osg::Vec3 cur = _on_curve[k];

    //e. scale to get final result
    if(mirror)
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() - side * interpolated.z()) * _scale;
    else
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() + side * interpolated.z()) * _scale;

    return ret;
}

std::vector <osg::Vec3> PalmParameter::getFrames()
{
    std::vector <osg::Vec3> ret;
    if(_on_curve.empty())
        return ret;

    float len = 0.0f;
    osg::Vec3 last = _on_curve[0];
    for(unsigned int i=1; i<_on_curve.size(); i++)
    {
        len += (_on_curve[i] - last).length();
        last = _on_curve[i];
    }
    float correction = len * 0.045f;//empirical

    unsigned int on_curve_size = _on_curve.size();
    for(unsigned int i=0; i<on_curve_size; i++)
    {
        osg::Vec3 tail1 = getFrameAt(i);
        osg::Vec3 tail2 = getFrameAt(i, true);
        osg::Vec3 cur = _on_curve[i];

        //to fix the misalignment problem caused by the connected mesh
        //but only apply to the second half of the branch
        osg::Vec3 translate = cur - (tail1 + tail2)*0.5f;
        translate.normalize();
        translate = translate * (i / float(on_curve_size)) * correction;

        ret.push_back(cur + translate);
        ret.push_back(tail1);
        ret.push_back(cur + translate);
        ret.push_back(tail2);
    }

    return ret;
}

std::vector <osg::Vec3> PalmParameter::getQuads(float aspect, bool debug, int debug_offset)
{
    std::vector <osg::Vec3> ret;
    if(aspect <= 0.0f)
        return ret;

    std::vector <osg::Vec3> frames = getFrames();

    //a. for each <head,tail> line, find its corresponding tangent on curve,
    //these 3 vectors are used to get a 2D basis
    for(unsigned int i=0; i<frames.size(); i+=2)
    {
        osg::Vec3 head = frames[i];
        osg::Vec3 tail = frames[i+1];
        osg::Vec3 tangent = getTangent(i/4);

        //check basis, ignore if not valid
        osg::Vec3 ht = tail - head;
        float h = ht.length();
        if(abs(ht * tangent) == h || h == 0.0f)
        {
            printf("PalmParameter::getQuads:parallel error");
            continue;
        }
        float w = h / aspect;

        //b. make the tangent orthogonal to ht to reduce texture distortion
        ht.normalize();
        tangent = (ht ^ tangent) ^ ht;
        tangent.normalize();
        ht = ht * h;

        //b--T--f
        //|     |
        //|  |  |
        //|     |
        //|  |  |   c. construct quad <a,b,e,f>
        //|     |
        //|  |  |
        //|     |
        //a--H--e
        osg::Vec3 a, b, e, f;
        a = head + tangent * 0.5f * w;
        b = a + ht;
        e = head - tangent * 0.5f * w;
        f = e + ht;

        ret.push_back(a);
        ret.push_back(b);
        ret.push_back(e);
        ret.push_back(f);

        //d. print out each quad as obj face
        if(debug)
        {
            printf("v %f %f %f\n", a.x(), a.y(), a.z());
            printf("v %f %f %f\n", b.x(), b.y(), b.z());
            printf("v %f %f %f\n", f.x(), f.y(), f.z());
            printf("v %f %f %f\n", e.x(), e.y(), e.z());
            int cnt = debug_offset + 2 * i;
            printf("f %d %d %d %d\n", cnt+1, cnt+2, cnt+3, cnt+4);
        }
    }

    return ret;
}

std::vector <osg::Vec2> PalmParameter::getTexCoords()
{
    std::vector <osg::Vec2> ret;

    ret.resize(onCurveSize() * 8);
    for(unsigned int i=0; i<ret.size(); i+=4)
    {
        ret[i] = osg::Vec2(0, 0);
        ret[i+1] = osg::Vec2(0, 1);
        ret[i+2] = osg::Vec2(1, 0);
        ret[i+3] = osg::Vec2(1, 1);
    }

    return ret;
}
