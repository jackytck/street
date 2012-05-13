#include "PalmParameter.h"
#include "Transformer.h"
#include <osg/Quat>

PalmParameter::PalmParameter(float noise): _noise(noise), _gravity(false)
{
    //generated from palm_parser.py
    _keys = std::vector <osg::Vec3> (14, osg::Vec3(0, 0, 0));
    _key_frames = std::vector <osg::Vec3> (14, osg::Vec3(0, 0, 0));

    // high V-shape
    /*
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
    */

    // flatter I-shape
    /*
    _keys[0] = osg::Vec3(0.883929, -0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948742, 0.000000, 1.697612);
    _keys[2] = osg::Vec3(2.968705, 0.000001, 2.447817);
    _keys[3] = osg::Vec3(4.148566, 0.000001, 3.235169);
    _keys[4] = osg::Vec3(5.651833, 0.000001, 3.992209);
    _keys[5] = osg::Vec3(6.700877, 0.000001, 4.478878);
    _keys[6] = osg::Vec3(7.801173, 0.000001, 4.826836);
    _keys[7] = osg::Vec3(9.117765, 0.000001, 4.958494);
    _keys[8] = osg::Vec3(10.208655, 0.000001, 4.986706);
    _keys[9] = osg::Vec3(11.261930, 0.000001, 4.883259);
    _keys[10] = osg::Vec3(12.390438, 0.000001, 4.610535);
    _keys[11] = osg::Vec3(13.443711, 0.000004, 4.234364);

    _key_frames[0] = osg::Vec3(1.060920, 0.344741, 5.299547);
    _key_frames[1] = osg::Vec3(1.530312, 0.343904, 5.636658);
    _key_frames[2] = osg::Vec3(2.137963, 0.496399, 5.634435);
    _key_frames[3] = osg::Vec3(2.952652, 0.386381, 5.558765);
    _key_frames[4] = osg::Vec3(3.107156, 0.289049, 4.963460);
    _key_frames[5] = osg::Vec3(3.670777, 0.285771, 4.356737);
    _key_frames[6] = osg::Vec3(3.826584, 0.381254, 3.424659);
    _key_frames[7] = osg::Vec3(3.592176, 0.579202, 2.609131);
    _key_frames[8] = osg::Vec3(3.469409, 0.640871, 1.821903);
    _key_frames[9] = osg::Vec3(3.044957, 0.766794, 1.203224);
    _key_frames[10] = osg::Vec3(2.230387, 0.583862, 0.612793);
    _key_frames[11] = osg::Vec3(1.025882, 0.008749, -0.000002);
    */

    // flat and parallel shape
    /*
    _keys[0] = osg::Vec3(0.883929, -0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948741, -0.000000, 1.697611);
    _keys[2] = osg::Vec3(2.968705, 0.000001, 2.447816);
    _keys[3] = osg::Vec3(4.148567, 0.000003, 3.235168);
    _keys[4] = osg::Vec3(5.651835, 0.000005, 3.992207);
    _keys[5] = osg::Vec3(6.700879, 0.000007, 4.478876);
    _keys[6] = osg::Vec3(7.801176, 0.000008, 4.826833);
    _keys[7] = osg::Vec3(9.117769, 0.000010, 4.958491);
    _keys[8] = osg::Vec3(10.208659, 0.000012, 4.986703);
    _keys[9] = osg::Vec3(11.261933, 0.000015, 4.883256);
    _keys[10] = osg::Vec3(12.390442, 0.000019, 4.610532);
    _keys[11] = osg::Vec3(13.443715, 0.000011, 4.234361);
    _keys[12] = osg::Vec3(13.443715, 0.000011, 4.234361);
    _keys[13] = osg::Vec3(14.412353, 0.000024, 3.905211);
    _keys[14] = osg::Vec3(14.412353, 0.000024, 3.905211);
    _keys[15] = osg::Vec3(14.412353, 0.000024, 3.905211);

    _key_frames[0] = osg::Vec3(1.619084, 0.041302, 5.273793);
    _key_frames[1] = osg::Vec3(1.834677, -0.113074, 5.180647);
    _key_frames[2] = osg::Vec3(2.180521, -0.320229, 4.950301);
    _key_frames[3] = osg::Vec3(2.151216, -0.191323, 4.696993);
    _key_frames[4] = osg::Vec3(1.977983, -0.072950, 4.510452);
    _key_frames[5] = osg::Vec3(2.379810, -0.185910, 4.307692);
    _key_frames[6] = osg::Vec3(2.451463, 0.124080, 4.066030);
    _key_frames[7] = osg::Vec3(2.552517, 0.386665, 4.205358);
    _key_frames[8] = osg::Vec3(2.602466, 0.423111, 4.035244);
    _key_frames[9] = osg::Vec3(2.753524, 0.749205, 4.125128);
    _key_frames[10] = osg::Vec3(2.707401, 1.028263, 3.904653);
    _key_frames[11] = osg::Vec3(2.286762, 1.124327, 3.068306);
    _key_frames[12] = osg::Vec3(2.545947, 0.695267, 2.225871);
    _key_frames[13] = osg::Vec3(1.851467, 0.496996, 1.613931);
    _key_frames[14] = osg::Vec3(1.903935, 0.282514, 0.705740);
    _key_frames[15] = osg::Vec3(2.164993, 0.179209, 0.051300);
    */

    // flat + parallel + round head shape
    /*
    _keys[0] = osg::Vec3(0.883929, -0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948739, -0.000000, 1.697609);
    _keys[2] = osg::Vec3(2.968705, 0.000002, 2.447814);
    _keys[3] = osg::Vec3(4.148568, 0.000005, 3.235165);
    _keys[4] = osg::Vec3(5.651839, 0.000009, 3.992203);
    _keys[5] = osg::Vec3(6.700884, 0.000012, 4.478871);
    _keys[6] = osg::Vec3(7.801183, 0.000015, 4.826828);
    _keys[7] = osg::Vec3(9.117777, 0.000017, 4.958484);
    _keys[8] = osg::Vec3(9.117777, 0.000017, 4.958484);
    _keys[9] = osg::Vec3(10.208667, 0.000019, 4.986695);
    _keys[10] = osg::Vec3(11.261941, 0.000023, 4.883246);
    _keys[11] = osg::Vec3(12.390450, 0.000029, 4.610520);
    _keys[12] = osg::Vec3(13.443723, 0.000023, 4.234348);
    _keys[13] = osg::Vec3(14.412360, 0.000038, 3.905196);

    _key_frames[0] = osg::Vec3(1.966012, -0.829573, 5.268725);
    _key_frames[1] = osg::Vec3(2.278292, -0.821551, 5.275563);
    _key_frames[2] = osg::Vec3(2.451481, -0.846261, 5.111340);
    _key_frames[3] = osg::Vec3(2.559528, -0.652321, 4.847157);
    _key_frames[4] = osg::Vec3(2.420502, -0.665277, 4.654540);
    _key_frames[5] = osg::Vec3(2.761193, -0.399279, 4.543908);
    _key_frames[6] = osg::Vec3(3.052726, -0.033925, 4.423780);
    _key_frames[7] = osg::Vec3(4.364782, -0.065473, 4.256228);
    _key_frames[8] = osg::Vec3(2.808153, -0.037576, 4.293646);
    _key_frames[9] = osg::Vec3(4.553224, 0.336446, 4.194273);
    _key_frames[10] = osg::Vec3(4.648286, 0.904590, 3.994246);
    _key_frames[11] = osg::Vec3(4.387308, 1.201336, 3.573984);
    _key_frames[12] = osg::Vec3(3.764152, 1.035572, 2.923845);
    _key_frames[13] = osg::Vec3(3.165639, 0.167697, -0.112111);
    */

    // downwards + parallel + round head shape
    /*
    _keys[0] = osg::Vec3(0.883929, -0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948739, -0.000000, 1.697608);
    _keys[2] = osg::Vec3(2.968704, 0.000002, 2.447814);
    _keys[3] = osg::Vec3(4.148568, 0.000005, 3.235165);
    _keys[4] = osg::Vec3(5.651839, 0.000008, 3.992203);
    _keys[5] = osg::Vec3(6.700885, 0.000012, 4.478873);
    _keys[6] = osg::Vec3(7.801183, 0.000014, 4.826830);
    _keys[7] = osg::Vec3(9.117777, 0.000016, 4.958488);
    _keys[8] = osg::Vec3(9.117777, 0.000016, 4.958488);
    _keys[9] = osg::Vec3(10.208667, 0.000019, 4.986700);
    _keys[10] = osg::Vec3(11.261942, 0.000022, 4.883253);
    _keys[11] = osg::Vec3(12.390450, 0.000028, 4.610528);
    _keys[12] = osg::Vec3(13.443725, 0.000022, 4.234355);
    _keys[13] = osg::Vec3(14.412363, 0.000038, 3.905203);

    _key_frames[0] = osg::Vec3(2.392214, -2.119610, 5.095464);
    _key_frames[1] = osg::Vec3(2.444726, -2.228047, 5.203313);
    _key_frames[2] = osg::Vec3(2.840532, -2.162443, 4.941717);
    _key_frames[3] = osg::Vec3(2.898482, -1.916741, 4.687342);
    _key_frames[4] = osg::Vec3(2.708086, -1.570847, 4.722943);
    _key_frames[5] = osg::Vec3(3.027918, -1.390282, 4.534034);
    _key_frames[6] = osg::Vec3(3.205586, -1.339838, 4.400910);
    _key_frames[7] = osg::Vec3(4.337639, -0.906544, 4.277610);
    _key_frames[8] = osg::Vec3(3.061746, -0.994793, 4.211329);
    _key_frames[9] = osg::Vec3(4.593148, -0.643569, 4.199039);
    _key_frames[10] = osg::Vec3(4.687363, -0.077307, 4.006274);
    _key_frames[11] = osg::Vec3(4.437125, 0.493707, 3.595784);
    _key_frames[12] = osg::Vec3(3.795799, 0.628956, 2.909116);
    _key_frames[13] = osg::Vec3(3.165640, 0.167698, -0.112114);
    */

    _keys[0] = osg::Vec3(0.883929, -0.000000, 0.761077);
    _keys[1] = osg::Vec3(1.948738, -0.000000, 1.697608);
    _keys[2] = osg::Vec3(2.968704, 0.000002, 2.447814);
    _keys[3] = osg::Vec3(4.148568, 0.000005, 3.235165);
    _keys[4] = osg::Vec3(5.651840, 0.000008, 3.992204);
    _keys[5] = osg::Vec3(6.700886, 0.000012, 4.478873);
    _keys[6] = osg::Vec3(7.801184, 0.000014, 4.826830);
    _keys[7] = osg::Vec3(9.117778, 0.000016, 4.958488);
    _keys[8] = osg::Vec3(9.117778, 0.000016, 4.958488);
    _keys[9] = osg::Vec3(10.208668, 0.000018, 4.986700);
    _keys[10] = osg::Vec3(11.261943, 0.000021, 4.883253);
    _keys[11] = osg::Vec3(12.390451, 0.000027, 4.610528);
    _keys[12] = osg::Vec3(13.443726, 0.000022, 4.234354);
    _keys[13] = osg::Vec3(14.412364, 0.000038, 3.905202);

    _key_frames[0] = osg::Vec3(2.392214, -1.477586, 5.095465);
    _key_frames[1] = osg::Vec3(2.579638, -1.591753, 5.201334);
    _key_frames[2] = osg::Vec3(2.600487, -1.329980, 5.003319);
    _key_frames[3] = osg::Vec3(2.663891, -1.109758, 4.789170);
    _key_frames[4] = osg::Vec3(2.533679, -1.113934, 4.798922);
    _key_frames[5] = osg::Vec3(3.027919, -1.195475, 4.534033);
    _key_frames[6] = osg::Vec3(3.205587, -0.909864, 4.400909);
    _key_frames[7] = osg::Vec3(4.337637, -0.768286, 4.277611);
    _key_frames[8] = osg::Vec3(3.061745, -0.548467, 4.211329);
    _key_frames[9] = osg::Vec3(4.593145, -0.643567, 4.199041);
    _key_frames[10] = osg::Vec3(4.687360, -0.077306, 4.006273);
    _key_frames[11] = osg::Vec3(4.437126, 0.493709, 3.595782);
    _key_frames[12] = osg::Vec3(3.795800, 0.628957, 2.909115);
    _key_frames[13] = osg::Vec3(3.165641, 0.167698, -0.112113);

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

void PalmParameter::setUpVec()
{
    int n = onCurveSize();
    if(n < 3)
    {
        printf("PalmParameter::setUpVec() error\n");
        return;
    }
    osg::Vec3 start = getOnCurve(0);
    osg::Vec3 mid = getOnCurve(int(n/2));
    osg::Vec3 end = getOnCurve(n-1);
    osg::Vec3 normal = (end - mid) ^ ((mid - start) ^ (end - mid));

    //if 3 points are collinear
    if(normal.x() == 0.0f && normal.y() == 0.0f && normal.z() == 0.0f)
    {
        normal = end - start;
        float tmp = normal.z();
        normal.z() = -normal.x();
        normal.x() = tmp;
        normal.y() = 0.0f;//prefer y to be 0 because we want to let the leaf parallel to the input image plane
    }

    normal.normalize();
    _up = normal;
}

osg::Vec3 PalmParameter::getUpVec()
{
    return _up;
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

    setUpVec();
}

float PalmParameter::setBezierCubic(osg::Vec3 ctr1, osg::Vec3 ctr2, osg::Vec3 ctr3, osg::Vec3 ctr4, float scale)
{
    float ret = scale;
    int steps = ((ctr2-ctr1).length() + (ctr3-ctr2).length() + (ctr4-ctr3).length()) * 28;
    //int steps = ((ctr2-ctr1).length() + (ctr3-ctr2).length() + (ctr4-ctr3).length()) * 4;
    //float slop = -1.0f;
    //if(ctr4.x()!=ctr1.x())
    //    slop = (ctr4.z()-ctr1.z()) / fabs(ctr4.x()-ctr1.x());//don't decrease steps if it's heading downwards
    //if(slop > 2)
    //    steps -= 5;
    //else if(slop < 1)//increase interpolation if it is flatter
    //    steps += 15;

    _on_curve = Transformer::interpolate_bezier_3(ctr1, ctr2, ctr3, ctr4, steps);

    //setup scale
    if(scale < 0)
    {
        _target_dist = 0.0f;
        for(unsigned int i=0; i<_on_curve.size()-1; i++)
        {
            osg::Vec3 a = _on_curve[i];
            osg::Vec3 b = _on_curve[i+1];
            _target_dist += (a - b).length();
        }

        if(_keys_dist != 0.0f && _target_dist != 0.0f)
        {
            _scale = _target_dist / _keys_dist * 0.50f;//hard-code: the widht of leaf
            ret = _scale;
        }
    }
    else
        _scale = scale;

    setUpVec();
    return ret;
}

void PalmParameter::enableGravity(bool gravity)
{
    _gravity = gravity;
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
    osg::Vec3 up = getUpVec();
    osg::Vec3 side = up ^ tangent;
    osg::Vec3 cur = _on_curve[k];

    //e. scale to get final result
    if(mirror)
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() - side * interpolated.z()) * _scale;
    else
        ret = cur + (tangent * interpolated.x() + up * interpolated.y() + side * interpolated.z()) * _scale;

    //f. add effect of gravity to target
    if(_gravity)
    {
        int sign = mirror ? 1 : -1;
        double hard_upper_limit = 25 * M_PI / 180;
        osg::Vec3 g(0, 0, -1);
        int max_ang = 50;//hard-code: maximum angle of rotation under the gravity effect
        int noise = rand() % 20 - 15;//hard-code: degree of noise added to the rotation
        float effect = (g - tangent * (g * tangent)).length();//length of orthgonal vector
        float rotate = sign * std::min((max_ang + noise) * M_PI / 180 * effect, hard_upper_limit);
        //rotate = 0;//debug
        osg::Quat q(rotate, tangent);
        ret = q * (ret - cur) + cur;

        //rotate above up vector as noise
        float noise2 = (rand() % 6 - 3) * M_PI / 180;
        osg::Quat q2(noise2, up);
        ret = q2 * (ret - cur) + cur;
    }

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
    //float correction = len * 0.045f;//empirical

    unsigned int on_curve_size = _on_curve.size();
    unsigned int keys_size = _keys.size();
    for(unsigned int i=0; i<on_curve_size; i++)
    {
        float t = float(i) / on_curve_size * (keys_size - 1);
        int low = floor(t);
        if(low == 0)
            continue;

        osg::Vec3 tail1 = getFrameAt(i);
        osg::Vec3 tail2 = getFrameAt(i, true);
        osg::Vec3 cur = _on_curve[i];

        //to fix the misalignment problem caused by the connected mesh
        //but only apply to the second half of the branch
        osg::Vec3 translate = osg::Vec3(0, 0, 0);//seems unnecessary
        //translate = cur - (tail1 + tail2)*0.5f;
        //translate.normalize();
        //translate = translate * (i / float(on_curve_size)) * correction;

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
