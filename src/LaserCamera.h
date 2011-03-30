#ifndef __LASERCAMERA_H
#define __LASERCAMERA_H

#include <osg/Vec2>
#include <osg/Vec3>
#include "BDLCamera.h"

/* A helper class for representing a laser camera
 * it stores 
 * a) the 3x4 projection matrix
 * b) the camera id
 * c) type of camera
 * d) camera parameters
 * Usage:
 * 1. LaserCamera()
 * 2. set_id_type()
 * 3. set_parameters()
 * 4. set_img_size()
 * 5. set_projection()
 * 6. project()
 * 6. project2tex()
 */
class LaserCamera
{
    public:

        LaserCamera();

        /* the bdl id and 
         * the type of camera: 0 == perspective, 1 == fisheye
         */
        void set_id_type(int id, int type);

        /* matlab convenion:
         * M = [a b c d; e f g h; i j k l]
         */
        void set_projection(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l);
        void set_projection(const BDLCamera& cam);

        /* the camera parameters from camera_info.txt
         * fx = 2312.401123
         * fy = 2318.612793
         * cx = 937.615051
         * cy = 1288.002075
         * k1 = -0.448866
         * k2 = 0.285165
         * k3 = -0.141900
         * p1 = 0.000702
         * p2 = -0.000289
         * fov_max = 92.021156
         * note: fish eye has no k3, p1 and p2
         */
        void set_parameters(float fx, float fy, float cx, float cy, float k1, float k2, float fov_max, float k3 = -23418715, float p1 = -23418715, float p2 = -23418715);
        void set_parameters(int info); //for the info-th camera info

        /* set the width and height of image
         * default to 1936 x 2592
         */
        void set_img_size(int w = 1936, int h = 2592);

        /* inverse of a 3x3 matrix
         */
        bool inverse3x3(float **m);

        /* distort the x_dir and y_dir according to
         * camera_info and projection formula
         */
        osg::Vec2 distort(float x_dir, float y_dir);

        /* project 3D points to 2D absolute img coords
         * origin at the upper-left corner
         */
        osg::Vec2 project(osg::Vec3 P);

        /* project 3D points to 2D texture coords
         * origin at the lower-left corner
         * values are capped to 1.0f if larger than it
         */
        osg::Vec2 project2tex(osg::Vec3 P);

        int _id, _type; //bdl_camera_id, 0: perspective, 1: fisheye
        float _projection[3][4]; //row then col
        float _fx, _fy, _cx, _cy, _k1, _k2, _k3, _p1, _p2, _fov_max; //camera_info.txt
        int _w, _h; //image width height
};

#endif
