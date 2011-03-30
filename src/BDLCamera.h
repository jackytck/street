#ifndef __BDLCAMERA_H
#define __BDLCAMERA_H

/* A lightweight helper class for representing a bdl camera
 * note: default copy constructor is enough
 * it stores 
 * a) the camera id
 * b) the 3x4 projection matrix
 */
class BDLCamera
{
    public:

        /* matlab convenion:
         * M = [a b c d; e f g h; i j k l]
         */
        BDLCamera(int id, float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l);

        int _id; //bdl_camera_id
        float _projection[3][4]; //row then col
};

#endif
