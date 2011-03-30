#ifndef __BDL_H
#define __BDL_H

#include <stdio.h>

typedef struct BDLImageCoord
{
    int cameraIndex;
    int flag;
    float u, v;
}BDLImageCoord;

typedef struct BDLPoint
{
    float x, y, z, w;
    //int flag, param;
    float r, g, b;
    //bool outlier;
    int imgCoordSize;
    BDLImageCoord *imgCoordPtr;
}BDLPoint;

class BDL
{
    public:
        BDL(const char * pathToBDL);
        ~BDL();
    //setter:
        int loadFromFile(); //success = 1, fail = 0
    //getter:
        int getPointSize();
        BDLPoint getPoint(int index); //for testing
        const char * getFilePath();
        /*
           getPoints();
           getCameraMatrices;
         */
    //private:
        int T_fread(FILE *input);
        char * _BDLFilePath;
        int _pointSize;
        BDLPoint *_points;
        /*
           BDLPoints;
           CameraMatrices;
       */
};

#endif
