#include "LineSegmentDetector.h"
extern "C" {
    #include "lsd.h"
}

LineSegmentDetector::LineSegmentDetector(const QImage& img): _img(img), _valid(false)
{
    _w = _img.width();
    _h = _img.height();
    if(_w > 0 && _h > 0 && !_img.isNull())
        _valid = true;
}

LineSegmentDetector::~LineSegmentDetector()
{
}

std::vector <osg::Vec4> LineSegmentDetector::run()
{
    std::vector <osg::Vec4> ret;

    if(!_valid)
        return ret;

    double *image = NULL;
    double *out;
    int x, y, i, n;

    //1. initialize lsd input
    image = (double *) malloc( _w * _h * sizeof(double) );
    if(!image)
    {
      printf("LineSegmentDetector::run(): error: not enough memory\n");
      return ret;
    }

    //2. fill the double array from data in QImage
    for(x=0; x<_w; x++)
        for(y=0; y<_h; y++)
        {
            QRgb c = _img.pixel(x, y);
            image[x + y * _w] = rgb2gray(qRed(c), qGreen(c), qBlue(c));
        }

    //3. LSD call
    out = lsd(&n, image, _w, _h);

    //4. fill output in std::vector
    for(i=0; i<n; i++)
    {
        osg::Vec4 v(out[7*i], out[7*i+1], out[7*i+2], out[7*i+3]);
        ret.push_back(v);
    }

    //5. debug output
    //int j;
    //printf("%d line segments found:\n",n);
    //for(i=0; i<n; i++)
    //{
    //    for(j=0; j<7; j++)
    //        printf("%f ", out[7*i+j]);
    //    printf("\n");
    //}

    //6. free memory
    free( (void *) image );
    free( (void *) out );

    return ret;
}

int LineSegmentDetector::rgb2gray(int r, int g, int b)
{
    return 0.2989 * r + 0.5870 * g + 0.1140 * b;
}

void LineSegmentDetector::test()
{
    double * image;
    double * out;
    int x,y,i,j,n;
    int X = 128;  /* x image size */
    int Y = 128;  /* y image size */

    /* create a simple image: left half black, right half gray */
    image = (double *) malloc( X * Y * sizeof(double) );
    if( image == NULL )
    {
      fprintf(stderr,"error: not enough memory\n");
      exit(EXIT_FAILURE);
    }
    for(x=0;x<X;x++)
    for(y=0;y<Y;y++)
      image[x+y*X] = x<X/2 ? 0.0 : 64.0; /* image(x,y) */


    /* LSD call */
    out = lsd(&n,image,X,Y);


    /* print output */
    printf("%d line segments found:\n",n);
    for(i=0;i<n;i++)
    {
      for(j=0;j<7;j++)
        printf("%f ",out[7*i+j]);
      printf("\n");
    }

    /* free memory */
    free( (void *) image );
    free( (void *) out );
}
