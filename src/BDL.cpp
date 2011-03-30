#include "BDL.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define CR 13            /* Decimal code of Carriage Return char */
#define LF 10            /* Decimal code of Line Feed char */
#define MAX_REC_LEN 1024 /* Maximum size of input buffer */

BDL::BDL(const char *pathToBDL): _pointSize(0)
{
    //deep copy the path
    _BDLFilePath = new char[strlen(pathToBDL)];
    strcpy(_BDLFilePath, pathToBDL);
}

BDL::~BDL()
{
    for(int i=0; i<_pointSize; i++)
            free(_points[i].imgCoordPtr);
    free(_points);
    //free file path
    delete[] _BDLFilePath;
}

int BDL::loadFromFile()
{
    FILE *inputFilePtr;                         /* Pointer to input file */
    inputFilePtr = fopen(_BDLFilePath, "r");    /* Open in TEXT mode */
    int iReadReturn = T_fread(inputFilePtr);    /* Read the file and print output */
    fclose(inputFilePtr);                       /* Close it */
    return (iReadReturn ? 0 : 1);               /* Exit with success or error code */
}

int BDL::getPointSize()
{
    return _pointSize;
}

BDLPoint BDL::getPoint(int index)
{
    return _points[index];
}

const char * BDL::getFilePath()
{
    return _BDLFilePath;
}

/******************************************************************************/
int BDL::T_fread(FILE *input) /* Use:       Read text file using fread()      */
    /*                                                   */
    /* Arguments: FILE *input                            */
    /*              Pointer to input file                */
    /*                                                   */
    /* Return:    int                                    */
    /*              0 = error                            */
    /*              1 = success                          */
    /******************************************************************************/
{
    /*
     *  This function reads the ENTIRE FILE into a character array and
     *  then parses the array to determine the contents of each line.
     *  This is lightning-fast, but may not work for large files. (See the
     *  notes preceding the call to calloc() in this function.)
     */

    int   isNewline;              /* Boolean indicating we've read a CR or LF */
    long  lFileLen;               /* Length of file */
    long  lIndex;                 /* Index into cThisLine array */
    long  lLineCount;             /* Current line number */
    long  lLineLen;               /* Current line length */
    long  lStartPos;              /* Offset of start of current line */
    long  lTotalChars;            /* Total characters read */
    char  cThisLine[MAX_REC_LEN]; /* Contents of current line */
    char *cFile;                  /* Dynamically allocated buffer (entire file) */
    char *cThisPtr;               /* Pointer to current position in cFile */

    fseek(input, 0L, SEEK_END);  /* Position to end of file */
    lFileLen = ftell(input);     /* Get file length */
    rewind(input);               /* Back to start of file */

    /*
     *  The next line attempts to reserve enough memory to read the
     *  entire file into memory (plus 1 byte for the null-terminator).
     *
     *  The program will simply quit if the memory isn't available.
     *  This normally won't happen on computers that use virtual
     *  memory (such as Windows PCs), but a real application should
     *  make provisions for reading the file in smaller blocks.
     *
     *  We could use malloc() to allocate the memory, but calloc()
     *  has the advantage of initializing all of the bits to 0, so
     *  we don't have to worry about adding the null-terminator
     *  (Essentially, every character initially IS a null-terminator).
     */

    cFile = (char *)(calloc(lFileLen + 1, sizeof(char)));

    if(cFile == NULL )
    {
        printf("\nInsufficient memory to read file.\n");
        return 0;
    }

    fread(cFile, lFileLen, 1, input); /* Read the entire file into cFile */

    lLineCount  = 0L;
    lTotalChars = 0L;

    cThisPtr    = cFile;              /* Point to beginning of array */

    int tokensIndex = 0;
    bool pointBracketOpen = false;
    char imgCoordList[1024];
    int pointIndex;

    while (*cThisPtr)                 /* Read until reaching null char */
    {
        lIndex    = 0L;                 /* Reset counters and flags */
        isNewline = 0;
        lStartPos = lTotalChars;

        while (*cThisPtr)               /* Read until reaching null char */
        {
            if (!isNewline)               /* Haven't read a CR or LF yet */
            {
                if (*cThisPtr == CR || *cThisPtr == LF) /* This char IS a CR or LF */
                    isNewline = 1;                        /* Set flag */
            }

            else if (*cThisPtr != CR && *cThisPtr != LF) /* Already found CR or LF */
                break;                                     /* Done with line */

            cThisLine[lIndex++] = *cThisPtr++; /* Add char to output and increment */
            ++lTotalChars;

        } /* end while (*cThisPtr) */

        cThisLine[lIndex] = '\0';     /* Terminate the string */
        ++lLineCount;                 /* Increment the line counter */
        lLineLen = strlen(cThisLine); /* Get length of line */

        /* Print the detail for this line */
        //PrintLine(cThisLine, lLineCount, lLineLen, lStartPos, NULL, 0);
        //printf("%s\n", cThisLine);
        char *NbPointOk = strstr(cThisLine, "NbPointOk");
        if(NbPointOk)
        {
            char tmp[3];
            sscanf(NbPointOk, "%s %d", tmp, &_pointSize);
            _points = (BDLPoint *) calloc(_pointSize, sizeof(BDLPoint));
            continue;
        }
        char *Point = strstr(cThisLine, "Point ");
        if(Point)
        {
            //printf("%s", Point);
            int index, flag, param;
            float x, y, z, w;
            sscanf(Point, "Point %d { %f %f %f %f Flag %d Param %d", &index, &x, &y, &z, &w, &flag, &param);
            //printf("index=%d flag=%d param=%u (%f %f %f %f)\n", index, flag, param, x, y, z, w);
            _points[index].x = x;
            _points[index].y = y;
            _points[index].z = z;
            _points[index].w = w;
            //change coordinates for osgviewer
            _points[index].y = z;
            _points[index].z = -y;

            /*
               _points[index].flag = flag;
               _points[index].param = param;
             */
            pointBracketOpen = true;
            tokensIndex = 0;
            memset(imgCoordList, 0, sizeof(imgCoordList));
            pointIndex = index;
            continue;
        }
        if(pointBracketOpen)
        {
            char delims[] = " ";
            char *result = NULL;
            result = strtok(cThisLine, delims);
            while(result != NULL)
            {
                if(strstr(result, "}"))
                {
                    //printf("%s\n", imgCoordList);
                    int size = (tokensIndex+1)/4;
                    _points[pointIndex].imgCoordSize = size;
                    _points[pointIndex].imgCoordPtr = (BDLImageCoord *) calloc(size, sizeof(BDLImageCoord));
                    char *tokens = NULL;
                    tokens = strtok(imgCoordList, delims);
                    int cnt = 0, imgPtrIndex = 0;
                    while(tokens != NULL)
                    {
                        switch(cnt%4)
                        {
                            case 0:
                                int ci;
                                sscanf(tokens, "%d", &ci);
                                _points[pointIndex].imgCoordPtr[imgPtrIndex].cameraIndex = ci;
                                break;
                            case 1:
                                int fi;
                                sscanf(tokens, "%d", &fi);
                                _points[pointIndex].imgCoordPtr[imgPtrIndex].flag = fi;
                                break;
                            case 2:
                                float ui;
                                sscanf(tokens, "%f", &ui);
                                _points[pointIndex].imgCoordPtr[imgPtrIndex].u = ui;
                                break;
                            case 3:
                                float vi;
                                sscanf(tokens, "%f", &vi);
                                _points[pointIndex].imgCoordPtr[imgPtrIndex].v = vi;
                                imgPtrIndex++;
                                break;
                        }
                        tokens = strtok(NULL, delims);
                        cnt++;
                    }
                    pointBracketOpen = false;
                    memset(imgCoordList, 0, sizeof(imgCoordList));
                    break;
                }
                sprintf(imgCoordList, "%s %s ", imgCoordList, result);
                //printf("%u:%s ", tokensIndex, result);
                result = strtok(NULL, delims);
                tokensIndex++;
            }
            continue;
        }
    } /* end while (cThisPtr <= cEndPtr) */

    //printf("Length of file array=%#x (dec %d)\n", strlen(cFile), strlen(cFile));

    free(cFile);
    return 1;

} /* end T_fread() */
