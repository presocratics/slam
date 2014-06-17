// feature class header

#ifndef FEATURE_H
#define FEATURE_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
using namespace cv;

class Feature{
    public:
        Vec3d X;
        Scalar RGB;
        int ID;
        int refFlag;

        // constructor
        Feature();
        ~Feature(); 
        Feature(Vec3d pos, Scalar color, int n, int ref );

        // accessor
        Vec3d getX();
        Scalar getRGB();
        int getID();
        int getRefFlag();
       
        // mutator
        void setX(Vec3d pos);
        void setRGB(Scalar color);
        void setID(int n);
        void setRefFlag(int ref);

};

#endif
