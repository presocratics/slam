// feature class header

#ifndef FEATURE_H
#define FEATURE_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
using namespace cv;

struct frame {
    cv::Vec3d world,body;
};				/* ----------  end of struct frame  ---------- */

typedef struct frame Frame;

class Feature{
    public:
        Scalar RGB;
        int ID;
        int refFlag;
        frame position;

        // constructor
        Feature();
        ~Feature(); 
        Feature(Vec3d pos, Scalar color, int n, int ref );

        // accessor
        Vec3d get_body();
        Scalar getRGB();
        int getID();
        int getRefFlag();
       
        // mutator
        void set_body(Vec3d pos);
        void setRGB(Scalar color);
        void setID(int n);
        void setRefFlag(int ref);

};

#endif
