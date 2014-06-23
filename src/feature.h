// feature class header

#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include "Quaternion.hpp"
using namespace cv;

struct frame {
    cv::Vec3d world,body;
};				/* ----------  end of struct frame  ---------- */

struct inits {
    Quaternion quaternion;
    double inverse_depth;
    cv::Vec3d anchor;
    cv::Point2d pib;
};				/* ----------  end of struct inits  ---------- */

typedef struct inits Inits;
typedef struct frame Frame;

class Feature{
    public:
        Scalar RGB;
        int ID;
        int refFlag;
        frame position;
        inits initial;

        // constructor
        Feature();
        ~Feature(); 
        Feature(Vec3d pos, Scalar color, int n, int ref );

        // accessor
        cv::Vec3d fromAnchor ( cv::Vec3d pos );
        cv::Matx33d rb2b ( Quaternion qbw );

        Vec3d get_body();
        Scalar getRGB();
        int getID();
        int getRefFlag();
       
        // mutator
        void set_body(Vec3d pos);
        void setRGB(Scalar color);
        void setID(int n);
        void setRefFlag(int ref);
        void set_initial_pib( cv::Point2d p );
        void set_initial_pib( cv::Vec3d p );
        void set_body_position( cv::Point2d, double d );
};

#endif
