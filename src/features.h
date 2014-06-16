// feature class header
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

class Feature{
    private:
        cv::Vec3d X;
        cv::Scalar RGB;
        int ID;
        int refFlag;
    public:
        // constructor
        Feature::Feature();
        Feature::Feature(Vec3d pos, Scalar color, int n, int ref );
        
        // accessor
        cv::Vec3d getX();
        cv::Scalar getRGB();
        int getID();
        int getRefFlag();
       
        // mutator
        void setX(Vec3d pos);
        void setRGB(Scalar color);
        void setID(int n);
        void setrefFlag(int ref);

}

