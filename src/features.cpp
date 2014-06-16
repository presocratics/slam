// feature class cpp
#include "features.h"

// constructor
Feature::Feature()
{
    X = Vec3d(0,0,0);
    RGB = Scalar(0,0,0);
    ID = -1;
    refFlag = -1;
}

Feature::Feature(Vec3d pos, Scalar color, int n, int ref)
{
   X = pos;
   RGB = color;
   ID = n;
   refFlag = ref;
}

// accessor
cv::Vec3d getX()
{
    return X;
}

cv::Scalar getRGB()
{
    return RGB;
}

int getID()
{
    return ID;
}

int getRefFlag
{
    return refFlag;
}

// mutator
void setX(Vec3d pos)
{
    X = pos;
}

void setRGB(Scalar color)
{
    RGB = color;
}

void setID(int n)
{
    ID = n;
}

void setRefFlag(int ref)
{
    refFlag = ref;
}
