// feature class cpp
#include "feature.h"

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

Feature::~Feature(){};

// accessor
Vec3d Feature::getX()
{
    return X;
}

Scalar Feature::getRGB()
{
    return RGB;
}

int Feature::getID()
{
    return ID;
}

int Feature::getRefFlag()
{
    return refFlag;
}

// mutator
void Feature::setX(Vec3d pos)
{
    X = pos;
}



void Feature::setRGB(Scalar color)
{
    RGB = color;
}

void Feature::setID(int n)
{
    ID = n;
}

void Feature::setRefFlag(int ref)
{
    refFlag = ref;
}
