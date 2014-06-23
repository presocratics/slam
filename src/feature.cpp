// feature class cpp
#include "feature.h"

// constructor
Feature::Feature()
{
    position.body = Vec3d(0,0,0);
    RGB = Scalar(0,0,0);
    ID = -1;
    refFlag = -1;
}

Feature::Feature(Vec3d pos, Scalar color, int n, int ref)
{
   position.body = pos;
   RGB = color;
   ID = n;
   refFlag = ref;
}

Feature::~Feature(){};

// accessor
Vec3d Feature::get_body()
{
    return position.body;
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
void Feature::set_body(Vec3d pos)
{
    position.body = pos;
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

    cv::Vec3d
Feature::fromAnchor ( cv::Vec3d pos )
{
    return initial.quaternion.rotation().t()*(pos - initial.anchor);
}		/* -----  end of method Feature::fromAnchor  ----- */


    cv::Matx33d
Feature::rb2b ( Quaternion qbw )
{
    return initial.quaternion.rotation().t() * qbw.rotation();
}		/* -----  end of method Feature::rb2b  ----- */

