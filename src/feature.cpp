/*
 * =====================================================================================
 *
 *       Filename:  feature.cpp
 *
 *    Description:  A tracked feature used in SLAM.
 *
 *        Version:  1.0
 *        Created:  06/24/2014 11:37:30 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */
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

    void
Feature::set_body_position ( cv::Point2d p, double d )
{
    position.body[0] = p.x;
    position.body[1] = p.y;
    position.body[2] = d;
    return ;
}		/* -----  end of method Feature::set_body_position  ----- */


    void
Feature::set_initial_pib ( cv::Vec3d p )
{
    initial.pib = cv::Point2d( p[0], p[1] );
    return ;
}		/* -----  end of method Feature::set_initial_pib  ----- */


    void
Feature::set_initial_pib ( cv::Point2d p )
{
    initial.pib = p;
    return ;
}		/* -----  end of method Feature::set_initial_pib  ----- */


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

