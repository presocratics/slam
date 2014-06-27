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
#define DINIT 5            /* Max initial depth */
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

Feature::Feature( cv::Vec3d anchor, Sensors sense, cv::Point2d pib )
{
    double idepth;
    cv::Vec3d pibr;

    add( atan2( pib.y, 1) * 180 / M_PI,
        sense.quaternion.euler()*180/M_PI, pibr );
    idepth = -sense.altitude / sin(pibr[1] / 180 * M_PI) * 2;

    set_initial_anchor(anchor);
    set_initial_quaternion(sense.quaternion);
    set_initial_pib(pib);
    set_body_position( pib, 1/idepth );
    return ;
}		/* -----  end of method Feature::Feature  ----- */

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

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  get_initial.quaternion
 *--------------------------------------------------------------------------------------
 */
    inline Quaternion
Feature::get_initial_quaternion (  ) const
{
    return initial.quaternion;
}		/* -----  end of method Feature::get_initial.quaternion  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  set_initial.quaternion
 *--------------------------------------------------------------------------------------
 */
    inline void
Feature::set_initial_quaternion ( Quaternion value )
{
    initial.quaternion	= value;
    return ;
}		/* -----  end of method Feature::set_initial.quaternion  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  get_anchor
 *--------------------------------------------------------------------------------------
 */
inline cv::Vec3d
Feature::get_initial_anchor (  ) const
{
    return initial.anchor;
}		/* -----  end of method Feature::get_anchor  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  set_anchor
 *--------------------------------------------------------------------------------------
 */
    inline void
Feature::set_initial_anchor ( cv::Vec3d value )
{
    initial.anchor	= value;
    return ;
}		/* -----  end of method Feature::set_anchor  ----- */

void Feature::set_body(Vec3d pos)
{
    position.body = pos;
}


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  get_body_position
 *--------------------------------------------------------------------------------------
 */
inline cv::Vec3d
Feature::get_body_position (  ) const
{
    return position.body;
}		/* -----  end of method Feature::get_body_position  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  set_body_position
 *--------------------------------------------------------------------------------------
 */
    inline void
Feature::set_body_position ( cv::Vec3d value )
{
    position.lastbody = get_body_position();
    position.body	= value;
    return ;
}		/* -----  end of method Feature::set_body_position  ----- */


    void
Feature::set_body_position ( cv::Point2d p, double d )
{
    set_body_position( cv::Vec3d(p.x,p.y,d) );
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
