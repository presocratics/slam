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
Feature::Feature( const Vec3d& pos, int n )
{
   set_body_position(pos);
   setID(n);
}

Feature::Feature( const cv::Vec3d& anchor, const Sensors& sense, const projection& match )
{
    initialize( anchor, sense, match, false );
    return ;
}		/* -----  end of method Feature::Feature  ----- */


    void
Feature::initialize ( const cv::Vec3d& anchor, const Sensors& sense, const
        projection& match, const bool extant )
{
    if( !extant )
    {
        double idepth;
        cv::Vec3d pibr;

        add( atan2( match.source.y, 1) * 180 / M_PI,
            sense.quaternion.euler()*180/M_PI, pibr );
        idepth = -sense.altitude / sin(pibr[1] / 180 * M_PI) * 2;
        idepth = fmin( idepth, DINIT );
        set_body_position( match.source, 1/idepth );
    }
    else
    {
        set_body_position( match.source, get_body_position()[2] );
    }
    setID( match.id );
    set_initial_anchor(anchor);
    set_initial_quaternion(sense.quaternion);
    set_initial_pib(match.source);
    set_noMatch(0);
    return ;
}		/* -----  end of method Feature::initialize  ----- */

// accessor

int Feature::getID() const { return ID; }

// mutator
Feature& Feature::set_body( const cv::Vec3d& pos)
{
    position.body = pos;
    return *this;
}

Feature& Feature::setID(int n)
{
    ID = n;
    return *this;
}

    cv::Vec3d
Feature::fromAnchor ( const cv::Vec3d& pos ) const
{
    return initial.quaternion.rotation().t()*(pos - initial.anchor);
}		/* -----  end of method Feature::fromAnchor  ----- */


    cv::Matx33d
Feature::rb2b ( const Quaternion& qbw ) const
{
    return initial.quaternion.rotation().t() * qbw.rotation();
}		/* -----  end of method Feature::rb2b  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  Feature
 *      Method:  Feature :: incNoMatch
 * Description:  Increments no match by 1.
 *--------------------------------------------------------------------------------------
 */
    int
Feature::incNoMatch ( )
{
    set_noMatch( get_noMatch()+1 );
    return get_noMatch();
}		/* -----  end of method Feature::incNoMatch  ----- */

