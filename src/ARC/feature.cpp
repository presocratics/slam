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
#define DINIT 100            /* Max initial depth */

void
euler2quaternion2 ( double phi, double theta, double psi, Quaternion& q )
{
    double q0 = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);
    double q1 = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
    double q2 = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
    double q3 = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);
    q.coord = cv::Vec4d(q0,q1,q2,q3);
    return;
}		/* -----  end of function euler2quaternion  ----- */


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
}        /* -----  end of method Feature::Feature  ----- */


    void
Feature::initialize ( const cv::Vec3d& anchor, const Sensors& sense, 
        const projection& match, const bool extant )
{
    if( !extant )
    {
        double idepth;
        cv::Vec3d pibr;
        //idepth = DINIT;
        idepth = 25.;
        set_body_position( match.source, 1/idepth );
    }
    else
    {
        set_body_position( match.source, get_body_position()[2] );
    }
    
    set_isRef(match.isRef());  
    setID( match.id );
    set_initial_anchor(anchor);

    set_initial_quaternion(sense.quat.get_value());
    set_initial_pib(match.source);
    set_noMatch(0);
    return ;
}       /* -----  end of method Feature::initialize  ----- */

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
}        /* -----  end of method Feature::fromAnchor  ----- */


    cv::Matx33d
Feature::rb2b ( const Quaternion& qbw ) const
{
    /* 
    std::cout << "init_quat: " << initial.quaternion.coord << std::endl;
    std::cout << "rb2b0: " << initial.quaternion.rotation().t() << std::endl;
    std::cout << "rb2b_curr: " << qbw.rotation() << std::endl;
     */ 
    return initial.quaternion.rotation().t() * qbw.rotation();
}        /* -----  end of method Feature::rb2b  ----- */


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
}        /* -----  end of method Feature::incNoMatch  ----- */

    cv::Vec3d
Feature::ppbHat ( const cv::Vec3d& X, const Quaternion& qbw ) const
{
    cv::Vec3d rv, xpbHat;
    xpbHat = this->xpbHat( X, qbw );
    rv = cv::Vec3d(
        xpbHat[1] / xpbHat[0],
        xpbHat[2] / xpbHat[0],
        0);
    return rv ;
}		/* -----  end of method Feature::ppbHat  ----- */

    cv::Vec3d
Feature::xpbHat ( const cv::Vec3d& X, const Quaternion& qbw ) const
{
    cv::Mat temp;
    const cv::Mat n = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    const cv::Mat S = cv::Mat::eye(3, 3, CV_64F) - 2 * n * n.t();
    temp = S * (Mat)qbw.rotation()*(Mat)this->xibHat();
    temp -= 2*n*n.t()*(Mat)X; 
    temp = (Mat)qbw.rotation().t() * temp;
    //std::cout << "X: " << X << std::endl;
    //std::cout << "s: " << S << std::endl;
    //std::cout << "Rw2b: " << (Mat)qbw.rotation().t() << std::endl;
    //std::cout << "xpbHat: " << temp << std::endl;
    return (cv::Vec3d) temp ;
}		/* -----  end of method Feature::xpbHat  ----- */

    cv::Vec3d 
Feature::xibHat() const 
{
    return cv::Vec3d( 1 / position.body[2],
            position.body[0] / position.body[2],
            position.body[1] / position.body[2]);
}
    cv::Vec3d 
Feature::xib0Hat( const cv::Vec3d& pos, const Quaternion& qbw) const
{
    cv::Vec3d rv;
    cv::Vec3d xibHat = this->xibHat();
    rv = rb2b(qbw)*xibHat;
    rv+=fromAnchor(pos);
    //std::cout << "xibHat: " << xibHat << std::endl;
    //std::cout << "rb2b: " << rb2b(qbw) << std:: endl;
    return rv;
}
    cv::Vec3d 
Feature::pib0Hat( const cv::Vec3d& pos, const Quaternion& qbw) const 
{
    cv::Vec3d xib0Hat=this->xib0Hat(pos, qbw);
    //std:: cout << "xib0Hat" << xib0Hat << std::endl;
    return cv::Vec3d(
            xib0Hat[1] / xib0Hat[0],
            xib0Hat[2] / xib0Hat[0],
            0);
}
