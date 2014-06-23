#include "view.hpp"

    View&
View::operator-= ( const View& rhs )
{
    this->altitude -= rhs.altitude;
    std::vector<Vfeat>::iterator it=this->features.begin();
    std::vector<Vfeat>::const_iterator rt=rhs.features.begin();
    for( ; it!=this->features.end(); ++it, ++rt )
    {
        it->current-=rt->current;
        it->initial-=rt->initial;
        it->reflection-=rt->reflection;
    } 
    return *this;
}		/* -----  end of method View::operator+=  ----- */

    void
Vfeat::set_views ( cv::Vec3d cur, cv::Vec3d init, cv::Vec3d refl )
{
    cv::Point2d c,r;
    c=cv::Point2d( cur[0], cur[1] );
    r=cv::Point2d( refl[0], refl[1] );
    set_views( c, init, r );
    return ;
}		/* -----  end of method View::set_views  ----- */

    void
Vfeat::set_views ( cv::Point2d cur, cv::Vec3d init, cv::Point2d refl )
{
    current=cur;
    reflection=refl;
    initial.x = init[0];
    initial.y = init[1];
    return ;
}		/* -----  end of method Vfeat::set_views  ----- */


