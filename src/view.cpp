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
    current.x = cur[0];
    current.y = cur[1];
    initial.x = init[0];
    initial.y = init[1];
    reflection.x = refl[0];
    reflection.y = refl[1];
    return ;
}		/* -----  end of method View::set_views  ----- */

