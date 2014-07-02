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
View::toMat ( cv::Mat& R )
{
    R.create(31,1,CV_64F);
    std::vector<double> bar;
    bar.push_back(altitude);
    std::vector<Vfeat>::iterator it=features.begin();
    for( ; it!=features.end(); ++it )
    {
        bar.push_back(it->current.x);
        bar.push_back(it->current.y);
        bar.push_back(it->initial.x);
        bar.push_back(it->initial.y);
        bar.push_back(it->reflection.x);
        bar.push_back(it->reflection.y);
    }
    R = cv::Mat(bar);
    return ;
}		/* -----  end of method View::toMat  ----- */


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
    cv::Point2d i;
    i = cv::Point2d( init[0], init[1] );
    set_views( cur, i, refl );
    return ;
}		/* -----  end of method Vfeat::set_views  ----- */

    void
Vfeat::set_views ( cv::Point2d cur, cv::Point2d init, cv::Point2d refl )
{
    current=cur;
    reflection=refl;
    initial=init;
    return ;
}		/* -----  end of method Vfeat::set_views  ----- */

