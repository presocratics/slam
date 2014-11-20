#include "view.hpp"

    void
View::toMat ( cv::Mat& R ) const
{
    R.create(31,1,CV_64F);
    std::vector<double> bar;
    bar.push_back(altitude);
    std::vector<Vfeat>::const_iterator it=features.begin();
    for( ; it!=features.end(); ++it )
    {
        bar.push_back(it->current.x);
        bar.push_back(it->current.y);
        bar.push_back(it->initial.x);
        bar.push_back(it->initial.y);
        /* do not add features with no reflection */
        if(it->reflection.x != 0) // isRef
        {
            //std::cout << "adding refl" << std::endl;
            bar.push_back(it->reflection.x);
            bar.push_back(it->reflection.y);
        }
    }
    R = cv::Mat(bar, true);
    return ;
}		/* -----  end of method View::toMat  ----- */

    void
subtract ( const View& lhs, const View& rhs, View& dst )
{
    dst.altitude=lhs.altitude-rhs.altitude;
    std::vector<Vfeat>::const_iterator lt=lhs.features.begin();
    std::vector<Vfeat>::const_iterator rt=rhs.features.begin();
    for( ; lt!=lhs.features.end(); ++lt, ++rt )
    {
        cv::Point2d current = lt->current-rt->current;
        cv::Point2d initial = lt->initial-rt->initial;
        cv::Point2d reflection = cv::Point2d(0,0);
        if(lt->reflection.x != 0)
        {
            reflection = lt->reflection-rt->reflection;
        }
        Vfeat vf( current, initial, reflection );
        dst.features.push_back(vf);
    } 
    return ;
}		/* -----  end of method View::subtract  ----- */

