#include "view.hpp"

    void
View::toMat ( cv::Mat& R )
{
    R.create(31,1,CV_64F);
    std::vector<double> bar;
    bar.push_back(altitude);
    std::vector<Vfeat*>::iterator it=features.begin();
    for( ; it!=features.end(); ++it )
    {
        bar.push_back((*it)->current.x);
        bar.push_back((*it)->current.y);
        bar.push_back((*it)->initial.x);
        bar.push_back((*it)->initial.y);
        bar.push_back((*it)->reflection.x);
        bar.push_back((*it)->reflection.y);
    }
    R = cv::Mat(bar);
    return ;
}		/* -----  end of method View::toMat  ----- */

    void
subtract ( const View& lhs, const View& rhs, View& dst )
{
    dst.features.reserve( lhs.features.size() );
    dst.set_altitude( lhs.get_altitude()-rhs.get_altitude() );
    std::vector<Vfeat*>::const_iterator lt=lhs.features.begin();
    std::vector<Vfeat*>::const_iterator rt=rhs.features.begin();
    for( ; lt!=lhs.features.end(); ++lt, ++rt )
    {
        Vfeat *vf = new Vfeat( (*lt)->current-(*rt)->current,
                (*lt)->initial-(*rt)->initial,
                (*lt)->reflection-(*rt)->reflection );
        dst.features.push_back(vf);
    } 
    return ;
}		/* -----  end of method View::subtract  ----- */

