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

