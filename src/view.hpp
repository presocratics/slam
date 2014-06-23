#ifndef  view_INC
#define  view_INC
#include <cv.h>

/*
 * =====================================================================================
 *        Class:  Vfeat
 *  Description:  Stores view features
 * =====================================================================================
 */
class Vfeat
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Vfeat (){;}                             /* constructor */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */
        void set_views( cv::Vec3d cur, cv::Vec3d init, cv::Vec3d refl );
        void set_views( cv::Point2d cur, cv::Vec3d init, cv::Point2d refl );
        

        /* ====================  DATA MEMBERS  ======================================= */
        cv::Point2d current, initial, reflection;



    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class Vfeat  ----- */

/*
 * =====================================================================================
 *        Class:  View
 *  Description:  Stores view information
 * =====================================================================================
 */
class View
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        View (){;}                             /* constructor */
        View ( int nf )
        {
            set_num_features(nf);
        }

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        void set_num_features( int nf )
        {
            features.resize(nf);
        }

        /* ====================  OPERATORS     ======================================= */
        View& operator-=(const View& rhs );

        /* ====================  DATA MEMBERS  ======================================= */
        double altitude;
        std::vector<Vfeat> features;

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class View  ----- */
inline View operator-(View lhs, const View& rhs)
{
    lhs-=rhs;
    return lhs;
}

#endif   /* ----- #ifndef view_INC  ----- */
