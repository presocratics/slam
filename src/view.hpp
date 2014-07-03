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
        Vfeat ( cv::Point2d cur, cv::Point2d init, cv::Point2d refl ) {
            set_views( cur, init, refl );
        }
        Vfeat ( cv::Vec3d cur, cv::Vec3d init, cv::Vec3d refl ) {
            set_views( cur, init, refl );
        }

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */
        void set_views( cv::Vec3d cur, cv::Vec3d init, cv::Vec3d refl );
        void set_views( cv::Point2d cur, cv::Vec3d init, cv::Point2d refl );
        void set_views( cv::Point2d cur, cv::Point2d init, cv::Point2d refl );
        

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

        /* ====================  ACCESSORS     ======================================= */
        void toMat(cv::Mat& R);

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

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
void subtract ( const View& lhs, const View& rhs, View& dst );
#endif   /* ----- #ifndef view_INC  ----- */
