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
        Vfeat ( const cv::Point2d& cur, const cv::Point2d& init, const cv::Point2d&
                refl ) : current(cur), initial(init), reflection(refl) {};
        Vfeat ( const cv::Vec3d& cur, const cv::Vec3d& init, const cv::Vec3d& refl ) :
                current(cv::Point2d(cur[0],cur[1])), initial(cv::Point2d(init[0],init[1])),
                reflection(cv::Point2d(refl[0],refl[1])) {};

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

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
