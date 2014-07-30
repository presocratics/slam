#ifndef  view_INC
#define  view_INC
#include <cv.h>

/*
 * =====================================================================================
 *        Class:  Vfeat
 *  Description:  Stores view features
 * =====================================================================================
 */
struct vfeat
{
        /* ====================  LIFECYCLE     ======================================= */
        vfeat ( const cv::Point2d& cur, const cv::Point2d& init, const cv::Point2d&
                refl ) : current(cur), initial(init), reflection(refl) { }

        vfeat ( const cv::Vec3d& cur, const cv::Vec3d& init, const cv::Vec3d& refl ) :
                current(cv::Point2d(cur[0],cur[1])), initial(cv::Point2d(init[0],init[1])),
                reflection(cv::Point2d(refl[0],refl[1])) { }

        cv::Point2d current, initial, reflection;
}; /* -----  end of class Vfeat  ----- */
typedef struct vfeat Vfeat;

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
        ~View()
        {
            for( size_t i=0; i<features.size(); ++i )
                delete features[i];
        }

        /* ====================  ACCESSORS     ======================================= */
        void toMat(cv::Mat& R);
        
        /*
         *--------------------------------------------------------------------------------------
         *       Class:  View
         *      Method:  get_altitude
         *--------------------------------------------------------------------------------------
         */
        inline double
        get_altitude (  ) const
        {
            return altitude;
        }		/* -----  end of method View::get_altitude  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  View
         *      Method:  set_altitude
         *--------------------------------------------------------------------------------------
         */
        inline View&
        set_altitude ( double value )
        {
            altitude	= value;
            return *this;
        }		/* -----  end of method View::set_altitude  ----- */
        
        /* ====================  DATA MEMBERS  ======================================= */
        std::vector<Vfeat*> features;
    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        double altitude;

}; /* -----  end of class View  ----- */
void subtract ( const View& lhs, const View& rhs, View& dst );
#endif   /* ----- #ifndef view_INC  ----- */
