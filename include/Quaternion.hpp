#ifndef  Quaternion_INC
#define  Quaternion_INC
#include <cv.h>

/*
 * =====================================================================================
 *        Class:  Quaternion
 *  Description:  Class for storing and transforming quaternion.
 * =====================================================================================
 */
class Quaternion
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Quaternion () : coord() {};
        Quaternion ( const cv::Vec4d& q ) : coord(q) {};

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */
        cv::Vec3d euler() const;
        cv::Matx33d rotation() const;

        /* ====================  DATA MEMBERS  ======================================= */
        cv::Vec4d coord;
    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class Quaternion  ----- */

#endif   /* ----- #ifndef Quaternion_INC  ----- */
