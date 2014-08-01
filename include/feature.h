// feature class header

#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include "Quaternion.hpp"
#include "Sensors.hpp"
#include "imagesensor.hpp"
using namespace cv;

struct frame {
    cv::Vec3d world,body;
};                /* ----------  end of struct frame  ---------- */

struct inits {
    Quaternion quaternion;
    double inverse_depth;
    cv::Vec3d anchor;
    cv::Point2d pib;
};                /* ----------  end of struct inits  ---------- */

typedef struct inits Inits;
typedef struct frame Frame;

class Feature{
    public:
        // constructor
        Feature(){};
        Feature( const Vec3d& pos, int n );
        Feature( const cv::Vec3d& anchor, const Sensors& sense, const projection& match );

        inits initial;

        // accessor
        int getID() const;

        cv::Vec3d fromAnchor ( const cv::Vec3d& pos ) const;
        cv::Matx33d rb2b ( const Quaternion& qbw ) const;

        cv::Vec3d xibHat() const;
        cv::Vec3d xib0Hat( const cv::Vec3d& pos, const Quaternion& qbw) const;
        cv::Vec3d pib0Hat( const cv::Vec3d& pos, const Quaternion& qbw) const;
        cv::Vec3d xpbHat ( const cv::Vec3d& X, const Quaternion& qbw ) const;
        cv::Vec3d ppbHat ( const cv::Vec3d& X, const Quaternion& qbw ) const;

        // mutator
        void initialize ( const cv::Vec3d& anchor, const Sensors& sense, const
                projection& match, const bool extant );
        int incNoMatch();
        Feature& set_body( const cv::Vec3d& pos);
        Feature& setID(int n);

            inline void
        set_initial_pib ( cv::Vec3d p )
        {
            set_initial_pib( cv::Point2d(p[0], p[1]) );
            return ;
        }        /* -----  end of method Feature::set_initial_pib  ----- */

            inline void
        set_initial_pib ( cv::Point2d p )
        {
            initial.pib = p;
            return ;
        }        /* -----  end of method Feature::set_initial_pib  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  get_world_position
         *--------------------------------------------------------------------------------------
         */
        inline cv::Vec3d
        get_world_position (  ) const
        {
            return position.world;
        }        /* -----  end of method Feature::get_world_position  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  get_body_position
         *--------------------------------------------------------------------------------------
         */
        inline cv::Vec3d
        get_body_position (  ) const
        {
            return position.body;
        }        /* -----  end of method Feature::get_body_position  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  set_body_position
         *--------------------------------------------------------------------------------------
         */
            inline void
        set_body_position ( cv::Vec3d value )
        {
            position.body    = value;
            return ;
        }        /* -----  end of method Feature::set_body_position  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  set_world_position
         *--------------------------------------------------------------------------------------
         */
            inline void
        set_world_position ( cv::Vec3d value )
        {
            position.world    = value;
            return ;
        }        /* -----  end of method Feature::set_world_position  ----- */
            inline void
        set_world_position ( cv::Point2d p, double d )
        {
            set_world_position( cv::Vec3d(p.x,p.y,d) );
            return ;
        }        /* -----  end of method Feature::set_body_position  ----- */
            inline void
        set_body_position ( cv::Point2d p, double d )
        {
            set_body_position( cv::Vec3d(p.x,p.y,d) );
            return ;
        }        /* -----  end of method Feature::set_body_position  ----- */
        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  get_anchor
         *--------------------------------------------------------------------------------------
         */
        inline cv::Vec3d
        get_initial_anchor (  ) const
        {
            return initial.anchor;
        }        /* -----  end of method Feature::get_anchor  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  set_anchor
         *--------------------------------------------------------------------------------------
         */
            inline void
        set_initial_anchor ( cv::Vec3d value )
        {
            initial.anchor    = value;
            return ;
        }        /* -----  end of method Feature::set_anchor  ----- */
        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  get_initial.quaternion
         *--------------------------------------------------------------------------------------
         */
            inline Quaternion
        get_initial_quaternion (  ) const
        {
            return initial.quaternion;
        }        /* -----  end of method Feature::get_initial.quaternion  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  set_initial.quaternion
         *--------------------------------------------------------------------------------------
         */
            inline void
        set_initial_quaternion ( Quaternion value )
        {
            initial.quaternion    = value;
            return ;
        }        /* -----  end of method Feature::set_initial.quaternion  ----- */
        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  get_noMatch
         *--------------------------------------------------------------------------------------
         */
        inline int
        get_noMatch (  ) const
        {
            return noMatch;
        }        /* -----  end of method Feature::get_noMatch  ----- */

        /*
         *--------------------------------------------------------------------------------------
         *       Class:  Feature
         *      Method:  set_noMatch
         *--------------------------------------------------------------------------------------
         */
            inline void
        set_noMatch ( int value )
        {
            noMatch    = value;
            return ;
        }        /* -----  end of method Feature::set_noMatch  ----- */


    private:
        frame position;
        int noMatch;
        int ID;

};
typedef std::pair<int,Feature*> featPair;
typedef std::map<int,Feature*> featMap;
typedef featMap::iterator featIter;
typedef std::vector<Feature*> active;
typedef active::iterator Fiter;
typedef active::const_iterator cFiter;

#endif
