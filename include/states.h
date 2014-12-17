// states class header
#ifndef STATES_H
#define STATES_H
#include <utility>
#include <vector>
#include <map>
#include "Quaternion.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include "feature.h"
#include "Sensors.hpp"
#include "imagesensor.hpp"
using namespace cv;
#define GRAVITY -9.80665            /* Standard acceleration due to free fall m/s^2 */
#define DMIN 0.3            /*  */

class States{

    public:
        // constructor
        States() : X(), V(), b() {};
        States( const cv::Mat& kx);

        Vec3d X;
        Vec3d V;
        Vec3d b;
        featMap feats;
        active features;
        int rf;
        int nrf;

        // accessor
        Vec3d getX();
        Vec3d getV();
        Vec3d getb();
        int getRows() const;
        int getNumFeatures() const;

        // mutator
        void clearContainers();
        void update_features( const ImageSensor& imgsense, const Sensors& sense );
        States& setX(const Vec3d& pos);
        States& setV(const Vec3d& vel);
        States& addFeature(const Feature& f);
        States& setb(const Vec3d& bias);
        void add( const States& a);
        States dynamics( const Sensors& s );
        void setMinMaxDepth(double minD, double MaxD);
        void set_rf_nrf(int r, int n);
        void toMat(Mat& outMat);

        //Operator
        States& operator*= ( const double& rhs );
        States& operator+= ( const States& rhs );
    private:
};
inline States operator*(States lhs, const double& rhs)
{
    lhs*=rhs;
    return lhs;
}

inline States operator+(States lhs, const States& rhs)
{
    lhs+=rhs;
    return lhs;
}


#endif
