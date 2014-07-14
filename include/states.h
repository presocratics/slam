// states class header
#ifndef STATES_H
#define STATES_H
#include <map>
#include "Quaternion.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include "feature.h"
#include "Sensors.hpp"
#include "imagesensor.hpp"
#include "config.hpp"
using namespace cv;
#define GRAVITY -9.80665            /* Standard acceleration due to free fall m/s^2 */
typedef std::map<int,Feature> featMap;
typedef featMap::iterator featIter;
typedef std::vector<projection>::iterator matchIter;
typedef std::vector<Feature>::iterator Fiter;


class States{

    public:
        Vec3d X;
        Vec3d V;
        Vec3d b;
        std::vector<Feature> features;
        std::map<int,Feature> feats;

        // constructor
        States();
        ~States();
        States(Vec3d pos, Vec3d vel, std::vector<Feature> feat, Vec3d bias, int n);
        States(cv::Mat kx);

        // accessor
        Vec3d getX();
        Vec3d getV();
        std::vector<Feature> getFeatures();
        Feature getFeature(int i);
        Vec3d getb();
        int getRows();
        int getNumFeatures();

        // mutator
        void update_features( ImageSensor *imgsense, Sensors sense );
        void end_loop (  );
        void setX(Vec3d pos);
        void setV(Vec3d vel);
        void setFeature(int i, Feature f);
        void addFeature(Feature f);
        void setb(Vec3d bias);
        void add(States a);
        States dynamics( Sensors s );
        //Operator
        States& operator*= ( const double& rhs );
        States& operator+= ( const States& rhs );
    private:
        int nf;
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
