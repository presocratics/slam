#ifndef MAIN_H
#define MAIN_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <cstdio>
#include "Quaternion.hpp"
#include "Sensors.hpp"
#include "feature.h"
#include "states.h"
#include "view.hpp"
#include "imagesensor.hpp"

#define QBIAS 0.002            /*  */
#define P0 1            /*  */
#define DMIN 0.3            /*  */
using namespace cv;

void blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl );
void jacobianH( const cv::Vec3d& X, const Quaternion& qbw, const Feature& feat,
        Mat& Hb, Mat& Hi );
void jacobianMotionModel( const States& mu, const Sensors& sense, Mat& F_out );
void measurementModel( const cv::Vec3d& old_pos, double alt, const std::vector<projection>& matches,
        const Quaternion& qbw, View& meas, View& hmu, Mat& H, States *mu );
void initG ( cv::Mat& G, int nf, double dt );
void initQ ( cv::Mat& Q, int nf, double Q0 );
void initR ( cv::Mat& R, int nf, double R0 );
void calcP ( cv::Mat& P, const cv::Mat& F, const cv::Mat& G, const cv::Mat& Q );
void calcK ( cv::Mat& K, const cv::Mat& H, const cv::Mat& P, const cv::Mat& R );
void updateP( cv::Mat& P, const cv::Mat& K, const cv::Mat& H );
void resizeP( cv::Mat& P, int nf );

#endif
