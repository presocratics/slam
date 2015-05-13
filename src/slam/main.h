#ifndef MAIN_H
#define MAIN_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <stdint.h>
#include <cstdio>
#include <sys/time.h>
#include "Quaternion.hpp"
#include "Sensors.hpp"
#include "feature.h"
#include "states.h"
#include "view.hpp"
#include "imagesensor.hpp"
#include "Sensor.hpp"

#define QBIAS 1e-5            /*  */
#define P0 2            /*  */
#define DMIN 0.3            /*  */

void blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl );
void jacobianMotionModel( const States& mu, const Sensors& sense, Mat& F_out, double dt );
void initG ( cv::Mat& G, int nf, double dt );
void initQ ( cv::Mat& Q, int nf, double Q0, double dt );
void initR ( cv::Mat& R, double R0, std::vector<int> refFlag );
void resizeP ( cv::Mat& P, int nf );
void calcP ( cv::Mat& P, const cv::Mat& F, const cv::Mat& G, const cv::Mat& Q );
void calcK ( cv::Mat& K, const cv::Mat& H, const cv::Mat& P, const cv::Mat& R );
void measurementModel( const cv::Vec3d& old_pos, double alt, const Quaternion& qbw,
        View& meas, View& hmu, Mat& H, States& mu );
void updateP ( cv::Mat& P, const cv::Mat& K, const cv::Mat& H );

#endif
