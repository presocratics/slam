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
//#include "feature.h"
#include "states.h"
//#include "view.hpp"
//#include "imagesensor.hpp"
#include "Sensor.hpp"

#define QBIAS 1e-5            /*  */
#define P0 2            /*  */
#define DMIN 0.3            /*  */

void blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl );
void jacobianMotionModel( const States& mu, const Sensors& sense, Mat& F_out, double dt );
void initG ( cv::Mat& G, int nf, double dt );
void initQ ( cv::Mat& Q, int nf, double Q0, double dt );

#endif
