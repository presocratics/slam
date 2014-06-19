#ifndef MAIN_H
#define MAIN_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <cstdio>
#include "Quaternion.hpp"
#include "Sensors.hpp"
#include "feature.h"
#include "states.h"
using namespace cv;

void blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl );
void ARC_compare ( cv::Mat cmat, char *fn, double thresh=0 );
void hexToVec ( const char *fn, vector<double>& vec );
void loadData(vector<double>& pibHist, vector<double>& ppbHist, vector<double>& refFlag, vector<double>& renewHist);
void loadData(vector<double>& aHist, vector<double>& altHist, vector<double>& dtHist, vector<double>& qbwHist, vector<double>& wHist);
void copyMat(Mat& src, Mat& dst);
void reshapeMat(vector<double> src, Mat& dst);
void reshapeMat3D(vector<double> src, Mat& dst);
void euler2Quaternion(Mat src, Mat& dst);
void jacobianH(States mu, Quaternion qbw, cv::Vec3d xb0w, Quaternion qb0w, int i, Mat& Hb, Mat& Hi, int k);
void motionModel(States mu, Quaternion qbw, cv::Vec3d a, cv::Vec3d w, Mat pibHat, int nf, double dt, States& f, Mat& F);
void measurementModel(int k, int nf, double alt, Mat pibHist, Mat pib0,
        Mat ppbHist, States mu, Quaternion qbw, vector<cv::Vec3d> xb0wHat, Mat xbb0Hat,
        vector<Quaternion> qb0w, vector<Matx33d> Rb2b0, Mat refFlag, int flagMeas, Mat& meas,
        Mat& hmu, Mat& H, Mat& pibHat, vector<Vec3d>& xiwHat);
vector<int> findIndex(const Mat& src, double val);

#endif
