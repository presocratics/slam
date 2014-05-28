#ifndef MAIN_H
#define MAIN_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace cv;

void loadData(vector<double>& pibHist, vector<double>& ppbHist, vector<double>& refFlag, vector<double>& renewHist);
void loadData(vector<double>& aHist, vector<double>& altHist, vector<double>& dtHist, vector<double>& qbwHist, vector<double>& rHist, vector<double>& wHist);
void copyMat(Mat& src, Mat& dst);
void reshapeMat(vector<double> src, Mat& dst);
void reshapeMat3D(vector<double> src, Mat& dst);
void quaternion2Rotation(Mat src, Mat& dst);
void quaternion2Euler(Mat src, Mat& dst);
void euler2Quaternion(Mat src, Mat& dst);
void jacobianH(Mat mu, Mat qbw, Mat xb0w, Mat qb0w, int i, Mat& Hb, Mat& Hi);
void motionModel(Mat mu, Mat qbw, Mat a, Mat w, Mat pibHat, int nf, double dt, Mat& f, Mat& F);
void measurementModel(int k, int nf, double alt, Mat pibHist, Mat pib0, Mat ppbHist, Mat mu, Mat qbw, Mat xb0wHat, Mat xbb0Hat, Mat qb0w, vector<Mat> Rb2b0, Mat refFlag, int flagMeas, Mat& meas, Mat& hmu, Mat& H, Mat& pibHat, Mat& xiwHat);


#endif