#include "main.h"


//#include "ourerr.hpp"
#define PINIT 1e-4            /*  */
#define THRESH 2e-10           /*  */
using std::cout; 
using std::cerr; 
using std::endl;



int main()
{
    // Configuration
	const int nf = 5;                            /* number of features */
	const int stepEnd = 4340;
	const int stepStart = 1700;
	const int flagBias = 1;
    const int flagMap = 1;
	const double d_max = 30;
	const double d_min = 0.3;
	const double d_init = 5;

    // Initial variables
	double d0 = 1;
	int j = 1;
	// Covariance Initialization
	double P0 = 1;					// 1 for simulation
	double Q0 = 1;				// 100 for simulation & 1 for experiments
	double R0 = 10;					// 10 for simulation & 1 for experiments

    // Declarations
	Mat d0Hist(stepEnd, nf, CV_64F, Scalar(0));
    vector<cv::Vec3d> xb0wHat(nf);
    vector<cv::Vec3d> xiwHat(nf);
	Mat xiwHatHist(stepEnd, nf, CV_64FC3, Scalar(0));

    vector<Quaternion> qb0w(nf);
    vector<States> muHist;

    vector<Matx33d> Rw2b0(nf);
	vector<Matx33d> Rb2b0(nf);

    vector<cv::Vec3d> pib0(nf);
    vector<Vec3d> xbb0Hat(nf);

    Sensors sense;
    States mu;
	Mat PHist(mu.rows, stepEnd, CV_64F);


	// Data History variables
	std::vector<double> pibHist_v;
	std::vector<double> ppbHist_v;
	std::vector<double> refFlag_v;
	std::vector<double> renewHist_v;

	// Load experimental data (vision: 1700 ~ 4340 automatic reflection and shore features)
	loadData(pibHist_v, ppbHist_v, refFlag_v, renewHist_v);

	// Reshape to Matrix
	Mat pibHist = Mat::zeros(stepEnd, 5, CV_64FC3);
	Mat ppbHist = Mat::zeros(stepEnd, 5, CV_64FC3);
	Mat refFlag = Mat::zeros(5, stepEnd, CV_64F);
	Mat renewHist = Mat::zeros(5, stepEnd, CV_64F);
	
	reshapeMat3D(pibHist_v, pibHist);
	reshapeMat3D(ppbHist_v, ppbHist);
	reshapeMat(refFlag_v, refFlag);
	reshapeMat(renewHist_v, renewHist);

    sense.set_acceleration( "../data/aHistF.hex", true );
    sense.set_altitude( "../data/altHist.hex", true );
    sense.set_dt( "../data/dtHist.hex", true );
    sense.set_quaternion( "../data/qbwHistF.hex", true );
    sense.set_angular_velocity( "../data/wHistF.hex", true );
    sense.update();

	clock_t startTime = clock();

	// State initialization
    // mu = [pos vel features bias]
    //    = [X    V  features  b  ]
	mu.X[2] = -1 * sense.altitude;


	for (int i = 0; i < nf; i++)
	{
        Feature ith_feature(Vec3d(pibHist.at<Vec3d>(stepStart - 1, i)[0],
            pibHist.at<Vec3d>(stepStart - 1, i)[1], 1/d0), Scalar(0,0,0), 0, 0);
        mu.addFeature(ith_feature);
	}
	
	Mat P = Mat::eye(6 + 3 * nf,6 + 3 * nf, CV_64F);
    blockAssign( P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(0,0) );

	// Inverse depth
	for (int i = 0; i < nf; i++)
	{
		P.at<double>(5 + 3*(i+1), 5 + 3*(i+1)) = P0;
	}

	if (flagBias == 1)
	{
		Mat temp = P.clone();
		P.release();
		P = Mat::eye(6 + 3 * nf + 3, 6 + 3 * nf + 3, CV_64F);
		copyMat(temp, P);
		mu.setb(Vec3d(0,0,0));
        blockAssign( P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(6+3*nf,6+3*nf) );
	}

	for (int k = stepStart-1; k < stepEnd; k++)
	{
        vector<int> renewIndex;
        int renewk, renewi;
        vector<cv::Vec3d> pibHat(nf);
        Mat G, Q, R, K;	
        Mat H = Mat::zeros(31, mu.rows, CV_64F);			// Measurement model ouputs
        View meas(nf);
        View hmu(nf);
        double altHat, alt_old;
        Matx33d Rb2w, Rw2b;
        alt_old = sense.altitude;
        cv::Vec3d r;
        States f;
        Mat F = Mat::zeros(mu.rows, mu.rows, CV_64F);
		// Read sensor measurements
        sense.update();
		renewk = renewi = -1;

        //TODO: Do we need this norm?
        //double qbw_norm;
        //qbw_norm = norm(qbw);
        //qbw*=(1/qbw_norm);
		//normalize qbw  -(!) NEED FIX ==================================================
		//for (int i = 0; i < 4; i++)
		//{
		//	sums = pow(sums, 0.5);
		//	qbw.at<double>(i, 0) = qbw.at<double>(i, 0)/sums;
		//}
		//std::cout << "sums " << sums << std::endl;

        Rb2w = sense.quaternion.rotation();
		Rw2b = Rb2w.t();
        r = sense.quaternion.euler();
        r *= 180 / M_PI; 

		// line 59
		for (int i = 0; i < nf; i++)
		{
            int renewZero, renewZero2;
            cv::Vec3d pibr;
			add( atan2(pibHist.at<Vec3d>(k, i)[1], 1) * 180 / M_PI, r, pibr );
			d0 = -sense.altitude / sin(pibr[1] / 180 * M_PI) * 2;

			d0 = fmin(d0,d_init);

            d0Hist.at<double>(k, i) = 1 / mu.features[i].X[2];
			// Expreiment: renew elements are piecewise constant
			renewZero = renewHist.at<double>(i, k - 1);
			renewZero2 = renewHist.at<double>(i, k);

			if (k == stepStart-1 || renewHist.at<double>(i, k) != renewZero)
			{
                Matx33d tempR;
				renewIndex = findIndex(renewHist, renewHist.at<double>(i, k));

				// Find max
				int tempCol, tempRow;
				for (int q = 0; q < renewIndex.size(); q++)
				{
					tempRow = renewIndex[q] / stepEnd;
					tempCol = renewIndex[q] % stepEnd;
					if (tempCol < k && tempCol > renewk)
					{
						renewk = tempCol;
						renewi = tempRow;
					}
				}

				//cout << "renewk" << renewk << endl;
				// If current signature existed before
				if (renewk != -1 && k < stepEnd)
				{
					d0 = d0Hist.at<double>(renewk, renewi);
				}


				// Location and orientation of each anchor
                xb0wHat[i] = mu.X;


                qb0w[i].coord = sense.quaternion.coord;
                tempR = sense.quaternion.rotation();

				Rw2b0[i]=tempR.t();

                pib0[i] = pibHist.at<Vec3d>(k, i);
				++j;

				// Re-initialize the state for a new feature

                mu.features[i].X = Vec3d( pibHist.at<Vec3d>(k, i)[0], 
                        pibHist.at<Vec3d>(k, i)[1] ,   1 / d0);
			} // if k


			// Position of the feature w.r.t. the anchor
			xbb0Hat[i] = Rw2b0[i]*(mu.X - xb0wHat[i]);
			
			Rb2b0[i] = Rw2b0[i] * Rb2w;

			// Leaving the final estimate of each feature's location
			if (k < stepEnd - 1 && renewHist.at<double>(i, k + 1) != renewZero2 || k == stepEnd)
			{
				xiwHatHist.at<Vec3d>(j, i) = xiwHat[i];

				// Removing bad features
				if (mu.features[i].X[2] < 1. / 10 || mu.features[i].X[2] > 1 / d_min)
				{
					xiwHatHist.at<Vec3d>(j, i) = cv::Vec3d(0,0,0);
				}
			}

		} // i loop
		
		// Saving the history of the estimates
        muHist.push_back(mu);
		
		for (int i = 0; i < mu.rows; i++)
		{
			PHist.at<double>(i, k) = sqrt(P.at<double>(i, i));
		}

		// Extended Kalman Filter Prediction
		for (int i = 0; i < nf; i++)
		{
            pibHat[i] = mu.features[i].X;
		}

		// Motion model
		motionModel(mu, sense.quaternion, sense.acceleration, sense.angular_velocity,
                nf, sense.dt, f, F);

		if (flagBias == 1)
		{
			F.at<double>(6 + 3 * nf, 6 + 3 * nf) = 1;
			F.at<double>(7 + 3 * nf, 7 + 3 * nf) = 1;
			F.at<double>(8 + 3 * nf, 8 + 3 * nf) = 1;
			F.at<double>(3, 6 + 3 * nf) = -1*sense.dt;
			F.at<double>(4, 7 + 3 * nf) = -1 * sense.dt;
			F.at<double>(5, 8 + 3 * nf) = -1 * sense.dt;
            f.V -= mu.b;
            f.b = cv::Vec3d(0,0,0);
		}
        States fxdt;

        // TODO: Encapsulate this in a * operator?
        f.X*=sense.dt;
        f.V*=sense.dt;
        
        for(int i=0; i < nf; i++)
        {
            f.features[i].X*=sense.dt;
        }

        mu.add(f);

		// Measurement model
		measurementModel(k, nf, sense.altitude, pibHist, pib0, ppbHist, mu,
            sense.quaternion, xb0wHat, xbb0Hat, qb0w, Rb2b0, refFlag.col(k).t(),
            0, meas, hmu, H, pibHat, xiwHat);

		if (flagBias == 1)
		{
            cv::Rect tempRect;
            Mat tempMat1;
			tempRect = Rect(6+3*nf, 0, 3, H.rows);
			tempMat1 = H(tempRect);
			tempMat1.setTo(0);
		}

		altHat = meas.altitude;

		G = sense.dt*Mat::eye(6 + 3 * nf, 6 + 3 * nf, CV_64F);
		G.at<double>(0, 0) = 0.5*pow(sense.dt, 2);
		G.at<double>(1, 1) = 0.5*pow(sense.dt, 2);
		G.at<double>(2, 2) = 0.5*pow(sense.dt, 2);

		if (flagBias == 1)
		{
			G = sense.dt*Mat::eye(6 + 3 * nf + 3, 6 + 3 * nf + 3, CV_64F);
			G.at<double>(0, 0) = 0.5*pow(sense.dt, 2);
			G.at<double>(1, 1) = 0.5*pow(sense.dt, 2);
			G.at<double>(2, 2) = 0.5*pow(sense.dt, 2);
			G.at<double>(6 + 3 * nf, 6 + 3 * nf) = 0.5*pow(sense.dt, 2);
			G.at<double>(7 + 3 * nf, 7 + 3 * nf) = 0.5*pow(sense.dt, 2);
			G.at<double>(8 + 3 * nf, 8 + 3 * nf) = 0.5*pow(sense.dt, 2);
		}

		// for 2nd street data set
		if (k > stepStart - 1 && altHat - alt_old < -0.6)
		{
			Q0 = 20;
		}

		Q = Q0*Mat::eye(mu.rows, mu.rows, CV_64F);

		if (flagBias == 1)
		{
			Q.at<double>(6 + 3 * nf, 6 + 3 * nf) = 0.002;
			Q.at<double>(7 + 3 * nf, 7 + 3 * nf) = 0.002;
			Q.at<double>(8 + 3 * nf, 8 + 3 * nf) = 0.002;
		}

		R = 0.1 / 770 * R0*Mat::eye(1+6*nf, 1+6*nf, CV_64F);

		// altimeter noise covariance
		R.at<double>(0, 0) = 0.0001*R0;

		for (int i = 0; i < nf; i++)
		{
			// current view measurement noise covariance
			R.at<double>(1 + 6 * i, 1 + 6 * i) = 0.1 / 770 * R0;
			R.at<double>(2 + 6 * i, 2 + 6 * i) = 0.1 / 770 * R0;

			// initial view measurement noise covariance
			R.at<double>(3 + 6 * i, 3 + 6 * i) = 10. / 770 * R0;
			R.at<double>(4 + 6 * i, 4 + 6 * i) = 10. / 770 * R0;

			// reflection measurment noise covariance
			R.at<double>(5 + 6 * i, 5 + 6 * i) = 10. / 770 * R0;
			R.at<double>(6 + 6 * i, 6 + 6 * i) = 10. / 770 * R0;
		}

		// EKF measurement update

        cv::Mat tmp1, tmp2;
        cv::Mat Pcopy = P.clone();
        
        /* Put multiplication on multiple lines for less error */
        tmp1 = F*P;
        tmp1 *= F.t();
        tmp2 = G*Q;
        tmp2 *= G.t();

		P = tmp1 + tmp2;

        cv::Mat temppp=H*P;
        temppp *= H.t();
        temppp += R;

        K = P*H.t();
        cv::Mat K2= K*temppp.inv();

        K=K.t();
        temppp=temppp.t();
        solve(temppp, K, K);
        K=K.t();

        //K *= temppp.inv();
		// error Check Passed: P, F, Q, H, G, temppp, K
        View foo = meas-hmu;
        vector<double> bar;
        bar.push_back(foo.altitude);
        vector<Vfeat>::iterator it=foo.features.begin();
        for( ; it!=foo.features.end(); ++it )
        {
            bar.push_back(it->current.x);
            bar.push_back(it->current.y);
            bar.push_back(it->initial.x);
            bar.push_back(it->initial.y);
            bar.push_back(it->reflection.x);
            bar.push_back(it->reflection.y);
        }
        Mat baz(bar);
        Mat kx = K*baz;
        States kmh;
        
        kmh.setX(Vec3d( kx.at<double>(0,0), kx.at<double>(1,0), kx.at<double>(2,0)) );
        kmh.setV(Vec3d( kx.at<double>(3,0), kx.at<double>(4,0), kx.at<double>(5,0)) );
        for(int i=0; i < nf; i++)
        {
            Feature tempfeat( Vec3d(  kx.at<double>(6+3*i,0),  kx.at<double>(7+3*i,0),
                kx.at<double>(8+3*i,0)), Scalar(0,0,0), 0, 0 );
            kmh.addFeature(tempfeat);
        }
        kmh.setb(Vec3d(  kx.at<double>(6+3*nf,0),  kx.at<double>(7+3*nf,0),  kx.at<double>(8+3*nf,0)));

        mu.add(kmh);


		//mu = mu + K*(meas - hmu);
		P = (Mat::eye(mu.rows, mu.rows, CV_64F) - K*H)*P;
		P = (P.t() + P) / 2;


		if (k%300 == 0)
            cout << k << endl;

	} //  k loop

	cout << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " seconds." << endl;


	int plotFlag = 1;

	if (plotFlag == 1)
	{
		int w = 800;
		int h = 1200;
		double scaleW = 10;
		double scaleH = 10;
		Mat plot = Mat::zeros(w, h, CV_8UC3);
		for (int i = stepStart - 1; i < stepEnd; i++)
		{
			circle(plot, Point(muHist[i-stepStart+1].X[1]*scaleW+w/2,
                        h/2-(muHist[i-stepStart+1].X[0]*scaleH + h/4 )), 3, Scalar(0, 10, 220));
		}
		for (int i = 0; i < stepEnd; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				//Point(xiwHatHist.at<Vec3d>(i, j)[0], xiwHatHist.at<Vec3d>(i, j)[0])
				if (xiwHatHist.at<Vec3d>(i, j)[0] + xiwHatHist.at<Vec3d>(i, j)[1] != 0.)
				{
					circle(plot, Point(xiwHatHist.at<Vec3d>(i, j)[1] * scaleW + w/2,
                                h/2 - (xiwHatHist.at<Vec3d>(i , j)[0] * scaleH + h/4  )),
                            2, Scalar(220, 120, 0));
				}
			}
		}
		//line(plot, Point(15, 20), Point(70, 50), Scalar(110, 220, 0), 2, 8);
		imshow("drawing", plot);
		waitKey(0);
	}
	return 0;
}


/*************************** FUNCTIONS ********************************************/

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  blockAssign
 *  Description:  Writes a submatrix to dst at the starting point tl
 * =====================================================================================
 */
    void
blockAssign ( cv::Mat dst, cv::Mat block, cv::Point tl )
{
    cv::Rect roi;
    cv::Mat sub_dst;

    roi = cv::Rect(tl, block.size());
    sub_dst = dst(roi);
    block.copyTo(sub_dst);
    return;
}		/* -----  end of function blockAssign  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  ARC_compare
 *  Description:  
 * =====================================================================================
 */
    void
ARC_compare ( cv::Mat cmat, char *fn, double thresh )
{
    cv::Mat diff;
    cv::Mat mlabmat(cmat.size(), CV_64F );
    vector<double> v;
    double minVal, maxVal;
    hexToVec( fn, v ); 
    reshapeMat( v, mlabmat );
    absdiff( cmat, mlabmat, diff );
    minMaxLoc( diff, &minVal, &maxVal );
    cout << "Min err: " << minVal << " " << "Max err: " << maxVal << endl;
    if( maxVal>thresh ) 
    {
        cerr << "Error too large." << endl;
        cerr << diff << endl;
        exit(EXIT_FAILURE);
    }
    return ;
}		/* -----  end of function ARC_compare  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  hexToVec
 *  Description:  Reads hex values from an ifstream and writes to a vector.
 * =====================================================================================
 */
void hexToVec ( const char *fn, vector<double>& vec )
{

	long long * iv = new long long;
	double * dv = (double*)iv;

    FILE *fp;
    char *line;
    size_t sz=32;

    line = (char *) calloc(sz, sizeof(char) );
    if( line==NULL ) {
        fprintf( stderr, "\ndynamic memory allocation failed\n" );
        exit( EXIT_FAILURE );
    }

	if ((fp = fopen(fn, "r")) == NULL);

    while( fgets( line, sz, fp )!=NULL )
    {
		*iv = std::strtoull(line, NULL, 16);
		vec.push_back(*dv);
    }
    fclose(fp);
    free(line);
    line = NULL;
    return;
}		/* -----  end of function hexToVec  ----- */

/************************************************************************************************
* Reads txt file and outputs vector
*
**************************************************************************************************/
void loadData(vector<double>& pibHist, vector<double>& ppbHist, vector<double>& refFlag, vector<double>& renewHist)
{

	// fill pibHist
    hexToVec( "../data/pibHist.hex", pibHist );
    hexToVec( "../data/ppbHist.hex", ppbHist );
    hexToVec( "../data/refFlag.hex", refFlag );
    hexToVec( "../data/renewHist.hex", renewHist );
}

void loadData(vector<double>& aHist, vector<double>& altHist, vector<double>& dtHist, vector<double>& qbwHist, vector<double>& wHist)
{
    hexToVec( "../data/aHist.hex", aHist );
    hexToVec( "../data/altHist.hex", altHist );
    hexToVec( "../data/dtHist.hex", dtHist );
    hexToVec( "../data/qbwHist.hex", qbwHist );
    hexToVec( "../data/wHist.hex", wHist );
}

/************************************************************************************************
* Copy Matrix from src to dst.
**************************************************************************************************/
void copyMat(Mat& src, Mat& dst)
{
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			dst.at<double>(i, j) = src.at<double>(i, j);
		}
	}
}

/************************************************************************************************
* Reshapes vector to Mat
**************************************************************************************************/
void reshapeMat(vector<double> src, Mat& dst)
{
	int rows = dst.rows;
	int cols = dst.cols;
	for (int j = 0; j < cols; j++)
	{
		for (int i = 0; i < rows; i++)
		{
			dst.at<double>(i, j) = src[rows*j + i];
		}
	}
}

/************************************************************************************************
* Reshapes vector to Mat with 3 Channels
**************************************************************************************************/
void reshapeMat3D(vector<double> src, Mat& dst)
{
	int rows = dst.rows;
	int cols = dst.cols;

	for (int k = 0; k < cols; k++)
	{
		for (int j = 0; j < rows; j++)
		{
			for (int i = 0; i < 3; i++)
			{
				dst.at<Vec3d>(j, k)[i] = src[3 * (rows*k + j) + i];
			}
		}
	}
}

/************************************************************************************************
* JacobianH. Note: 'i' here should be 1 less 'i' in matlab
**************************************************************************************************/
void jacobianH(States mu, Quaternion qbw, cv::Vec3d xb0w, Quaternion qb0w, int i, Mat& Hb, Mat& Hi )
{
	double xbw1 = mu.X[0];
	double xbw2 = mu.X[1];
	double xbw3 = mu.X[2];
	double qbw1 = qbw.coord[0];
	double qbw2 = qbw.coord[1];
	double qbw3 = qbw.coord[2];
	double qbw4 = qbw.coord[3];

	double pib1 = mu.features[i].X[0];
	double pib2 = mu.features[i].X[1];
	double pib3 = mu.features[i].X[2];

	double xb0w1 = xb0w[0];
	double xb0w2 = xb0w[1];
	double xb0w3 = xb0w[2];
	double qb0w1 = qb0w.coord[0];
	double qb0w2 = qb0w.coord[1];
	double qb0w3 = qb0w.coord[2];
	double qb0w4 = qb0w.coord[3];

	Hb = (Mat_<double>(4, 6) <<
		-(2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
        (pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
        -(2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
        0 , 0, 0,
	-(2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
    (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
    (pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) - ((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
    0, 0, 0,
	0, 0,
    (4 * qbw1*qbw4 + 4 * qbw2*qbw3) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) + ((4 * qbw1*qbw3 - 4 * qbw2*qbw4)*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + ((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) , 2),
    0, 0, 0,
	0, 0,
    ((4 * qbw1*qbw3 - 4 * qbw2*qbw4)*((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) + (2 * qbw1*qbw4 - 2 * qbw2*qbw3)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) + (pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) , 2) - (2 * pow(qbw1 , 2) + 2 * pow(qbw2 , 2) - 2 * pow(qbw3 , 2) - 2 * pow(qbw4 , 2)) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))), 0, 0, 0);

	Hi = (Mat_<double>(4, 3) <<
		-((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / (pib3*((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) - (((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / (pib3*pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2)), (((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / (pib3*pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2)) - ((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / (pib3*((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)), (((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pow(pib3 , 2) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pow(pib3 , 2) + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pow(pib3 , 2)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) + ((((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pow(pib3 , 2) + (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pow(pib3 , 2) - (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pow(pib3 , 2))*(((2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(xb0w3 - xbw3) - (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(xb0w1 - xbw1) + (xb0w2 - xbw2)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) + pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w4)+2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w2)-2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
	-((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / (pib3*((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) - (((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / (pib3*pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2)), (((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / (pib3*pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2)) - ((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / (pib3*((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)), ((pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pow(pib3 , 2) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pow(pib3 , 2) + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pow(pib3 , 2)) / ((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) + ((((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pow(pib3 , 2) + (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pow(pib3 , 2) - (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pow(pib3 , 2))*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(xb0w2 - xbw2) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(xb0w1 - xbw1) - ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4)) / pib3 + (xb0w3 - xbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (pib1*((2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw2 - 2 * qbw3*qbw4))) / pib3 + (pib2*((pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))*(pow((qb0w1) , 2) + pow((qb0w2) , 2) - pow((qb0w3) , 2) - pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w3)+2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 + 2 * qbw2*qbw4) + (2 * (qb0w1)*(qb0w4)-2 * (qb0w2)*(qb0w3))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3)) / pow(((2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(xb0w2 - xbw2) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(xb0w3 - xbw3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw2 + 2 * qbw3*qbw4) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw3 - 2 * qbw2*qbw4)) / pib3 + (xb0w1 - xbw1)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (pib1*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) - (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)) + (2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(2 * qbw1*qbw4 + 2 * qbw2*qbw3))) / pib3 + (pib2*((2 * (qb0w1)*(qb0w3)-2 * (qb0w2)*(qb0w4))*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2)) - (2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow((qb0w1) , 2) - pow((qb0w2) , 2) - pow((qb0w3) , 2) + pow((qb0w4) , 2)) + (2 * (qb0w1)*(qb0w2)+2 * (qb0w3)*(qb0w4))*(2 * qbw1*qbw4 - 2 * qbw2*qbw3))) / pib3) , 2),
	((((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - ((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) / pib3 + ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3)*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + ((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) , 2) - (pow((2 * qbw1*qbw2 - 2 * qbw3*qbw4) , 2) / pib3 - pow( (2 * qbw1*qbw4 + 2 * qbw2*qbw3) , 2) / pib3 + pow( (pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)), 2) / pib3) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))), -(((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + ((2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + ((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) - ((((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + ((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) / pib3 - ((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + ((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) , 2), ((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pow(pib3 , 2) + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pow(pib3 , 2) + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pow(pib3 , 2)) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*((2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pow(pib3 , 2) - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pow(pib3 , 2) + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pow(pib3 , 2)) + ((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pow(pib3 , 2) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pow(pib3 , 2) + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pow(pib3 , 2))*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) - (((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) - (2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + ((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2)))*((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pow(pib3 , 2) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pow(pib3 , 2) + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pow(pib3 , 2)) + (2 * qbw1*qbw3 - 2 * qbw2*qbw4)*((2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pow(pib3 , 2) - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pow(pib3 , 2) + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pow(pib3 , 2)) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pow(pib3 , 2) + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pow(pib3 , 2) + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pow(pib3 , 2))*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1 , 2) + pow(qbw2 , 2) - pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1 , 2) - pow(qbw2 , 2) + pow(qbw3 , 2) - pow(qbw4 , 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1 , 2) - pow(qbw2 , 2) - pow(qbw3 , 2) + pow(qbw4 , 2))) , 2),
	((((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - ((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))) / pib3 + ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3)*((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) + (2 * qbw1*qbw4 - 2 * qbw2*qbw3)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) + (pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))), 2) - (((2 * qbw1*qbw4 - 2 * qbw2*qbw3)*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + ((2 * qbw1*qbw4 + 2 * qbw2*qbw3)*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + ((2 * qbw1*qbw2 - 2 * qbw3*qbw4)*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))), -(pow((2 * qbw1*qbw3 + 2 * qbw2*qbw4), 2) / pib3 + pow((2 * qbw1*qbw4 - 2 * qbw2*qbw3), 2) / pib3 - pow((pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2)) , 2) / pib3) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))) - ((((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + ((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))) / pib3 - ((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3)*((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) + (2 * qbw1*qbw4 - 2 * qbw2*qbw3)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) + (pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))), 2), ((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pow(pib3, 2) + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pow(pib3, 2) + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pow(pib3, 2)) + (2 * qbw1*qbw4 - 2 * qbw2*qbw3)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pow(pib3, 2) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pow(pib3, 2) + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pow(pib3, 2)) + ((2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pow(pib3, 2) - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pow(pib3, 2) + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pow(pib3, 2))*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / ((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))) - (((2 * qbw1*qbw3 + 2 * qbw2*qbw4)*((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3) + (2 * qbw1*qbw4 - 2 * qbw2*qbw3)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) + (pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3))*((2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pow(pib3, 2) - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pow(pib3, 2) + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pow(pib3, 2)) + (2 * qbw1*qbw3 - 2 * qbw2*qbw4)*((2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pow(pib3, 2) - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pow(pib3, 2) + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pow(pib3, 2)) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pow(pib3, 2) + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pow(pib3, 2) + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pow(pib3, 2))*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)))) / pow(((2 * qbw1*qbw3 - 2 * qbw2*qbw4)*(2 * xbw3 + (2 * qbw1*qbw3 - 2 * qbw2*qbw4) / pib3 - (pib2*(pow(qbw1, 2) + pow(qbw2, 2) - pow(qbw3, 2) - pow(qbw4, 2))) / pib3 + (pib1*(2 * qbw1*qbw4 + 2 * qbw2*qbw3)) / pib3) + (2 * qbw1*qbw2 + 2 * qbw3*qbw4)*((pib1*(pow(qbw1, 2) - pow(qbw2, 2) + pow(qbw3, 2) - pow(qbw4, 2))) / pib3 - (2 * qbw1*qbw2 + 2 * qbw3*qbw4) / pib3 + (pib2*(2 * qbw1*qbw4 - 2 * qbw2*qbw3)) / pib3) - ((pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2)) / pib3 + (pib1*(2 * qbw1*qbw2 - 2 * qbw3*qbw4)) / pib3 + (pib2*(2 * qbw1*qbw3 + 2 * qbw2*qbw4)) / pib3)*(pow(qbw1, 2) - pow(qbw2, 2) - pow(qbw3, 2) + pow(qbw4, 2))), 2));

}

void motionModel(States mu, Quaternion qbw, cv::Vec3d a, cv::Vec3d w, int nf,
        double dt, States& f, Mat& F_out)
{

    f = mu.dynamics( qbw, a, w );

    Mat Fb = (Mat_<double>(6, 6) << 0, 0, 0, 
            pow(qbw.coord[0], 2) - pow(qbw.coord[1], 2) - pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            2 * qbw.coord[0]*qbw.coord[1] - 2 * qbw.coord[2]*qbw.coord[3],
            2 * qbw.coord[0]*qbw.coord[2] + 2 * qbw.coord[1]*qbw.coord[3],
            0, 0, 0,
            2 * qbw.coord[0]*qbw.coord[1] + 2 * qbw.coord[2]*qbw.coord[3],
            - pow(qbw.coord[0] , 2) + pow(qbw.coord[1] , 2) - pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            2 * qbw.coord[1]*qbw.coord[2] - 2 * qbw.coord[0]*qbw.coord[3],
            0, 0, 0,
            2 * qbw.coord[0]*qbw.coord[2] - 2 * qbw.coord[1]*qbw.coord[3],
            2 * qbw.coord[0]*qbw.coord[3] + 2 * qbw.coord[1]*qbw.coord[2],
            -pow(qbw.coord[0] , 2) - pow(qbw.coord[1] , 2) + pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            0, 0, 0, 0, w[2], -w[1],
            0, 0, 0, -w[2], 0, w[0],
            0, 0, 0, w[1], -w[0], 0);


    Mat Fi = Mat::zeros(nf*3, nf*3, CV_64F);
    Mat FiTemp;
    Mat Fib = Mat::zeros(nf*3, 6, CV_64F);
    Mat Fib_ith;
    Mat Fi_ith = Mat::zeros(3,nf*3,CV_64F);
    Mat Fi_ith_1;
    Mat Fi_ith_2;
    Mat Fi_ith_3;

    for (int i = 0; i < nf; i++)
    {
        Matx13d pib( 
            mu.features[i].X[0],
            mu.features[i].X[1],
            mu.features[i].X[2]
        );
        double pib1 = mu.features[i].X[0];
        double pib2 = mu.features[i].X[1];
        double pib3 = mu.features[i].X[2];


        FiTemp = (Mat_<double>(3, 3) <<
                    (pib * Matx31d( mu.V[2], w[0], -2*w[1]))(0,0),
                    w[2] + pib1*w[0],
                    pib1*mu.V[2] - mu.V[0],
                    -w[2] - pib2*w[1], 
                    pib3*mu.V[2] - pib1*w[1] + 2 * pib2*w[0],
                    pib2*mu.V[2] - mu.V[1],
                    -pib3*w[1],
                    pib3*w[0],
                    2 * pib3*mu.V[2] - pib1*w[1] + pib2*w[0]);
        // work on Fib
        Fib_ith = (Mat_<double>(3, 6) <<
                    0, 0, 0, -pib3, 0, pib1*pib3,
                    0, 0, 0, 0, -pib3, pib2*pib3,
                    0, 0, 0, 0, 0, pow(pib3, 2));

        blockAssign(Fib, Fib_ith, Point(0, 3*i));


        // work on Fi

        // work on Fi_ith -> LHS of line 40
        Fi_ith_1 = Mat::zeros(3, 3 * (i), CV_64F);
        Fi_ith_2 = FiTemp;
        Fi_ith_3 = Mat::zeros(3, 3 * (nf-i-1), CV_64F);
        blockAssign(Fi_ith, Fi_ith_1, Point(0,0));
        blockAssign(Fi_ith, Fi_ith_2, Point(Fi_ith_1.cols,0));
        blockAssign(Fi_ith, Fi_ith_3,
        Point(Fi_ith_1.cols+FiTemp.cols,0));
        blockAssign(Fi, Fi_ith, Point(0,3*i));
    }
    Mat temp1 = Mat::eye(mu.rows, mu.rows, CV_64F);
    F_out.setTo(0);
    blockAssign(F_out, Fb, Point(0,0));
    blockAssign(F_out, Fib, Point(0,Fb.rows));
    blockAssign(F_out, Fi,Point(Fib.cols,Fb.rows));
    F_out = dt*F_out + temp1;
}

/************************************************************************************************
* measurementModel
* assumes output matrix to be initialized to 0.
**************************************************************************************************/
void measurementModel(int k, int nf, double alt, Mat pibHist, vector<cv::Vec3d> pib0,
    Mat ppbHist, States mu, Quaternion qbw, vector<cv::Vec3d> xb0wHat, vector<cv::Vec3d> xbb0Hat,
    vector<Quaternion> qb0w, vector<cv::Matx33d> Rb2b0, Mat refFlag, int flagMeas,
    View& meas, View& hmu, Mat& H, vector<cv::Vec3d>& pibHat, vector<cv::Vec3d>& xiwHat)
{
    H=cv::Mat::zeros(H.size(),CV_64F);
	Mat n = (Mat_<double>(3, 1) << 0, 0, 1);
	Mat S = Mat::eye(3, 3, CV_64F) - 2 * n * n.t();

    cv::Matx33d Rb2w, Rw2b;
    Rb2w = qbw.rotation();
	Rw2b = Rb2w.t();

	Mat Hb;
	Mat Hi;
    std::vector<Vfeat>::iterator mi=meas.features.begin(), hi=hmu.features.begin();
	for (int i = 0; i < nf; i++, ++mi, ++hi)
	{
        cv::Vec3d pib0Hat, ppbHat, xibHat, xib0Hat, xpbHat;

        pibHat[i] = mu.features[i].X;
	
		xibHat = cv::Vec3d(
				1 / pibHat[i][2],
				pibHat[i][0] / pibHat[i][2],
				pibHat[i][1] / pibHat[i][2]);

        Mat temp;
        temp =  (Mat)xbb0Hat[i] + (cv::Mat)Rb2b0[i] * (cv::Mat)xibHat;
        xib0Hat = (cv::Vec3d) temp;

        pib0Hat = cv::Vec3d(
				xib0Hat[1] / xib0Hat[0],
				xib0Hat[2] / xib0Hat[0],
                0);

        temp = S * (Mat)Rb2w*(Mat)xibHat;
        temp -= 2*n*n.t()*(Mat)mu.X; 
        temp = (Mat)Rw2b * temp;
        xpbHat = (cv::Vec3d) temp;

		ppbHat = cv::Vec3d(
			xpbHat[1] / xpbHat[0],
			xpbHat[2] / xpbHat[0],
            0);

        add(mu.X,(Mat)Rb2w*(Mat)xibHat, xiwHat[i] );

		jacobianH(mu, qbw, xb0wHat[i], qb0w[i], i, Hb, Hi);


		// 0: all
		if (flagMeas == 0)
		{
			meas.altitude = alt;							// altitude
            mi->set_views( pibHist.at<Vec3d>(k,i), pib0[i], ppbHist.at<Vec3d>(k,i) );
            hmu.altitude = -mu.X[2];
            hi->set_views( pibHat[i], pib0Hat, ppbHat );

			H.row(0).col(2).setTo(-1);
            // For each feature
            Mat Hfeat = Mat::zeros(6,24,CV_64F);
            blockAssign( Hfeat, Mat::eye(2,2,CV_64F), Point(6+3*i,0) );
            blockAssign( Hfeat, Hb, Point(0,2) );
            blockAssign( Hfeat, Hi, Point(6+3*i,2) );
            
            blockAssign( H, Hfeat, Point(0,1+6*i) );

			// features without reflection
			if (refFlag.at<double>(0, i) == 0)
			{
				meas.features[i].reflection = cv::Point2d(0,0);
			}
		}
    } // end for loop

	/*Remove Unavailable reflection measurements */

	// NEED TO CONVERT
	//hmu = hmu(~ismember(1:size(hmu, 1), find(meas == 0)), :);
	//H = H(~ismember(1:size(H, 1), find(meas == 0)), :);
	//meas = meas(~ismember(1:size(meas, 1), find(meas == 0)), :);
	
}

/************************************************************************************************
* Find indexes of elements equal to val in src: 1D array
**************************************************************************************************/
vector<int> findIndex(const Mat& src, double val)
{
	vector<int> out;
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			if (src.at<double>(i, j) == val)
			{
				out.push_back(src.cols*i + j);
			}
		}
	}
	return out;
}
