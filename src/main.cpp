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
    const bool fb=true;
    const int flagMap = 1;
	const double d_max = 30;
	const double d_min = 0.3;
	const double d_init = 5;

    // Initial variables
	double d0 = 1;
	// Covariance Initialization
	double P0 = 1;					// 1 for simulation
	double Q0 = 1;				// 100 for simulation & 1 for experiments
	double R0 = 10;					// 10 for simulation & 1 for experiments

    // Declarations
	Mat d0Hist(stepEnd, nf, CV_64F, Scalar(0));

    ImageSensor imgsense("../data/bodyHist2.hex", true );
    Sensors sense;
    States mu;
    mu.features.resize(nf);


	// Data History variables

	// Load experimental data (vision: 1700 ~ 4340 automatic reflection and shore features)

	// Reshape to Matrix
	
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

    double scaleW = 10;
    double scaleH = 10;
    int width=800;
    int height=800;
    Mat rtplot = Mat::zeros(width, height, CV_8UC3);
	for (int k = stepStart; k < stepEnd; k++)
	{
        cv::Vec3d old_pos;
        Mat G, Q, R, K, H;	
        Mat kx, eeMat;
        View meas(nf), hmu(nf);
        View estimateError(nf);
        double altHat, alt_old;
        States f, kmh;

        alt_old = sense.altitude;
        Mat F = Mat::zeros(mu.getRows(), mu.getRows(), CV_64F);
		// Update sensors
        sense.update();
        imgsense.update();
        mu.update_features( &imgsense, sense );

        old_pos = mu.X; // Need this for fromAnchor in measurementModel

        f = mu.dynamics( sense, fb );           // Motion model
        f*=sense.dt;
		jacobianMotionModel(mu, sense, F, fb );
        mu+=f;

		measurementModel( old_pos, sense.altitude, imgsense.matches, 
                sense.quaternion, 0, fb, meas, hmu, H, mu );

		altHat = meas.altitude;

        initG( G, nf, sense.dt, fb);
        if( k>stepStart && altHat-alt_old<-0.6 )
        {
            Q0 = 20;
        }
        initQ( Q, nf, Q0, fb );
        initR( R, nf, R0 );

		// EKF measurement update
        calcP( P, F, G, Q );
        calcK( K, H, P, R );
        updateP( P, K, H );

        estimateError = meas-hmu;
        estimateError.toMat(eeMat);
        kx = K*eeMat;

        kmh = States(kx);
        mu+=kmh;



		if (k%300 == 0)
            cout << k << endl;

        Fiter feat=mu.features.begin();
        for( int i=0; feat!=mu.features.end(); ++feat, ++i )
        {
            d0Hist.at<double>(k, i) = feat->position.body[2];
        }
        // Real time plotting.
        circle(rtplot, Point(mu.X[1]*scaleW+width/2,
            height/2-(mu.X[0]*scaleH + height/4 )), 3, Scalar(0, 10, 220));
        mu.end_loop();

	} //  k loop
    for( featIter fi=mu.feats.begin(); fi!=mu.feats.end(); ++fi )
    {
        if( fi->second.position.body[2]>1./10 &&
            fi->second.position.body[2]<1/d_min )
        {
            circle( rtplot, Point(fi->second.position.world[1] * scaleW + width/2,
                        height/2 - (fi->second.position.world[0] * scaleH + height/4  )),
                    3, Scalar(0, 120, 0));
        }

    }

	cout << double(clock() - startTime) / (double)CLOCKS_PER_SEC << " seconds." << endl;

    imshow("drawing", rtplot);
    waitKey(0);
	return 0;
}


/*************************** FUNCTIONS ********************************************/

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  updateP
 *  Description:  
 * =====================================================================================
 */
    void
updateP ( cv::Mat& P, cv::Mat K, cv::Mat H )
{
    Mat kh = K*H;
    P = (Mat::eye(kh.size(), CV_64F) - K*H)*P;
    P = (P.t() + P) / 2;
    return;
}		/* -----  end of function updateP  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcK
 *  Description:  
 * =====================================================================================
 */
    void
calcK ( cv::Mat& K, cv::Mat H, cv::Mat P, cv::Mat R )
{
    cv::Mat tmp=H*P;
    tmp *= H.t();
    tmp += R;

    K = P*H.t();
    cv::Mat K2= K*tmp.inv();

    K=K.t();
    tmp=tmp.t();
    solve(tmp, K, K);
    K=K.t();
    return;
}		/* -----  end of function calcK  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcP
 *  Description:  
 * =====================================================================================
 */
    void
calcP ( cv::Mat& P, cv::Mat F, cv::Mat G, cv::Mat Q )
{
    cv::Mat tmp1, tmp2;
    /* Put multiplication on multiple lines for less error */
    tmp1 = F*P;
    tmp1 *= F.t();
    tmp2 = G*Q;
    tmp2 *= G.t();
    P = tmp1 + tmp2;
    return;
}		/* -----  end of function calcP  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initR
 *  Description:  
 * =====================================================================================
 */
    void
initR ( cv::Mat& R, int nf, double R0 )
{
    // for 2nd street data set
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
    return;
}		/* -----  end of function initR  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initQ
 *  Description:  
 * =====================================================================================
 */
    void
initQ ( cv::Mat& Q, int nf, double Q0, bool flagbias )
{
    Q = Q0*Mat::eye(6+3*nf+3, 6+3*nf+3, CV_64F);
    if( flagbias==true )
    {
        Q.at<double>(6 + 3 * nf, 6 + 3 * nf) = 0.002;
        Q.at<double>(7 + 3 * nf, 7 + 3 * nf) = 0.002;
        Q.at<double>(8 + 3 * nf, 8 + 3 * nf) = 0.002;
    }
    return;
}		/* -----  end of function initq  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initG
 *  Description:  
 * =====================================================================================
 */
    void
initG ( cv::Mat& G, int nf, double dt, bool flagbias )
{
    G = dt*Mat::eye(6 + 3 * nf, 6 + 3 * nf, CV_64F);
    G.at<double>(0, 0) = 0.5*dt*dt;
    G.at<double>(1, 1) = 0.5*dt*dt;
    G.at<double>(2, 2) = 0.5*dt*dt;
    if (flagbias == true)
    {
        G = dt*Mat::eye(6 + 3 * nf + 3, 6 + 3 * nf + 3, CV_64F);
        G.at<double>(0, 0) = 0.5*dt*dt;
        G.at<double>(1, 1) = 0.5*dt*dt;
        G.at<double>(2, 2) = 0.5*dt*dt;
        G.at<double>(6 + 3 * nf, 6 + 3 * nf) = 0.5*dt*dt;
        G.at<double>(7 + 3 * nf, 7 + 3 * nf) = 0.5*dt*dt;
        G.at<double>(8 + 3 * nf, 8 + 3 * nf) = 0.5*dt*dt;
    }
    return;
}		/* -----  end of function initG  ----- */
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

	double pib1 = mu.features[i].position.body[0];
	double pib2 = mu.features[i].position.body[1];
	double pib3 = mu.features[i].position.body[2];

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

void jacobianMotionModel(States mu, Sensors sense, Mat& F_out, bool flagbias )
{
    int nf;
    Quaternion qbw;
    double dt;
    cv::Vec3d w;

    qbw = sense.quaternion;
    dt = sense.dt;
    w=sense.angular_velocity;
    nf=mu.getNumFeatures();

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

    for (int i = 0; i<nf; i++)
    {
        Matx13d pib( 
            mu.features[i].position.body[0],
            mu.features[i].position.body[1],
            mu.features[i].position.body[2]
        );
        double pib1 = mu.features[i].position.body[0];
        double pib2 = mu.features[i].position.body[1];
        double pib3 = mu.features[i].position.body[2];


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
    Mat temp1 = Mat::eye(mu.getRows(), mu.getRows(), CV_64F);
    F_out.setTo(0);
    blockAssign(F_out, Fb, Point(0,0));
    blockAssign(F_out, Fib, Point(0,Fb.rows));
    blockAssign(F_out, Fi,Point(Fib.cols,Fb.rows));
    F_out = dt*F_out + temp1;
    if( flagbias==true )
    {
        F_out.at<double>(6 + 3 * nf, 6 + 3 * nf) = 1;
        F_out.at<double>(7 + 3 * nf, 7 + 3 * nf) = 1;
        F_out.at<double>(8 + 3 * nf, 8 + 3 * nf) = 1;
        F_out.at<double>(3, 6 + 3 * nf) = -1*dt;
        F_out.at<double>(4, 7 + 3 * nf) = -1 * dt;
        F_out.at<double>(5, 8 + 3 * nf) = -1 * dt;
    }
}

/************************************************************************************************
* measurementModel
* assumes output matrix to be initialized to 0.
**************************************************************************************************/
void measurementModel( cv::Vec3d old_pos, double alt, std::vector<projection> matches,
        Quaternion qbw, int flagMeas, bool flagbias, View& meas, View& hmu, Mat& H, States& mu )
{
    int nf;
    nf=mu.getNumFeatures();

    H=cv::Mat::zeros(6*nf+1,mu.getRows(),CV_64F);
	Mat n = (Mat_<double>(3, 1) << 0, 0, 1);
	Mat S = Mat::eye(3, 3, CV_64F) - 2 * n * n.t();

	Mat Hb;
	Mat Hi;

    std::vector<projection>::iterator match=matches.begin();
    std::vector<Feature>::iterator feat=mu.features.begin();
    std::vector<Vfeat>::iterator mi=meas.features.begin(), hi=hmu.features.begin();
	for (int i = 0; i < nf; i++, ++mi, ++hi, ++feat, ++match)
	{
        cv::Vec3d pib0Hat, ppbHat, xibHat, xib0Hat, xpbHat, pibHat;

        pibHat = mu.features[i].position.body;
	
		xibHat = cv::Vec3d(
				1 / pibHat[2],
				pibHat[0] / pibHat[2],
				pibHat[1] / pibHat[2]);

        Mat temp;
        temp =  (Mat)feat->fromAnchor(old_pos) + (cv::Mat)feat->rb2b(qbw) * (cv::Mat)xibHat;
        xib0Hat = (cv::Vec3d) temp;

        pib0Hat = cv::Vec3d(
				xib0Hat[1] / xib0Hat[0],
				xib0Hat[2] / xib0Hat[0],
                0);

        temp = S * (Mat)qbw.rotation()*(Mat)xibHat;
        temp -= 2*n*n.t()*(Mat)mu.X; 
        temp = (Mat)qbw.rotation().t() * temp;
        xpbHat = (cv::Vec3d) temp;

		ppbHat = cv::Vec3d(
			xpbHat[1] / xpbHat[0],
			xpbHat[2] / xpbHat[0],
            0);

        add(mu.X,(Mat)qbw.rotation()*(Mat)xibHat, mu.features[i].position.world );

		jacobianH(mu, qbw, mu.features[i].initial.anchor, mu.features[i].initial.quaternion, i, Hb, Hi);


		// 0: all
		if (flagMeas == 0)
		{
			meas.altitude = alt;							// altitude
            mi->set_views( match->source, feat->initial.pib, match->reflection );
            hmu.altitude = -mu.X[2];
            hi->set_views( pibHat, pib0Hat, ppbHat );

			H.row(0).col(2).setTo(-1);
            // For each feature
            Mat Hfeat = Mat::zeros(6,24,CV_64F);
            blockAssign( Hfeat, Mat::eye(2,2,CV_64F), Point(6+3*i,0) );
            blockAssign( Hfeat, Hb, Point(0,2) );
            blockAssign( Hfeat, Hi, Point(6+3*i,2) );
            
            blockAssign( H, Hfeat, Point(0,1+6*i) );
		}
    } // end for loop
    if (flagbias==true)
    {
        cv::Rect tempRect;
        Mat tempMat1;
        tempRect = Rect(6+3*nf, 0, 3, H.rows);
        tempMat1 = H(tempRect);
        tempMat1.setTo(0);
    }

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
