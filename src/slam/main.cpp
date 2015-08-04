/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  SLAM algorithm for reflection code.
 *          Input should be files/FIFO in the following format:
 *          SENSORS:
 *                  altitude: float
 *              acceleration: float,float,float
 *                        dt: float
 *                quaternion: float,float,float,float
 *          angular velocity: float,float,float
 *
 *          IMAGE:
 *          ID1,X1,Y1,X2,Y2
 *          ...
 *          IDN,X1,Y1,X2,Y2
 *          \n (end of timestep denoted by newline)
 *
 *          Output to STDOUT:
 *          X,V
 *          ID1,X,Y,Z
 *          ...
 *          IDN,X,Y,Z
 *          \n (end of timestep denoted by newline)
 *
 *        Version:  1.0
 *        Created:  07/07/2014 05:45:59 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (MHM), miller7@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */

#include "main.h"
#include <iostream>
#include <fstream>

//#include "ourerr.hpp"
using std::cout; 
using std::cerr; 
using std::endl;
using std::vector;
using cv::Mat;
using cv::Rect;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  readMat
 *  Description:  
 * =====================================================================================
 */
    void
hexToVec ( const char *fn, vector<double>& out )
{
    union u_tag {
        uint64_t ival;
        double dval;
    } uval;

    FILE *fp;
    char *line;
    size_t sz=32;

    line = (char *) calloc(sz, sizeof(char) );
    if (line==NULL) {
        fprintf(stderr, "\ndynamic memory allocation failed\n" );
        exit(EXIT_FAILURE);
    }

    if ((fp=fopen(fn, "r"))==NULL)
        err_sys("fopen");

    while (fgets(line, sz, fp)!=NULL) {
        sscanf(line, "%lx", &uval.ival);
        out.push_back(uval.dval);
    }
    fclose(fp);
    free (line);
    line	= NULL;


    return ;
}		/* -----  end of function readMat  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  help
 *  Description:  
 * =====================================================================================
 */
    void
help ( const char *str)
{
    fprintf(stderr, "Usage: %s [measurements] < sensors\n", str);
    return;
}		/* -----  end of function help  ----- */

int main( int argc, char **argv )
{
    double Q0,R0,P0=1e-5;
    /* Handle input */
    if (argc<2 || !strcmp(argv[1],"-h")) {
        help(argv[0]);
        exit(EXIT_FAILURE);
    }
    if (argc>2) {
        Q0=atof(argv[2]);
        R0=atof(argv[3]);
        P0=atof(argv[4]);
    }

    /* Initialize */
    States mu(P0);
    Sensors sense;
    ImageSensor imgsense( argv[1], false );

    cv::Mat P=1e-3*cv::Mat::eye(9,9,CV_64F);
    blockAssign(P, 1e-6*cv::Mat::eye(3,3,CV_64F), cv::Point(6,6));

    /* Set initial conditions */
    cv::Vec3d old_pos;
    int u;

    mu.setb(cv::Vec3d(0,0,0));
    u=sense.update();
    if (u & UPDATE_POS) {
        cv::Vec3d pos=sense.pos.get_value();
        mu.X[0]=pos[0];
        mu.X[1]=pos[1];
        mu.X[2]=pos[2];
    }
    if ( u & UPDATE_VELB) {
        mu.V=sense.velb.get_value();
    }
    if (u & UPDATE_VEL ) {
        Matx33d Rb2w, Rw2b; 
        Rb2w = sense.quat.get_value().rotation();
        Rw2b = Rb2w.t();
        gemm(Rw2b,sense.vel.get_value(),1,Mat(),0,mu.V);
    }
    if (!(u & UPDATE_IMG)) {
        fprintf(stderr, "No image measurement at first time step.\n");
        exit(EXIT_FAILURE);
    }
    imgsense.update();
    mu.update_features(imgsense, sense,P);
    old_pos=mu.X;

    /* Enter main loop */
    int iter=0;
    int meascount=0;
    int nf;
    Mat plot=Mat::zeros(400,400,CV_8UC1);
    while (u!=-1) {
        double dt;
        cv::Mat G, Q, R, K, H, F;
        Mat kx, eeMat;
        States f, kmh;
        View meas, hmu, estimateError;
        
        u=sense.update();
        if (u & UPDATE_ACC) dt=sense.acc.get_dt();
        else if (u & UPDATE_ANG) dt=sense.ang.get_dt();
        else if (u & UPDATE_QUAT) dt=sense.quat.get_dt();
        else if (u & UPDATE_ALT) dt=sense.alt.get_dt();
        else if (u==0)
        {
            fprintf(stderr, "No update, so what are we doing here?\n");
            exit(EXIT_FAILURE);
        }
        if (u & UPDATE_IMG ) {
            old_pos=mu.X;
            imgsense.update();
            meascount++;
            // Read in new features
            mu.update_features(imgsense, sense, P);
            
            nf=mu.getNumFeatures();
        }
        // TODO Lake of Woods only
        sense.alt.set_value(-1.34);
        nf=mu.getNumFeatures();
        
        if (u & UPDATE_POS) {
            cv::Vec3d pos=sense.pos.get_value();
            mu.X[0]=pos[0];
            mu.X[1]=pos[1];
            mu.X[2]=pos[2];
        }
        if (u & UPDATE_VEL ) {
            Matx33d Rb2w, Rw2b; 
            Rb2w = sense.quat.get_value().rotation();
            Rw2b = Rb2w.t();
            gemm(Rw2b,sense.vel.get_value(),1,Mat(),0,mu.V);
        }
        if (u & UPDATE_VELB ){
            mu.V=sense.velb.get_value();
        }
        jacobianMotionModel(mu, sense, F, dt);
        f=mu.dynamics(sense,dt);
        mu+=f;
        if (u & UPDATE_VEL) {
        }

        if (u & UPDATE_IMG && mu.features.size()>0)
        {
            measurementModel(mu.X, sense.alt.get_value(), imgsense.matches,
                   sense.quat.get_value(), meas, hmu, H, mu);
        }

        initQ(Q, nf, dt,Q0);
        Mat maskF(F!=F);
        Mat maskP(P!=P);
        calcP(P,F,Q);
        maskP=Mat(P!=P);
        printf("%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\n",sense.get_time(),
                mu.X[0],mu.X[1],mu.X[2],
                mu.V[0],mu.V[1],mu.V[2],
                mu.b[0],mu.b[1],mu.b[2]);

        if (u & UPDATE_IMG && mu.features.size()>0 ) 
        {
            std::vector<int> rf;
            for (size_t i=0; i<meas.features.size(); ++i) {
                if (meas.features[i].reflection==NONREF) {
                    rf.push_back(0);
                } else {
                    rf.push_back(1);
                }
            }
            initR(R, rf, R0);
            
            P=.5*P+.5*P.t();
            P+=1e-18*Mat::eye(P.rows,P.cols,CV_64F);
            calcK(K,H,P,R);
            updateP(P,K,H);
            subtract(meas,hmu,estimateError);
            cv::Point2d mean(0,0);
            int k=0;

            estimateError.toMat(eeMat);
            Mat hmuMat;
            hmu.toMat(hmuMat);
            kx=K*eeMat;
            Mat kxv=kx(cv::Rect(0,0,1,3));
            Mat Rv=(Mat)sense.quat.get_value().rotation();
            kmh=States(kx);
            cv::Vec3d tx=kmh.X;
            cv::Vec3d tv=kmh.V;
            mu+=kmh;
        }

        ++iter;
    } 
    return 0;
}


void jacobianMotionModel( const States& mu, const Sensors& sense, Mat& F_out, double dt )
{
    F_out = cv::Mat::zeros(mu.getRows(), mu.getRows(), CV_64F);
    int nf;
    Quaternion qbw;
    cv::Vec3d w;

    qbw = sense.quat.get_value();
    w=sense.ang.get_value();
    nf=mu.getNumFeatures();

    Mat Fb = cv::Mat::zeros(6,6,CV_64F);
    Mat Fb1 = (Mat_<double>(6, 6) << 0, 0, 0,
            pow(qbw.coord[0], 2) - pow(qbw.coord[1], 2) - pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            2 * qbw.coord[0]*qbw.coord[1] - 2 * qbw.coord[2]*qbw.coord[3], 2 * qbw.coord[0]*qbw.coord[2] + 2 * qbw.coord[1]*qbw.coord[3],
            0, 0, 0, 
            2 * qbw.coord[0]*qbw.coord[1] + 2 * qbw.coord[2]*qbw.coord[3], - pow(qbw.coord[0] , 2) + pow(qbw.coord[1] , 2) - pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            2 * qbw.coord[1]*qbw.coord[2] - 2 * qbw.coord[0]*qbw.coord[3],
            0, 0, 0,
            2 * qbw.coord[0]*qbw.coord[2] - 2 * qbw.coord[1]*qbw.coord[3], 2 * qbw.coord[0]*qbw.coord[3] + 2 * qbw.coord[1]*qbw.coord[2], -pow(qbw.coord[0] , 2)
            - pow(qbw.coord[1] , 2) + pow(qbw.coord[2] , 2) + pow(qbw.coord[3] , 2),
            0, 0, 0, 0, w[2], -w[1], // TODO: These all go to zero when using CORRIMU Span data
            0, 0, 0, -w[2], 0, w[0],
            0, 0, 0, w[1], -w[0], 0);
    blockAssign(Fb,Fb1,Point(0,0));
    Mat Fi = Mat::zeros(nf*3, nf*3, CV_64F);
    Mat Fib = Mat::zeros(nf*3, 6, CV_64F);

    for (int i = 0; i<nf; i++)
    {
        Mat FiTemp;
        Mat Fib_ith;
        Mat Fi_ith = Mat::zeros(3,nf*3,CV_64F);
        Mat Fi_ith_1;
        Mat Fi_ith_2;
        Mat Fi_ith_3;

        Matx13d pib( 
            mu.features[i].get_body_position()[0],
            mu.features[i].get_body_position()[1],
            mu.features[i].get_body_position()[2]
        );
        double pib1 = mu.features[i].get_body_position()[0];
        double pib2 = mu.features[i].get_body_position()[1];
        double pib3 = mu.features[i].get_body_position()[2];

        FiTemp = (Mat_<double>(3, 3) <<
                    pib3*mu.V[0]-2*pib1*w[2] +pib2*w[1],
                    w[0]+pib1*w[1],
                    pib1*mu.V[0] - mu.V[1],
                    -w[0]-pib2*w[2],
                    pib3*mu.V[0]-pib1*w[2]+2*pib2*w[1],
                    pib2*mu.V[0]-mu.V[2],
                    -pib3*w[2],
                    pib3*w[1],
                    2*pib3*mu.V[0]-pib1*w[2]+pib2*w[1]);

        /* new Fib 11/10/14 */
        Fib_ith = (Mat_<double>(3, 6) <<
                    0, 0, 0, pib1*pib3, -pib3, 0,
                    0, 0, 0, pib2*pib3, 0, -pib3,
                    0, 0, 0, pow(pib3, 2), 0, 0);


        blockAssign(Fib, Fib_ith, Point(0, 3*i));

        blockAssign(Fi, FiTemp, Point(3*i,3*i));
    }  
    Mat temp1 = Mat::eye(mu.getRows(), mu.getRows(), CV_64F);
    F_out.setTo(0);
    blockAssign(F_out, Fb, Point(0,0));
    blockAssign(F_out, Fib, Point(0,Fb.rows));
    blockAssign(F_out, Fi,Point(Fib.cols,Fb.rows));
    F_out = dt*F_out + temp1;

    F_out.at<double>(3, 6+3*nf) = -1*dt; // use 6,7,8 for bias up front
    F_out.at<double>(4, 7+3*nf) = -1*dt;
    F_out.at<double>(5, 8+3*nf) = -1*dt;
}

/************************************************************************************************
* measurementModel
* assumes output matrix to be initialized to 0.
* TODO: which output matrix? Just set to zeros then...
**************************************************************************************************/
void measurementModel( const cv::Vec3d& old_pos, double alt, const vector<projection>& matches,
        const Quaternion& qbw, View& meas, View& hmu, Mat& H, const States& mu )
{
    meas.altitude = -alt;                            // altitude
    hmu.altitude = -mu.X[2];
    H=cv::Mat::zeros(1+6*mu.rf+4*mu.nrf,mu.getRows(),CV_64F);
    H.at<double>(0,2)=-1;
    Mat Hb;
    Mat Hi;

    cFiter feat=mu.features.begin();
    int index = 1;
    for (int i=0; feat!=mu.features.end(); ++i, ++feat)
    {
        // find corresponding match
        bool found=false;
        cMatchIter match=matches.begin();
        while (match!=matches.end()) {
            if (match->id==feat->getID()) {
                found=true;
                break;
            }
            ++match;
        }
        if (found==false) {
            fprintf(stderr,"Cannot find match.\n");
            exit(EXIT_FAILURE);
        }
        jacobianH(mu.X, qbw, *feat, Hb, Hi);
        if( feat->initial.isRef )
        {           
            meas.features.push_back(Vfeat( match->source, feat->initial.pib, match->reflection ));
            hmu.features.push_back(Vfeat( feat->get_body_position(), 
            feat->pib0Hat(old_pos, qbw), feat->ppbHat(mu.X, qbw) ));
        }
        else
        {
            meas.features.push_back(Vfeat( match->source, feat->initial.pib ));
            hmu.features.push_back(Vfeat( feat->get_body_position(), feat->pib0Hat(old_pos, qbw) ));
        }
        H.row(0).col(2).setTo(-1);
        // For each feature

        Mat Hfeat = Mat::zeros(6,mu.getRows(),CV_64F);
        blockAssign( Hfeat, Mat::eye(2,2,CV_64F), Point(6+3*i,0) );
        blockAssign( Hfeat, Hb, Point(0,2) );
        blockAssign( Hfeat, Hi, Point(6+3*i,2) );

        if( feat->initial.isRef )
        {
            blockAssign( H, Hfeat, Point(0,index) );
            index += 6;
        }
        else
        {
            Mat Hfeat_roi(Hfeat, Rect(0,0,Hfeat.cols,4));
            blockAssign( H, Hfeat_roi, Point(0,index) );
            index += 4;
        }
    } // end for loop
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initG
 *  Description:  
 * =====================================================================================
 */
    void
initG ( cv::Mat& G, int nf, double dt )
{
    G = Mat::eye(9+3*nf, 9+3*nf, CV_64F);
    return;
}        /* -----  end of function initG  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initQ
 *  Description:  
 * =====================================================================================
 */
    void
initQ ( cv::Mat& Q, int nf, double dt, double Q0)
{
    Mat G=(Mat_<double>(6,1) <<
            dt*dt/2,
            dt*dt/2,
            dt*dt/2,
            dt,
            dt,
            dt);
    double acccov=Q0;

    Mat GGt=G*acccov*G.t();

    Q = 1e-5*Mat::eye(9+3*nf, 9+3*nf, CV_64F);
    blockAssign(Q,GGt,cv::Point(0,0));

    blockAssign(Q, 1e-9*cv::Mat::eye(3,3, CV_64F), cv::Point(6+3*nf,6+3*nf) );
    return;
}        /* -----  end of function initq  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initR
 *  Description:  
 * =====================================================================================
 */
    void
initR ( cv::Mat& R, const std::vector<int>& refFlag, double R0 )
{
    vector<double> vecR;
    vecR.push_back(5);

    for (int i=0; i<refFlag.size(); i++)
    {
        // current view measurement noise covariance
        vecR.push_back(R0);
        vecR.push_back(R0);

        // initial view measurement noise covariance
        vecR.push_back(R0);
        vecR.push_back(R0);

        if(refFlag[i])
        {
            // reflection measurment noise covariance
            vecR.push_back(R0);
            vecR.push_back(R0);
        }
    }
    // Possibly unnecessary intermediate step ensure data is copied out of
    // vector, not shared.
    Mat row(vecR,true);
    R = Mat::diag(row);
    return;
}        /* -----  end of function initR  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcP
 *  Description:  
 * =====================================================================================
 */
    void
calcP ( cv::Mat& P, const cv::Mat& F, const cv::Mat& Q )
{
    P = F*P*F.t() + Q;
    return;
}        /* -----  end of function calcP  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcK
 *  Description:  
 * =====================================================================================
 */
    void
calcK ( cv::Mat& K, const cv::Mat& H, const cv::Mat& P, const cv::Mat& R)
{
    cv::Mat tmp=H*P;
    tmp *= H.t();
    tmp += R;

    K = P*H.t();

    K=K.t();
    tmp=tmp.t();
    if (!solve(tmp, K, K,DECOMP_CHOLESKY))
        cerr << "HPHt+R is singular." << endl;
    K=K.t();
    return;
}        /* -----  end of function calcK  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  updateP
 *  Description:  
 * =====================================================================================
 */
    void
updateP ( cv::Mat& P, const cv::Mat& K, const cv::Mat& H )
{
    Mat kh = K*H;
    Mat ikh;
    ikh = (Mat::eye(kh.size(), CV_64F) - kh)*P;
    P=ikh;
    P = (ikh.t() + ikh) / 2;
    return;
}   

/************************************************************************************************
* JacobianH. Note: 'i' here should be 1 less 'i' in matlab
**************************************************************************************************/

void jacobianH( const cv::Vec3d& X, const Quaternion& qbw, const Feature& feat, Mat& Hb, Mat& Hi )
{
    cv::Vec3d xb0w;
    Quaternion qb0w;

    xb0w=feat.get_initial_anchor();
    qb0w=feat.get_initial_quaternion();

    double xbw1 = X[0];
    double xbw2 = X[1];
    double xbw3 = X[2];
    double qbw1 = qbw.coord[0];
    double qbw2 = qbw.coord[1];
    double qbw3 = qbw.coord[2];
    double qbw4 = qbw.coord[3];

    double pib1 = feat.get_body_position()[0];
    double pib2 = feat.get_body_position()[1];
    double pib3 = feat.get_body_position()[2];

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
