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
#define PINIT 1e-4            /*  */
#define THRESH 2e-10           /*  */
#define MAXLINE 1024
using std::cout; 
using std::cerr; 
using std::endl;

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
    /* Handle input */
    if (argc<2 || !strcmp(argv[1],"-h")) {
        help(argv[0]);
        exit(EXIT_FAILURE);
    }

    /* Initialize */
    const double Q0=25; 
    const double R0=25;
    States mu, mu_prev;
    Sensors sense;
    ImageSensor imgsense( argv[1], false );

    mu.X[2] = -1; /* Constant altitude for now */

    cv::Mat P=cv::Mat::eye(9,9,CV_64F);
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(0,0));
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(6,6));

    mu.setb(cv::Vec3d(0,0,0));

    const double scaleW=.6;
    const double scaleH=.6;
    const int width=640;
    const int height=480;
    cv::Mat rtplot=cv::Mat::zeros(height, width, CV_8UC3);

    /* Set initial conditions */
    sense.update();

    /* Enter main loop */
    int u;
    int iter=0;
    while ((u=sense.update())!=-1)
    {
        double dt;
        cv::Vec3d old_pos;
        cv::Mat G, Q, R, K, H, F;
        Mat kx, eeMat;
        States f, kmh;
        View meas, hmu, estimateError;
        int nf;

        nf=0;
        if (u & UPDATE_ACC) dt=sense.acc.get_dt();
        else if (u & UPDATE_ANG) dt=sense.ang.get_dt();
        else if (u & UPDATE_QUAT) dt=sense.quat.get_dt();
        else if (u==0)
        {
            fprintf(stderr, "No update, so what are we doing here?\n");
            exit(EXIT_FAILURE);
        }
        if (u & UPDATE_IMG) {
            // Read in new features
            // Update features in mu
            // SetMinMaxDepth (should this happen during update features?)
        }
        /*
        if ((iter++%14)==0) {
            // Read in new features
            imgsense.update();
            matchIter m=imgsense.matches.begin();
            for (;m!=imgsense.matches.end();++m) {
                cout << m->source << endl;
            }
            // Update features in mu
            //mu.update_features(imgsense, sense);
        }
        */
        
        dt=0.02;
        old_pos=mu.X;
        f=mu.dynamics(sense);
        f*=dt; // TODO: why isn't this inside dynamics?
        mu+=f;

        /*
        jacobianMotionModel(mu, sense, F, dt);

        if (u & UPDATE_IMG) 
        {
            measurementModel(old_pos, 0, sense.quat.get_value().coord, meas, hmu, H, mu);
            resizeP(P,nf);
        }

        initG(G, nf, dt);
        initQ(Q, nf, Q0, dt);
        std::vector<int> rf;
        initR(R, R0, rf);
        calcP(P,F,G,Q);

        if (u & UPDATE_IMG) 
        {
            calcK(K,H,P,R);
            updateP(P,K,H);
            subtract(meas,hmu,estimateError);
            estimateError.toMat(eeMat);
            kx=K*eeMat;
            kmh=States(kx);
            mu+=kmh;
        }

                    */
        circle(rtplot, cv::Point(mu.X[0]*scaleW+width/2,
                    height/2+(-mu.X[1]*scaleH)), .1, cv::Scalar(0,10,220));
        //cv::imshow("foo", rtplot);
        //cv::waitKey(2);
        //std::cout << sense.acc.get_dt() << ",ACC,"<< sense.acc.get_value() << std::endl;
        //std::cout << sense.quat.get_dt() << ",QUAT,"<< sense.quat.get_value().coord << std::endl;
        //std::cout << F <<std::endl;
        //std::cout << sense.quat.get_value().coord << std::endl;
        //std::cout << sense.quat.get_value().rotation() << std::endl;
    }
    cv::imshow("foo", rtplot);
    cv::waitKey(0);
    return 0;
}

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
}        /* -----  end of function blockAssign  ----- */

void jacobianMotionModel( const States& mu, const Sensors& sense, Mat& F_out, double dt )
{
    F_out = cv::Mat::zeros(mu.getRows(), mu.getRows(), CV_64F);
    int nf;
    Quaternion qbw;
    cv::Vec3d w;

    qbw = sense.quat.get_value();
    //w=200*sense.ang.get_value();
    w=sense.ang.get_value();
    nf=mu.getNumFeatures();

    Mat Fb = cv::Mat::zeros(6,6,CV_64F);
    Mat Fb1 = (Mat_<double>(6, 6) << 0, 0, 0, 
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
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0);
    blockAssign(Fb,Fb1,Point(0,0));
    Mat Fi = Mat::zeros(nf*3, nf*3, CV_64F);
    Mat FiTemp;
    Mat Fib = Mat::zeros(nf*3, 9, CV_64F);
    Mat Fib_ith;
    Mat Fi_ith = Mat::zeros(3,nf*3,CV_64F);
    Mat Fi_ith_1;
    Mat Fi_ith_2;
    Mat Fi_ith_3;

    /*
    for (int i = 0; i<nf; i++)
    {
        Matx13d pib( 
            mu.features[i]->get_body_position()[0],
            mu.features[i]->get_body_position()[1],
            mu.features[i]->get_body_position()[2]
        );
        double pib1 = mu.features[i]->get_body_position()[0];
        double pib2 = mu.features[i]->get_body_position()[1];
        double pib3 = mu.features[i]->get_body_position()[2];

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
        */

        /* new Fib 11/10/14 */
    /*
        Fib_ith = (Mat_<double>(3, 6) <<
                    0, 0, 0, pib1*pib3, -pib3, 0,
                    0, 0, 0, pib2*pib3, 0, -pib3,
                    0, 0, 0, pow(pib3, 2), 0, 0);


        blockAssign(Fib, Fib_ith, Point(0, 3*i));

        Fi_ith_1 = Mat::zeros(3, 3 * (i), CV_64F);
        Fi_ith_2 = FiTemp;
        Fi_ith_3 = Mat::zeros(3, 3 * (nf-i-1), CV_64F);
        blockAssign(Fi_ith, Fi_ith_1, Point(0,0));
        blockAssign(Fi_ith, Fi_ith_2, Point(Fi_ith_1.cols,0));
        blockAssign(Fi_ith, Fi_ith_3,
        Point(Fi_ith_1.cols+FiTemp.cols,0));
        blockAssign(Fi, Fi_ith, Point(0,3*i));
    }  
    */
    Mat temp1 = Mat::eye(mu.getRows(), mu.getRows(), CV_64F);
    F_out.setTo(0);
    blockAssign(F_out, Fb, Point(0,0));
    blockAssign(F_out, Fib, Point(0,Fb.rows));
    blockAssign(F_out, Fi,Point(Fib.cols,Fb.rows));
    F_out = dt*F_out + temp1;

    blockAssign(F_out, cv::Mat::eye(3,3,CV_64F), cv::Point(6,6));
    F_out.at<double>(3, 6) = -1*dt;
    F_out.at<double>(4, 7) = -1*dt;
    F_out.at<double>(5, 8) = -1*dt;
}

/************************************************************************************************
* measurementModel
* assumes output matrix to be initialized to 0.
**************************************************************************************************/
void measurementModel( const cv::Vec3d& old_pos, double alt, const Quaternion& qbw,
       View& meas, View& hmu, Mat& H, States& mu )
{
    meas.altitude = alt;                            // altitude
    hmu.altitude = -mu.X[2];
    H=cv::Mat::zeros(1,mu.getRows(),CV_64F);
    H.row(0).col(2).setTo(-1);
    //Mat Hb;
    //Mat Hi;

    //cMatchIter match=matches.begin();
    //Fiter feat=mu.features.begin();
    //int index = 1;
    /*
    for (int i=0; feat!=mu.features.end(); ++i,  ++feat, ++match)
    {
        cv::Vec3d dst;
        add(mu.X,(Mat)qbw.rotation()*(Mat)(*feat)->xibHat(), dst );
        (*feat)->set_world_position(dst);

        jacobianH(mu.X, qbw, **feat, Hb, Hi);
        
        meas.features.push_back(Vfeat( match->source, (*feat)->initial.pib, match->reflection ));
        if( (*feat)->initial.isRef )
        {           
            hmu.features.push_back(Vfeat( (*feat)->get_body_position(), 
            (*feat)->pib0Hat(old_pos, qbw), (*feat)->ppbHat(mu.X, qbw) ));
        }
        else
        {
            hmu.features.push_back(Vfeat( (*feat)->get_body_position(), 
            (*feat)->pib0Hat(old_pos, qbw), cv::Vec3d(0,0,0) ));

        }
        H.row(0).col(2).setTo(-1);
        // For each feature

        Mat Hfeat = Mat::zeros(6,mu.getRows(),CV_64F);
        blockAssign( Hfeat, Mat::eye(2,2,CV_64F), Point(9+3*i,0) );
        blockAssign( Hfeat, Hb, Point(0,2) );
        blockAssign( Hfeat, Hi, Point(9+3*i,2) );

        if( (*feat)->initial.isRef )
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
    */
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
initQ ( cv::Mat& Q, int nf, double Q0, double dt )
{
    Q0*=dt*dt;
    Q = Q0*Mat::eye(9+3*nf, 9+3*nf, CV_64F);
    Q.at<double>(0,0) *= 0.001*Q0;
    Q.at<double>(1,1) *= 0.001*Q0;
    Q.at<double>(2,2) *= 0.001*Q0;

    blockAssign(Q, QBIAS*dt*dt*cv::Mat::eye(3,3, CV_64F), cv::Point(6,6) );
    return;
}        /* -----  end of function initq  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  initR
 *  Description:  
 * =====================================================================================
 */
    void
initR ( cv::Mat& R, double R0, std::vector<int> refFlag )
{
    vector<double> vecR;
    vecR.push_back(0.15*R0);

    /*
    for (int i = 0; i < refFlag.size(); i++)
    {
        // current view measurement noise covariance
        vecR.push_back(0.01 / 770 * R0);
        vecR.push_back(0.01 / 770 * R0);

        // initial view measurement noise covariance
        vecR.push_back(10. / 770 * R0);
        vecR.push_back(10. / 770 * R0);

        if(refFlag[i])
        {
            // reflection measurment noise covariance
            vecR.push_back(1. / 770 * R0);
            vecR.push_back(1. / 770 * R0);      
        }
    }
    */
    // Possibly unnecessary intermediate step ensure data is copied out of
    // vector, not shared.
    Mat row(vecR,true);
    R = Mat::diag(row);
    return;
}        /* -----  end of function initR  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  resizeP
 *  Description:  
 * =====================================================================================
 */
    void
resizeP ( cv::Mat& P, int nf )
{
    int nf_old;
    // Determine previous nf
    nf_old=(P.rows-9)/3;
    if( nf<nf_old ) // Select a smaller ROI
    {
        P=P( cv::Rect(0,0, 9+3*nf, 9+3*nf) );
    }
    else if( nf>nf_old ) // Increase size and initialize new features.
    {
        cv::Mat bigP;
        bigP = cv::Mat::zeros(9+3*nf, 9+3*nf, CV_64F);
        blockAssign(bigP, P, cv::Point(0,0));
        for( int i=9+3*nf_old; i<9+3*nf; i+=3 )
        {
            bigP.at<double>( i, i ) = P0;
            bigP.at<double>( i+1, i+1 ) = P0;
            bigP.at<double>( i+2, i+2 ) = P0;
        }
        P=bigP;
    }
    return;
}        /* -----  end of function resizeP  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcP
 *  Description:  
 * =====================================================================================
 */
    void
calcP ( cv::Mat& P, const cv::Mat& F, const cv::Mat& G, const cv::Mat& Q )
{
    P = F*P*F.t() + G*Q*G.t();
    return;
}        /* -----  end of function calcP  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  calcK
 *  Description:  
 * =====================================================================================
 */
    void
calcK ( cv::Mat& K, const cv::Mat& H, const cv::Mat& P, const cv::Mat& R )
{
    cv::Mat tmp=H*P;
    tmp *= H.t();
    tmp += R;

    K = P*H.t();

    K=K.t();
    tmp=tmp.t();
    solve(tmp, K, K);
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
    P = (Mat::eye(kh.size(), CV_64F) - kh)*P;
    //P = (P.t() + P) / 2;
    return;
}   

/************************************************************************************************
* JacobianH. Note: 'i' here should be 1 less 'i' in matlab
**************************************************************************************************/
/* 

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



} */
