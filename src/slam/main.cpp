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
    if (argc>1)
    {
        if (!strcmp(argv[1],"-h" ))
        {
            help(argv[0]);
            exit(EXIT_FAILURE);
        }
        else
        {
            //TODO: get meas filename
        }
    }

    /* Intialize */
    const double Q0=25; // 100 for simulation and 1 for experiments
    //const double R0=25; // 10 for simulation and 1 for experiments
    const double R0=1; // 10 for simulation and 1 for experiments
    States mu, mu_prev;
    Sensors sense;

    mu.X[2] = -1; /* Constant altitude for now */

    cv::Mat P=cv::Mat::eye(9,9,CV_64F);
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(0,0));
    blockAssign(P, PINIT*cv::Mat::eye(3,3,CV_64F), cv::Point(6,6));

    mu.setb(cv::Vec3d(0,0,0));

    const double scaleW=1;
    const double scaleH=1;
    const int width=640;
    const int height=480;
    cv::Mat rtplot=cv::Mat::zeros(height, width, CV_8UC3);


    /* Enter main loop */
    int u;
    while ((u=sense.update())!=-1)
    {
        double dt;
        cv::Vec3d old_pos;
        cv::Mat G, Q, R, K, H, F;
        States f, kmh;
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
        dt=0.02;
        old_pos=mu.X;
        f=mu.dynamics(sense);
        f*=dt;
        jacobianMotionModel(mu, sense, F, dt);
        mu+=f;

        initG(G, nf, dt);
        initQ(Q, nf, Q0, dt);
        circle(rtplot, cv::Point(mu.X[0]*scaleW+width/2,
                    height/2+(-mu.X[1]*scaleH)), .1, cv::Scalar(0,10,220));
        //cv::imshow("foo", rtplot);
        //cv::waitKey(2);
        //std::cout << sense.acc.get_dt() << ",ACC,"<< sense.acc.get_value() << std::endl;
        //std::cout << sense.quat.get_dt() << ",QUAT,"<< sense.quat.get_value().coord << std::endl;
        //std::cout << mu.V <<std::endl;
        std::cout << F <<std::endl;
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
    w=sense.ang.get_value();
    //nf=mu.getNumFeatures();
    nf=0;

    Mat Fb = cv::Mat::zeros(9,9,CV_64F);
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
            0, 0, 0, 0, w[2], -w[1],
            0, 0, 0, -w[2], 0, w[0],
            0, 0, 0, w[1], -w[0], 0);
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
    F_out.at<double>(4, 7) = -1 * dt;
    F_out.at<double>(5, 8) = -1 * dt;
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
