/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/21/2014 03:00:06 PM
 *       Revision:  none
 *       Compiler:  g++
 *
 *         Author:  Hong-Bin Yoon (HBY), yoon48@illinois.edu
 *   Organization:  Aerospace Robotics and Controls Lab (ARC)
 *
 * =====================================================================================
 */

#define _USE_MATH_DEFNES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define MAXLINE 1024


/* GLOBAL VARIABLES */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getSphericalLat
 *  Description:  
 * =====================================================================================
 */
    float
getSphericalLat ( float lat, float h )
{
    /*  Semi-Major Axis */
    float A = 6378137;
    /* 1/f Reciprocal of Flattening */
    float f = 1/298.257223563;

    float e2, Rc, p, z, r, sphericalLat;
    e2 = f*(2-f);
    Rc = A/ ( sqrt(1-pow(e2,2)* pow(sin(lat*M_PI/180),2)) );
    p = (Rc + h) * cos(lat * M_PI/180);
    z = (Rc * (1-pow(e2,2)) + h ) * sin(lat*M_PI/180);
    r = sqrt(pow(p,2)+pow(z,2));

    sphericalLat = asin(z/r)*180/M_PI;

    return sphericalLat;
}		/* -----  end of function getSphericalLat  ----- */

#include <iostream>
int main(int argc, char** argv)
{
	using namespace std;

    if( argc!=1 )
    {
        printf("Usage: %s\n Reads raw GPS data as triple long, lat, alt from STDIN. \n",argv[0]);
        exit(EXIT_FAILURE);
    }
    char* line;
    line	= (char*)calloc ( (size_t)(MAXLINE), sizeof(char) );
    if ( line==NULL ) {
        fprintf ( stderr, "\ndynamic memory allocation failed\n" );
        exit (EXIT_FAILURE);
    }

    /*  Semi-Major Axis */
    float A = 6378137;
    /* 1/f Reciprocal of Flattening */
    float f = 1/298.257223563;
    float b = A*(1-f);
    float e2 = 1-pow((b/A),2);

    while( fgets( line, MAXLINE, stdin )!=NULL )
    {
        double lon, lat, GPS_lat, alt;
        sscanf(line, "%lf,%lf,%lf", &GPS_lat, &lon, &alt);
        GPS_lat = GPS_lat/pow(10,7);
        alt = alt/pow(10,3) + 1.021;
        lon = lon/pow(10,7);
        //printf("GPS_RAW:%lf,%lf,%lf\n",GPS_lat,lon,alt);
        lat = getSphericalLat(GPS_lat, alt);
        double Nphi = A /sqrt(1-e2*pow(sin(lat),2) );
        lat = lat*M_PI/180;
        lon = lon*M_PI/180;
        //printf("%lf,%lf,%lf\n", lat,lon,alt);
        //printf("%lf,%lf\n",Nphi,alt);
        double x, y, z;
        x = (Nphi + alt)*cos(lat)*cos(lon);
        y = (Nphi + alt)*cos(lat)*sin(lon);
        z = alt;

        printf("%lf,%lf,%lf\n",x,y,z);
    }

    free (line);
    line	= NULL;


	return 0;
}

