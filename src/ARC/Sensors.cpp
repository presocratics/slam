#include "Sensors.hpp"
#include "ourerr.hpp"

    int
Sensors::update ( )
{
    int rv=0;
    int updated=0;
    if (time==-1)
    {
        rv=scanf("%lf,%[^,\n],%s", &time, type, vals);
    }
    t=time;

    while (time-t<EPS)
    {
        if (rv==EOF) return -1;
        cv::Vec3d val3;
        cv::Vec4d val4;
        if (!strcmp(type, "ACC"))
        {
            updated|=UPDATE_ACC;
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            acc.set_value(time, val3);
        }
        else if (!strcmp(type, "ANG"))
        {
            updated|=UPDATE_ANG;
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            ang.set_value(time, val3);
        }
        else if (!strcmp(type, "IMG"))
        {
            updated|=UPDATE_IMG;
            img.set_value(time, 0);
        }
        else if (!strcmp(type, "QUAT"))
        {
            updated|=UPDATE_QUAT;
            sscanf(vals, "%lf,%lf,%lf,%lf", &val4[0], &val4[1], &val4[2], &val4[3]);
            quat.set_value(time, val4);
        }
        else if (!strcmp(type, "ALT"))
        {
            updated|=UPDATE_ALT;
            sscanf(vals, "%lf", &val3[0]);
            alt.set_value(time, val3[0]);
        }
        else if (!strcmp(type, "POS"))
        {
            updated|=UPDATE_POS;
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            pos.set_value(time, val3);
        }
        else if (!strcmp(type, "VEL"))
        {
            updated|=UPDATE_VEL;
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            vel.set_value(time, val3);
        }
        else if (!strcmp(type, "INIT"))
        {
            updated|=UPDATE_INIT;
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            init.set_value(time, val3);
        }
        rv=scanf("%lf,%[^,\n],%s", &time, type, vals);
    }
    return updated;
}		/* -----  end of method Sensors::update  ----- */

