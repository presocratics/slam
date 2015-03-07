#include "Sensors.hpp"
#include "ourerr.hpp"
#define EPS 0.02 // Maximum delta to consider part of the same timestep.

    void
Sensors::update ( )
{
    if (time==-1)
    {
        scanf("%lf,%[^,],%s", &time, type, vals);
    }
    t=time;

    while (time-t<EPS)
    {
        cv::Vec3d val3;
        cv::Vec4d val4;
        if (!strcmp(type, "ACC"))
        {
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            acc.set_value(time, val3);
        }
        else if (!strcmp(type, "ANG"))
        {
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            ang.set_value(time, val3);
        }
        else if (!strcmp(type, "IMG"))
        {
            sscanf(vals, "%lf,%lf,%lf", &val3[0], &val3[1], &val3[2]);
            img.set_value(time, val3);
        }
        else if (!strcmp(type, "QUAT"))
        {
            sscanf(vals, "%lf,%lf,%lf,%lf", &val4[0], &val4[1], &val4[2], &val4[3]);
            quat.set_value(time, val4);
        }
        scanf("%lf,%[^,],%s", &time, type, vals);
    }
    return ;
}		/* -----  end of method Sensors::update  ----- */

