#include "pid.h"
#include <QDebug>
pid::pid(float kp,float ki,float kd,float max_output):kp(kp),ki(ki),kd(kd),max_output(max_output)
{

}

float pid::pid_calc(float get, float set)
{
    err[NOW] = set - get;
    pout = kp * err[NOW];
    iout += ki * err[NOW];
    dout = kd * (err[NOW] - err[LAST]);

//    abs_limit(&(iout), integral_limit);
    out = pout + iout + dout;
//    qDebug()<<out;
    abs_limit(&(iout), 20);
    abs_limit(&(out), max_output);

    err[LAST]  = err[NOW];

    return out;
}
void pid::pid_reset() {
    iout=0;
    err[NOW]=0;
    err[LAST]=0;
}
void pid::abs_limit(float *a,float max)
{
    if(*a>max)
        *a=max;
    if(*a<-max)
        *a=-max;
}
