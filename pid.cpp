#include "pid.h"

pid::pid(float kp,float ki,float kd):kp(kp),ki(ki),kd(kd)
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
//    abs_limit(&(out), max_output);

    err[LAST]  = err[NOW];

    return out;
}
