#ifndef PID_H
#define PID_H

#define NOW 0
#define LAST 1

class pid
{
public:
    pid(float kp,float ki,float kd);
    float pid_calc(float get, float set);
    float kp,ki,kd;
    float pout=0.0f,iout=0.0f,dout=0.0f,out=0.0f;
    float err[2];
};

#endif // PID_H
