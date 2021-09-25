#include "PID.h"
extern     int16 targetSpeed1, targetSpeed2, targetSpeed3, targetSpeed4;
extern int16  realSpeed1, realSpeed2, realSpeed3, realSpeed4;
float constrain_float(float amt, float low, float high) //ÏÞ·ùº¯Êý
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
void Pid_Init(pid * pid)
{
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_i_sum = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_last_error =0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
    pid->T =2;
    pid->ID=0;
}
float Pid_LocCtrl(pid* pid, float error)
{
    /* ÀÛ»ýÎó²î */
    pid->integrator += error;

    /* Îó²îÏÞ·ù */
    constrain_float(pid->integrator, -pid->imax, pid->imax);
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
float Pid_IncCtrl(pid* pid, float error)
{
    float a=0.1,b=0.8;
    pid->out -=  pid->out_i_sum;

   pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    if(pid->ID ==1||pid->ID==5)
    {
        if(targetSpeed1>=0)
        {
            if((realSpeed1-targetSpeed1)>(targetSpeed1)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
        else
        {
            if((targetSpeed1-realSpeed1)>(-targetSpeed1)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
    }
    if(pid->ID ==2||pid->ID==6)
    {
        if(targetSpeed2>=0)
        {
            if((realSpeed2-targetSpeed2)>(targetSpeed2)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
        else
        {
            if((targetSpeed2-realSpeed2)>(-targetSpeed2)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
    }
    if(pid->ID ==3||pid->ID==7)
    {
        if(targetSpeed3>=0)
        {
            if((realSpeed3-targetSpeed3)>(targetSpeed3)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
        else
        {
            if((targetSpeed3-realSpeed3)>(-targetSpeed3)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
    }
    if(pid->ID ==4||pid->ID==8)
    {
        if(targetSpeed4>=0)
        {
            if((realSpeed4-targetSpeed4)>(targetSpeed4)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
        else
        {
            if((targetSpeed4-realSpeed4)>(-targetSpeed4)*a)
            pid->out_i_sum=pid->out_i_sum*b;
            else
            pid->out_i_sum += pid->out_i;
        }
    }
    pid->out += pid->out_p + pid->out_d;

   pid->out = pid->out + pid->out_i_sum;
    return pid->out;

  /*  float q0,q1,q2;

    q0 = pid->kp * (1 + pid->T / pid->ki + pid->kd / pid->T);
    q1 = -pid->kp * (1 + 2 * pid->kd / pid->T);
    q2 = pid->kp * pid->kd / pid->T;
    pid->out = pid->out + q0 * error + q1 * pid->last_error + q2 * pid->last_last_error;

    pid->last_last_error =pid->last_error;
    pid->last_error = error;*/

}


