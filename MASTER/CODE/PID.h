#ifndef PID__H
#define PID__H
#include "common.h"
typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_i_sum;
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                T;
    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_last_error;
    float                last_derivative;//< 上次误差与上上次误差之差
    uint32               last_t;     //< 上次时间//XX
    int                  ID;
}pid;
void Pid_Init(pid* pid);

float constrain_float(float amt, float low, float high);

float Pid_LocCtrl(pid* pid, float error);

float Pid_IncCtrl(pid* pid, float error);
#endif
