#ifndef MotorControl__H
#define MotorControl__H
#include "PID.h"
#include "zf_pwm.h"
#include "SEEKFREE_ABSOLUTE_ENCODER.h"
/*
 * 电机pid参数
 *
 *     3(左前)   2(右前)
 *
 *     4(左后)   1(右后)
 */
#define MOTOR_FREQUENCY    17000
#define motor1_PWM    PWM4_CH1_B6
#define motor1_DIR    A15
#define motor4_PWM    PWM4_CH2_B7
#define motor4_DIR      B3
#define motor2_PWM    PWM4_CH3_B8
#define motor2_DIR    B4
#define motor3_PWM    PWM4_CH4_B9
#define motor3_DIR    B5
#define MAIN_ROAD MT9V03X_H*1/2
void Motor_Init(void);
void Enc_Init(void);
void Motor_Ctrl(float motor1, float motor2, float motor3, float motor4);
void Speed_Ctrl(int16_t x, int16_t y, float z);
void control_loc(int16_t error);
void movecontrol(void);
void pid_init(void);
void rotate(float angle,int dir,float velocity);
void moveclear(void);
extern pid pid1,pid2,pid3,pid4;
extern int16 Encoder[4];

#endif
