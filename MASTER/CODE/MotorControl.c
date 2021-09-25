 #include "MotorControl.h"
/*
 * 电机pid参数
 *
 *     3(左前)   2(右前)
 *
 *     4(左后)   1(右后)
 */
#define LEN 20
extern float currentyaw;
extern uint8_t leftside[MT9V03X_H],rightside[MT9V03X_H],midline[MT9V03X_H];
extern uint32 sectionbuff[LEN];
extern int16_t StartVelocityY;
extern int16_t startdir ;
int stopflag=0;
pid ramppid,pid1,pid2,pid3,pid4,locpid,angelpid,encoderpid,startpid1,startpid2,startpid3,startpid4,acrosspid1,acrosspid2,acrosspid3,acrosspid4,acrossangelpid;
extern int32_t movecount;
int16 movemode = 0;     //  0 停车   1  前行  2 逆行  3左平移 4 右平移
int16 lastmovemode = 0;
extern int8 cameraerr;
int startflag = 0;
int rotateflag =0;
extern int16 angle;
int16 targetSpeed1 = 0, targetSpeed2 = 0, targetSpeed3 = 0, targetSpeed4 = 0;
int16  realSpeed1 = 0, realSpeed2 = 0, realSpeed3 = 0, realSpeed4 = 0;
int64 stopcnt=0;
extern int16_t isbranch;
extern uint8 dat;
void Motor_Init(void)
{
    pwm_init(motor1_PWM,MOTOR_FREQUENCY, 0);
    gpio_init(motor1_DIR, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(motor2_PWM,MOTOR_FREQUENCY, 0);
    gpio_init(motor2_DIR, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(motor3_PWM,MOTOR_FREQUENCY, 0);
    gpio_init(motor3_DIR, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(motor4_PWM,MOTOR_FREQUENCY, 0);
    gpio_init(motor4_DIR, GPO, 0, GPIO_PIN_CONFIG);
    pid_init();
}
void Enc_Init(void)
{
    encoder_init_spi(ABS_ENCODER_SPI_PC1_PIN);
    encoder_init_spi(ABS_ENCODER_SPI_PC2_PIN);
    encoder_init_spi(ABS_ENCODER_SPI_PC3_PIN);
    encoder_init_spi(ABS_ENCODER_SPI_PC4_PIN);

}
void Motor_Ctrl(float motor1duty, float motor2duty, float motor3duty, float motor4duty)
{

   /* static float motor1Last = 0, motor2Last = 0, motor3Last = 0, motor4Last = 0;
    motor1duty = motor1Last + constrain_float(motor1duty - motor1Last, -1000, 1000);
    motor2duty = motor2Last + constrain_float(motor2duty - motor2Last, -1000, 1000);
    motor3duty = motor3Last + constrain_float(motor3duty - motor3Last, -1000, 1000);
    motor4duty = motor4Last + constrain_float(motor4duty - motor4Last, -1000, 1000);*/

    motor1duty=constrain_float(motor1duty, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    motor2duty=constrain_float(motor2duty, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    motor3duty=constrain_float(motor3duty, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    motor4duty=constrain_float(motor4duty, -PWM_DUTY_MAX, PWM_DUTY_MAX);

  /* motor1Last = motor1duty;
   motor2Last = motor2duty;
   motor3Last = motor3duty;
   motor4Last = motor4duty;*/

    if(motor1duty>0)
     {   gpio_set(motor1_DIR, 1);
         pwm_duty(motor1_PWM,(uint32)motor1duty);
     }
     else
     {   gpio_set(motor1_DIR, 0);
         pwm_duty(motor1_PWM,(uint32)(-motor1duty));
     }
     if(motor2duty>0)
     {
         gpio_set(motor2_DIR, 0);
         pwm_duty(motor2_PWM,(uint32)motor2duty);
     }
     else
     {   gpio_set(motor2_DIR, 1);
         pwm_duty(motor2_PWM,(uint32)(-motor2duty));
     }
     if(motor3duty>0)
     {   gpio_set(motor3_DIR, 1);
         pwm_duty(motor3_PWM,(uint32)motor3duty);
     }
     else
     {   gpio_set(motor3_DIR, 0);
         pwm_duty(motor3_PWM,(uint32)(-motor3duty));
     }
     if(motor4duty>0)
     {
         gpio_set(motor4_DIR, 1);
         pwm_duty(motor4_PWM,(uint32)motor4duty);
     }
     else
     {
         gpio_set(motor4_DIR, 0);
         pwm_duty(motor4_PWM,(uint32)(-motor4duty));
     }
}

uint8 test1[] = "seekfree\n";
void Speed_Ctrl(int16 x, int16 y, float z)
{
   // static int stopcount=0;

    targetSpeed1 =  x  - y - z;
    targetSpeed2 =  x  + y - z;
    targetSpeed3 =  x  - y + z;
    targetSpeed4 =  x  + y +  z;
    realSpeed1 = Encoder[2];
     realSpeed2 = Encoder[1];
     realSpeed3 = -Encoder[0];
    realSpeed4 = -Encoder[3];

    //uart_putchar(UART_2,50);
    //uart_putbuff(UART_2,test1,sizeof(test1)-1);
   //if(realSpeed1==0 ||realSpeed2==0|| realSpeed3==0|| realSpeed4==0 )  stopcount ++;
    //if(stopcount>250) stopflag=1;
    //printf("%16d\r\n",realSpeed1);
    //data_conversion(realSpeed1,realSpeed2,realSpeed3,realSpeed4,virtual_scope_data);
    //ANO_DT_send_int16((int16_t)realSpeed1,(int16_t)realSpeed2,(int16_t)realSpeed3,(int16_t)realSpeed4,(int16_t)TargetVelocityY,0,0,0);

 /* if(movemode ==3|| movemode ==4 ||movemode ==8 || movemode==9)  //自转
    {     Pid_IncCtrl(&startpid1,(float)(targetSpeed1-realSpeed1));
           Pid_IncCtrl(&startpid2,(float)(targetSpeed2-realSpeed2));
           Pid_IncCtrl(&startpid3,(float)(targetSpeed3-realSpeed3));
           Pid_IncCtrl(&startpid4,(float)(targetSpeed4-realSpeed4));
          Motor_Ctrl(startpid1.out,startpid2.out,startpid3.out,startpid4.out);
    }*/
   if(movemode==0 ||movemode==2 ||movemode ==7 ||movemode ==6 ||movemode==11||movemode==10||movemode==14||movemode==13)  //前后
   {
            Pid_IncCtrl(&pid1,(float)(targetSpeed1-realSpeed1));
            Pid_IncCtrl(&pid2,(float)(targetSpeed2-realSpeed2));
            Pid_IncCtrl(&pid3,(float)(targetSpeed3-realSpeed3));
            Pid_IncCtrl(&pid4,(float)(targetSpeed4-realSpeed4));
            pid1.out=constrain_float(pid1.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            pid2.out=constrain_float(pid2.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            pid3.out=constrain_float(pid3.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            pid4.out=constrain_float(pid4.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            Motor_Ctrl(pid1.out,pid2.out,pid3.out,pid4.out);
   }
    else if(movemode==5||movemode==3||movemode==4||movemode==8||movemode==9||movemode==1)     //左右
    {
            Pid_IncCtrl(&acrosspid1,(float)(targetSpeed1-realSpeed1));
            Pid_IncCtrl(&acrosspid2,(float)(targetSpeed2-realSpeed2));
            Pid_IncCtrl(&acrosspid3,(float)(targetSpeed3-realSpeed3));
            Pid_IncCtrl(&acrosspid4,(float)(targetSpeed4-realSpeed4));
            acrosspid1.out=constrain_float(acrosspid1.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            acrosspid2.out=constrain_float(acrosspid2.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            acrosspid3.out=constrain_float(acrosspid3.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            acrosspid4.out=constrain_float(acrosspid4.out, -PWM_DUTY_MAX, PWM_DUTY_MAX);
            Motor_Ctrl(acrosspid1.out,acrosspid2.out,acrosspid3.out,acrosspid4.out);
    }

  /* char txt1[4];
   char txt2[4];
   char txt3[4];
   char txt4[4];
   char txt5[4];
   char txt6[4];
   char txt7[4];
   char txt8[4];
   char txt9[4];
   char txt10[4];
   char txt11[4];
   char txt12[4];
   char txt13[4];
   char txt14[4];
   char txt15[4];
   sprintf(txt1, "%d",realSpeed1);
   uart_putstr(UART_2,txt1);
   uart_putchar(UART_2,',');
   sprintf(txt2, "%d",realSpeed2);
   uart_putstr(UART_2,txt2);
   uart_putchar(UART_2,',');
   sprintf(txt3, "%d",realSpeed3);
   uart_putstr(UART_2,txt3);
   uart_putchar(UART_2,',');
   sprintf(txt4, "%d",realSpeed4);
   uart_putstr(UART_2,txt4);
   uart_putchar(UART_2,',');
   sprintf(txt5, "%d",targetSpeed1);
   uart_putstr(UART_2,txt5);
   uart_putchar(UART_2,',');
   sprintf(txt6, "%d",targetSpeed2);
   uart_putstr(UART_2,txt6);
   uart_putchar(UART_2,',');
   sprintf(txt7, "%d",targetSpeed3);
   uart_putstr(UART_2,txt7);
   uart_putchar(UART_2,',');
   sprintf(txt8, "%d",targetSpeed4);
   uart_putstr(UART_2,txt8);
   uart_putchar(UART_2,',');
   sprintf(txt9, "%d",movemode);
   uart_putstr(UART_2,txt9);
   uart_putchar(UART_2,',');

   if(movemode==0 ||movemode==2 ||movemode ==7 ||movemode ==6 ||movemode==11||movemode==10||movemode==1||movemode==14||movemode==13)  //前后
   { sprintf(txt10, "%d",(int16)pid1.out);
   uart_putstr(UART_2,txt10);
   uart_putchar(UART_2,',');
   sprintf(txt11, "%d",(int16)pid2.out);
   uart_putstr(UART_2,txt11);
   uart_putchar(UART_2,',');
   sprintf(txt12, "%d",(int16)pid3.out);
   uart_putstr(UART_2,txt12);
   uart_putchar(UART_2,',');
   sprintf(txt13, "%d",(int16)pid4.out);
   uart_putstr(UART_2,txt13);
   }
   else if(movemode==5||movemode==3||movemode==4||movemode==8||movemode==9)
   {
       sprintf(txt10, "%d",(int16)acrosspid1.out);
           uart_putstr(UART_2,txt10);
           uart_putchar(UART_2,',');
           sprintf(txt11, "%d",(int16)acrosspid2.out);
           uart_putstr(UART_2,txt11);
           uart_putchar(UART_2,',');
           sprintf(txt12, "%d",(int16)acrosspid3.out);
           uart_putstr(UART_2,txt12);
           uart_putchar(UART_2,',');
           sprintf(txt13, "%d",(int16)acrosspid4.out);
           uart_putstr(UART_2,txt13);
   }
   uart_putchar(UART_2,',');
   sprintf(txt14, "%d",(int16)cameraerr);
   uart_putstr(UART_2,txt14);
   uart_putchar(UART_2,',');
   sprintf(txt15, "%d",(int16)isbranch);
   uart_putstr(UART_2,txt15);

   uart_putchar(UART_2,'\r');
   uart_putchar(UART_2,'\n');*/
   //if((pid1.out>9000)||(pid1.out<-9000)||(pid2.out>9000)||(pid2.out<-9000)||(pid3.out>9000)||(pid3.out<-9000)||(pid4.out>9000)||(pid4.out<-9000)) stopcnt =70000;
}
void movecontrol(void)
{
    float expectedanglevelocity;//expectedvelocityZ;
    //cameraerr = 0;
    if(movemode==5||movemode==10)    expectedanglevelocity = Pid_LocCtrl(&acrossangelpid,(float)cameraerr);
    else if(movemode==13)    expectedanglevelocity = Pid_LocCtrl(&ramppid,(float)cameraerr);
    else expectedanglevelocity = Pid_LocCtrl(&angelpid,(float)cameraerr);

   // expectedanglevelocity = (float)TargetVelocityX;
   // expectedvelocityZ     = Pid_LocCtrl(&encoderpid,expectedanglevelocity-(float)icm_gyro_z);
   // ips114_showfloat(150, 2,expectedanglevelocity-(float)icm_gyro_z, 2, 2);
  //  ips114_showfloat(50, 6,expectedanglevelocity, 2, 2);
  //  ips114_showint8(50, 6,cameraerr);


  //  if(stopcnt>60000) movemode =12;
    if(dat ==1)  movemode=2;
     switch(movemode)
     {
     case 0:
      //   if(lastmovemode!=0) moveclear();
        Speed_Ctrl(0,0,0);
       //moveclear();

        // Motor_Ctrl(0,0,0,0);
         lastmovemode =0;
         break;
     case 1:  //出库
       //  if(lastmovemode!=1) moveclear();
         if(startdir==1)
           Speed_Ctrl(0,-110,0);
         else if(startdir==2)
             Speed_Ctrl(0,110,0);
         lastmovemode =1;
         break;
     case 2: //前行
      //   if(lastmovemode!=2) moveclear();
        // if(lastmovemode !=2) moveclear();
         //Speed_Ctrl(0,TargetVelocityX,0);
         //Speed_Ctrl(0,-TargetVelocityY,-expectedanglevelocity);
         //Speed_Ctrl(0,TargetVelocityY,expectedanglevelocity);
         if(lastmovemode!=2)  moveclear();
         Speed_Ctrl(TargetVelocityX,0,-expectedanglevelocity);
         //Speed_Ctrl(0,TargetVelocityY,expectedanglevelocity);
         //Speed_Ctrl(0,TargetVelocityX,0);
         lastmovemode =2;
         break;
     case 3: //第一次顺时针旋转30
       //  if(lastmovemode!=3) moveclear();
         if(lastmovemode !=3) moveclear();
         Speed_Ctrl(0,0,(float)150);
         pwm_duty(PWM2_CH1_A0,1020);
        // pwm_duty(PWM2_CH1_A0,700);
         lastmovemode =3;
         break;
     case 4:  //第二次逆时针转150
       //  if(lastmovemode!=4) moveclear();
         if(lastmovemode !=4) moveclear();
         Speed_Ctrl(0,0,-(float)150);
         pwm_duty(PWM2_CH1_A0,550);
        // pwm_duty(PWM2_CH1_A0,230);
         lastmovemode =4;
         break;
     case 5: //第一次平移
       //  if(lastmovemode!=5) moveclear();
        // if(lastmovemode !=5) moveclear();
         if(lastmovemode!=5)  moveclear();
         Speed_Ctrl(0,TargetVelocityY,-expectedanglevelocity);
         //Speed_Ctrl(TargetVelocityX,0,-expectedanglevelocity);
        // pwm_duty(PWM2_CH1_A0,250);
         //Speed_Ctrl(0,20,0);
         lastmovemode =5;
         break;
     case 6://逆行
        // if(lastmovemode!=6) moveclear();
        // if(lastmovemode !=6) moveclear();
         if(lastmovemode!=6)  moveclear();
         Speed_Ctrl(-TargetVelocityX,0,-expectedanglevelocity);
         lastmovemode =6;
         break;
     case 7:  //前行减速
         //if(lastmovemode!=7) moveclear();
         Speed_Ctrl(35,0,-expectedanglevelocity);    //   120速度  35减速
         lastmovemode=7;
         break;
     case 8: //顺时针150
        // if(lastmovemode!=8) moveclear();
         //if(lastmovemode !=8) moveclear();
        // pwm_duty(PWM2_CH1_A0,700);
         Speed_Ctrl(0,0,(float)100);
         lastmovemode =8;
         break;
     case 9:// 第二次逆时针30
         //if(lastmovemode !=9) moveclear();
        // pwm_duty(PWM2_CH1_A0,230);
         Speed_Ctrl(0,0,-(float)100);
         lastmovemode =9;
         break;
     case 10:
        // if(lastmovemode!=10) moveclear();
         Speed_Ctrl(0,TargetVelocityY/2,-expectedanglevelocity);
         lastmovemode=10;
         break;
     case 11:
        // if(lastmovemode!=11) moveclear();
         Speed_Ctrl(-40,0,-expectedanglevelocity);
         lastmovemode=11;
         break;
     case 12:
         //if(lastmovemode!=12) moveclear();
         Motor_Ctrl(0,0,0,0);
         moveclear();
         lastmovemode=12;
         break;
     case 13:
         //if(lastmovemode!=13) moveclear();
         Speed_Ctrl(80,0,-expectedanglevelocity);
         lastmovemode=13;
         break;
     case 14:
         Speed_Ctrl(120,0,-expectedanglevelocity);    // 入库速度
         break;
     case 15:
         Motor_Ctrl(0,0,0,0);
          pwm_duty(PWM2_CH1_A0,990);
         break;
     case 16:
         Motor_Ctrl(0,0,0,0);
          pwm_duty(PWM2_CH1_A0,530);
         break;
     }
}
void pid_init(void)
{
    Pid_Init(&pid1);
    Pid_Init(&pid2);
    Pid_Init(&pid3);
    Pid_Init(&pid4);
    Pid_Init(&angelpid);
    Pid_Init(&encoderpid);
    Pid_Init(&startpid1);
    Pid_Init(&startpid2);
    Pid_Init(&startpid3);
    Pid_Init(&startpid4);
    Pid_Init(&acrosspid1);
    Pid_Init(&acrosspid2);
    Pid_Init(&acrosspid3);
    Pid_Init(&acrosspid4);
    pid1.ID = 1;
    pid2.ID = 2;
    pid3.ID = 3;
    pid4.ID = 4;
    acrosspid1.ID=5;
    acrosspid2.ID=6;
    acrosspid3.ID=7;
    acrosspid4.ID=8;

  /*  pid1.kp =3.5;
    pid1.ki =0.5;
    pid1.kd =0;

    pid2.kp =3.5;                                                               //  3(左前)   2(右前)
    pid2.ki =0.5;
    pid2.kd =0;

                             //                                                     4(左后)   1(右后)
    pid3.kp =3.5;
    pid3.ki =0.5;
    pid3.kd =0;

    pid4.kp =3.5;
    pid4.ki =0.5;
    pid4.kd =0;

    angelpid.kp = 1.5;
    angelpid.ki = 0;
    angelpid.kd = 0;

    encoderpid.kp = 1.5;
    encoderpid.ki = 0;
    encoderpid.kd = 0;*/

    flash_page_read(FLASH_SECTION_15, FLASH_PAGE_0, sectionbuff, 20);
    acrossangelpid.kp = (float)sectionbuff[0]*1.0/10;
    acrossangelpid.kd = (float)sectionbuff[4]*1.0/10;
    TargetVelocityX = (int16)sectionbuff[1];
    TargetVelocityZ =(int16)sectionbuff[10];
    TargetVelocityY =(int16)sectionbuff[7];

    //  Kp  4  Ki  6
      pid1.kp =(float)sectionbuff[2]*1.0/10;   //4
      pid1.ki =(float)sectionbuff[3]*1.0/10;    //4.9
      pid1.kd =0;    //0

      pid2.kp =(float)sectionbuff[2]*1.0/10  ;                                                               //  3(左前)   2(右前)
      pid2.ki =(float)sectionbuff[3]*1.0/10;
      pid2.kd =0;

                               //                                                     4(左后)   1(右后)
      pid3.kp =(float)sectionbuff[2]*1.0/10  ;
      pid3.ki =(float)sectionbuff[3]*1.0/10   ;
      pid3.kd =0;

      pid4.kp =(float)sectionbuff[2]*1.0/10;
      pid4.ki =(float)sectionbuff[3]*1.0/10;
      pid4.kd =0;

   /*   startpid1.kp =(float)sectionbuff[8]*1.0/10;
      startpid2.kp =(float)sectionbuff[8]*1.0/10   + 1.25;
      startpid3.kp =(float)sectionbuff[8]*1.0/10   + 3;
      startpid4.kp =(float)sectionbuff[8]*1.0/10;

      startpid1.ki =(float)sectionbuff[9]*1.0/10;
      startpid2.ki =(float)sectionbuff[9]*1.0/10    +1.25;
      startpid3.ki =(float)sectionbuff[9]*1.0/10    + 3;
      startpid4.ki =(float)sectionbuff[9]*1.0/10;*/

      angelpid.kp =(float)sectionbuff[5]*1.0/10;
      angelpid.ki = 0;
      angelpid.kd =(float)sectionbuff[6]*1.0/10;

   /*   acrossangelpid.kp=1.29;//1.1;
      acrossangelpid.ki=0;
      acrossangelpid.kd=0.5;//1.1;*/

      acrosspid1.kp=(float)sectionbuff[8]*1.0/10;
      acrosspid2.kp=(float)sectionbuff[10]*1.0/10;
      acrosspid3.kp=(float)sectionbuff[12]*1.0/10;
      acrosspid4.kp=(float)sectionbuff[14]*1.0/10;

      acrosspid1.ki=(float)sectionbuff[9]*1.0/10;
      acrosspid2.ki=(float)sectionbuff[11]*1.0/10;
      acrosspid3.ki=(float)sectionbuff[13]*1.0/10;
      acrosspid4.ki=(float)sectionbuff[15]*1.0/10;

      ramppid.kp = 1;
      ramppid.kd = 0.5;
     // encoderpid.kp =(float)sectionbuff[6]*1.0/10;
     // encoderpid.kd =(float)sectionbuff[7]*1.0/10;
}
void rotate(float angle,int dir,float velocity)   //dir  1 左转  2  右转
{
   static  int i=0;
  static int stopflag=0;
   if(movemode ==1&&i==0)
   {
    currentyaw = 0;
    i=1;
   }

        if(dir == 1)Speed_Ctrl(0,0,-velocity);
        else if(dir == 2 && stopflag ==0)Speed_Ctrl(0,0,velocity);
        if(dir == 2)     if(fabs(angle+currentyaw)<5)     stopflag=1;

     if(stopflag ==1 )Speed_Ctrl(0,0,0);
    ips114_showfloat(150, 1,currentyaw, 3, 2);
    ips114_showfloat(150, 2,angle+currentyaw, 3, 2);
}
void moveclear(void)
{
    pid1.integrator =0;
    pid2.integrator =0;
    pid3.integrator =0;
    pid4.integrator =0;
    pid1.last_error =0;
    pid2.last_error =0;
    pid3.last_error =0;
    pid4.last_error =0;
    pid1.last_last_error =0;
    pid2.last_last_error =0;
    pid3.last_last_error =0;
    pid4.last_last_error =0;
    pid1.last_derivative =0;
    pid2.last_derivative =0;
    pid3.last_derivative =0;
    pid4.last_derivative =0;
    pid1.out_i_sum = 0;
    pid2.out_i_sum = 0;
    pid3.out_i_sum = 0;
    pid4.out_i_sum = 0;

    pid1.out = 0;
    pid2.out = 0;
    pid3.out = 0;
    pid4.out = 0;

    acrosspid1.integrator =0;
    acrosspid2.integrator =0;
    acrosspid3.integrator =0;
    acrosspid4.integrator =0;
    acrosspid1.last_error =0;
    acrosspid2.last_error =0;
    acrosspid3.last_error =0;
    acrosspid4.last_error =0;
    acrosspid1.last_derivative =0;
    acrosspid2.last_derivative =0;
    acrosspid3.last_derivative =0;
    acrosspid4.last_derivative =0;
    acrosspid1.out_i_sum = 0;
    acrosspid2.out_i_sum = 0;
    acrosspid3.out_i_sum = 0;
    acrosspid4.out_i_sum = 0;
    acrosspid1.last_last_error =0;
    acrosspid2.last_last_error =0;
    acrosspid3.last_last_error =0;
    acrosspid4.last_last_error =0;
    acrosspid1.out = 0;
    acrosspid2.out = 0;
   acrosspid3.out = 0;
    acrosspid4.out = 0;
}
