/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/


#include "headfile.h"



void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_BRK_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SPI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void RTCAlarm_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void USBWakeUp_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void USBHD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#define LINE_LEN                13              //数据长度
uint8 temp_buff[LINE_LEN];                      //从机向主机发送数据BUFF
extern  int8 cameraerr;
int16 movemode = 0;
uint8 key4_status = 1, key3_status = 1;
uint8 key4_last_status, key3_last_status;
uint8 key4_flag, key3_flag;
uint8 start = 0;
extern uint8_t Branchprotection,Rampprotection;
float angle = 0;
int startflag =0;
extern uint8_t isBranch,isZebra,isRamp,isRoundabout;
extern uint8_t startdir;
void get_sensor_data(void)
{
    //这里仅仅是提供一个模拟数据
    get_icm20602_gyro_spi();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议处理数据
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void process_data(void)
{
  //  if(startflag ==0)  cameraerr =(90-fabs((int8)angle))*4/3;
   // if(startflag==0&&cameraerr<20)  cameraerr = 20;
    temp_buff[0] = 0xD8;                         //帧头
    temp_buff[1] = 0xB0;                         //功能字
    temp_buff[2] = movemode>>8;        //数据高8位
    temp_buff[3] = movemode&0xFF;      //数据低8位

    temp_buff[4] = 0xB1;                         //功能字
    temp_buff[5] = ((int16)angle)>>8;       //数据高8位
    temp_buff[6] = ((int16)angle)&0xFF;     //数据低8位

    temp_buff[7] = 0xB2;                         //功能字
    temp_buff[8] = (int16)isRoundabout>>8;            //数据高8位
    temp_buff[9] = (int16)isRoundabout&0xFF;          //数据低8位

    temp_buff[10] = 0xB3;
    temp_buff[11] = cameraerr;

    temp_buff[12] = 0xEE;                        //帧尾

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4中断服务函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
void EXTI0_IRQHandler(void)
{
    if(SET == EXTI_GetITStatus(EXTI_Line0))
    {
        if(mt9v03x_finish_flag)
        {  ImageProcess();
        mt9v03x_finish_flag = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI1_IRQHandler(void)
{
    if(SET == EXTI_GetITStatus(EXTI_Line1))
    {
        if(camera_type == CAMERA_BIN_UART)
            ov7725_uart_vsync();
        else if(camera_type == CAMERA_GRAYSCALE)
            mt9v03x_vsync();
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

}

void EXTI2_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{


}

void EXTI4_IRQHandler(void)
{


}

void EXTI9_5_IRQHandler(void)
{


}

void EXTI15_10_IRQHandler(void)
{

}

void ADC1_2_IRQHandler(void)
{


}

void TIM1_BRK_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Break);

    }
}

void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    }
}

void TIM1_TRG_COM_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Trigger) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Trigger);

    }
    if(TIM_GetITStatus(TIM1, TIM_IT_COM) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_COM);

    }
}

void TIM1_CC_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

    }
    if(TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

    }
    if(TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

    }
    if(TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

    }
}

void TIM2_IRQHandler(void)
{

    float deangle;
    static int branchflag1=0;
    static int branchflag2=0;
    static int branchflag3=0;
    static int branchflag4=0;
    static int zebraflag = 0;
    deangle = (float)icm_gyro_z* 2000.0 / 32767;
    static int count = 0;
    static int sound = 0;


    //ips114_showint8(0,0,isBranch);
    //pwm_init(PWM4_CH4_B9,sound,5000);
    sound +=  10;
    // start
    if(movemode==1)
    {
        angle+= deangle*0.05;
        if(angle>90||angle<-90) {startflag=1;  movemode =2;}
    }


   //  branch
    if(isBranch == 1)
    {
        movemode = 2;
        count = 0;
    }
   if(isBranch == 2)
   {
      if(branchflag1==0)  angle =0;
       branchflag1 =1;
       if(movemode == 3) angle+= deangle*0.05;
       if(count <= 3)
       {
           movemode = 7;
           count++;
       }
       else movemode = 3;
       if(angle>5 ||angle <-5) {movemode = 0;isBranch = 3; count = 0;branchflag1 = 0;Branchprotection = 0;}
   }
   if(isBranch == 3)
   {
       if(count <= 1)
       {
           movemode = 0;
           count++;
       }
       /*else if(count <= 15)
       {
           movemode = 15;
           count++;
       }*/
       else movemode = 5;
   }
   if(isBranch == 4)
   {
       count = 0;
       movemode = 5;
   }

   if(isBranch == 5)
   {
       if(branchflag2==0)  angle =0;
        branchflag2 =1;
        if(movemode == 4) angle+= deangle*0.05;
        if(count <= 0)
        {
            movemode = 5;
            count++;
        }
        else movemode =4;
        if(angle>105 ||angle <-105) {movemode =0;isBranch =6; count = 0;branchflag2 = 0;Branchprotection = 0;Rampprotection = 150;}
   }
   if(isBranch == 6 && start == 1 && isRamp % 2 == 0 && isZebra != 3)
   {
       if(count <= 1)
       {
           count++;
           movemode = 0;
       }
       /*else if(count <= 15)
       {
           movemode = 16;
           count++;
       }
       else if(count >= 100 && count <= 102)
       {
           count++;
           movemode = 0;
       }*/
       else if(count > 102 && count <= 104)
       {
           count++;
           movemode = 12;
       }
       else movemode = 2;
   }
   if(isBranch == 7)
   {
       movemode = 2;
       count = 0;
   }
   if(isBranch == 8)
   {
       if(branchflag3==0)  angle =0;
        branchflag3 =1;
        if(movemode == 3) angle+= deangle*0.05;
        if(count <= 3)
        {
            count++;
            movemode = 7;
        }
        else movemode =3;
        //ips114_showint16(100,100,angle);
        if(angle>105 ||angle <-105) {movemode =0;isBranch =9; count = 0;branchflag3 = 0;Branchprotection = 0;}
   }
   if(isBranch == 9)
   {
       if(count <= 1)
       {
           movemode = 0;
           count++;
       }
       /*else if(count<= 15)
       {
           movemode = 15;
           count++;
       }*/
       else movemode = 5;
   }
   if(isBranch == 10)
   {
       count = 0;
       movemode = 5;
   }
   if(isBranch == 11)
   {
       if(branchflag4==0)  angle =0;
        branchflag4=1;
        if(movemode == 4) angle+= deangle*0.05;
        if(count <= 0)
        {
            movemode = 5;
            count++;
        }
        movemode =4;
        if(angle>10 ||angle <-10) {movemode =0;isBranch =0; count = 0;movemode = 2;branchflag4 = 0;Branchprotection = 0;Rampprotection = 150;}
   }
   if(isBranch == 0 && start == 1 && isRamp % 2 == 0 && isZebra != 3)
   {
       if(count <= 1)
       {
           count++;
           movemode = 0;
       }
       /*else if(count <= 15)
       {
           count++;
           movemode = 16;
       }*/
       else if(count >= 100 && count <= 102)
       {
           count++;
           movemode = 0;
       }
       else if(count > 102 && count <= 104)
       {
           count++;
           movemode = 12;
       }
       else movemode = 2;
   }
   if(isZebra == 3)
   {
       if(zebraflag == 0) angle = 0;
       zebraflag = 1;
       angle+= deangle*0.05;
       if(angle > 60 || angle < -60) movemode = 0;
   }
   /*if(isRamp==1)
   {
       if(rampflag==0)
           {angle = 0;
           rampflag =1;

           }
       if(rampflag == 1)
       {
           deangle = (float)icm_gyro_x* 2000.0 / 32767;
                  angle+=deangle*0.05;
       }
       if((angle < -8 && isBranch == 0) || (angle > 8 && isBranch == 6))

       {
           rampflag = 0;
           isRamp = 2;
           pwm_duty(PWM4_CH4_B9,5000);
       }
   }
   if(isRamp == 2)
   {
       deangle = (float)icm_gyro_x* 2000.0 / 32767;
       angle+=deangle*0.05;
       if((isBranch == 0 && angle >= 0) || (isBranch == 6 && angle <= 0))
       {
           isRamp = 3;
           pwm_duty(PWM4_CH4_B9,5000);
       }
   }
   if(isRamp == 3)
   {
       if(rampflag == 0)
       {
           if(movemode == 2) movemode = 7;
           else if(movemode == 6) movemode = 11;
       }
       rampflag++;
       if(rampflag > 40)
       {
           if(movemode == 7) movemode = 2;
           else if(movemode == 11) movemode = 6;
           rampflag = 0;
           isRamp = 0;
       }
   }*/
   // if(angle>30 || angle <-30) movemode =2;
    //ips114_showfloat(50,0,deangle,5,2);
   //ips114_showint16(50,0,(int16)angle);


    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        GPIO_PIN_SET(D2);                           //A0引脚拉高

        get_sensor_data();                          //获取传感器数据。
        process_data();                             //根据协议处理数据，并存入temp_buff中。
        uart_putbuff(UART_3, temp_buff, 13);  //通过串口3将数据发送出去。

        GPIO_PIN_RESET(D2);                         //A0引脚拉低

        key4_last_status = key4_status;

        key4_status = gpio_get(B8);


        if(key4_status && !key4_last_status)    key4_flag = 1;



        if(key4_flag)
        {
            key4_flag = 0;//使用按键之后，应该清除标志位
            movemode =1;

            /*count = 101;
            movemode = 2;
            start = 1;*/
        }

        key3_last_status = key3_status;
        key3_status = gpio_get(B12);
        if(key3_status && !key3_last_status) key3_flag = 1;
        if(key3_flag == 1)
        {
            key3_flag = 0;
            if(startdir == 1) startdir = 2;
            else startdir = 1;
        }

        if(movemode == 1)
        {
            if(count <= 15)
            {
                count++;
            }
            else
            {
                count = 101;
                movemode = 2;
                start = 1;
            }
        }


        //ips114_showint8(0,0,cameraerr);

        //gpio_toggle(C5);
    }
}

void TIM3_IRQHandler(void)
{

    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update );

    }
}

//void TIM4_IRQHandler(void)
//{
//    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM4, TIM_IT_Update );
//
//    }
//}

void USART1_IRQHandler(void)
{

}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        if(camera_type == CAMERA_BIN_UART)
            ov7725_cof_uart_interrupt();
        else if(camera_type == CAMERA_GRAYSCALE)
            mt9v03x_uart_callback();
    }
}

void USART3_IRQHandler(void)
{

}

void DMA1_Channel4_IRQHandler(void)
{
    if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);
        if(camera_type == CAMERA_BIN_UART)
            ov7725_uart_dma();
        else if(camera_type == CAMERA_GRAYSCALE)
            mt9v03x_dma();
    }
}

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{

}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{

  while (1)
  {
  }
}



