#include "headfile.h"
#define E_START 0              //准备状态
#define E_OK 1                 //成功
#define E_FRAME_HEADER_ERROR 2 //帧头错误
#define E_FRAME_RTAIL_ERROR 3  //帧尾错误

#define LINE_LEN 13 //数据长度
#define LEN 20
uint32 sectionbuf[LEN];
uint32 sectionbuff[LEN];
float startangel = 0;
float currentyaw;
float currentpitch;
float dangle;
uint8 temp_buff[LINE_LEN]; //主机用于接收数据的BUFF
vuint8 uart_flag;          //接收数据标志位
int16 Encoder[4];
int16_t TargetVelocityX = 30; //期望速度
int16_t TargetVelocityY = 0;
int16_t TargetVelocityZ = 0;
int16_t startdir =1;   //1 left 2 right
int16_t isbranch=0;
extern int32 encoder1integrator,encoder2integrator,encoder3integrator,encoder4integrator;

extern int16_t cameraerr;
extern pid encoderpid,angelpid;
extern int16 movemode;
extern pid startpid1,startpid2,startpid3,startpid4,acrosspid1,acrosspid2,acrosspid3,acrosspid4,acrossangelpid;
//int startdir = 0;    // 1 左出库   2 右出库
int16 angle;
void ips_show(int i);

uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;

void get_slave_data(uint8 data)
{
    static uint8 uart_num = 0;
    temp_buff[uart_num++] = data;

    if (1 == uart_num)
    {
        //接收到的第一个字符不为0xD8，帧头错误
        if (0xD8 != temp_buff[0])
        {
            uart_num = 0;
            uart_flag = E_FRAME_HEADER_ERROR;
        }
    }

    if (LINE_LEN == uart_num)
    {
        uart_flag = E_OK;
        //接收到最后一个字节为0xEE
        if (0xEE == temp_buff[LINE_LEN - 1])
        {
            uart_flag = E_OK;
        }
        else //接收到最后一个字节不是0xEE，帧尾错误
        {
            uart_flag = E_FRAME_RTAIL_ERROR;
        }
        uart_num = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议对从机发送过来的数据，进行数据解析
//  @param      *line                           串口接收到的数据BUFF
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void data_analysis(uint8 *line)
{
    if (line[1] == 0xB0)
        movemode = ((int16)line[2] << 8) | line[3];
    if (line[4] == 0xB1)
        angle = ((int16)line[5] << 8) | line[6];
    if (line[7] == 0xB2)
        isbranch= ((int16)line[8] << 8) | line[9];
    if (line[10] == 0xB3)
        cameraerr = (line[11]);
    icm_gyro_x *= 2000.0 / 32767;
    icm_gyro_y *= 2000.0 / 32767;
    dangle = (float)icm_gyro_z* 2000.0 / 32767;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口3中断服务函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    uint8 dat;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除串口接收中断标志位
        dat = USART_ReceiveData(USART3);                //获取串口数据
        get_slave_data(dat);                            //将每一个字节的串口数据存入temp_buff中。
    }
}
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void)
{
    static int i = 0;

    if (SET == EXTI_GetITStatus(EXTI_Line2))
    {
        EXTI_ClearITPendingBit(EXTI_Line2);

        while (uart_flag != 1)  ;                     //等待接收数据
        uart_flag = 0;            //清空标志位
        data_analysis(temp_buff); //数据解析


        movecontrol();
       // if(movemode == 1)currentyaw +=  0.002*(float)icm_gyro_z;
        // control_loc((int16)cameraerr);
        key1_last_status = key1_status;
        key2_last_status = key2_status;
        key3_last_status = key3_status;

        key1_status = gpio_get(B0);
        key2_status = gpio_get(B1);
        key3_status = gpio_get(B2);

        if (key1_status && !key1_last_status)
            key1_flag = 1;
        if (key2_status && !key2_last_status)
            key2_flag = 1;
        if (key3_status && !key3_last_status)
            key3_flag = 1;

        if (key1_flag)
        {

            key1_flag = 0;
            switch(i)
            {
    /*        case 0: startdir=1;break;
            case 1:TargetVelocityX+=5;break;
            case 2: pid1.kp+=1;break;
            case 3: pid1.ki+=0.1;break;
            case 4: pid1.kd+=1;break;
            case 5: angelpid.kp+=0.1;break;
            case 6: angelpid.kd+=0.1;break;
            case 7: TargetVelocityY+=5;break;
            case 8: acrosspid1.kp+=1;break;
            case 9: acrosspid1.ki+=0.1;break;
            case 10: acrosspid2.kp+=1;break;
            case 11: acrosspid2.ki+=0.1;break;
            case 12: acrosspid3.kp+=1;break;
            case 13: acrosspid3.ki+=0.1;break;
            case 14: acrosspid4.kp+=1;break;
            case 15: acrosspid4.ki+=0.1;break;*/

           //  case 0: startdir=1;break;


            case 0:TargetVelocityX+=5;break;
            case 1: pid1.kp+=1;break;
            case 2: pid1.ki+=0.1;break;
            case 3: angelpid.kp+=0.1;break;
            case 4: angelpid.kd+=0.1;break;
            case 5: TargetVelocityY+=5;break;
            case 6: acrossangelpid.kp+=0.1;break;
            case 7: acrossangelpid.kd+=0.1;break;

            case 8: acrosspid1.kp+=1;break;
             case 9: acrosspid1.ki+=0.1;break;
             case 10: acrosspid2.kp+=1;break;
             case 11: acrosspid2.ki+=0.1;break;
             case 12: acrosspid3.kp+=1;break;
             case 13: acrosspid3.ki+=0.1;break;
             case 14: acrosspid4.kp+=1;break;
             case 15: acrosspid4.ki+=0.1;break;


            }
          //  sectionbuf[0]=(uint32)startdir;
            sectionbuf[0]=(uint32)(acrossangelpid.kp*10);
            sectionbuf[1]=(uint32)TargetVelocityX;
            sectionbuf[2]=(uint32)(pid1.kp*10);
            sectionbuf[3]=(uint32)(pid1.ki*10);
            sectionbuf[4]=(uint32)(acrossangelpid.kd*10);
            sectionbuf[5]=(uint32)(angelpid.kp*10);
            sectionbuf[6]=(uint32)(angelpid.kd*10);
            sectionbuf[7]=(uint32)TargetVelocityY;
            sectionbuf[8]=(uint32)(acrosspid1.kp*10);
            sectionbuf[9]=(uint32)(acrosspid1.ki*10);
            sectionbuf[10]=(uint32)(acrosspid2.kp*10);
            sectionbuf[11]=(uint32)(acrosspid2.ki*10);
            sectionbuf[12]=(uint32)(acrosspid3.kp*10);
            sectionbuf[13]=(uint32)(acrosspid3.ki*10);
            sectionbuf[14]=(uint32)(acrosspid4.kp*10);
            sectionbuf[15]=(uint32)(acrosspid4.ki*10);


            flash_page_program(FLASH_SECTION_15, FLASH_PAGE_0, sectionbuf, 20);

        }

        if (key2_flag)
        {

            key2_flag = 0;
            switch(i)
            {

            //case 0: startdir=2;break;

            case 0:TargetVelocityX-=5;break;
            case 1: pid1.kp-=1;break;
            case 2: pid1.ki-=0.1;break;
            case 3: angelpid.kp-=0.1;break;
            case 4: angelpid.kd-=0.1;break;
            case 5: TargetVelocityY-=5;break;
            case 6: acrossangelpid.kp-=0.1;break;
            case 7: acrossangelpid.kd-=0.1;break;

            case 8: acrosspid1.kp-=1;break;
             case 9: acrosspid1.ki-=0.1;break;
             case 10: acrosspid2.kp-=1;break;
             case 11: acrosspid2.ki-=0.1;break;
             case 12: acrosspid3.kp-=1;break;
             case 13: acrosspid3.ki-=0.1;break;
             case 14: acrosspid4.kp-=1;break;
             case 15: acrosspid4.ki-=0.1;break;

            }
              sectionbuf[0]=(uint32)(acrossangelpid.kp*10);
              sectionbuf[1]=(uint32)TargetVelocityX;
              sectionbuf[2]=(uint32)(pid1.kp*10);
              sectionbuf[3]=(uint32)(pid1.ki*10);
              sectionbuf[4]=(uint32)(acrossangelpid.kd*10);
              sectionbuf[5]=(uint32)(angelpid.kp*10);
              sectionbuf[6]=(uint32)(angelpid.kd*10);
              sectionbuf[7]=(uint32)TargetVelocityY;
              sectionbuf[8]=(uint32)(acrosspid1.kp*10);
              sectionbuf[9]=(uint32)(acrosspid1.ki*10);
              sectionbuf[10]=(uint32)(acrosspid2.kp*10);
              sectionbuf[11]=(uint32)(acrosspid2.ki*10);
              sectionbuf[12]=(uint32)(acrosspid3.kp*10);
              sectionbuf[13]=(uint32)(acrosspid3.ki*10);
              sectionbuf[14]=(uint32)(acrosspid4.kp*10);
              sectionbuf[15]=(uint32)(acrosspid4.ki*10);


            flash_page_program(FLASH_SECTION_15, FLASH_PAGE_0, sectionbuf, 20);
        }

        if (key3_flag)
        {
            i++;
            if (i > 16)
                i = 0;

            key3_flag = 0;

        }

        //systick_delay_ms(10);//延时，按键程序应该保证调用时间不小于10ms
       ips_show(i);
    }
}
extern pid angelpid;
//uint8_t startflag = 0;
uint8_t B0state = 0;
uint8_t B1state = 0;
uint8_t A9state = 0;
uint8_t A10state = 0;

void ips_show(int i)
{
    static int lasti;
  //  ips114_clear(WHITE);

    ips114_showint16(50, 0, TargetVelocityX);
    ips114_showfloat(50, 1, pid1.kp, 2, 2);
    ips114_showfloat(50, 2, pid1.ki, 2, 2);
   ips114_showfloat(50, 3, angelpid.kp, 2, 2);
    ips114_showfloat(50, 4, angelpid.kd, 2, 2);
    ips114_showint16(50,5,TargetVelocityY);
    ips114_showfloat(50,6,acrossangelpid.kp,2,2);
    ips114_showfloat(50,7,acrossangelpid.kd,2,2);

    ips114_showint8(100,0,cameraerr);

    ips114_showfloat(150,0,acrosspid1.kp,3,2);
    ips114_showfloat(150,1,acrosspid1.ki,3, 2);
    ips114_showfloat(150,2,acrosspid2.kp,3,2);
    ips114_showfloat(150,3,acrosspid2.ki,3, 2);
    ips114_showfloat(150,4,acrosspid3.kp,3,2);
    ips114_showfloat(150,5,acrosspid3.ki,3, 2);
    ips114_showfloat(150,6,acrosspid4.kp,3,2);
    ips114_showfloat(150,7,acrosspid4.ki,3, 2);

    if(i>7) i-=8;
    for(int k = 4; k <= 12; k++)
    {
        ips114_drawpoint(0, 16 * i + k,WHITE);
        ips114_drawpoint(1, 16 * i + k, WHITE);
        ips114_drawpoint(2, 16 * i + k, WHITE);
        ips114_drawpoint(3, 16 * i + k, WHITE);
        ips114_drawpoint(4, 16 * i + k, WHITE);
        ips114_drawpoint(5, 16 * i + k, WHITE);
        if(i != lasti)
        {ips114_drawpoint(0, 16 * lasti + k, BLACK);
        ips114_drawpoint(1, 16 * lasti + k, BLACK);
        ips114_drawpoint(2, 16 * lasti + k,BLACK);
        ips114_drawpoint(3, 16 * lasti + k, BLACK);
        ips114_drawpoint(4, 16 * lasti + k, BLACK);
        ips114_drawpoint(5, 16 * lasti + k,BLACK);}
    }
    lasti = i;
    //printf("%d\r\n",i);
}

int main(void)
{
    DisableGlobalIRQ();
    board_init(); //务必保留，本函数用于初始化MPU 时钟 调试串口
     //530 990
    ips114_init();
    pwm_init(PWM2_CH1_A0, 50, 550); //舵机初始化   250 - 1250   230   700   1170
    Motor_Init();
    Enc_Init();

    timer_pit_interrupt_ms(TIMER_3, 2);
    nvic_init(TIM3_IRQn, 0, 0, ENABLE);
    //  uart_init(UART_1,115200,UART1_TX_A9,UART1_RX_A10);
    // uart_rx_irq(UART_1,ENABLE);
    // nvic_init((IRQn_Type)(53 + UART_1), 0, 1, ENABLE);
    // EnableGlobalIRQ(0);
    // while(startflag == 0);
    // mt9v03x_init();
    uart_init(UART_2, 115200, UART2_TX_A2, UART2_RX_A3);
 //   uart_rx_irq(UART_2, ENABLE);                           //默认抢占优先级0 次优先级0。
 //   nvic_init((IRQn_Type)(53 + UART_2), 1, 7, ENABLE);     //将串口3的抢占优先级设置为最高，次优先级设置为最高。

    uart_init(UART_3, 115200, UART3_TX_B10, UART3_RX_B11); //串口3初始化，波特率460800
    uart_rx_irq(UART_3, ENABLE);                           //默认抢占优先级0 次优先级0。
    nvic_init((IRQn_Type)(53 + UART_3), 0, 1, ENABLE);     //将串口3的抢占优先级设置为最高，次优先级设置为最高。
    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据
    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据
    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据

    gpio_interrupt_init(D2, RISING, GPIO_INT_CONFIG); //A0初始化为GPIO 上升沿触发
    nvic_init(EXTI2_IRQn, 1, 1, ENABLE);              //EXTI0优先级配置，抢占优先级1，次优先级1

    /*gpio_init(A9,GPI,0,GPIO_Mode_IPD);
    gpio_init(A10,GPI,0,GPIO_Mode_IPD);*/

    EnableGlobalIRQ(0);
    while (1)
    {
    }
}
