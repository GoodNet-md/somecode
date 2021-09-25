#include "ImageProcess.h"

#define MAIN_ROAD MT9V03X_H * 3 / 5

int8_t cameraerr = 0, lastcameraerr = 0;

extern int16 movemode;

uint8_t leftside[MT9V03X_H], rightside[MT9V03X_H], midline[MT9V03X_H], width[MT9V03X_H];
uint8_t ZebraFlag = 0, BranchFlag = 0;
uint8_t leftjumpingpoint[5], rightjumpingpoint[5], leftjumpingsum, rightjumpingsum;
uint8_t leftextremepoint[5][2], rightextremepoint[5][2], leftextremesum, rightextremesum;        //1极左2极右
uint8_t leftlose, rightlose;

uint8_t isRoundabout = 0, isBranch = 0, isZebra = 0, isRamp = 0, isCross = 0;

uint8_t threshold;

uint8_t startdir = 1;
uint8_t Rampcount = 0;
uint16_t Branchprotection = 51, Rampprotection = 201;

void ImageProcess(void)
{
    //threshold = GetOSTU();
    threshold = 140;
    Get_01_Value();

    GetSide();
    findextremepoint();
    pwm_duty(PWM4_CH4_B9,0);

    /*if(isRoundabout == 0 && (isBranch == 0 || isBranch == 6) && isZebra % 2 == 0 && isCross == 0 && isRamp % 2 == 0)
    {
        checkZebra(startdir);
    }
    if(isZebra % 2 == 1)
    {
        ZebraProcess(startdir);
    }*/

    if(isRoundabout == 0 && (isBranch == 0 || isBranch == 6) && isZebra % 2 == 0 && isCross == 0 && isRamp % 2 == 0)
    {
        checkRoundabout();
    }
    if(isRoundabout != 0)
    {
        RoundaboutProcess();
    }

    if(isRoundabout == 0 && (isBranch == 0 || isBranch == 6) && isZebra % 2 == 0 && isCross == 0 && Branchprotection > 50 && isRamp % 2 == 0)
    {
        checkBranch();
    }
    if(isBranch != 0 && isBranch != 6)
    {
        BranchProcess();
    }
    if(Branchprotection <= 50) Branchprotection++;
    if(Rampprotection <= 200) Rampprotection++;

    if(isRoundabout == 0 && (isBranch == 0 || isBranch == 6) && isZebra % 2 == 0 && isCross == 0 && isRamp % 2 == 0 && Rampprotection > 200)
    {
        checkRamp();
    }
    if(isRamp % 2 == 1)
    {
        pwm_duty(PWM4_CH4_B9,5000);
        if(Rampcount <= 15)
        {
            Rampcount++;
        }
        else if(Rampcount <= 120 )
        {
            Rampcount++;
            if(isRamp % 4 == 3) movemode = 13;
        }
        else
        {
            isRamp++;
            movemode = 2;
            Rampprotection = 0;
        }
    }

    if(isRoundabout == 0 && (isBranch == 0 || isBranch == 6) && isZebra % 2 == 0 && isCross == 0)
    {
        checkCross();
    }
    if(isCross == 1)
    {
        if(isRamp % 2 == 1) isRamp++;
        CrossProcess();
    }

    //printside();
    //ips114_showuint8(100,50,rightside[0] - leftside[0]);
    //ips114_showuint8(100,100,rightside[49] - leftside[49]);

    //ips114_displayimage032_zoom(mt9v03x_image[0],MT9V03X_W,MT9V03X_H,IPS114_X_MAX,IPS114_Y_MAX);
    //ips114_showuint8(50,50,leftextremepoint[0][0]);
    //ips114_showuint8(150,50,rightextremepoint[0][0]);
    //ips114_showuint8(100,50,isZebra);
    //ips114_showint16(100,100,icm_gyro_y);
    //ips114_showuint8(100,100,isRoundabout);

    if(isRamp % 4 == 1)
    {
        cameraerr = MT9V03X_W / 2 - (leftside[45] + rightside[45]) / 2;
    }
    else if(isRamp % 4 == 3)
    {
        cameraerr = MT9V03X_W / 2 - (leftside[30] + rightside[30]) / 2;
    }
    else
    {
        if(leftside[25] < rightside[25])
            midline[25] = (leftside[25] + rightside[25]) / 2;
        else midline[25] = MT9V03X_W / 2 + 8;
        if(isBranch != 1 && isBranch != 7 && isBranch != 3 && isBranch != 4 && isBranch != 9 && isBranch != 10)
        {
                cameraerr = MT9V03X_W / 2 - midline[25];
        }
        else if(isBranch == 3 || isBranch == 4 || isBranch == 9 || isBranch == 10)
            cameraerr = MT9V03X_W / 2 - (leftside[MAIN_ROAD+15] + rightside[MAIN_ROAD+15]) / 2;
        else cameraerr = MT9V03X_W / 2 - (leftside[MAIN_ROAD+10] + rightside[MAIN_ROAD+10]) / 2;
    }


    cameraerr -= 8;
    //ips114_showuint8(100,150,cameraerr);
    if(movemode == 5 && Branchprotection <= 15) cameraerr = 0;

    if(lastcameraerr  > cameraerr + 50 || lastcameraerr  < cameraerr - 50) cameraerr = lastcameraerr / 2 + cameraerr / 2;
    if(cameraerr > 65) cameraerr = 65;
    else if(cameraerr < - 65) cameraerr = -65;

    lastcameraerr = cameraerr;
}

void printside()
{
    int16_t i, j;
    for(i = 0; i <= MT9V03X_H - 1; i++)
    {
        for(j = 0; j <= MT9V03X_W - 1; j++)
        {
            mt9v03x_image[i][j] = 255;
        }
    }
    for(i = 0; i <= MT9V03X_H - 1; i++)
    {
        mt9v03x_image[i][leftside[i]] = 0;
        mt9v03x_image[i][rightside[i]] = 0;
    }
    for(i = 0; i <= MT9V03X_W - 1; i++)
        mt9v03x_image[MAIN_ROAD][i] = 0;
}

void Get_01_Value()
{
    uint8_t i, j;
    for(i = 0; i <= MT9V03X_H - 1; i++)
    {
        for(j = 0; j <=MT9V03X_W - 1; j++)
        {
            if(mt9v03x_image[i][j] >= threshold) mt9v03x_image[i][j] = 255;
            else mt9v03x_image[i][j] = 0;
        }
    }
}

void clearside()
{
    uint8_t i;
    for(i = 0; i <= MT9V03X_H-1 ; i++)
    {
        leftside[i] = 0;
        rightside[i] = MT9V03X_W - 1;
        width[i] = 36 + i * 8 / 5;
    }
    leftlose = rightlose = 0;
    leftjumpingsum = rightjumpingsum = 0;
    leftextremesum = rightextremesum = 0;
    for(i = 0; i <= 4; i++)
    {
        leftjumpingpoint[i] = rightjumpingpoint[i] = 0;
        leftextremepoint[i][0] = leftextremepoint[i][1] = rightextremepoint[i][0] = rightextremepoint[i][1] = 0;
    }
}

void GetSide()
{
    int16_t i, j;
    clearside();
    for(i = MT9V03X_H - 1; i >= MT9V03X_H * 5 / 6; i--)
    {
        j = 1;
        while(j <= MT9V03X_W / 2)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
            {
                leftside[i] = j;
                break;
            }
            j++;
        }
        j = MT9V03X_W - 2;
        while(j >= MT9V03X_W / 2)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
            {
                rightside[i] = j;
                break;
            }
            j--;
        }
        if(i <= MT9V03X_H - 5)
        {
            findjumpingpoint(i);
            if(leftside[i] == 0) leftlose++;
            if(rightside[i] == 0) rightlose++;
        }
    }
   for(i = MT9V03X_H * 5 / 6 - 1; i >= 0; i--)
   {
        if(leftside[i+1] != 0)
        {
            j = (leftside[i+1] - 25 >= 1)?(leftside[i+1] - 25):1;
            while(j <= leftside[i+1] + 25 && j <= MT9V03X_W - 2)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
                {
                    leftside[i] = j;
                    break;
                }
                j++;
            }
        }
        else
        {
            j = 1;
            while(j <= MT9V03X_W * 3 / 5 && j <= rightside[i+1])
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
                {
                    leftside[i] = j;
                    break;
                }
                j++;
            }
        }

        if(rightside[i+1] != MT9V03X_W - 1)
        {
            j = (rightside[i+1] + 25 <= MT9V03X_W - 2)?(rightside[i+1] + 25):MT9V03X_W - 2;
            while(j >= 1 && j >= rightside[i+1] - 25)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                {
                    rightside[i] = j;
                    break;
                }
                j--;
            }
        }
        else
        {
            j = MT9V03X_W - 2;
            while(j >= leftside[i+1] && j >= MT9V03X_W * 2 / 5)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                {
                    rightside[i] = j;
                    break;
                }
                j--;
            }
        }

        if(leftside[i] == 0 && i >= MT9V03X_H / 5) leftlose++;
        if(rightside[i] == MT9V03X_W - 1 && i >= MT9V03X_H / 5) rightlose++;

        //找跳变点
        if(i >= MT9V03X_H / 10) findjumpingpoint(i);
   }
}

void findjumpingpoint(uint8_t now)
{
    if(leftside[now+1] >= MT9V03X_W / 6 && ((leftside[now] == 0 && leftside[now+2] != 0)
            || (leftside[now+2] == 0 && leftside[now] != 0)) && leftjumpingsum < 5)
    {
        leftjumpingpoint[leftjumpingsum] = now + 1;
        leftjumpingsum++;
    }
    if(rightside[now+1] <= MT9V03X_W * 5 / 6 && ((rightside[now] == MT9V03X_W - 1 && rightside[now+2] != MT9V03X_W - 1)
            || (rightside[now+2] == MT9V03X_W - 1 && rightside[now] != MT9V03X_W - 1))&& rightjumpingsum < 5)
    {
        rightjumpingpoint[rightjumpingsum] = now+1;
        rightjumpingsum++;
    }
}

void findextremepoint()
{
    int16_t i;
    uint8_t leftpre = MT9V03X_H - 6, rightpre = MT9V03X_H - 6, leftpre2 = MT9V03X_H - 5, rightpre2 = MT9V03X_H - 5;
    uint8_t leftpre3 = MT9V03X_H - 4, rightpre3 = MT9V03X_H - 4, leftpre4 = MT9V03X_H - 3, rightpre4 = MT9V03X_H - 3;
    for(i = MT9V03X_H - 6; i >= MT9V03X_H / 15; i--)
    {
        if(leftside[leftpre2] < leftside[leftpre] && leftside[leftpre] < leftside[i]
                && leftside[leftpre2] < leftside[leftpre3] && leftside[leftpre3] < leftside[leftpre4] && leftextremesum <= 5
                && leftside[leftpre] != 0 && leftside[leftpre2] != 0 && leftside[i] != 0
                && leftside[leftpre3] != 0 && leftside[leftpre4] != 0)
        {
            leftextremepoint[leftextremesum][0] = leftpre2;
            leftextremepoint[leftextremesum][1] = 1;
            leftextremesum++;
        }
        if(leftside[leftpre2] > leftside[leftpre] && leftside[leftpre] > leftside[i]
                && leftside[leftpre2] > leftside[leftpre3] && leftside[leftpre3] > leftside[leftpre4] && leftextremesum <= 5
                && leftside[leftpre] != 0 && leftside[leftpre2] != 0 && leftside[i] != 0
                && leftside[leftpre3] != 0 && leftside[leftpre4] != 0)
        {
            leftextremepoint[leftextremesum][0] = leftpre2;
            leftextremepoint[leftextremesum][1] = 2;
            leftextremesum++;
        }

        if(rightside[rightpre2] < rightside[rightpre] && rightside[rightpre] < rightside[i]
                && rightside[rightpre2] < rightside[rightpre3] && rightside[rightpre3] < rightside[rightpre4] && rightextremesum <= 5
                && rightside[rightpre] != MT9V03X_W - 1 && rightside[rightpre2] != MT9V03X_W - 1 && rightside[i] != MT9V03X_W - 1
                && rightside[rightpre3] != MT9V03X_W - 1 && rightside[rightpre4] != MT9V03X_W - 1)
        {
            rightextremepoint[rightextremesum][0] = rightpre;
            rightextremepoint[rightextremesum][1] = 1;
            rightextremesum++;
        }
        if(rightside[rightpre2] > rightside[rightpre] && rightside[rightpre] > rightside[i]
                && rightside[rightpre2] > rightside[rightpre3] && rightside[rightpre3] > rightside[rightpre4] && rightextremesum <= 5
                && rightside[rightpre] != MT9V03X_W - 1 && rightside[rightpre2] != MT9V03X_W - 1 && rightside[i] != MT9V03X_W - 1
                && rightside[rightpre3] != MT9V03X_W - 1 && rightside[rightpre4] != MT9V03X_W - 1)
        {
            rightextremepoint[rightextremesum][0] = rightpre;
            rightextremepoint[rightextremesum][1] = 2;
            rightextremesum++;
        }

        if(leftside[i] != leftside[leftpre])
        {
            leftpre4 = leftpre3;
            leftpre3 = leftpre2;
            leftpre2 = leftpre;
            leftpre = i;
        }
        if(rightside[i] != rightside[rightpre])
        {
            rightpre4 = rightpre3;
            rightpre3 = rightpre2;
            rightpre2 = rightpre;
            rightpre = i;
        }

        if(leftextremesum > 5 && rightextremesum > 5) break;
    }
}

short GetOSTU (void)
{
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  unsigned char HistoGram[256];              //

  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //初始化灰度直方图

  for (j = 0; j < MT9V03X_H; j++)
  {
    for (i = 0; i < MT9V03X_W; i++)
    {
      HistoGram[mt9v03x_image[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }

  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MinValue + 1 == MaxValue)
    return MinValue;        // 图像中只有二个颜色

  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数

  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数
    PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
    OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
    PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
    MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
}

void checkRoundabout()                     //1是左环2是右环
{
    int16_t i, j;
    uint8_t leftcontinue = 1, rightcontinue = 1;
    //uint8_t zebra = 1;
    for(i = MT9V03X_H * 4 / 5; i >= MT9V03X_H / 10; i--)
    {
        if(leftside[i] < leftside[i+1] || (leftside[i] - leftside[i+1] > 2 && leftside[i] >= leftside[i+1])) leftcontinue = 0;
        if(rightside[i] > rightside[i+1] || (rightside[i+1] - rightside[i] > 2 && rightside[i+1] >= rightside[i])) rightcontinue = 0;
        //uint8_t count = 0;
        //for(j = 2; j <= MT9V03X_W - 3; j++)
        //{
        //    if(mt9v03x_image[i][j] == 255 && mt9v03x_image[i][j+1] == 0) count++;
        //}
        //if(count >= 3) zebra = 0;
    }
    //if(zebra == 0) return;
    if(leftlose >= MT9V03X_H / 6 && rightlose <= 2 && leftjumpingsum > 0
         && rightjumpingsum == 0 && leftjumpingpoint[0] >= MT9V03X_H / 2 && rightcontinue == 1)//左环
    {
        uint8_t count = 0, pre = leftside[leftjumpingpoint[0]];
        for(i = leftjumpingpoint[0] + 1; i <= MT9V03X_H - 1; i++)
        {
            if(pre != 0)
            {
                for(j = pre; j >= 0; j--)
                {
                    if((mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255) || j == 0)
                    {
                        count++;
                        pre = j;
                        break;
                    }
                }
                if(j == 0 && i == leftjumpingpoint[0] + 1) break;
            }
            else
            {
                uint8_t check = 1;
                for(j = leftside[i]; j >= 0; j--)
                {
                    if(mt9v03x_image[i][j] == 255)
                    {
                        check = 0;
                        break;
                    }
                }
                if(check == 0) break;
                else count++;
            }
            if(count >= MT9V03X_H / 7) break;
        }
        if(count >= MT9V03X_H / 10 && leftside[leftjumpingpoint[0]-1] == 0)
        {
            isRoundabout = 1;
            pwm_duty(PWM4_CH4_B9,5000);
        }
    }

    if(leftlose <= 2 && rightlose >= MT9V03X_H / 6 && rightjumpingsum > 0
         && leftjumpingsum == 0 && rightjumpingpoint[0] >= MT9V03X_H / 2 && leftcontinue == 1) //右环
    {
        uint8_t count = 0, pre = rightside[rightjumpingpoint[0]];
        for(i = rightjumpingpoint[0] + 1; i <= MT9V03X_H - 1; i++)
        {
            if(pre != MT9V03X_W - 1)
            {
                for(j = pre; j <= MT9V03X_W - 1; j++)
                {
                    if((mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255) || j == MT9V03X_W - 1)
                    {
                        count++;
                        pre = j;
                        break;
                    }
                }
                if(count >= MT9V03X_H / 7) break;
                if(j == MT9V03X_W - 1 && i == rightjumpingpoint[0] + 1) break;
            }
            else
            {
                uint8_t check = 1;
                for(j = rightside[i]; j <= MT9V03X_W - 1; j++)
                {
                    if(mt9v03x_image[i][j] == 255)
                    {
                        check = 0;
                        break;
                    }
                }
                if(check == 0) break;
                else count++;
            }
        }
        if(count >= MT9V03X_H / 10 && rightside[rightjumpingpoint[0]-1] == MT9V03X_W - 1)
        {
            isRoundabout = 2;
            pwm_duty(PWM4_CH4_B9,5000);
        }
    }

    //偏入环
        /*if(rightlose < 2 && rightcontinue == 1 && leftextremesum >= 1 && leftextremepoint[0][0] <= MT9V03X_H / 3 && rightjumpingsum == 0)   //左环
        {
            uint8_t allwhite = 1;
            for(i = leftextremepoint[0][0] + 5; i >= leftextremepoint[0][0] - 5; i--)
            {
                for(j = leftside[i] + 2; i <= rightside[i] - 2; i++)
                {
                    if(mt9v03x_image[i][j] == 0)
                    {
                        allwhite = 0;
                        break;
                    }
                }
                if(allwhite == 0) break;
            }
            if(allwhite == 1) isRoundabout = 1;
        }
        else if(leftlose < 2 && leftcontinue == 1 && rightextremesum >= 1 && rightextremepoint[0][0] <= MT9V03X_H / 3 && leftjumpingsum == 0)   //右环
        {
            uint8_t allwhite = 1;
            for(i = rightextremepoint[0][0] + 5; i >= rightextremepoint[0][0] - 5; i--)
            {
                for(j = rightside[i] - 2; i >= leftside[i] + 2; i--)
                {
                    if(mt9v03x_image[i][j] == 0)
                    {
                        allwhite = 0;
                        break;
                    }
                }
                if(allwhite == 0) break;
            }
            if(allwhite == 1) isRoundabout = 2;
        }*/

}

void ImageAddingLine(uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY)
{
    int i = 0;
       float k = 0.0f, b = 0.0f;
       switch(status)
       {
         case 1:
           {
               k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
               b = (float)startX - (float)startY * k;

               for(i = startY; i < endY; i++)
               {
                   leftside[i] = (uint8_t)(k * i + b);
               }
               break;
           }
         case 2:
           {
               k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
               b = (float)startX - (float)startY * k;

               for(i = startY; i < endY; i++)
               {
                   rightside[i] = (uint8_t)(k * i + b);
               }
               break;
           }
       }
}

/*void RoundaboutProcess()
{
    int16_t i, j;
    switch(isRoundabout)
    {
        case 1:
        {
            uint8_t X1,X2,Y1,Y2;
            if(leftextremesum >= 1 && leftextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3)
            {
                Y1 = leftextremepoint[0][0];
                X1 = leftside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = rightside[Y1] - width[Y1];
            }

            if(leftjumpingsum >= 1 && leftjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 5)
            {
                Y2 = leftjumpingpoint[0];
                X2 = leftside[Y2];
            }
            else
            {
                Y2 = MT9V03X_H - 3;
                X2 = rightside[Y2] - width[Y2];
            }
            ImageAddingLine(1,X1,Y1,X2,Y2);
            if(leftextremepoint[0][0] >= MT9V03X_H * 2 / 5 && leftextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3
                    && leftextremesum > 0)
            {
                isRoundabout = 3;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 2:
        {
            //pwm_duty(PWM4_CH4_B9,1000);
            uint8_t X1,X2,Y1,Y2;
            if(rightextremesum >= 1 && rightextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3)
            {
                Y1 = rightextremepoint[0][0];
                X1 = rightside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = leftside[Y1] + width[Y1];
            }

            if(rightjumpingsum >= 1 && rightjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 5)
            {
                Y2 = rightjumpingpoint[0];
                X2 = rightside[Y2];
            }
            else
            {
                Y2 = MT9V03X_H - 3;
                X2 = leftside[Y2] + width[Y2];
            }
            ImageAddingLine(2,X1,Y1,X2,Y2);
            if(rightextremepoint[0][0] >= MT9V03X_H * 2 / 5 && rightextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3
                    && rightextremesum > 0)
            {
                isRoundabout = 4;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 3:
        {
            uint8_t rightcontinue = 1;
            for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
            {
                if(leftside[i] > leftside[i-1] || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]))
                    rightcontinue = i;
                if(rightcontinue != 1)
                    break;
            }
            if(leftjumpingsum >= 1 && rightjumpingsum == 0 && leftside[leftjumpingpoint[0]+1] == 0)
            {
                ImageAddingLine(2, leftside[leftjumpingpoint[0]], leftjumpingpoint[0], rightside[MT9V03X_H-3], MT9V03X_H-3);
                for(i = leftjumpingpoint[0] - 1; i >= 0; i--)
                {
                    rightside[i] = leftside[i] = 0;
                    for(j = rightside[i+1]; j >= 0; j--)
                    {
                        if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                        {
                            rightside[i] = j;
                            break;
                        }
                    }
                }
            }
            else if(rightjumpingsum >= 1)
            {
                i = MT9V03X_H - 3;
                while(leftside[i] > MT9V03X_W - 1 - width[i] && leftside[i] != 0)
                    i--;
                //uint8_t pointX;
                //if(leftside[i] <= MT9V03X_W - 1 - width[i]) pointX = leftside[i] + width[i];
                //else pointX = MT9V03X_W - 1;
                ImageAddingLine(2, rightside[rightjumpingpoint[0]] - 5, rightjumpingpoint[0], MT9V03X_W - 1, i);
                for(i = rightjumpingpoint[0] + 3; i >= 0; i--)
                {
                    leftside[i] = 0;
                    if(rightside[i] == 0) break;
                }
            }
            else if(rightcontinue == 1)
            {
                for(i = 5; i <= MAIN_ROAD; i++)
                {
                    if(leftside[i] < leftside[i+1] || (leftside[i] >= leftside[i+1] && leftside[i] - leftside[i+1] > 2))
                        break;
                }
                if(i <= MAIN_ROAD && i >= 7)
                    ImageAddingLine(2, leftside[i], i, rightside[MT9V03X_H-3], MT9V03X_H - 3);
            }
            if(rightjumpingpoint[0] >= MT9V03X_H * 3 / 5)
            {
                isRoundabout = 5;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 4:
        {
            pwm_duty(PWM4_CH4_B9,3000);
            uint8_t leftcontinue = 1;
            for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
            {
                if(leftside[i] > leftside[i-1] || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]))
                    leftcontinue = i;
                if(leftcontinue != 1)
                    break;
            }
            if(rightjumpingsum >= 1 && leftjumpingsum == 0 && rightside[rightjumpingpoint[0]+1] == MT9V03X_W - 1)
            {
                ImageAddingLine(1, rightside[rightjumpingpoint[0]], rightjumpingpoint[0], leftside[MT9V03X_H-3], MT9V03X_H-3);
                for(i = rightjumpingpoint[0] - 1; i >= 0; i--)
                {
                    leftside[i] = rightside[i] = MT9V03X_W - 1;
                    for(j = leftside[i+1]; j <= MT9V03X_W - 2; j++)
                    {
                        if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] ==255)
                        {
                            leftside[i] = j;
                            break;
                        }
                    }
                }
            }
            else if(leftjumpingsum >= 1)
            {
                i = MT9V03X_H - 3;
                while(rightside[i] < width[i] && rightside[i] != MT9V03X_W - 1)
                    i--;
                //uint8_t pointX;
                //if(rightside[i] >= width[i]) pointX = rightside[i] - width[i];
                //else pointX = 0;
                ImageAddingLine(1, leftside[leftjumpingpoint[0]] + 5, leftjumpingpoint[0], 0, i);
                for(i = leftjumpingpoint[0] + 2; i >= 0; i--)
                {
                    rightside[i] = MT9V03X_W - 1;
                    if(leftside[i] == 0) break;
                }
            }
            else if(leftcontinue == 1)
            {
                for(i = 5; i <= MAIN_ROAD; i++)
                {
                    if(rightside[i] > rightside[i+1] || (rightside[i] <= rightside[i+1] && rightside[i+1] - rightside[i] > 2))
                        break;
                }
                if(i <= MAIN_ROAD && i >= 7)
                    ImageAddingLine(1, rightside[i], i, leftside[MT9V03X_H-3], MT9V03X_H - 3);
            }
            if(leftjumpingpoint[0] >= MT9V03X_H * 3 / 5)
            {
                isRoundabout = 6;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 5:
        {
            uint8_t check = 1, checkpre = 0;
            for(i = 10; i >= 1; i--)
            {
                if(!check) break;
                if(mt9v03x_image[2][MT9V03X_W / 20 * i] == 255)
                {
                    check = 0;
                    break;
                }
                for(j = 3; j < MT9V03X_H; j++)
                {
                    if(j == MT9V03X_H - 1)
                    {
                        check = 0;
                        break;
                    }
                    else
                    {
                        if(mt9v03x_image[j][MT9V03X_W / 20 * i] == 255)
                        {
                            if(checkpre > j) check = 0;
                            checkpre = j;
                            break;
                        }
                    }
                }
            }
            if(leftjumpingsum >= 1)
            {
                for(i = leftjumpingpoint[0]; i >= 0; i--)
                    leftside[i] = 0;

            }
            if(check == 1)
            {
                isRoundabout = 7;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 6:
        {
            pwm_duty(PWM4_CH4_B9,5000);
            uint8_t check = 1, checkpre = 0;
            for(i = 10; i <= 19; i++)
            {
                if(!check) break;
                if(mt9v03x_image[2][MT9V03X_W / 20 * i] == 255)
                {
                    check = 0;
                    break;
                }
                for(j = 3; j < MT9V03X_H; j++)
                {
                    if(j == MT9V03X_H - 1)
                    {
                        check = 0;
                        break;
                    }
                    else
                    {
                        if(mt9v03x_image[j][MT9V03X_W / 20 * i] == 255)
                        {
                            if(checkpre > j) check = 0;
                            checkpre = j;
                            break;
                        }
                    }
                }
            }
            if(rightjumpingsum >= 1)
            {
                for(i = rightjumpingpoint[0]; i >= 0; i--)
                    rightside[i] = MT9V03X_W - 1;
            }
            if(check == 1)
            {
                isRoundabout = 8;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 7:
        {
            uint8_t X1,Y1,X2,Y2;
            for(i = 2; i <= MT9V03X_H - 1; i++)
            {
                if(mt9v03x_image[i][2] == 255) break;
            }
            X1 = width[i-1], Y1 = i - 1;
            if(leftextremesum >= 1)
            {
                X2 = rightside[rightextremepoint[0][0]], Y2 = rightextremepoint[0][0];
            }
            else
            {

                    X2 = MT9V03X_W - 1;

                Y2 = MT9V03X_H - 3;
            }
            ImageAddingLine(2,X1,Y1,X2,Y2);
            for(i = Y2; i >= 0; i--)
            {
                leftside[i] = 0;
            }
            if(mt9v03x_image[2][2] == 255 && rightextremesum == 0)
            {
                isRoundabout = 9;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 8:
        {
            //pwm_duty(PWM4_CH4_B9,7000);
            uint8_t X1,Y1,X2,Y2;
            for(i = 2; i <= MT9V03X_H - 1; i++)
            {
                if(mt9v03x_image[i][MT9V03X_W-3] == 255) break;
            }
            X1 = MT9V03X_W - 1 - width[i-1] / 2, Y1 = i - 1;
            if(leftextremesum >= 1)
            {
                X2 = leftside[leftextremepoint[0][0]], Y2 = leftextremepoint[0][0];
            }
            else
            {

                    X2 = 0;

                Y2 = MT9V03X_H - 3;
            }
            ImageAddingLine(1,X1,Y1,X2,Y2);
            for(i = Y2; i >= 0; i--)
            {
                rightside[i] = MT9V03X_W - 1;
            }
            if(mt9v03x_image[2][MT9V03X_W-3] == 255 && leftextremesum == 0)
            {
                isRoundabout = 10;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 9:
        {
            uint8_t leftcontinue = 0, pointX = 0;
            for(i = 1; i <= MT9V03X_H - 5; i++)
            {
                if((leftside[i] > leftside[i-1] && leftside[i] - leftside[i-1] > 1)
                        || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]) || leftside[i] == 0)
                {
                    pointX = i - 1;
                    if(i >= 5) break;
                }
                else leftcontinue++;
            }
            uint8_t rightcontinue = 1;
            for(j = 7; j <= MT9V03X_H * 1 / 2; j++)
            {
                if((rightside[j] < rightside[j-1] && rightside[j-1] - rightside[j] > 1)
                        || (rightside[j] - rightside[j-1] > 4 && rightside[j] >= rightside[j-1]) || rightside[j] == MT9V03X_W - 1)
                {
                    rightcontinue = j - 1;
                    break;
                }
            }
            if(leftjumpingsum >= 1 && rightjumpingsum == 0 && leftside[leftjumpingpoint[0]+1] == 0)
            {
                ImageAddingLine(1,leftside[leftjumpingpoint[0]],leftjumpingpoint[0],rightside[MT9V03X_H-3]-width[MT9V03X_H-3],MT9V03X_H-3);
            }
            else if(rightjumpingsum == 0 && leftcontinue >= MT9V03X_H / 7)
            {
                ImageAddingLine(1,leftside[i-1],i-1,rightside[MT9V03X_H-5]-width[MT9V03X_H-5],MT9V03X_H-5);
            }
            else if(rightcontinue != 1)
            {
                if(rightside[MT9V03X_H-3] == MT9V03X_W - 1)
                    ImageAddingLine(2, 2, 0, MT9V03X_W - 1, MT9V03X_H - 3);
                else
                {
                    for(i =MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                    {
                        if(rightside[i-1] == MT9V03X_W - 1)
                            break;
                    }
                    if(i >= MAIN_ROAD)
                    {
                        ImageAddingLine(2, 2, 0, rightside[i], i);
                    }
                }
            }

            if((leftjumpingsum >= 1 && rightcontinue == 1 && leftjumpingpoint[0] >= MT9V03X_H * 3 / 5 && leftside[leftjumpingpoint[0]+1] == 0)
                    || (rightcontinue == 1 && pointX >= MT9V03X_H * 3 / 5 + 2))
            {
                isRoundabout = 0;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 10:
        {
            //pwm_duty(PWM4_CH4_B9,9000);
            uint8_t rightcontinue = 0, pointX = 0;
            for(i = 1; i <= MT9V03X_H - 5; i++)
            {
                if((rightside[i] < rightside[i-1] && rightside[i-1] - rightside[i] > 1)
                        || (rightside[i] - rightside[i-1] > 2 && rightside[i] >= rightside[i-1]) || rightside[i] == MT9V03X_W - 1)
                {
                    pointX = i - 1;
                    if(i >= 5) break;
                }
                else rightcontinue++;
            }
            uint8_t leftcontinue = 1;
            for(j = 7; j <= MT9V03X_H * 1 / 2; j++)
            {
                if((leftside[j] > leftside[j-1] && leftside[j] - leftside[j-1] > 1)
                        || (leftside[j-1] - leftside[j] > 4 && leftside[j] <= leftside[j-1]) || leftside[j] == 0)
                {
                    leftcontinue = j - 1;
                    break;
                }
            }
            if(rightjumpingsum >= 1 && leftjumpingsum == 0 && rightside[rightjumpingpoint[0]+1] == MT9V03X_W - 1)
            {
                ImageAddingLine(2,rightside[rightjumpingpoint[0]],rightjumpingpoint[0],leftside[MT9V03X_H-3]+width[MT9V03X_H-3],MT9V03X_H-3);
            }
            else if(leftjumpingsum == 0 && rightcontinue >= MT9V03X_H / 10)
            {
                ImageAddingLine(2,rightside[i-1],i-1,leftside[MT9V03X_H-5]+width[MT9V03X_H-5],MT9V03X_H-5);
            }
            else if(leftcontinue != 1)
            {
                if(leftside[MT9V03X_H-3] == 0)
                    ImageAddingLine(1, MT9V03X_W - 3, 0, 0, MT9V03X_H - 3);
                else
                {
                    for(i = MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                    {
                        if(leftside[i-1] == 0)
                            break;
                    }
                    if(i >= MAIN_ROAD)
                    {
                        ImageAddingLine(1, MT9V03X_W - 3, 0, leftside[i], i);
                    }
                }
            }

            if((rightjumpingsum >= 1 && leftcontinue == 1 && rightjumpingpoint[0] >= MT9V03X_H * 3 / 5 && rightside[rightjumpingpoint[0]+1] == MT9V03X_W - 1)
                    || (leftcontinue == 1 && pointX >= MT9V03X_H * 3 / 5 + 2))
            {
                isRoundabout = 0;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
    }
}*/

void RoundaboutProcess()
{
    int16_t i, j;
    switch(isRoundabout)
    {
        case 1:
        {
            uint8_t X1,X2,Y1,Y2;
            if(leftextremesum >= 1 && leftextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3)
            {
                Y1 = leftextremepoint[0][0];
                X1 = leftside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = rightside[Y1] - width[Y1];
            }

            if(leftjumpingsum >= 1 && leftjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 5)
            {
                Y2 = leftjumpingpoint[0];
                X2 = leftside[Y2];
            }
            else
            {
                Y2 = MT9V03X_H - 3;
                X2 = rightside[Y2] - width[Y2];
            }
            ImageAddingLine(1,X1,Y1,X2,Y2);
            if(leftextremepoint[0][0] >= MT9V03X_H * 3 / 10
                    && leftextremesum > 0)
            {
                isRoundabout = 3;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 2:
        {
            //pwm_duty(PWM4_CH4_B9,1000);
            uint8_t X1,X2,Y1,Y2;
            if(rightextremesum >= 1 && rightextremepoint[0][0] <= MT9V03X_H - MT9V03X_H / 3)
            {
                Y1 = rightextremepoint[0][0];
                X1 = rightside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = leftside[Y1] + width[Y1];
            }

            if(rightjumpingsum >= 1 && rightjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 5)
            {
                Y2 = rightjumpingpoint[0];
                X2 = rightside[Y2];
            }
            else
            {
                Y2 = MT9V03X_H - 3;
                X2 = leftside[Y2] + width[Y2];
            }
            ImageAddingLine(2,X1,Y1,X2,Y2);
            if(rightextremepoint[0][0] >= MT9V03X_H * 3 / 10
                    && rightextremesum > 0)
            {
                isRoundabout = 4;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 3:
        {
            uint8_t rightcontinue = 1;
            for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
            {
                if(leftside[i] > leftside[i-1] || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]))
                    rightcontinue = i;
                if(rightcontinue != 1)
                    break;
            }
            if(leftjumpingsum >= 1 && rightjumpingsum == 0 && leftside[leftjumpingpoint[0]+1] == 0)
            {
                ImageAddingLine(2, leftside[leftjumpingpoint[0]], leftjumpingpoint[0], rightside[MT9V03X_H-3], MT9V03X_H-3);
                for(i = leftjumpingpoint[0] - 1; i >= 0; i--)
                {
                    rightside[i] = leftside[i] = 0;
                    for(j = rightside[i+1]; j >= 0; j--)
                    {
                        if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                        {
                            rightside[i] = j;
                            break;
                        }
                    }
                }
            }
            else if(rightjumpingsum >= 1)
            {
                i = MT9V03X_H - 3;
                while(leftside[i] > MT9V03X_W - 1 - width[i] && leftside[i] != 0)
                    i--;
                //uint8_t pointX;
                //if(leftside[i] <= MT9V03X_W - 1 - width[i]) pointX = leftside[i] + width[i];
                //else pointX = MT9V03X_W - 1;
                ImageAddingLine(2, rightside[rightjumpingpoint[0]] - 5, rightjumpingpoint[0], MT9V03X_W - 1, i);
                for(i = rightjumpingpoint[0] + 3; i >= 0; i--)
                {
                    leftside[i] = 0;
                    if(rightside[i] == 0) break;
                }
            }
            else if(rightcontinue == 1)
            {
                for(i = 5; i <= MAIN_ROAD; i++)
                {
                    if(leftside[i] < leftside[i+1] || (leftside[i] >= leftside[i+1] && leftside[i] - leftside[i+1] > 2))
                        break;
                }
                if(i <= MAIN_ROAD && i >= 7)
                    ImageAddingLine(2, leftside[i], i, rightside[MT9V03X_H-3], MT9V03X_H - 3);
            }
            if(rightjumpingpoint[0] >= MT9V03X_H * 3 / 5)
            {
                isRoundabout = 5;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 4:
        {
            pwm_duty(PWM4_CH4_B9,3000);
            uint8_t leftcontinue = 1;
            for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
            {
                if(leftside[i] > leftside[i-1] || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]))
                    leftcontinue = i;
                if(leftcontinue != 1)
                    break;
            }
            if(rightjumpingsum >= 1 && leftjumpingsum == 0 && rightside[rightjumpingpoint[0]+1] == MT9V03X_W - 1)
            {
                ImageAddingLine(1, rightside[rightjumpingpoint[0]], rightjumpingpoint[0], leftside[MT9V03X_H-3], MT9V03X_H-3);
                for(i = rightjumpingpoint[0] - 1; i >= 0; i--)
                {
                    leftside[i] = rightside[i] = MT9V03X_W - 1;
                    for(j = leftside[i+1]; j <= MT9V03X_W - 2; j++)
                    {
                        if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] ==255)
                        {
                            leftside[i] = j;
                            break;
                        }
                    }
                }
            }
            else if(leftjumpingsum >= 1)
            {
                i = MT9V03X_H - 3;
                while(rightside[i] < width[i] && rightside[i] != MT9V03X_W - 1)
                    i--;
                //uint8_t pointX;
                //if(rightside[i] >= width[i]) pointX = rightside[i] - width[i];
                //else pointX = 0;
                ImageAddingLine(1, leftside[leftjumpingpoint[0]] + 5, leftjumpingpoint[0], 0, i);
                for(i = leftjumpingpoint[0] + 2; i >= 0; i--)
                {
                    rightside[i] = MT9V03X_W - 1;
                    if(leftside[i] == 0) break;
                }
            }
            else if(leftcontinue == 1)
            {
                for(i = 5; i <= MAIN_ROAD; i++)
                {
                    if(rightside[i] > rightside[i+1] || (rightside[i] <= rightside[i+1] && rightside[i+1] - rightside[i] > 2))
                        break;
                }
                if(i <= MAIN_ROAD && i >= 7)
                    ImageAddingLine(1, rightside[i], i, leftside[MT9V03X_H-3], MT9V03X_H - 3);
            }
            if(leftjumpingpoint[0] >= MT9V03X_H * 3 / 5)
            {
                isRoundabout = 6;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 5:
        {
            uint8_t check = 1, checkpre = 0;
            for(i = 10; i >= 1; i--)
            {
                if(!check) break;
                if(mt9v03x_image[2][MT9V03X_W / 20 * i] == 255)
                {
                    check = 0;
                    break;
                }
                for(j = 3; j < MT9V03X_H; j++)
                {
                    if(j == MT9V03X_H - 1)
                    {
                        check = 0;
                        break;
                    }
                    else
                    {
                        if(mt9v03x_image[j][MT9V03X_W / 20 * i] == 255)
                        {
                            if(checkpre > j) check = 0;
                            checkpre = j;
                            break;
                        }
                    }
                }
            }
            if(leftjumpingsum >= 1)
            {
                for(i = leftjumpingpoint[0]; i >= 0; i--)
                    leftside[i] = 0;

            }
            if(check == 1)
            {
                isRoundabout = 7;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 6:
        {
            pwm_duty(PWM4_CH4_B9,5000);
            uint8_t check = 1, checkpre = 0;
            for(i = 10; i <= 19; i++)
            {
                if(!check) break;
                if(mt9v03x_image[2][MT9V03X_W / 20 * i] == 255)
                {
                    check = 0;
                    break;
                }
                for(j = 3; j < MT9V03X_H; j++)
                {
                    if(j == MT9V03X_H - 1)
                    {
                        check = 0;
                        break;
                    }
                    else
                    {
                        if(mt9v03x_image[j][MT9V03X_W / 20 * i] == 255)
                        {
                            if(checkpre > j) check = 0;
                            checkpre = j;
                            break;
                        }
                    }
                }
            }
            if(rightjumpingsum >= 1)
            {
                for(i = rightjumpingpoint[0]; i >= 0; i--)
                    rightside[i] = MT9V03X_W - 1;
            }
            if(check == 1)
            {
                isRoundabout = 8;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 7:
        {
            uint8_t X1,Y1,X2,Y2;
            for(i = 2; i <= MT9V03X_H - 1; i++)
            {
                if(mt9v03x_image[i][2] == 255) break;
            }
            X1 = width[i-1], Y1 = i - 1;
            if(leftextremesum >= 1)
            {
                X2 = rightside[rightextremepoint[0][0]], Y2 = rightextremepoint[0][0];
            }
            else
            {

                    X2 = MT9V03X_W - 1;

                Y2 = MT9V03X_H - 3;
            }
            ImageAddingLine(2,X1,Y1,X2,Y2);
            for(i = Y2; i >= 0; i--)
            {
                leftside[i] = 0;
            }
            if(mt9v03x_image[2][2] == 255 && rightextremesum == 0)
            {
                isRoundabout = 9;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 8:
        {
            //pwm_duty(PWM4_CH4_B9,7000);
            uint8_t X1,Y1,X2,Y2;
            for(i = 2; i <= MT9V03X_H - 1; i++)
            {
                if(mt9v03x_image[i][MT9V03X_W-3] == 255) break;
            }
            X1 = MT9V03X_W - 1 - width[i-1] / 2, Y1 = i - 1;
            if(leftextremesum >= 1)
            {
                X2 = leftside[leftextremepoint[0][0]], Y2 = leftextremepoint[0][0];
            }
            else
            {

                    X2 = 0;

                Y2 = MT9V03X_H - 3;
            }
            ImageAddingLine(1,X1,Y1,X2,Y2);
            for(i = Y2; i >= 0; i--)
            {
                rightside[i] = MT9V03X_W - 1;
            }
            if(mt9v03x_image[2][MT9V03X_W-3] == 255 && leftextremesum == 0)
            {
                isRoundabout = 10;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 9:
        {
            uint8_t leftcontinue = 0, pointX = 0, pointY = 0,start2 = 0;
            for(i = MT9V03X_H / 3; i <=MAIN_ROAD + 5; i++)
            {
                if((leftside[i] > leftside[i-1] && leftside[i] - leftside[i-1] > 1)
                        || (leftside[i-1] - leftside[i] > 4 && leftside[i] <= leftside[i-1]) || leftside[i] == 0)
                {
                    pointY = i;
                    break;
                }
            }

            for(i = 1; i <= MT9V03X_H - 5; i++)
            {
                if((leftside[i] > leftside[i-1] && leftside[i] - leftside[i-1] > 1)
                        || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]) || leftside[i] == 0)
                {
                    pointX = i - 1;
                    if(start2 == 1) break;
                    if(start2 == 0 && i >= MT9V03X_H / 2) break;
                }
                else
                {
                    leftcontinue++;
                    if(start2 == 0 && i >= MT9V03X_H / 7) start2 = 1;
                }
            }
            uint8_t rightcontinue = 1;
            for(j = MT9V03X_H / 3; j <= MT9V03X_H * 4 / 5; j++)
            {
                if((rightside[j] < rightside[j-1] && rightside[j-1] - rightside[j] > 1)
                        || (rightside[j] - rightside[j-1] > 4 && rightside[j] >= rightside[j-1]))
                {
                    rightcontinue = j - 1;
                    break;
                }
                if(rightside[j] == MT9V03X_W - 1 && rightside[j-1] == MT9V03X_W - 1 && rightside[j-2] == MT9V03X_W - 1) break;
            }
            if(leftjumpingsum >= 1 && rightjumpingsum == 0 && leftside[leftjumpingpoint[0]+1] == 0)
            {
                if(rightside[MT9V03X_H-3] >= width[MT9V03X_H-3])
                    ImageAddingLine(1,leftside[leftjumpingpoint[0]],leftjumpingpoint[0],rightside[MT9V03X_H-3]-width[MT9V03X_H-3],MT9V03X_H-3);
                else ImageAddingLine(1,leftside[leftjumpingpoint[0]],leftjumpingpoint[0],0,MT9V03X_H-3);
            }
            else if(rightjumpingsum == 0 && leftcontinue >= MT9V03X_H / 10)
            {
                if(rightside[MT9V03X_H-5] >= width[MT9V03X_H-5])
                    ImageAddingLine(1,leftside[i-1],i-1,rightside[MT9V03X_H-5]-width[MT9V03X_H-5],MT9V03X_H-5);
                else ImageAddingLine(1,leftside[i-1],i-1,0,MT9V03X_H-5);
            }
            else if(rightcontinue != 1)
            {
                if(rightside[MT9V03X_H-3] == MT9V03X_W - 1)
                    ImageAddingLine(2, 2, 0, MT9V03X_W - 1, MT9V03X_H - 3);
                else
                {
                    for(i =MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                    {
                        if(rightside[i-1] == MT9V03X_W - 1)
                            break;
                    }
                    if(i >= MAIN_ROAD)
                    {
                        ImageAddingLine(2, 2, 0, rightside[i], i);
                    }
                }
            }

            if((rightcontinue == 1 && pointX >= MT9V03X_H * 3 / 5 && leftcontinue >= MT9V03X_H / 10) || (rightcontinue == 1 && pointY > MAIN_ROAD)
                    )
            {
                isRoundabout = 0;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
        case 10:
        {
            //pwm_duty(PWM4_CH4_B9,9000);
            uint8_t rightcontinue = 0, pointX = 0,pointY = 0, start2 = 0;



            for(i = MT9V03X_H / 3; i <= MAIN_ROAD + 5; i++)
            {
                if((rightside[i] < rightside[i-1] && rightside[i-1] - rightside[i] > 1)
                        || (rightside[i] - rightside[i-1] > 4 && rightside[i] >= rightside[i-1]) || rightside[i] == MT9V03X_W - 1)
                {
                    pointY = i;
                    break;
                }
            }

            for(i = 1; i <= MT9V03X_H - 5; i++)
            {
                if((rightside[i] < rightside[i-1] && rightside[i-1] - rightside[i] > 1)
                        || (rightside[i] - rightside[i-1] > 2 && rightside[i] >= rightside[i-1]) || rightside[i] == MT9V03X_W - 1)
                {
                    pointX = i - 1;
                    if(start2 == 1) break;
                    if(start2 == 0 && i >= MT9V03X_H / 2) break;
                }
                else
                {
                    rightcontinue++;
                    if(start2 == 0 && i >= MT9V03X_H / 7) start2 = 1;
                }
            }
            uint8_t leftcontinue = 1;
            for(j = MT9V03X_H / 3; j <= MT9V03X_H * 4 / 5; j++)
            {
                if((leftside[j] > leftside[j-1] && leftside[j] - leftside[j-1] > 1)
                        || (leftside[j-1] - leftside[j] > 4 && leftside[j] <= leftside[j-1]))
                {
                    leftcontinue = j - 1;
                    break;
                }
                if(leftside[j] == 0 && leftside[j-1] == 0 && leftside[j-2] == 0) break;
            }
            if(rightjumpingsum >= 1 && leftjumpingsum == 0 && rightside[rightjumpingpoint[0]+1] == MT9V03X_W - 1)
            {
                if(leftside[MT9V03X_H-3]+width[MT9V03X_H-3] <= MT9V03X_W - 1)
                    ImageAddingLine(2,rightside[rightjumpingpoint[0]],rightjumpingpoint[0],leftside[MT9V03X_H-3]+width[MT9V03X_H-3],MT9V03X_H-3);
                else ImageAddingLine(2,rightside[rightjumpingpoint[0]],rightjumpingpoint[0],MT9V03X_W-1,MT9V03X_H-3);
            }
            else if(leftjumpingsum == 0 && rightcontinue >= MT9V03X_H / 10)
            {
                if(leftside[MT9V03X_H-5]+width[MT9V03X_H-5] <= MT9V03X_W - 1)
                    ImageAddingLine(2,rightside[i-1],i-1,leftside[MT9V03X_H-5]+width[MT9V03X_H-5],MT9V03X_H-5);
                else ImageAddingLine(2,rightside[i-1],i-1,MT9V03X_W-1,MT9V03X_H-5);
            }
            else if(leftcontinue != 1)
            {
                if(leftside[MT9V03X_H-3] == 0)
                    ImageAddingLine(1, MT9V03X_W - 3, 0, 0, MT9V03X_H - 3);
                else
                {
                    for(i = MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                    {
                        if(leftside[i-1] == 0)
                            break;
                    }
                    if(i >= MAIN_ROAD)
                    {
                        ImageAddingLine(1, MT9V03X_W - 3, 0, leftside[i], i);
                    }
                }
            }

            if((leftcontinue == 1 && pointX > MT9V03X_H * 3 / 5 && rightcontinue >= MT9V03X_H / 10) || (leftcontinue == 1 && pointY > MAIN_ROAD)
                    )
            {
                isRoundabout = 0;
                pwm_duty(PWM4_CH4_B9,5000);
            }
            break;
        }
    }
}

void checkBranch()
{
    int16_t i, j, k;
    if(leftextremesum >= 1 && rightextremesum >= 1 && leftextremepoint[0][1] == 2 && rightextremepoint[0][1] == 1 &&
       ((leftextremepoint[0][0] >= rightextremepoint[0][0] && leftextremepoint[0][0] - rightextremepoint[0][0] <= 15) ||
         (rightextremepoint[0][0] >= leftextremepoint[0][0] && rightextremepoint[0][0] - leftextremepoint[0][0] <= 15)))       //正入
    {
        if(isBranch == 0 || isBranch == 6)
        {
            if(isBranch == 0) i = leftextremepoint[0][0];
            else i = rightextremepoint[0][0];
            for(; i >= 3; i--)
            {
                if(mt9v03x_image[i][MT9V03X_W / 2] == 0)
                    break;
            }
            if(i >= 3)
            {
                uint8_t lefttmp = 0, righttmp = 0, leftpre = MT9V03X_W / 2, rightpre = MT9V03X_W / 2;
                for(k = i; k > i - 3; k--)
                {
                    for(j = MT9V03X_W / 2; j >= 1; j--)
                    {
                        if(mt9v03x_image[k][j] == 255 && mt9v03x_image[k][j-1] == 255 && mt9v03x_image[k][j+1] == 0)
                        {
                            if(leftpre > j)
                            {
                                lefttmp++;
                                leftpre = j;
                            }
                            break;
                        }
                    }
                    for(j = MT9V03X_W / 2; j <= MT9V03X_W - 2; j++)
                    {
                        if(mt9v03x_image[k][j] == 255 && mt9v03x_image[k][j+1] == 255 && mt9v03x_image[k][j-1] == 0)
                        {
                            if(rightpre < j)
                            {
                                righttmp++;
                                rightpre = j;
                            }
                            break;
                        }
                    }
                }
                if(lefttmp >= 3 && righttmp >= 3)
                {
                    isBranch ++;
                    if(isBranch >= 12) isBranch -= 12;
                    pwm_duty(PWM4_CH4_B9,2500);
                }
            }
        }
    }
}

void BranchProcess()
{
    switch(isBranch)
    {
        case 1:
        {
            if(mt9v03x_image[MT9V03X_H/6][MT9V03X_W/2] == 0)
            {
                isBranch = 2;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }
        case 3:
        {
            if(leftextremesum >= 1 && rightextremesum >= 1 && leftextremepoint[0][1] == 2 && rightextremepoint[0][1] == 1 &&
              ((leftextremepoint[0][0] >= rightextremepoint[0][0] && leftextremepoint[0][0] - rightextremepoint[0][0] <= 10) ||
              (rightextremepoint[0][0] >= leftextremepoint[0][0] && rightextremepoint[0][0] - leftextremepoint[0][0] <= 10)))
            {
                if(Branchprotection <= 50) break;
                isBranch = 4;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }
        case 4:
        {
            if(mt9v03x_image[MT9V03X_H*2/5][MT9V03X_W/2] == 0)
            {
                isBranch = 5;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }

        case 7:
        {
            if(mt9v03x_image[MT9V03X_H/6][MT9V03X_W/2] == 0)
            {
                isBranch = 8;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }
        case 9:
        {
            if(leftextremesum >= 1 && rightextremesum >= 1 && leftextremepoint[0][1] == 2 && rightextremepoint[0][1] == 1 &&
              ((leftextremepoint[0][0] >= rightextremepoint[0][0] && leftextremepoint[0][0] - rightextremepoint[0][0] <= 10) ||
              (rightextremepoint[0][0] >= leftextremepoint[0][0] && rightextremepoint[0][0] - leftextremepoint[0][0] <= 10)))
            {
                if(Branchprotection <= 50) break;
                isBranch = 10;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }
        case 10:
        {
            if(mt9v03x_image[MT9V03X_H*2/5][MT9V03X_W/2] == 0)
            {
                isBranch = 11;
                pwm_duty(PWM4_CH4_B9,2500);
            }
            break;
        }
    }
}

void checkCross()
{
    int16_t i, j;
    uint8_t leftpartside[10],rightpartside[10];
    for(i = 5; i <= MT9V03X_H - 1; i += 5)
    {
        for(j = MT9V03X_W * 3 / 4; j >= 1; j--)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
            {
                leftpartside[i/5] = j;
                break;
            }
        }
        if(j == 0) leftpartside[i/5] = 0;
        for(j = MT9V03X_W * 1 / 4; j <= MT9V03X_W - 2; j++)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
            {
                rightpartside[i/5] = j;
                break;
            }
        }
        if(j == MT9V03X_W - 1) rightpartside[i/5] = MT9V03X_W - 1;
    }
    if(leftside[MT9V03X_H/2-5] == 0 && rightside[MT9V03X_H/2-5] == MT9V03X_W - 1)
    {
        int8_t bothlose = 0, onelose = 0, properwidth = 1;
        for(i = 1; i <= 9; i++)
        {
            if(leftpartside[i] == 0 && rightpartside[i] == MT9V03X_W - 1) bothlose++;
            else if(leftpartside[i] == 0 && rightpartside[i] != MT9V03X_W - 1) onelose--;
            else if(rightpartside[i] == MT9V03X_W - 1 && leftpartside[i] != 0) onelose++;
            else
            {
                if(i >= 6) continue;
                if(leftpartside[i] >= rightpartside[i]) properwidth = 0;
                else
                {
                    if((width[i*5] >= rightpartside[i] - leftpartside[i] && width[i*5] <= rightpartside[i] - leftpartside[i] + 5)
                            || (width[i*5] <= rightpartside[i] - leftpartside[i] && width[i*5] + 5 >= rightpartside[i] - leftpartside[i]))
                    {

                    }
                    else properwidth = 0;
                }
            }
        }
        if(bothlose >= 3 && onelose <= 2 && onelose >= -2 && properwidth == 1)
        {
            for(i = 9; i > 0; i--)
            {
                if(leftpartside[i] == 0 || rightpartside[i] == MT9V03X_W - 1)
                    break;
            }
            if(i != 9 && i != 0 && leftpartside[i+1] > 5 && rightpartside[i+1] < MT9V03X_W - 6)
            {
                uint8_t leftlineA = 0, leftlineB = 0, rightlineA = 0, rightlineB = 0;
                for(j = (i + 1) * 5; j >= 0; j--)
                {
                    if(mt9v03x_image[j][leftpartside[i+1]-5] == 0 && mt9v03x_image[j-1][leftpartside[i+1]-5] == 0 && mt9v03x_image[j+1][leftpartside[i+1]-5] == 255)
                        leftlineA = j;
                    if(mt9v03x_image[j][leftpartside[i+1]-5] == 0 && mt9v03x_image[j+1][leftpartside[i+1]-5] == 0 && mt9v03x_image[j-1][leftpartside[i+1]-5] == 255)
                        leftlineB = j;
                    if(leftlineA != 0 && leftlineB != 0) break;
                }
                for(j = (i + 1) * 5; j >= 0; j--)
                {
                    if(mt9v03x_image[j][rightpartside[i+1]+5] == 0 && mt9v03x_image[j-1][rightpartside[i+1]+5] == 0 && mt9v03x_image[j+1][rightpartside[i+1]+5] == 255)
                        rightlineA = j;
                    if(mt9v03x_image[j][rightpartside[i+1]+5] == 0 && mt9v03x_image[j+1][rightpartside[i+1]+5] == 0 && mt9v03x_image[j-1][rightpartside[i+1]+5] == 255)
                        rightlineB = j;
                    if(rightlineA != 0 && rightlineB != 0) break;
                }
                //ips114_showuint8(50,50,leftlineA);
                //ips114_showuint8(50,100,leftlineB);
                //ips114_showuint8(150,50,rightlineA);
                //ips114_showuint8(150,100,rightlineB);
                if(leftlineA != 0 && leftlineB != 0 && rightlineA != 0 && rightlineB != 0 && leftlineA < leftlineB &&
                        rightlineA < rightlineB && leftlineB - leftlineA + 5>= rightlineB - rightlineA && leftlineB - leftlineA <= rightlineB - rightlineA + 5)
                {
                    isCross = 1;
                    pwm_duty(PWM4_CH4_B9,7500);
                }
            }
            else if(i == 9)
            {
                uint8_t tmp = 0;
                for(i = 9; i > 0; i--)
                {
                    if((leftpartside[i] != 0 && rightpartside[i] != MT9V03X_W - 1) && tmp == 0)
                        tmp = 1;
                    if(tmp == 1 && (leftpartside[i] == 0 || rightpartside[i] == MT9V03X_W - 1))
                        tmp = 2;
                }
                if(tmp == 1)
                {
                    isCross = 1;
                    pwm_duty(PWM4_CH4_B9,7500);
                }
            }
        }
    }
}

void CrossProcess()
{
    pwm_duty(PWM4_CH4_B9,7500);
    int16_t i, j;
    uint8_t lineA = 0, lineB = 0;
    float kA, kB, leftbottom, rightbottom;
    for(i = MT9V03X_H / 2; i >= MT9V03X_H / 10 - 1; i--)
    {
        for(j = MT9V03X_W * 7 / 10; j >= 0; j--)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
            {
                leftside[i] = j;
                break;
            }
        }
        for(j = MT9V03X_W * 3 / 10; j <= MT9V03X_W - 1; j++)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j] == 255)
            {
                rightside[i] = j;
                break;
            }
        }
    }
    for(i = MT9V03X_H / 2; i >= MT9V03X_H / 10; i--)
    {
        if(leftside[i] != 0 && rightside[i] != MT9V03X_W - 1 && leftside[i+1] != 0 && rightside[i+1] != MT9V03X_W - 1 &&
                (leftside[i] >= leftside[i+1] && leftside[i] - leftside[i+1] <= 2) && (rightside[i+1] >= rightside[i] && rightside[i+1] - rightside[i] <= 2))
            break;
    }
    lineB = i;
    for(i = lineB; i >= MT9V03X_H / 10; i--)
    {
        if((leftside[i-1] >= leftside[i] && leftside[i-1] - leftside[i] <= 2) && (rightside[i] >= rightside[i-1] && rightside[i] - rightside[i-1] <= 2))
            continue;
        else
        {
            break;
        }
    }
    lineA = i;
    if(leftside[MT9V03X_H/2] != 0 && rightside[MT9V03X_H/2] != MT9V03X_W - 1 && leftside[MAIN_ROAD] != 0 && rightside[MAIN_ROAD] != MT9V03X_W - 1)
    {
        isCross = 0;
    }
    if(lineA == lineB)
    {
        ImageAddingLine(1,leftside[lineA],lineA,0,MT9V03X_H - 1);
        ImageAddingLine(2,rightside[lineA],lineA,MT9V03X_W - 1, MT9V03X_H - 1);
    }
    else
    {
        kA = ((float)lineA - (float)lineB) / ((float)leftside[lineA] - (float)leftside[lineB]);
        kB = ((float)lineA - (float)lineB) / ((float)rightside[lineA] - (float)rightside[lineB]);
        leftbottom = (float)leftside[lineB] + (MT9V03X_H - 1 - (float)lineB) / kA;
        rightbottom = (float)rightside[lineB] + (MT9V03X_H - 1 - (float)lineB) / kB;
        if(leftbottom < 0) leftbottom = 0;
        if(rightbottom > MT9V03X_W - 1) rightbottom = MT9V03X_W - 1;
        ImageAddingLine(1, leftside[lineA], lineA, (uint8_t)leftbottom, MT9V03X_H - 1);
        ImageAddingLine(2, rightside[lineA], lineA, (uint8_t)rightbottom, MT9V03X_H - 1);
    }
}

void checkZebra(uint8_t side)
{
    int16_t i, j;
    uint8_t leftcontinue = 1, rightcontinue = 1;
    uint8_t count2 = 0;
    for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
    {
        if(leftside[i] > leftside[i-1] || (leftside[i-1] - leftside[i] > 2 && leftside[i] <= leftside[i-1]))
            leftcontinue = i;
        if(rightside[i] < rightside[i-1] || (rightside[i] - rightside[i-1] > 2 && rightside[i] >= rightside[i-1]))
            rightcontinue = i;
        if(leftcontinue != 1 && rightcontinue != 1)
            break;
    }
    for(i = MT9V03X_H / 2; i >= MT9V03X_H / 5; i--)
    {
        uint8_t count = 0;
        if(side == 1)
        {
            for(j = 2; j < rightside[i]; j++)
            {
                if(mt9v03x_image[i][j-2] == 255 && mt9v03x_image[i][j-1] == 255 && mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0)
                {
                    count++;
                }
            }
        }
        else
        {
            for(j = MT9V03X_W - 3; j > leftside[i]; j--)
            {
                if(mt9v03x_image[i][j+2] == 255 && mt9v03x_image[i][j+2] == 255 && mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0)
                {
                    count++;
                }
            }
        }
        if(count >= 6) count2++;
    }
    if(side == 1 && rightcontinue == 1 && count2 >= 2 && leftcontinue != 1)
    {
        isZebra++;
        //pwm_duty(PWM4_CH4_B9,2500);
    }
    if(side == 2 && leftcontinue == 1 && count2 >= 2 && rightcontinue != 1)
    {
        isZebra++;
        //pwm_duty(PWM4_CH4_B9,2500);
    }
}

void ZebraProcess(uint8_t side)
{
    int16_t i;
    pwm_duty(PWM4_CH4_B9,2500);
    if(isZebra <= 100)
    {
        uint8_t Y1 = MT9V03X_W / 10, Y2, tmp = 1, X1, X2;
        if(side == 1)
        {
            for(i = MT9V03X_H - 5; i > MAIN_ROAD; i--)
            {
                if(leftside[i-1] < leftside[i] || (leftside[i-1] >= leftside[i] && leftside[i-1] - leftside[i] > 2))
                    break;
            }
            Y2 = i;
            for(i = MT9V03X_H / 10; i <= MT9V03X_H / 5; i++)
            {
                if(leftside[i+1] > leftside[i] || (leftside[i+1] <= leftside[i] && leftside[i] - leftside[i+1] > 2))
                    break;
            }
            Y1 = i;

            for(i = MAIN_ROAD + 10; i >= MT9V03X_H / 5; i--)
            {
                if(leftside[i-1] < leftside[i] || (leftside[i-1] >= leftside[i] && leftside[i-1] - leftside[i] > 2))
                    tmp = 0;
            }
            if(tmp == 1)
            {
                isZebra++;
                pwm_duty(PWM4_CH4_B9,2500);
            }

            if(rightside[Y1] >= leftside[Y1] && rightside[Y1] - leftside[Y1] >= width[Y1] - 10 &&
                    rightside[Y1] - leftside[Y1] <= width[Y1] + 10)
                X1 = leftside[Y1];
            else X1 = rightside[Y1] - width[Y1];
            if(rightside[Y2] >= leftside[Y2] && rightside[Y2] - leftside[Y2] >= width[Y2] - 10 &&
                    rightside[Y2] - leftside[Y2] <= width[Y2] + 10)
                X2 = leftside[Y2];
            else X2 = rightside[Y2] - width[Y2];
            ImageAddingLine(1, X1, Y1, X2, Y2);
        }
        else
        {
            for(i = MT9V03X_H - 5; i > MAIN_ROAD; i--)
            {
                if(rightside[i-1] > rightside[i] || (rightside[i-1] <= rightside[i] && rightside[i] - rightside[i-1] > 2))
                    break;
            }
            Y2 = i;
            for(i = MT9V03X_H / 10; i <= MT9V03X_H / 5; i++)
            {
                if(rightside[i+1] < rightside[i] || (rightside[i+1] >= rightside[i] && rightside[i+1] - rightside[i] > 2))
                    break;
            }
            Y1 = i;

            for(i = MAIN_ROAD; i >= MT9V03X_H / 5; i--)
            {
                if(rightside[i-1] > rightside[i] || (rightside[i-1] <= rightside[i] && rightside[i] - rightside[i-1] > 2))
                    tmp = 0;
            }
            if(tmp == 1)
            {
                isZebra++;
                pwm_duty(PWM4_CH4_B9,2500);
            }

            if(rightside[Y1] >= leftside[Y1] && rightside[Y1] - leftside[Y1] >= width[Y1] - 10 &&
                    rightside[Y1] - leftside[Y1] <= width[Y1] + 10)
                X1 = rightside[Y1];
            else X1 = leftside[Y1] + width[Y1];
            if(rightside[Y2] >= leftside[Y2] && rightside[Y2] - leftside[Y2] >= width[Y2] - 10 &&
                    rightside[Y2] - leftside[Y2] <= width[Y2] + 10)
                X2 = rightside[Y2];
            else X2 = leftside[Y2] + width[Y2];
            ImageAddingLine(2, X1, Y1, X2, Y2);
        }
    }
    else
    {
        movemode = 14;
        if(side == 1)
        {
            for(i = MT9V03X_H / 10; i <= MT9V03X_H / 3; i++)
            {
                if(leftside[i+1] > leftside[i] || (leftside[i+1] <= leftside[i] && leftside[i] - leftside[i+1] >2))
                    break;
            }
            if(leftside[i] < rightside[i] && rightside[i] - leftside[i] >= width[i] - 10 && rightside[i] - leftside[i] <= width[i] + 10)
                ImageAddingLine(2, leftside[i], i, rightside[MT9V03X_H-5], MT9V03X_H - 5);
            for(i = MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                if(leftside[i] > leftside[i-1] || (leftside[i] < rightside[i] &&
                        (rightside[i] - leftside[i] < width[i] - 10 || rightside[i] - leftside[i] > width[i] + 10)) || leftside[i] >= rightside[i])
                    break;
            for(; i >= MAIN_ROAD; i--)
                leftside[i] = 0;
        }
        else
        {
            for(i = MT9V03X_H / 10; i <= MT9V03X_H / 3; i++)
            {
                if(rightside[i+1] < rightside[i] || (rightside[i+1] >= rightside[i] && rightside[i+1] - rightside[i] > 2))
                    break;
            }
            if(leftside[i] < rightside[i] && rightside[i] - leftside[i] >= width[i] - 10 && rightside[i] - leftside[i] <= width[i] + 10)
                ImageAddingLine(1, rightside[i], i, leftside[MT9V03X_H-5], MT9V03X_H - 5);
            for(i = MT9V03X_H - 3; i >= MAIN_ROAD; i--)
                if(rightside[i] < rightside[i-1] || (leftside[i] < rightside[i] &&
                        (rightside[i] - leftside[i] < width[i] - 10 || rightside[i] - leftside[i] > width[i] + 10)) || leftside[i] >= rightside[i])
                    break;
            for(; i >= MAIN_ROAD; i--)
                rightside[i] = MT9V03X_W - 1;
        }
    }
}

void checkRamp()
{
    int16_t i;
    if(leftlose >= 5 || rightlose >= 5) return;
    for(i = MT9V03X_H - 5; i >= MT9V03X_H / 5; i--)
    {
        if(!((leftside[i] >= leftside[i+1] && leftside[i] - leftside[i+1] <= 3)
                && (rightside[i] <= rightside[i+1] && rightside[i+1] - rightside[i] <= 3)
                && leftside[i] <= MT9V03X_W / 2 && rightside[i] >= MT9V03X_W / 2))
        {
            return;
        }
    }
    for(i = MT9V03X_H * 7 / 10; i >= MT9V03X_H * 2 / 5; i -= MT9V03X_H / 10)
    {
        if(rightside[i] - leftside[i] + i / 2 < width[i] + 20)
            return;
    }
    isRamp++;
    //if(isRamp >= 2) isRamp -= 2;
    Rampcount = 0;
    pwm_duty(PWM4_CH4_B9,5000);
}
