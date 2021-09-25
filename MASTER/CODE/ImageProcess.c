#include "ImageProcess.h"

short camera_threshold=0;

int8_t dx[8] = {-1,-1,0,1,1,1,0,-1}, dy[8] = {0,1,1,1,0,-1,-1,-1};

uint8_t leftside[MT9V03X_H],rightside[MT9V03X_H],midline[MT9V03X_H],width[MT9V03X_H];
uint8_t ZebraFlag = 0, BranchFlag = 0;
uint8_t leftjumpingpoint[5],rightjumpingpoint[5],leftjumpingsum,rightjumpingsum;
uint8_t leftextremepoint[5][2],rightextremepoint[5][2],leftextremesum,rightextremesum;        //1极左2极右
uint8_t leftlose, rightlose,alllose;
int8 cameraerr=0;
uint8_t isRoundabout = 0;

uint8_t threshold;

void ImageProcess(void)
{
    uint8_t i, j;
    threshold = GetOSTU ();
    Get_01_Value();

    GetSide();
    findextremepoint();

    if(isRoundabout == 0)
    {
        checkRoundabout();
    }

    if(isRoundabout != 0)
    {
        RoundaboutProcess();
    }

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


   // ips114_displayimage032_zoom(mt9v03x_image[0], MT9V03X_W, MT9V03X_H, IPS114_X_MAX, IPS114_Y_MAX);
   // ips114_showuint8(100,100,isRoundabout);
   // ips114_showuint8(150,50,rightjumpingsum);
   // ips114_showuint8(150,100,rightextremesum);
   // ips114_showuint8(150,150,rightextremepoint[0][0]);
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
        midline[i] = MT9V03X_W / 2;
        width[i] = 36 + i * 6 / 5;
    }
    leftlose = rightlose = alllose = 0;
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
        if(j == MT9V03X_H - 1) j = MT9V03X_W / 2;
        else
            j = (midline[i+1] <= MT9V03X_W - 10)?midline[i+1] : MT9V03X_W - 10;
        while(j >= 1)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
            {
                leftside[i] = j;
                break;
            }
            j--;
        }
        if(j == MT9V03X_H - 1) j = MT9V03X_W / 2;
        else
            j = (midline[i+1] >= 9)?midline[i+1] : 9;
        while(j <= MT9V03X_W - 2)
        {
            if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
            {
                rightside[i] = j;
                break;
            }
            j++;
        }
        midline[i] = (leftside[i] + rightside[i]) / 2;
        if(i <= MT9V03X_H - 5)
        {
            findjumpingpoint(i);
        }
    }
   for(i = MT9V03X_H * 5 / 6 - 1; i >= 0; i--)
   {
        if(leftside[i+1] != 0)
        {
            j = (leftside[i+1] + 6 <= MT9V03X_W - 2)?(leftside[i+1] + 6) : MT9V03X_W - 2;
            while(j >= 1 && j >= leftside[i+1] - 10)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
                {
                    leftside[i] = j;
                    break;
                }
                j--;
            }
        }
        else
        {
            j = midline[i+1];
            while(j >= 1)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
                {
                    leftside[i] = j;
                    break;
                }
                j--;
            }
        }

        if(rightside[i+1] != MT9V03X_W - 1)
        {
            j = (rightside[i+1] - 6 >= 1)?(rightside[i+1] - 6) : 1;
            while(j <= MT9V03X_W - 2 && j <= rightside[i+1] + 10)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                {
                    rightside[i] = j;
                    break;
                }
                j++;
            }
        }
        else
        {
            j = midline[i+1];
            while(j <= MT9V03X_W - 2)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                {
                     rightside[i] = j;
                     break;
                }
                j++;
            }
        }

        midline[i] = (leftside[i] + rightside[i]) / 2;

        if(leftside[i] == 0) leftlose++;
        if(rightside[i] == MT9V03X_W - 1) rightlose++;
        if(leftside[i] == 0 && rightside[i] == MT9V03X_W - 1) alllose++;

        //找跳变点
        if(i >= MT9V03X_H / 10) findjumpingpoint(i);
   }
}

void findjumpingpoint(uint8_t now)
{
    if(leftside[now+1] >= MT9V03X_W / 6 && ((leftside[now] == 0 && leftside[now+2] != 0)
            || (leftside[now+2] == 0 && leftside[now] != 0)) && leftjumpingsum < 5)
    {
        leftjumpingpoint[leftjumpingsum] = now+1;
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
    uint8_t leftpre = MT9V03X_H - 4, rightpre = MT9V03X_H - 4, leftpre2 = MT9V03X_H - 3, rightpre2 = MT9V03X_H - 3;
    for(i = MT9V03X_H - 4; i >= MT9V03X_H / 15; i--)
    {
        if(leftside[leftpre] < leftside[i] && leftside[leftpre] < leftside[leftpre2] && leftextremesum <= 5
                && leftside[leftpre] != 0 && leftside[leftpre2] != 0 && leftside[i] != 0)
        {
            leftextremepoint[leftextremesum][0] = leftpre;
            leftextremepoint[leftextremesum][1] = 1;
            leftextremesum++;
        }
        if(leftside[leftpre] > leftside[i] && leftside[leftpre] > leftside[leftpre2] && leftextremesum <= 5
                && leftside[leftpre] != 0 && leftside[leftpre2] != 0 && leftside[i] != 0)
        {
            leftextremepoint[leftextremesum][0] = leftpre;
            leftextremepoint[leftextremesum][1] = 2;
            leftextremesum++;
        }

        if(rightside[rightpre] < rightside[i] && rightside[rightpre] < rightside[rightpre2] && rightextremesum <= 5
           && rightside[rightpre] != MT9V03X_W-1 && rightside[rightpre2] != MT9V03X_W-1 && rightside[i] != MT9V03X_W-1)
        {
            rightextremepoint[rightextremesum][0] = rightpre;
            rightextremepoint[rightextremesum][1] = 1;
            rightextremesum++;
        }
        if(rightside[rightpre] > rightside[i] && rightside[rightpre] > rightside[rightpre2] && rightextremesum <= 5
           && rightside[rightpre] != MT9V03X_W-1 && rightside[rightpre2] != MT9V03X_W-1 && rightside[i] != MT9V03X_W-1)
        {
            rightextremepoint[rightextremesum][0] = rightpre;
            rightextremepoint[rightextremesum][1] = 2;
            rightextremesum++;
        }

        if(leftside[i] != leftside[leftpre])
        {
            leftpre2 = leftpre;
            leftpre = i;
        }
        if(rightside[i] != rightside[rightpre])
        {
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
    if(leftlose >= MT9V03X_H / 6 && rightlose <= 2 && leftjumpingsum > 0
         && rightjumpingsum == 0 && rightextremesum == 0 && leftjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 3 - 1)//左环
    {
        uint8_t count = 0, pre = leftside[leftjumpingpoint[0]];
        for(i = leftjumpingpoint[0] + 1; i <= MT9V03X_H - 1 || count < MT9V03X_H / 7; i++)
        {
            for(j = pre - 1; j >= 0; j--)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j+1] == 0 && mt9v03x_image[i][j-1] == 255)
                {
                    count++;
                    pre = j;
                    break;
                }
            }
        }
        if(count >= MT9V03X_H / 7) isRoundabout = 1;
    }

    if(leftlose <= 2 && rightlose >= MT9V03X_H / 6 && rightjumpingsum > 0
         && leftjumpingsum == 0 && leftextremesum == 0 && rightjumpingpoint[0] >= MT9V03X_H - MT9V03X_H / 3 - 1) //右环
    {
        uint8_t count = 0, pre = rightside[rightjumpingpoint[0]];
        for(i = rightjumpingpoint[0] + 1; i <= MT9V03X_H - 1 || count < MT9V03X_H / 7; i++)
        {
            for(j = pre + 1; j <= MT9V03X_W - 1; j++)
            {
                if(mt9v03x_image[i][j] == 0 && mt9v03x_image[i][j-1] == 0 && mt9v03x_image[i][j+1] == 255)
                {
                    count++;
                    pre = j;
                    break;
                }
            }
        }
        if(count >= MT9V03X_H / 7) isRoundabout = 2;
        ips114_showuint8(50,100,count);
    }
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

void RoundaboutProcess()
{
   // int16_t i;
    switch(isRoundabout)
    {
        case 1:
        {
            uint8_t X1,X2,Y1,Y2;
            if(leftextremesum >= 1 && leftextremepoint[0][0] <= MT9V03X_H - MT9V03X_H/3)
            {
                Y1 = leftextremepoint[0][0];
                X1 = leftside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = rightside[Y1] - width[Y1];
            }

            if(leftjumpingsum >= 1 && leftjumpingpoint[0] >= MT9V03X_H - MT9V03X_H/5)
            {
                Y2 = leftjumpingpoint[0];
                X2 = leftside[Y2];
            }
            else
            {
                Y2 = MT9V03X_H - 3;
                X2 = rightside[Y2]-width[Y2];
            }
            ImageAddingLine(1,X1,Y1,X2,Y2);
            break;
        }
        case 2:
        {
            uint8_t X1,X2,Y1,Y2;
            if(rightextremesum >= 1 && rightextremepoint[0][0] <= MT9V03X_H - MT9V03X_H/3)
            {
                Y1 = rightextremepoint[0][0];
                X1 = rightside[Y1];
            }
            else
            {
                Y1 = MT9V03X_H / 15;
                X1 = leftside[Y1] + width[Y1];
            }

            if(rightjumpingsum >= 1 && rightjumpingpoint[0] >= MT9V03X_H - MT9V03X_H/5)
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
            break;
        }
    }
}
