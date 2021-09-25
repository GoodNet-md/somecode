#ifndef __IMAGE__
#define __IMAGE__

#include "headfile.h"
void ImageProcess(void);
void Get_01_Value();
void GetSide();
void clearside();
short GetOSTU (void);
void findjumpingpoint(uint8_t now);
void findextremepoint();
void checkRoundabout();
void RoundaboutProcess();
void checkBranch();
void BranchProcess();
void checkCross();
void checkZebra(uint8_t side);
void ZebraProcess(uint8_t side);
void printside();
void checkRamp();
void CrossProcess();

void ImageAddingLine(uint8_t status, uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY);

#endif
