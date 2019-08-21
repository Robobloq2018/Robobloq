#include "RB_LEDMATRIX.h"

RB_LEDMatrix  LEDMatrix(2);



unsigned char picturDataA[20]={
0x00,0x00,0x20,0x10,0x50,0x28,0x88,0x44,0x00,0x00,
0x10,0x20,0x08,0x40,0x07,0x80,0x03,0x00,0x00,0x00};



void setup() {
  // put your setup code here, to run once:
    LEDMatrix.RB_LEDMatrix_PageAllWrite(0x00,0xFF,20);
    delay(1000);
    LEDMatrix.RB_LEDMatrix_PageAllWrite(0x00,0x00,20);
    delay(1000);
    LEDMatrix.RB_LEDMatrix_PageWrite(0x00,picturDataA);
    delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
