
#include <RB_RGBLEDMATRIX.h>

RB_RGBLEDMATRIX  Ledmatrix(2);

unsigned short RGB_LED_test[72] = 
{
    0,0,0,0,0,0,
    1.1,1,1,1,1,
    0,0,0,0,0,0,
    1.1,1,1,1,1,
    0,0,0,0,0,0,
    1.1,1,1,1,1,
    0,0,0,0,0,0,
    1.1,1,1,1,1,
    0,0,0,0,0,0,
    1.1,1,1,1,1,
    0,0,0,0,0,0,
    1.1,1,1,1,1
  
};


void setup() {
   Serial.begin(115200);
   Ledmatrix.RB_RGBLEDMATRIX_Init(0X77);
   Ledmatrix.RB_RGBLEDMATRIX_Init(0X74);
   Ledmatrix.RGBLEDMATRIX_DISPALY1(RGB_LED_test,RGB_LED_test);
   
}

void loop() {
  // put your main code here, to run repeatedly:

  
}
