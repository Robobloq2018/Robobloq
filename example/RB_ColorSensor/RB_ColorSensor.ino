#include "RB_COLORSENSOR.h"



RB_COLORSENSOR   colorsensor(2,TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_4X);

uint16_t  red,green,blue,white;

unsigned char color_value ;

void setup()
{
       
      Serial.begin(115200);
     
}

void loop()
{
     colorsensor.getRawData(&red,&green,&blue,&white); ////采集颜色传感器
     Serial.print("red:");
     Serial.println(red);
     Serial.print("green:");
     Serial.println(green);
     Serial.print("blue:");
     Serial.println(blue);
     Serial.print("white:");
     Serial.println(white);

     color_value = colorsensor.GetColor();
     Serial.println(color_value);


     
}
