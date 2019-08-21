#include "RB_ADC.h"
#include "RB_MP3.h"

//RB_MP3   MP3(2);
RB_ADC   RB_ADC;

uint16_t value = 0;

void setup() {
     Serial.begin(115200);
     delay(2000);
     
  //    MP3.RB_MP3_Star(0x03,0x0f);
  //    MP3.RB_MP3_Star(0X04,0X02);
  //    MP3.RB_MP3_END();
      

 //     pinMode(RBPort[1].clk,OUTPUT);
 //     pinMode(RBPort[1].dat,OUTPUT);

 //      pinMode(RBPort[2].clk,INPUT);
 //     pinMode(RBPort[2].dat,INPUT);

 //      pinMode(RBPort[3].clk,OUTPUT);
 //     pinMode(RBPort[3].dat,OUTPUT);

 //          pinMode(RBPort[4].clk,OUTPUT);
 //      pinMode(RBPort[4].dat,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
     Serial.print("1:");
     value = RB_ADC.RB_ADC_Read(1);
     Serial.print(value);
     Serial.print(":");
     value =analogRead(ADC_OUT);
     Serial.print(value);
     Serial.print("  ");
     
      Serial.print("2:");
     value = RB_ADC.RB_ADC_Read(2);
     Serial.print(value);
     Serial.print(":");
     value =analogRead(ADC_OUT);
      Serial.print(value);
     Serial.print("  ");
     
      Serial.print("3:");
     value = RB_ADC.RB_ADC_Read(3);
     Serial.print(value);
     Serial.print(":");
     value =analogRead(ADC_OUT);
      Serial.print(value);
     Serial.print("  ");
     
      Serial.print("4:");
     value = RB_ADC.RB_ADC_Read(4);
     Serial.print(value);
     Serial.print(":");
     value =analogRead(ADC_OUT);
      Serial.print(value);
     Serial.println("  ");
}
