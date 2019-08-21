#include "RB_TEMPERATURE.h"

RB_TEMPERATURE temp1(2);


float  value = 0;


  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

   
   value = temp1.GET_TTEMPERATURE(1);
   Serial.print("Wendu1:");  
   Serial.println(value);  
   value = temp1.GET_TTEMPERATURE(2);
   Serial.print("Wendu2:");  
   Serial.println(value);    
   delay(500);

}
