#include "RB_GASSENSOR.h"


RB_GASSENSOR   gas(2);


uint16_t gas_avalue =  0;


void setup() {
  // put your setup code here, to run once:
     Serial.begin(115200);
     
         
}

void loop() {
  // put your main code here, to run repeatedly:
    gas_avalue = gas.GetGasValue();
    Serial.print("Robobloq GasSensor  analog value is ");
    Serial.println(gas_avalue);
    delay(500);
    
}
