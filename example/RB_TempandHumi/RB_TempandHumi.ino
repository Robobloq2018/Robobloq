#include "RB_TEMPANDHUMI.h" 

RB_TempAndHumi TempAndHumi(2);

uint8_t temp =0 ;
uint8_t humi = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
    TempAndHumi.Update() ;
    temp = TempAndHumi.GetTemperature();
    humi = TempAndHumi.GetHumidity();
     Serial.print("temp:");
     Serial.println(temp);
    Serial.print("humi:");
     Serial.println(humi);
     delay(1500);
    
}
