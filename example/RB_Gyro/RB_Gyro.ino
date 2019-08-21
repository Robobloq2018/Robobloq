#include "RB_GYRO.h"


RB_GYRO   gyro(2);

uint8_t Device_ID;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gyro.begin();
  delay(200);
  gyro.ReadData(0x75,&Device_ID,1);
  Serial.print("Who am I:");
  Serial.println(Device_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro.Update();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ());  
  
}