#include "RB_DCMOTOR.h"

RB_DCMotor  RB_DCMotor(1);

void setup()
{
    Serial.begin(9600); 
   
    RB_DCMotor.RB_DCMotor_SetSpeed(0x50,0xA3,0X01,0X01,80);
    delay(500);
    RB_DCMotor.RB_DCMotor_SetSpeed(0x50,0xA3,0X02,0X00,80);
    delay(500);
    RB_DCMotor.RB_DCMotor_SetSpeed(0x50,0xA3,0X01,0X01,80);
    delay(500);
    RB_DCMotor.RB_DCMotor_SetSpeed(0x50,0xA3,0X02,0X00,80);
    delay(500);
}

void loop()
{


}
