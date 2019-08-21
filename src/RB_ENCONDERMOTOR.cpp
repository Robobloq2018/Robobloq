#include "RB_ENCONDERMOTOR.h"
#include <util/delay.h>
 
/*
 *   1 
 */
RB_EncoderMotor:: RB_EncoderMotor(uint8_t port)
{  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    _Port_A = encoder_Port[port].port_A;
    _Port_B = encoder_Port[port].port_B;
    _Port_PWM_A = encoder_Port[port].pwm_A;
    _Port_PWM_B = encoder_Port[port].pwm_B;

    pinMode(_Port_A,INPUT_PULLUP);
    pinMode(_Port_B,INPUT_PULLUP);
    pinMode(_Port_PWM_A,OUTPUT);
    pinMode(_Port_PWM_B,OUTPUT); 
  


    if(_Port_A == 18) 
    {
      _IntterrruptNum = 5;
       TCCR1A=0xAB;   TCCR1B=0x01;
       OCR1B=0;       OCR1A=0; 
    }
    else if(_Port_A==19){
      _IntterrruptNum = 4; 
      TCCR4A=0xAB;TCCR4B=0x01;
      OCR4B=0; OCR4A=0;
    }
    else if(_Port_A == 20){
        _IntterrruptNum = 3;
    }
    else if(_Port_A == 21) {
        _IntterrruptNum = 2;
    }
    else if(_Port_A == 3){
        _IntterrruptNum = 1;
    }
    else if(_Port_A == 2){
       _IntterrruptNum = 0;
     }
    SetMotorPwm(0);
    SetPulsePos(0);
    _Start_speed_time = millis();  
   #endif 
}
/*
 * 2
 */
uint8_t RB_EncoderMotor::GetIntterrruptNum(void){
    return _IntterrruptNum;
}
/*
 *  3
 */
uint8_t RB_EncoderMotor::GetPortA(void){
     return _Port_A;
}
/*
 *  4
 */
uint8_t RB_EncoderMotor::GetPortB(void){
     return _Port_B;
}
/*
 * 5
 */
long RB_EncoderMotor::GetPulsePos(void){
      return encoder_structure.pulsePos;  
}
/*
 *  6
 */
void RB_EncoderMotor::SetPulsePos(long pulsePos){
     encoder_structure.pulsePos  =  pulsePos;
}
/*
 *  7 
 */
void RB_EncoderMotor::PulsePosPlus(void){
     encoder_structure.pulsePos++;
}
/*
 *  8 
 */
void RB_EncoderMotor::PulsePosMinus(void){
     encoder_structure.pulsePos --;
}
/*
 *  9 
 */
void RB_EncoderMotor::SetCurrentSpeed(double speed){
     encoder_structure.currentSpeed = speed;
}
/*
 *  10
 */
double  RB_EncoderMotor::GetCurrentSpeed(void){
     return encoder_structure.currentSpeed;
}
/*
 *  11
 */
int32_t RB_EncoderMotor::GetCurrentPwm(void){
     return encoder_structure.currentPwm;
}
/*
 *   11_2
 */
void RB_EncoderMotor::SetTarPWM(int16_t pwm_value) 
 { 
   encoder_structure.mode = PWM_MODE; 
   encoder_structure.targetPwm = constrain(pwm_value,-99,99);
 } 

/*
 *  12
 */
void RB_EncoderMotor::SetMotorPwm(int32_t pwm)
{ 
    pwm = constrain(pwm,-100,100);
    if(encoder_structure.previousPwm != pwm){
         encoder_structure.previousPwm = pwm;
   }
   else{
        return;
    }
   encoder_structure.currentPwm = pwm;
   #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   if(RB_EncoderMotor::GetIntterrruptNum()==4){
       if(pwm>0){
            pwm = pwm*1023/100;
            OCR4A = 0 ; OCR4B = pwm;
       }
       else {
            pwm = abs(pwm)*1023/100;
            OCR4A = pwm ; OCR4B = 0;
       }
   }
   else if(RB_EncoderMotor::GetIntterrruptNum()==5){
       if(pwm>0){
            pwm = pwm*1023/100;
            OCR1A = pwm; OCR1B = 0;
       }
       else {
            pwm = abs(pwm)*1023/100;
            OCR1A = 0 ; OCR1B = pwm;
       } 
   }
   #endif
}
/*
 * 13
 */
void RB_EncoderMotor::UpdateSpeed(void)
{
   if((millis()-_Start_speed_time)>20) 
   {
      uint16_t  dt = millis -  _Start_speed_time;
      SetCurrentSpeed(((GetPulsePos() - encoder_structure.previousPos) *(1000.0/dt)*60.0) /(encoder_structure.pulseEncoder * encoder_structure.ratio));
      encoder_structure.previousPos = GetPulsePos();
      _Start_speed_time = millis();
   } 
}
/*
 *  14
 */
void RB_EncoderMotor::UpdateCurPos(void)
{
  encoder_structure.currentPos = (long)((encoder_structure.pulsePos/(encoder_structure.pulseEncoder * encoder_structure.ratio)) * 360);
}
/*
 *  15
 */
void RB_EncoderMotor::GetCurPos(void)
{
    return encoder_structure.currentPos;
 }
 /*
  *  16
  */
void RB_EncoderMotor::RunSpeed(float speed)
{
      encoder_structure.mode = PID_MODE; 
      encoder_structure.motionState = MOTION_WITHOUT_POS; 
      encoder_structure.targetSpeed = speed; 
      _Lock_flag = false; 
}
/*
 *  17
 */
void RB_EncoderMotor::SetSpeed(float speed) 
 { 
     encoder_structure.motionState = MOTION_WITHOUT_POS; 
     encoder_structure.targetSpeed = speed; 
      _Lock_flag = false; 
 } 
 /*
  *  18
  */
void RB_EncoderMotor::Move(long position,float speed,int16_t extId,cb callback) 
{ 
    encoder_structure.targetPos = encoder_structure.currentPos + position; 
    MoveTo(encoder_structure.targetPos,speed,extId,callback);
} 
/*
 *  19
 */
void RB_EncoderMotor::MoveTo(long position,float speed,int16_t extId,cb callback) 
{ 
         encoder_structure.targetSpeed = speed; 
         _extId = extId; 
         _Lock_flag = false; 
        encoder_structure.mode = PID_MODE; 
        encoder_structure.motionState = MOTION_WITH_POS; 
        encoder_structure.targetPos = position; 
        _callback = callback; 
        if(DistanceToGo() > 0)  { 
               _Dir_lock_flag = true; 
        } 
        else { 
             _Dir_lock_flag = false; 
        } 
} 
/*
 *  20
 */
long RB_EncoderMotor::DistanceToGo(void) { 
      return encoder_structure.targetPos - encoder_structure.currentPos; 
} 
/*
 *  21
 */
void RB_EncoderMotor::SetSpeedPid(float p,float i,float d) 
{ 
    encoder_structure.PID_speed.P = p; 
    encoder_structure.PID_speed.I = i; 
    encoder_structure.PID_speed.D = d; 
} 
/*
 * 22
 */
void RB_EncoderMotor::SetPosPid(float p,float i,float d) 
 { 
   encoder_structure.PID_pos.P = p; 
   encoder_structure.PID_pos.I = i; 
   encoder_structure.PID_pos.D = d; 
} 
/*
 *  23
 */
void RB_EncoderMotor::SetRatio(int16_t RatioValue)
{
    encoder_structure.ratio = RatioValue;
 }
 /*
  * 24
  */
void RB_EncoderMotor::SetMotionMode(int16_t motionMode) 
 { 
   encoder_structure.mode = motionMode; 
 } 
/*
 *  25
 */
void RB_EncoderMotor::PwmMove(void) 
{ 
   if(millis() - _Encoder_move_time > 5) 
  { 
    _Encoder_move_time = millis(); 
    encoder_structure.currentPwm = round(0.9 * encoder_structure.currentPwm + 0.1 * encoder_structure.targetPwm); 
    if((abs(encoder_structure.currentPwm) <= 10) && (encoder_structure.targetPwm == 0)) 
     { 
       encoder_structure.currentPwm = 0; 
     } 
   } 
} 
/*
 *  26
 */
boolean RB_EncoderMotor::IsTarPosReached(void) 
 { 
   return _Lock_flag; 
 } 

/*
 * 27
 */
void RB_EncoderMotor::Loop(void) 
 { 
   UpdateCurPos(); 
   UpdateSpeed(); 
   if(encoder_structure.mode == PWM_MODE) 
   { 
     PwmMove(); 
     SetMotorPwm(encoder_structure.currentPwm); 
   } 
 } 










