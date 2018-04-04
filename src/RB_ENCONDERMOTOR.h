#ifndef  RB_EncoderMotor_H
#define  RB_EncoderMotor_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 


#define  BIT10     10
#define  BIT9      9
#define  BIT8      8 


#define PWM_MAX     99

#define DIRECT_MODE       0x00
#define PID_MODE          0x01
#define PWM_MODE          0x02
#define PWM_PID_MODE      0X03

#define MOTION_WITH_POS        0x00
#define MOTION_WITHOUT_POS     0x01

#define PWM_MIN_OFFSET                 10
#define ENCODER_POS_DEADBAND           10
#define DECELERATION_DISTANCE_PITCH    6 

typedef   struct 
{
     float  P,I,D;
     float  Setpoint,Output,Integral,differential,last_error;  
} PID_internal;

typedef struct
{
     float limit;
     float target;
     float now;
     float kp;
     float ki;
     float kd;
     float e0;
     float e1;
     float e2;
     float output;
     long  current_pos;
     long  previout_pos;
} PID_PWM_internal;

typedef struct
{
     float      mode;
     uint8_t    motionState;

     int32_t   pulseEncoder;
     int32_t   currentPwm;
     int32_t   targetPwm;
     int32_t   previousPwm;

     double    currentSpeed;
     double    targetSpeed;
     double    previousSpeed;
     double    ratio;

     long     currentPos;
     long     targetPos;
     long     previousPos;
     long     pulsePos;

     PID_internal   PID_speed;
     PID_internal   PID_pos;
     PID_PWM_internal PID_pwm;
}RB_Encoder_type;

typedef void (*cb)(int16_t,int16_t); 


class RB_EncoderMotor
{
  public :
     /*
     *   1 
     */
RB_EncoderMotor:: RB_EncoderMotor(uint8_t port);
/*
 * 2
 */
uint8_t RB_EncoderMotor::GetIntterrruptNum(void);
/*
 *  3
 */
uint8_t RB_EncoderMotor::GetPortA(void);
/*
 *  4
 */
uint8_t RB_EncoderMotor::GetPortB(void);
/*
 * 5
 */
long RB_EncoderMotor::GetPulsePos(void);
/*
 *  6
 */
void RB_EncoderMotor::SetPulsePos(long pulsePos);
/*
 *  7 
 */
void RB_EncoderMotor::PulsePosPlus(void);
/*
 *  8 
 */
void RB_EncoderMotor::PulsePosMinus(void);
/*
 *  9 
 */
void RB_EncoderMotor::SetCurrentSpeed(double speed);
/*
 *  10
 */
double  RB_EncoderMotor::GetCurrentSpeed(void);
/*
 *  11
 */
int32_t RB_EncoderMotor::GetCurrentPwm(void);
/*
 * 
 */
void RB_EncoderMotor::SetTarPWM(int16_t pwm_value);
/*
 *  12
 */
void RB_EncoderMotor::SetMotorPwm(int32_t pwm);
/*
 * 13
 */
void RB_EncoderMotor::UpdateSpeed(void);
/*
 *  14
 */
void RB_EncoderMotor::UpdateCurPos(void);
/*
 *  15
 */
void RB_EncoderMotor::GetCurPos(void);
 /*
  *  16
  */
void RB_EncoderMotor::RunSpeed(float speed);
/*
 *  17
 */
void RB_EncoderMotor::SetSpeed(float speed) ;
 /*
  *  18
  */
void RB_EncoderMotor::Move(long position,float speed,int16_t extId,cb callback) ;
/*
 *  19
 */
void RB_EncoderMotor::MoveTo(long position,float speed,int16_t extId,cb callback) ;
/*
 *  20
 */
long RB_EncoderMotor::DistanceToGo(void) ;
/*
 *  21
 */
void RB_EncoderMotor::SetSpeedPid(float p,float i,float d) ;
/*
 * 22
 */
void RB_EncoderMotor::SetPosPid(float p,float i,float d) ;
/*
 *  23
 */
void RB_EncoderMotor::SetRatio(int16_t RatioValue);
 /*
  * 24
  */
void RB_EncoderMotor::SetMotionMode(int16_t motionMode) ;
/*
 *  25
 */
void RB_EncoderMotor::PwmMove(void) ;
/*
 *  26
 */
boolean RB_EncoderMotor::IsTarPosReached(void) ;
/*
 *  27
 */
void RB_EncoderMotor::Loop(void);
 
protected:
      volatile RB_Encoder_type encoder_structure;
       long _Start_speed_time;
       long _Encoder_move_time;
       long _pid_time;
       uint8_t _Slot;
       uint8_t _Port_A;
       uint8_t _Port_B;
       uint8_t _Port_PWM_A;
       uint8_t _Port_PWM_B;
       uint8_t _IntterrruptNum;
       boolean _Lock_flag;
       boolean _Dir_lock_flag;
       cb _callback;
       uint8_t _extId;
       int16_t _Encoder_output; 
};



#endif 

