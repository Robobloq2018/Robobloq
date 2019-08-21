#ifndef  _RB_Port_H
#define  _RB_Port_H


#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>


#define RB_PORT_USER_DEFINED 


#define   Other_Device                   0xff
#define   None_Device                    0x00
#define   Encode_Motor_Divice            0x01
#define   Ultrasonic_Distance_Sensor     0x02
#define   LED_Matraix_Blue               0x03
#define   Line_Follower_Sensor           0x04
#define   Servo_Device                   0x05
#define   Temp_And_Humi_Sensor           0x06
#define   Light_Sensor                   0x07
#define   Sound_Sensor                   0x08
#define   DC_Motor_Device                0x09
#define   PIR_Sensor                     0x0A
#define   MP3_Sensor                     0X0B
#define   RGBLED_Matraix                 0X0C
#define   Color_Sensor                   0X0D
#define   Gyro_Sensor                    0X0E
#define   Touch_Sensor                   0X0F  

#define   Tempture_Sensors               0X10
#define   Line_Follower_Array_Sensors    0X11
#define   DigitalDisplay_Device          0X12 
#define   RGBLED_Array_Device            0X13


#define   JoyStick_Sensor                0X14  
#define   Flame_Sensor                   0X15
#define   Gas_Sensor                     0X16
#define   Potentimeter_Sensor                   0X17 


        


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)


#define NC    (0)
#define A0    54
#define A1    55
#define A2    56
#define A3    57
#define A4    58
#define A5    59
#define A6    60
#define A7    61
#define A8    62
#define A9    63
#define A10   64
#define A11   65

#define PWM0   5
#define PWM1   2
#define PWM2  46 
#define PWM3  45


 
#define LowPowr_Pin    66
#define MOTOR_PWM0     11
#define MOTOR_PWM1     12
#define MOTOR_PWM2     6
#define MOTOR_PWM3     7
#define MOTOR_Charge_Pin    20
#define MOTOR_Sleep_Pin     17
#define MOTOR_Fault_Pin     16 

#define ADC_A          22
#define ADC_B          23
#define ADC_C          24
#define ADC_OUT        67
#define ENA_A          19
#define ENA_B           9
#define ENB_A          18 
#define ENB_B           8


#define RGB_LED_Pin    37
#define Buzzer_Pin     36
#define USER_KEY_Pin   35

#define  EEPROM_ADDRESS    1 






typedef struct 
{
   uint8_t clk;
   uint8_t dat;
} RB_Port_Sig;
typedef struct 
{
   uint8_t  port_A;
   uint8_t  port_B;
   uint8_t  pwm_A;
   uint8_t  pwm_B; 
} RB_Port_Encoder;


extern RB_Port_Sig  RBPort[9];
extern RB_Port_Encoder encoder_Port[3];

#else  

#define NC       0
#define A0      14
#define A1      15
#define A2      16
#define A3      17
#define A4      18
#define A5      19
#define PWM0    3
#define PWM1    11
#define ADC_OUT  A6
#define ADC_A    12 
#define ADC_B    13
#define ADC_C    7 





#define RGB_LED_Pin    2
#define Buzzer_Pin     4
#define USER_KEY_Pin   8

#define MOTOR_PWM0     5 
#define MOTOR_PWM1     6
#define MOTOR_PWM2     9 
#define MOTOR_PWM3     10


typedef struct 
{
   uint8_t clk;
   uint8_t dat;
} RB_Port_Sig;

extern RB_Port_Sig  RBPort[5];

#endif

class RB_Port 
{
   public :
          /*  1
           * 
           */
         RB_Port(void);
         /*  2
          * 
          */
         RB_Port(uint8_t port);
         /* 3
          * 
          */
         uint8_t  GetPort(void);    
         /*
          * 
          */
         uint8_t  GetClk(void);
         /*   
          *    
          */
         uint8_t GetDat(void);
         /*
          * 
          */
          bool DReadClk(void);
          /*
           * 
           */
          bool DReadDat(void);
          /*
           * 
           */
          void DWriteClk(bool value);
          /*
           * 
           */
          void DWriteDat(bool value );
          /*
           * 
           */
          int16_t AReadClk(void);
          /*
           * 
           */
          int16_t AReadDat(void);
          /*
           * 
           */
          void ResetPort(uint8_t port);
 protected:
         uint8_t  _clk;
         uint8_t  _dat;
         uint8_t  _port;
            
} ; 

#endif 
