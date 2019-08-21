#ifndef _RB_QMIND_PLUS_H
#define _RB_QMIND_PLUS_H



#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "avr/interrupt.h"
#include "avr/io.h"
#include "RB_ADC.h"
#include "RB_BUZZER.h"
#include "RB_DCMOTOR.h"

#include "RB_LEDMATRIX.h"
#include "RB_LIGHTSENSOR.h"
#include "RB_LINEFOLLOWER.h"
#include "RB_PIRSENSOR.h"
#include "RB_PORT.h"
#include "RB_RGBLED.h"
#include "RB_SERIAL_TASK.h"
#include "RB_SERVO.h"
#include "RB_SOFTI2CMASTER.h"
#include "RB_SOUNDSENSOR.h"
#include "RB_TEMPANDHUMI.h"
#include "RB_ULTRASONIC.h"
#include "avr/wdt.h"
#include "RB_MP3.h"
#include "RB_RGBLEDMATRIX.h"
#include "RB_COLORSENSOR.h"
#include "RB_GYRO.h"
#include "RB_TOUCHSENSOR.h"

#include "RB_DIGITALDISPLAY.h"
#include "RB_TEMPERATURE.h"

#include "RB_JOYSTICK.h"
#include "RB_FLAMESENSOR.h"
#include "RB_GASSENSOR.h"
#include "RB_POTENTIOMETER.h"

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)    //Qoooers
#include "RB_ENCONDERMOTOR.h" 


 
#define DeviceType                            1          //主控类型          
#define HardwareVersion                       2          //硬件版本
#define SoftwareVersion                       11        //软件版本     


/*
 *    WORK MODE
 */

 #define Remote_Control_Mode               0X00      //遥控模式
 #define Ultrasonic_Mode                   0X01      //超声波蔽障模式
 #define Line_Follower_Mode                0x02      //巡线模式



#else                                            //Qscout

#include "RB_DCMOTORONBOARD.h"	
/*
 *  Hardware Information
 */

#define DeviceType                            0x02          //设备类型   K2
#define HardwareVersion                       0x01          //硬件版本
#define SoftwareVersion                       0x07          //软件版本


/*
 *    WORK MODE
 */

 #define Remote_Control_Mode               0X00           //遥控模式
 #define Ultrasonic_Mode                   0X01           //超声波蔽障模式
 #define Line_Follower_Mode                0x02          //巡线模式
 #define Dinosaur_Ultrasonic_Mode         0x03          //恐龙模式蔽障模式
 #define Alligator_Ultrasonic_Mode         0x04          //鳄鱼传统蔽障模式



#endif
/*
 *   Action
 */

#define   Hardware                            0x01          //查询版本信息
#define   Device_Type                         0x02          //查询设备信息
#define   Device_All                          0x03          //查询所有设备信息
#define   Device_Motor                        0x04          //查询电机接口信息
#define   Device_Port                         0x05          //查询端口信息
#define   Led_SET                             0X10          //设置LED颜色
#define   MoterSpeed_SET                      0x11          //设置电机速度
#define   UlSensorLed_Set                     0x12          //超声波灯光设置
#define   Buzzer_Set                          0x13          //蜂鸣器设置
#define   ExPanel_Set                         0x14          //表情面板设置
#define   LowPorw_Set                         0x15          //低电压主动上报
#define   Button_Set                          0x16          //按键设置
#define   MoterTurs_Set                       0x17          //设置电机圈数
#define   Mode_Change                         0x18          //模式切换
#define   Servo_Set                           0x19          //舵机控制
#define   ExDCMotor_Set                       0x1A          //外置电机控制
#define   RGBLEDMatrix_Set                    0x1B          //RGB矩阵屏设置
#define   MP3_Set                             0x1C          //MP3设置
#define   TouchSensor_Set                     0x1D          //触摸传感器主动上报设置
#define   DigitalDisplay_Set                  0x1E          //4段数码管显示设置
#define   RGBLEDArray_Set                     0x1F          //4个RGBLED显示设置



#define   UlSensorDistance_Read               0xA1          //超声波距离读取
#define   Button_Read                         0xA2          //按键读取
#define   Power_Read                          0xA3          //读取电压
#define   Line_Follower_Read                  0xA4          //巡线传感器数据读取
#define   Humi_And_Temp_Read                  0xA5          //温湿度传感器数据读取
#define   Light_Read                          0xA6          //光线传感器数据读取
#define   Sound_Read                          0xA7          //声音传感器数据读取
#define   PIRSensor_Read                      0xA8          //人体红外传感器读取
#define   GyroSensor_Read                     0xA9          //陀螺仪传感器数据读取
#define   ColorSensor_Read                    0XAA          //颜色传感器读取
#define   TouchSensor_Read                    0XAB          //触摸传感器读取
#define   TemptureSensors_Read                0XAC          //2路温度传感读取
#define   Line_Follower_Array_Read            0XAD          //6路巡线传感器读取



#define   JoyStickSensor_Read                 0XAE          //遥感读取
#define   FlameSensor_Read                    0XAF          //火焰读取
#define   GasSensor_Read                      0XB0          //气体读取      
#define   Potentimeter_Read                   0XB1          //电位器读取




 
 




#endif
