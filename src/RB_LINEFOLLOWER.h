#ifndef RB_LineFollower_H
#define RB_LineFollower_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 
#define  S1_IN_S2_IN         (0X00)
#define  S1_IN_S2_OUT        (0X01)
#define  S1_OUT_S2_IN        (0X02)
#define  S1_OUT_S2_OUT       (0X03)



class RB_LineFollower:public RB_Port
{
 public :
  RB_LineFollower(void);
  RB_LineFollower(uint8_t port);
  void SetPin(uint8_t Sensor1,uint8_t Sensor2);
  uint8_t ReadSensors(void);
  bool ReadSensor1(void); 
  bool ReadSensor2(void);
 private :
  volatile uint8_t _Sensor1;
  volatile uint8_t _Sensor2;
  
  
};



#endif 
