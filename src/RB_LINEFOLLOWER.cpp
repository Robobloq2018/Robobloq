#include "RB_LINEFOLLOWER.h"




RB_LineFollower::RB_LineFollower(void):RB_Port(0)
{
  
}

RB_LineFollower::RB_LineFollower(uint8_t port):RB_Port(port)
{
   _Sensor1 = RBPort[port].clk;
   _Sensor2 = RBPort[port].dat;
   pinMode(_Sensor1,INPUT);
   pinMode(_Sensor2,INPUT);
}


void RB_LineFollower::SetPin(uint8_t Sensor1,uint8_t Sensor2)
{
   _Sensor1 = Sensor1;
   _Sensor2 = Sensor2;
   pinMode(_Sensor1,INPUT);
   pinMode(_Sensor2,INPUT); 
}

uint8_t RB_LineFollower::ReadSensors(void)
{
    uint8_t state = S1_IN_S2_IN;
    bool    s1State = digitalRead(_Sensor1);
    bool    s2State = digitalRead(_Sensor2); 
    state  = ((1&s1State)<<1)|s2State;
    return(state);
}

bool RB_LineFollower::ReadSensor1(void)
{
  return (digitalRead(_Sensor1));
}

bool RB_LineFollower::ReadSensor2(void)
{
   return (digitalRead(_Sensor2));  
}

