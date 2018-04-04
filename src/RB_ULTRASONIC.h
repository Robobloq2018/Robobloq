#ifndef RB_Ultrasonic_H
#define RB_Ultrasonic_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"


 class RB_Ultrasonic : public RB_SoftI2CMaster
{
  public : 
     RB_Ultrasonic(void);
     RB_Ultrasonic(uint8_t port);
     void RB_Ultrasonic_SetRGB(uint8_t SlaveAddress,uint8_t RegisterAddress,uint8_t Red,uint8_t Green,uint8_t Blue);
     double distanceCm(uint16_t = 250);
     double distanceInch(uint16_t = 180);
     long   measure(unsigned long = 30000);
     double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
	    double Uldistance(double max_distance=300);
	 void   Swap(double  A[], int i, int j);
	 void   BubbleSort(double A[], int n);
   private:
      volatile uint8_t  _SignalPin;
      volatile bool _measureFlag;
      volatile long _lastEnterTime;
      volatile float _measureValue;
};
#endif 
