#ifndef _RB_TempAndHumi_H
#define _RB_TempAndHumi_H


#include<stdint.h>
#include<stdbool.h>
#include<Arduino.h>

#include "RB_PORT.h" 


class RB_TempAndHumi:public RB_Port
{
	public :
	         RB_TempAndHumi(void);
 			     RB_TempAndHumi(uint8_t port);
			     SetPin(uint8_t pin);
	         void Update(void);
	         uint8_t GetHumidity(void);
	         uint8_t GetTemperature(void);
		 uint8_t GetHumidity_2(void);
	         uint8_t GetTemperature_2(void);
			     uint8_t GetValue(uint8_t index);
			     double  GetFahrenheit(void);
			     double  GetKelvin(void);
			     double  GetdewPoint(void);
			     double  getPointFast(void);
	private :
             uint8_t Humidity;
             uint8_t Temperature;
             uint8_t _DataPin;			 
    
};






#endif
