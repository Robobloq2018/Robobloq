#ifndef RB_MP3_H
#define RB_MP3_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 
#include <SoftwareSerial.h>


class RB_MP3:public RB_Port,public SoftwareSerial
{
	public:
	   RB_MP3(void);
	   RB_MP3(uint8_t port);
     void      RB_MP3_Star(uint8_t mode ,uint8_t data); 
     uint8_t   RB_MP3_Mode(void);
     void      RB_MP3_Set(uint8_t mode);
	 void      RB_MP3_END(void);
 protected:
    uint8_t _mode;

};





#endif
