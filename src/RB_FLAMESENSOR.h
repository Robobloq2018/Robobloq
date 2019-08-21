#ifndef RB_FLAMESENSOR_H
#define RB_FLAMESENSOR_H


#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#include "RB_PORT.h" 


class RB_FLAMESENSOR:public RB_Port
{
	public:
	             RB_FLAMESENSOR(void);
	             RB_FLAMESENSOR(uint8_t port);
            void SetPin(uint8_t sigpin_dig,uint8_t sigpin_ana);
        uint16_t GetFlameAnalog(void);
        uint8_t  GetFlameDValue(void);
        uint16_t GetFlameAValue(void);

 private:
        uint8_t  _SigPin_Digital;
	    uint8_t  _SigPin_Analog;
};


#endif
