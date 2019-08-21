#ifndef RB_OneWire_h
#define RB_OneWire_h

#ifdef __cplusplus

#include <stdint.h>



#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif






#include "OneWire_direct_regtype.h"

class RB_OneWire
{
  private:
    IO_REG_TYPE bitmask;
    volatile IO_REG_TYPE *baseReg;
  public:
     RB_OneWire(void);
	 RB_OneWire(uint8_t pin);
	 void reset(uint8_t pin);
	 uint8_t reset(void);
	 bool readIO(void);
	 void write_bit(uint8_t v);
	 uint8_t read_bit(void);
	 void write(uint8_t v, uint8_t power=0);
	 void write_bytes(const uint8_t *buf, uint16_t count, bool power =0);
	 uint8_t read();
	 void read_bytes(uint8_t *buf, uint16_t count);
	 void select(const uint8_t rom[8]);
	 void skip();
	 void depower();
    




};



#endif // __cplusplus
#endif // OneWire_h
