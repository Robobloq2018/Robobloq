

#include "RB_TOUCHSENSOR.h"


RB_TOUCHSENSOR   cap(2);

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

unsigned char  data[4];

void setup() {
  while (!Serial);        // needed to keep leonardo/micro from starting too fast!

  Serial.begin(115200);

  


}

void loop() {

   cap.RB_TOUCHSENSOR_ReadArray(0xB0,data,4);
   Serial.write(data[0]);
   Serial.write(data[1]);
   Serial.write(data[2]);
   Serial.write(data[3]);
   data[1] = 0x00;
   data[2] = 0x00;
   delay(100);
   currtouched = cap.RB_TOUCHSENSOR_ReadValue();
 //   Serial.println(currtouched);
/*
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
    }
  }

  // reset our state
  lasttouched = currtouched;
  delay(100);
  // comment out this line for detailed data from the sensor!
  return;
*/
}
