#include "RB_LINEFOLLOWARRAY.h"


RB_LINEFOLLOWARRAY   line(2);

uint8_t line_value =0;
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   line_value = line.RB_LINEFOLLOWARRAY_ReadValue();
   Serial.print("LineFollowArray Value:");
   Serial.println(line_value);
   Serial.print("LineFollowArray Bin:");
   Serial.println(line_value,BIN);
}
