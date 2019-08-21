
#include "RB_JOYSTICK.h"


RB_JOYSTICK   joystick(2);


int16_t x = 0;    /* a variable for the Joystick's x value */
int16_t y = 0;    /* a variable for the Joystick's y value */
float angle = 0;       /* The relative angle of XY */
float OffCenter = 0;    /* offset with the center */

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
    /* read the both joystick axis values: */
  x = joystick.ReadJoystickX();
  y = joystick.ReadJoystickY();
  OffCenter = joystick.OffCenter();

  /* print the results to the serial monitor: */
  Serial.print("Joystick X = ");
  Serial.print(x);
  Serial.print("  Joystick Y = ");
  Serial.print(y);
  Serial.print("  OffCenter= ");
  Serial.println(OffCenter);
  /* wait 10 milliseconds before the next loop */
  delay(10);
}
