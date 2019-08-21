#include "RB_BUZZER.h"


RB_Buzzer Buzzer(Buzzer_Pin);


void setup() {
  // put your setup code here, to run once:
      for (int thisNote = 0; thisNote < 29; thisNote++) {
                  int noteDuration = 1000/noteDurations[thisNote];
                  Buzzer.tone(melody[thisNote],noteDuration);
                  int pauseBetweenNotes = noteDuration*0.6 ;
                  delay(pauseBetweenNotes);
                  Buzzer.noTone();
                 } 
}

void loop() {
  // put your main code here, to run repeatedly:

}
