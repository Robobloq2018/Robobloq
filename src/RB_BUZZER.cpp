#include "RB_BUZZER.h"
#include "avr/wdt.h"

// notes in the melody:
int melody[] = {
NOTE_G4,//5  
NOTE_G4,//5
NOTE_A4,//6
NOTE_G4,//5
NOTE_C5,//1.
NOTE_B4,//7
0,
NOTE_G4,//5
NOTE_G4,//5
NOTE_A4,//6
NOTE_G4,//5
NOTE_D5,//2.
NOTE_C5,//1.
0,
NOTE_G4,//5
NOTE_G4,//5
NOTE_G5,//5.
NOTE_E5,//3.
NOTE_C5,//1.
NOTE_B4,//7
NOTE_A4,//6
0,
NOTE_F5,//4.
NOTE_F5,//4.
NOTE_E5,//3.
NOTE_C5,//1.
NOTE_D5,//2.
NOTE_C5,//1.
0,
};

int noteDurations[] = {
  8,8,4,4,4,4,
  4,
  8,8,4,4,4,4,
  4,
  8,8,4,4,4,4,2,
  8,
  8,8,4,4,4,2,
  4,
};



/*
 * 
 */
RB_Buzzer::RB_Buzzer(int pin)
{
  buzzer_pin = pin;
}


/*
 * 
 */
void RB_Buzzer::setPin(int pin)
{
  buzzer_pin = pin;
}
 
/*
 * 
 */
 void RB_Buzzer::tone(int pin ,uint16_t frequency,uint32_t duration)
 {
    buzzer_pin= pin ;
    int period = 1000000L / frequency ;
    int pulse = period /2;
    pinMode(buzzer_pin ,OUTPUT);
    for(long i = 0; i< duration *1000L;i+=period)
    {
      digitalWrite(buzzer_pin,HIGH);
      delayMicroseconds(pulse);
      digitalWrite(buzzer_pin,LOW);
      delayMicroseconds(pulse);
      wdt_reset();
    }
  }
  /*
   * 
   */
   void RB_Buzzer::tone(uint16_t frequency,uint32_t duration)
  {
     int period = 1000000L / frequency ;
    int pulse = period /2;
    pinMode(buzzer_pin ,OUTPUT);
    for(long i = 0; i< duration *1000L;i+=period)
    {
      digitalWrite(buzzer_pin,HIGH);
      delayMicroseconds(pulse);
      digitalWrite(buzzer_pin,LOW);
      delayMicroseconds(pulse);
      wdt_reset();
    }    
  }
  /*
   * 
   */
  void RB_Buzzer::noTone(int pin)
{
  buzzer_pin = pin;
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);
}
/*
 * 
 */
 void RB_Buzzer::noTone(void)
{
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);
}
