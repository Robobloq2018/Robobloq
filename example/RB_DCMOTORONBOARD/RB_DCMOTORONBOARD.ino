
#include <RB_DCMOTORONBOARD.h>

RB_DcmotorOnBoard RB_DcmotorOnBoard;



void setup()
{
    Serial.begin(115200);
    RB_DcmotorOnBoard.RB_DcmotorOnBoard_Init();
    delay(1000);
    RB_DcmotorOnBoard.run(90,90);
    delay(1000);
    RB_DcmotorOnBoard.run(-90,-90);
    delay(1000);
 }


 void loop()
 {
  
  
  }
