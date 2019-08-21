#include "RB_QMIND.h"

RB_RGBLed           RGBLED(2,2);
RB_COLORSENSOR      Color1(1,TCS34725_INTEGRATIONTIME_2_4MS,TCS34725_GAIN_60X);
RB_DcmotorOnBoard   RB_DcmotorOnBoard;
RB_LineFollower     LineFollower_2(2);

 unsigned char  Forward  = 42 ;
 unsigned char  Turn     = 42 ;
 unsigned char  Color_check = 0;
  unsigned char  LineFollowFlag = 10;

  uint16_t red,blue,green,white;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  RB_DcmotorOnBoard.RB_DcmotorOnBoard_Init();
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
    RGBLED.setColor(0, 0, 0, 50);
    RGBLED.show();
    if (digitalRead(USER_KEY_Pin)==LOW) {
      RGBLED.setColor(0, 50, 0, 50);
      RGBLED.show();
      while(true){
            Color1.getRawData(&red,&green,&blue,&white);
            Color_check =0;
          if((red>40)&&(green<40)&&(blue<40)&&((red-green)>25))
            {
              Color_check = 1;
            }
            if((green>45)&&(red<45)&&(blue<45)&&((green-red)>15))
            {
              Color_check = 2;
            }
             if((blue>30)&&(red<35)&&(green<35)&&((blue-green)>3))
            {
              Color_check = 3;
            }
            Serial.print(red);
            Serial.print(",");
            Serial.print(green);
            Serial.print(",");
            Serial.println(blue);
            
            switch(Color_check)
            {
                case 1:RGBLED.setColor(0, 50, 0, 0);
                       RGBLED.show();
                       RB_DcmotorOnBoard.run(0,0);
                       while(1); 
                       break;
                case 2:RGBLED.setColor(0, 0, 50, 0);
                       RGBLED.show();
                       RB_DcmotorOnBoard.run(95,-80);
                       delay(300);
                       
                       break;
                case 3:RGBLED.setColor(0, 0, 0, 50);
                       RGBLED.show();
                       RB_DcmotorOnBoard.run(45,45);
                       delay(250);
                       break;
                default:
                       RGBLED.setColor(0, 0, 0, 0);
                       RGBLED.show();
                       LineFolloweMode();
                       break;
              
            } 
            
       }
    }
}


void LineFolloweMode(void)
{
    uint8_t  val =   LineFollower_2.ReadSensors();
    switch(val)
       {
         case S1_IN_S2_IN:    //全黑
              RB_DcmotorOnBoard.run(42,42);
              LineFollowFlag = 10;
              break;
         case S1_IN_S2_OUT:   //黑白
              RB_DcmotorOnBoard.run(42,42);
              if(LineFollowFlag>1)LineFollowFlag--;
              break;
         case S1_OUT_S2_IN:      //白黑  
              RB_DcmotorOnBoard.run(42,42);
              if(LineFollowFlag<20)LineFollowFlag++; 
              break;
         case S1_OUT_S2_OUT:    // 全白
              if(LineFollowFlag==10)
                  {RB_DcmotorOnBoard.run(75,-50); }
              if(LineFollowFlag<10)
                  {RB_DcmotorOnBoard.run(-75,75);}
              if(LineFollowFlag>10)
                  { RB_DcmotorOnBoard.run(75,-75);}
              break;  
         default:
              break; 
       }
  
  
}
