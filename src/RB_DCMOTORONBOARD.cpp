


#include "RB_DCMOTORONBOARD.h"






RB_DcmotorOnBoard::RB_DcmotorOnBoard_Init(void)
{ 

     pinMode(MOTOR_PWM0, OUTPUT);  
     pinMode(MOTOR_PWM1, OUTPUT);
     pinMode(MOTOR_PWM2, OUTPUT);  
     pinMode(MOTOR_PWM3, OUTPUT);  

  
     
  
     TCCR1A=0xAD;           //A、B、C通道非反转波形
     TCCR1B=0x0B;           //8分频，计数器设置为10位

      
      OCR1A  = 0;
      OCR1B  = 0;


      delay(500);
      
}

void RB_DcmotorOnBoard::RB_DcmotorOnBoardM1_Run(int32_t speed)
{    
  
      bool M_Dir = true;
      speed = constrain(speed,-100,100);
      if(speed < 0 ) {
            M_Dir = false;
      }
      else {
            M_Dir = true ;
      }
      
      if(M_Dir == true) {
           speed = abs(speed)*255/100;
           if(speed==0)
              pinMode(MOTOR_PWM1, INPUT);  
           else 
              pinMode(MOTOR_PWM1, OUTPUT);  
           pinMode(MOTOR_PWM0, INPUT); 
           analogWrite(MOTOR_PWM1,speed);
          
       }  
      else {
           speed = abs(speed)*255/100;
             if(speed==0)
                 pinMode(MOTOR_PWM0, INPUT);  
             else 
               pinMode(MOTOR_PWM0, OUTPUT); 
           pinMode(MOTOR_PWM1, INPUT);  
          analogWrite(MOTOR_PWM0,speed);
       }
      _M1_Speed =  speed;
}

void RB_DcmotorOnBoard::RB_DcmotorOnBoardM2_Run(int32_t speed)
{  
      
      bool M_Dir = true;
      speed = constrain(speed,-100,100);
      if(speed < 0 ) {
            M_Dir = false;
      }
      else {
            M_Dir = true ;
      } 

      if(M_Dir == true) {
         speed =  (abs(speed))*255/100; 
         if(speed==0)
                 pinMode(MOTOR_PWM2, INPUT);  
             else 
               pinMode(MOTOR_PWM2, OUTPUT);   
         pinMode(MOTOR_PWM3, INPUT); 
    
            OCR1A  = speed;
            OCR1B  = 0;

      }  
      else {
          speed =  (abs(speed))*255/100; 
          if(speed==0)
                 pinMode(MOTOR_PWM3, INPUT);  
             else 
               pinMode(MOTOR_PWM3, OUTPUT); 
          pinMode(MOTOR_PWM2, INPUT); 
            OCR1B  = speed;
            OCR1A  = 0;

          _M2_Speed = speed;
      }
}

void RB_DcmotorOnBoard::run(int32_t speed1,int32_t speed2)
 {   
    
     RB_DcmotorOnBoard::RB_DcmotorOnBoardM1_Run(speed1);
     RB_DcmotorOnBoard::RB_DcmotorOnBoardM2_Run(speed2);

 
  }
void RB_DcmotorOnBoard::stop(void)
 {
     RB_DcmotorOnBoard::RB_DcmotorOnBoardM1_Run(0);
     RB_DcmotorOnBoard::RB_DcmotorOnBoardM2_Run(0);
  }

 void RB_DcmotorOnBoard::SetSpeed( short Left_Speed, short Right_Speed)
 {    
     Left_Speed = constrain(Left_Speed,-250,250);
     Right_Speed= constrain(Right_Speed,-250,250);
     if(Left_Speed<0)
       {
           analogWrite(MOTOR_PWM0, abs(Left_Speed));
           analogWrite(MOTOR_PWM1, 0);  
       }
     else {
           analogWrite(MOTOR_PWM1, abs(Left_Speed));
           analogWrite(MOTOR_PWM0, 0);  
     }
     if(Right_Speed<0)
     {
        OCR1A = 0 ; OCR1B = abs(Right_Speed); 
     }
     else 
     { 
         OCR1B = 0 ; OCR1A = abs(Right_Speed); 
     } 

  }
  
