#include "RB_ADC.h"
#include "RB_BUZZER.h"
#include "RB_PORT.h"
#include "RB_RGBLED.h"
#include "RB_ULTRASONIC.h"
#include "RB_DCMOTORONBOARD.h"
#include "RB_LINEFOLLOWARRAY.h"


#define   auto_mode            0
#define   linefollow_mode      1
#define   ulrasonic_mode       2


RB_RGBLed             RGBled(RGB_LED_Pin,2);
RB_Buzzer             Buzzer(Buzzer_Pin);
RB_DcmotorOnBoard     RB_DcmotorOnBoard;
RB_LINEFOLLOWARRAY    *linefollowarray=NULL;
RB_Ultrasonic         Ultrasonic(3); 
RB_ADC                adc;

uint8_t work_mode = linefollow_mode;
bool    Hardware_Button_flag = false;
bool    Button_Down          = false;

void setup() {
  // put your setup code here, to run once:
      pinMode(USER_KEY_Pin,INPUT);     
      Serial.begin(115200);  
      Buzzer.noTone(); 
      RGBled.setpin(RGB_LED_Pin); 
      RGBled.setColor(0,0,0,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,50,0,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,0,50,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,0,0,50);
      RGBled.show();
      delay(500);
       RGBled.setColor(0,50,0,0);
      RGBled.show();
      delay(500);
      Buzzer.tone(500,500); 
      RB_DcmotorOnBoard.RB_DcmotorOnBoard_Init(); 
      Serial.println("RoboCup");
      Serial.println("Version:1.1.0");
      delay(1000);
     
      
   
      

      
}

void loop() {
  // put your main code here, to run repeatedly:
     Button_Process();
     switch(work_mode)
        {
            case  auto_mode: 
                  break;
            case  linefollow_mode:
                  LineFollow_mode();
                  break;
            case  ulrasonic_mode:
                  Ulrasonic_mode();
                  break;      
        }
}


void Button_Process(void)
{
       if(digitalRead(USER_KEY_Pin) == HIGH)
              { Hardware_Button_flag = true;}
       if((digitalRead(USER_KEY_Pin) == LOW) && (Hardware_Button_flag == true)){
               Button_Down = true;  Hardware_Button_flag = false;
       }
       if(Button_Down==true){
          Button_Down = false;
          switch(work_mode)
             {
               case  auto_mode:
                     RGBled.setColor(0,0,50,0);
                     RGBled.show();
                     work_mode = linefollow_mode;
                     break;
               case linefollow_mode:
                     RGBled.setColor(0,0,0,50);
                     RGBled.show();
                     work_mode = ulrasonic_mode;
                     break;
               case ulrasonic_mode:
                      RGBled.setColor(0,50,0,0);
                      RGBled.show();
                      RB_DcmotorOnBoard.SetSpeed(0,0);   
                      work_mode = auto_mode;
                       break;                       
             }
       }
}



void  LineFollow_mode(void)
{
     
     uint8_t array_value = 0;
     static  uint8_t status = 0;
     if(adc.RB_ADC_Read(2)==18)
     {
     if(linefollowarray==NULL)linefollowarray = new RB_LINEFOLLOWARRAY(2); 
     array_value = linefollowarray->RB_LINEFOLLOWARRAY_ReadValue() ;
     Serial.println(array_value,BIN); //输出6路灰度传感器
     }
     else
     {  
        if(linefollowarray!=NULL){
           delete  linefollowarray;
           linefollowarray = NULL;
           Serial.println(0); //输出6路灰度传感器
        }  
     }
     if(array_value==0){
          status = 0;
     }
     else if(array_value == 0x3F) {
          switch(status)  {
              case 0: RB_DcmotorOnBoard.SetSpeed(50,50);  break;
              case 1: RB_DcmotorOnBoard.SetSpeed(-40,85); break;
              case 2: RB_DcmotorOnBoard.SetSpeed(20,80);  break;
              case 3: RB_DcmotorOnBoard.SetSpeed(85,20);  break;
              case 4: RB_DcmotorOnBoard.SetSpeed(85,-40); break;  
              case 5: RB_DcmotorOnBoard.SetSpeed(50,50);  break;         
            }  
      }
      else {
           if((array_value&0x20)==0) {
                              status = 1;
                              RB_DcmotorOnBoard.SetSpeed(-40,85);  
            }
           else if((array_value&0x10)==0){
                              status = 2;
                              RB_DcmotorOnBoard.SetSpeed(20,80);
           }
           else if((array_value&0x02)==0) {
                             status = 3;
                             RB_DcmotorOnBoard.SetSpeed(85,20);
           }
           else if((array_value&0x01)==0){
                            status = 3;
                            RB_DcmotorOnBoard.SetSpeed(85,-40);
           }
           else   {
                            status  =5 ;
                            RB_DcmotorOnBoard.SetSpeed(50,50);
           }
      }
}


void Ulrasonic_mode(void)
{
    static   unsigned char flag  = 0;
    static   unsigned char last_flag = 0;
    uint8_t  randnum;

     
     double Distance = Ultrasonic.Uldistance();
     Distance = Kalman_Filter(Distance,0.1,4);

     Serial.println(Distance); //输出超声波距离传感器数据
     delay(10);
     if(Distance>35)  { flag = 0; last_flag = 0; }
     else if((Distance>=22)&&(Distance<=35))  { 
          if(flag==0)  {flag =1;}
          else { last_flag  =1 ;flag = 1; }
     }    
     else  {       
            if(flag ==0){flag =1;}
            else if(flag ==1) {flag =2;}
            else { last_flag = 2;flag = 2; }
     }
     switch(last_flag) {
             case 0:
                   RB_DcmotorOnBoard.SetSpeed(55,55);
                   RGBled.setColor(0,0,0,50);
                   RGBled.show();
                   break;
             case 1:
                   RGBled.setColor(0,50,0,0);
                   RGBled.show();
                   randnum = random(200);
                   if(randnum>100){ RB_DcmotorOnBoard.SetSpeed(-200,200); }
                   else {RB_DcmotorOnBoard.SetSpeed(200,-200);}
                   delay(350);
                   break;
             case 2: 
                   RGBled.setColor(0,0,50,0);
                   RGBled.show();
                   RB_DcmotorOnBoard.SetSpeed(-90,-90);
                   delay(300);
                   randnum = random(200);
                   if(randnum>100){ RB_DcmotorOnBoard.SetSpeed(-98,98);}
                   else { RB_DcmotorOnBoard.SetSpeed(98,-98);}
                   delay(300);
                   break; 
             
     }
}
float Kalman_Filter( const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
  float R = MeasureNoise_R;
  float Q = ProcessNiose_Q;

  static  float x_last;

  float x_mid = x_last;
  float x_now;

  static  float p_last;

  float p_mid ;
  float p_now;
  float kg; 

  x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
  p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
  kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
  x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
    
  p_now=(1-kg)*p_mid;//最优值对应的covariance 

  p_last = p_now; //更新covariance值
  x_last = x_now; //更新系统状态值

  return x_now;   
}

