/***********************************************
 *           Date              : 02/01/2018
 *           File Name         : Firmware for Qmind.ino
 *           Author            : Allen
 *           Updated           : Allen
 *           http://www.robobloq.com/ 
 *           History:
 *           <Version>         <Time>             <Author>                    <Descr>
 *           2.1.1             2017/11/13         Allen                      Build the New
 *           2.1.2             2017/12/27         Allen                      出厂固件
 *           2.1.3             2018/01/02         Allen                      新增加巡线传感器数据读取
 *           2.1.3             2018/01/03         Allen                      新增加舵机驱动控制
 *           2.1.4             2018/01/09         Allen                      新增DHT11 温湿度传感器
 *           2.1.4             2018/02/26         Allen                      新增光线传感器
 *           2.1.4             2018/02/26         Allen                      新增声音传感器
 *           2.1.5             2018/07/04         Allen                      串口初始化放在前面，解决蓝牙电平不稳定导致的问题
 *           2.1.6             2018/07/27         Allen                      新增加陀螺仪、颜色、MP3、RGBLED传感器通信固件
 *           2.1.6             2018/07/27         Allen                      修复超声波传感器
 *           6.1.6             2018/09/01         Allen                      大象固件
 *********************************************/





#include "RB_QMIND.h"
 

  uint8_t     Work_Mode      = 0 ;
  char        Work_Mode_Port = 0;

/*
 * 
 */
RB_RGBLed RGBled(0,2);
RB_Buzzer Buzzer(Buzzer_Pin);
RB_Serial RB_Serial;
RB_Ultrasonic    *Ultrasonic   = NULL ;
RB_LEDMatrix     *LEDMatrix    = NULL;
RB_LineFollower  *LineFollower = NULL ;
RB_SERVO         *Servo1       = NULL ;
RB_TempAndHumi   *TempAndHumi  = NULL ;
RB_LightSensor   *LightSensor  = NULL ;
RB_SoundSensor   *SoundSensor  = NULL ;
RB_DCMotor       *DCmotor      = NULL ;  
RB_PirSensor     *PirSensor    = NULL ;

RB_MP3           *MP3          = NULL;
RB_RGBLEDMATRIX  *RGBLEDMatrix = NULL;
RB_COLORSENSOR   *COLORSENSOR  = NULL;
RB_GYRO          *GYRO  = NULL;
RB_TOUCHSENSOR   *TouchSensor  = NULL;



RB_DcmotorOnBoard RB_DcmotorOnBoard;
RB_ADC            RB_ADC;

uint8_t      Button_text_flag = 0;
bool         Button_Down = false;
bool         Button_flag   = false;
bool         Hardware_Button_flag =false;
uint8_t      LineFollowFlag = 10;

bool        Low_Porwer_flag = 0;
uint8_t     Power_Send_count =0;
uint32_t    Power_ADC_count = 0;
uint8_t     flag = 0;
uint16_t     white_count = 0;
volatile unsigned short RGB_LED_test[72];
volatile unsigned short RGB_LED_test2[72];
volatile uint8_t d1[16],d2[16],d3[16],d4[16];


void setup()
{
  // put your setup code here, to run once:
      pinMode(USER_KEY_Pin,INPUT);     
      RB_Serial.Serial_begin(115200);   
      delay(300);
      Buzzer.noTone(); 
      RGBled.setpin(RGB_LED_Pin); 
      RGBled.setColor(0,0,0,0);
      RGBled.show();
      delay(300);
      RGBled.setColor(0,30,0,0);
      RGBled.show();
      delay(300);
      RGBled.setColor(0,0,30,0);
      RGBled.show();
      delay(300);
      RGBled.setColor(0,0,0,30);
      RGBled.show();
      delay(300);
      Buzzer.tone(500,500); 
      RB_DcmotorOnBoard.RB_DcmotorOnBoard_Init();
      SendReset();
      wdt_enable(WDTO_2S);
      wdt_reset();
    
   
}
void loop() 
{       

        wdt_reset();
        SerialCheakEvent();
        Button_Process();
        if(Low_Porwer_flag){
          Power_Process();
        }
        switch(Work_Mode)
           {
              case Remote_Control_Mode:
                      SensorType_Update(); 
                      break;
            }

          
}
void SerialDataAnalysis(bool ack)
{   
    
    char TX_CheckSum =0;
    bool Set_Mode_Flag = 0;
    uint8_t  i =0; 
    double distance ; 
    int    distance_1 ; 
    int V2 = 0;
    uint8_t TX_Count = 0;
    switch((uint8_t)RX_BUF[ack][4])
       {
        case Hardware://获取版本信息
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     TX_BUF[TX_Count++] = 6;
                     TX_BUF[TX_Count++] = HardwareVersion;
                     TX_BUF[TX_Count++] = SoftwareVersion;
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                         TX_CheckSum += TX_BUF[i];
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     
                     break; 
           case Device_Type:
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];    
                     switch(RX_BUF[ack][5])
                     {   
                        case 1:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(1);
                                  break;
                        case 2:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(2);
                                  break;
                        case 3:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(3);
                                  break;
                        case 4:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(4);
                                  break;
                        default:
                                  break;  
                    }
                   
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break; 
           case Device_Motor:
                     
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     TX_BUF[TX_Count++] = 1;
                     TX_BUF[TX_Count++] = 1;
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;            
          case Device_Port:
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(1);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(2);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(3);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(4);
                     TX_BUF[TX_Count++] = 0;
                     TX_BUF[TX_Count++] = 0;
                     TX_BUF[TX_Count++] = 0;
                     TX_BUF[TX_Count++] = 0;
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
        case Led_SET:
                      switch(RX_BUF[ack][5]) //端口号
                      {
                        case 0:  RGBled.setColor(0,(uint8_t)RX_BUF[ack][6],(uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                                 RGBled.show();
                                 break;
                        case -4: RGBled.setColor(1,(uint8_t)RX_BUF[ack][6],(uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                                 RGBled.show();
                                 break;
                        case -5: RGBled.setColor(2,(uint8_t)RX_BUF[ack][6],(uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                                 RGBled.show();
                                 break;
                        default: break;
                      }
                      Set_Mode_Flag =1;
                      break;
        case MoterSpeed_SET:
                      switch(RX_BUF[ack][5])
                       {
                         case 0:RB_DcmotorOnBoard.run (RX_BUF[ack][6],RX_BUF[ack][7]);
                                 if((RX_BUF[ack][6]==0)&&(RX_BUF[ack][7]==0))
                                     {
                                        Work_Mode_Port  = 0;
                                        Work_Mode       = 0;
                                      }
                                 break;
                         case -1:RB_DcmotorOnBoard.RB_DcmotorOnBoardM1_Run(RX_BUF[ack][6]);
                                 break;
                         case -2:RB_DcmotorOnBoard.RB_DcmotorOnBoardM2_Run(RX_BUF[ack][6]);
                                 break;
                         default: break;
                       }
                       Set_Mode_Flag =1;
                       break;
        case UlSensorLed_Set:
                       if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Ultrasonic_Distance_Sensor)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                       if(Ultrasonic == NULL){
                             Ultrasonic = new RB_Ultrasonic(RX_BUF[ack][5]);
                       }
                       else if(Ultrasonic->GetPort() != RX_BUF[ack][5]){
                            delete Ultrasonic;
                            Ultrasonic = new RB_Ultrasonic(RX_BUF[ack][5]);  
                        }
                        Ultrasonic->RB_Ultrasonic_SetRGB(0x40,0XA1,(uint8_t)RX_BUF[ack][6],(uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                        Set_Mode_Flag =1;
                        break;
        case Button_Set:
                      switch(RX_BUF[ack][5])
                       {
                         case 0x00:
                                break;
                         case -7:if(RX_BUF[ack][6])
                                        Button_flag = 1;
                                   else 
                                        Button_flag = 0;
                                break;
                         default: break; 
                       }
                       Set_Mode_Flag =1;
                       break;
                      
        case LowPorw_Set:
                      switch(RX_BUF[ack][6])
                      {
                        case 0x00:Low_Porwer_flag = 0;
                             break;
                        case 0x01:Low_Porwer_flag = 1;
                             break;
                        default:break;   
                      }
                      Power_ADC_count = 0; 
                      Power_Send_count = 0;  
                      Set_Mode_Flag =1;
                      break;
        case Buzzer_Set:
                       uint16_t fre ,dur;
                       uint8_t  fre_H ,fre_L,dur_H,dur_L;
                       fre_H = (uint8_t)RX_BUF[ack][6];
                       fre_L = (uint8_t)RX_BUF[ack][7];
                       dur_H = (uint8_t)RX_BUF[ack][8];
                       dur_L = (uint8_t)RX_BUF[ack][9];
                       fre = (uint16_t)fre_H<<8|fre_L;
                       dur = (uint16_t)dur_H<<8|dur_L;
                       switch(RX_BUF[ack][5])
                         {
                          case -6:  Buzzer.tone(fre,dur);
                                    Buzzer.noTone(); 
                                    break;
                          default:  break;
                         }
                         Set_Mode_Flag =1;
                         break;
        case ExPanel_Set:
                      if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=LED_Matraix_Blue)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                      if(LEDMatrix == NULL){
                        LEDMatrix = new RB_LEDMatrix( RX_BUF[ack][5]);
                      }
                      else if(LEDMatrix->GetPort() != RX_BUF[ack][5]){
                          delete LEDMatrix;
                          LEDMatrix = new RB_LEDMatrix(RX_BUF[ack][5]);
                      }
                      LEDMatrix->RB_LEDMatrix_PageWrite( 0x00,&RX_BUF[ack][6]);
                      Set_Mode_Flag = 1;
                      break;
                      
        case Mode_Change:
                      Work_Mode_Port = RX_BUF[ack][5]; 
                      Work_Mode = RX_BUF[ack][6];
                      Set_Mode_Flag = 1;
                      break;
        case Servo_Set:
                       if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Servo_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                  
                      if(Servo1 == NULL){
                          Servo1 = new RB_SERVO(RX_BUF[ack][5]);
                          Servo1->RB_SERVO_INIT();
                      }
                      else if(Servo1->GetPort() != RX_BUF[ack][5])
                      {
                          delete Servo1;
                          Servo1 = new RB_SERVO(RX_BUF[ack][5]);
                          Servo1->RB_SERVO_INIT();
                      }
                      switch(RX_BUF[ack][6])
                      {
                          case 0:Servo1->RB_SERVO_Write((uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                                 break;
                          case 1:Servo1->RB_SERVO_Write_Servo1((uint8_t)RX_BUF[ack][7]);
                                 break;
                          case 2:Servo1->RB_SERVO_Write_Servo2((uint8_t)RX_BUF[ack][7]);
                                 break;
                      }
             
                      Set_Mode_Flag = 1;
                      break;  
           case ExDCMotor_Set:
                         if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=DC_Motor_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                           if(DCmotor == NULL){
                               DCmotor = new RB_DCMotor(RX_BUF[ack][5]);
                           }
                          else if(DCmotor->GetPort() != RX_BUF[ack][5])
                          {
                             delete DCmotor;
                             DCmotor = new RB_DCMotor(RX_BUF[ack][5]);
                          }
                          switch(RX_BUF[ack][6])
                               {
                                  case 0:if(RX_BUF[ack][7]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X01,abs(RX_BUF[ack][7])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X00,abs(RX_BUF[ack][7])); }
                                         if(RX_BUF[ack][8]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X01,abs(RX_BUF[ack][8])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X00,abs(RX_BUF[ack][8])); }   
                                        break;
                                  case 1:if(RX_BUF[ack][7]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X01,abs(RX_BUF[ack][7])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X00,abs(RX_BUF[ack][7])); }
                                        break;
                                  case 2:if(RX_BUF[ack][8]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X01,abs(RX_BUF[ack][8])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X00,abs(RX_BUF[ack][8])); }   
                                        break;
                                }
            
                      Set_Mode_Flag = 1;
                      break; 
        case RGBLEDMatrix_Set:
                       if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=RGBLED_Matraix )
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                       if(RGBLEDMatrix== NULL)
                        { 
                          if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==RGBLED_Matraix){
                          RGBLEDMatrix = new RB_RGBLEDMATRIX(RX_BUF[ack][5]);
                          RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X77);
                          RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X74);   
                          RGBLEDMatrix->RGBLEDMATRIX_Start(d1,d2,d3,d4); 
                          }
                         }
                     else if(RGBLEDMatrix->GetPort() != RX_BUF[ack][5]){
                           if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==RGBLED_Matraix){
                              delete RGBLEDMatrix;
                              RGBLEDMatrix = new RB_RGBLEDMATRIX(RX_BUF[ack][5]);
                              RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X77);
                              RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X74); 
							  RGBLEDMatrix->RGBLEDMATRIX_Start(d1,d2,d3,d4);
                           }
                      }
                       memset(RGB_LED_test,0,72);
                       for(int i=0;i<72;i++)
                       {
                          RGB_LED_test[i] = (uint16_t)(RX_BUF[ack][78+i]);
                       }
                       memset(RGB_LED_test2,0,72);
                        for(int i=0;i<72;i++)
                       {
                          RGB_LED_test2[i] = (uint16_t)(RX_BUF[ack][6+i]);
                        }
                       RGBLEDMatrix->RGBLEDMATRIX_DATA(RGB_LED_test2,d3,d4); 
                       RGBLEDMatrix->RGBLEDMATRIX_DATA(RGB_LED_test,d1,d2);
                       RGBLEDMatrix->RGBLEDMATRIX_Start(d1,d2,d3,d4);
                       Set_Mode_Flag =1;
                       break;
         case MP3_Set : 
                      if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=MP3_Sensor)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                     if(MP3 == NULL)
                        { 
                          
                          MP3 = new RB_MP3(RX_BUF[ack][5]);
                         }
                      else if(MP3->GetPort() != RX_BUF[ack][5]){
                          delete MP3;
                          MP3 = new RB_MP3(RX_BUF[ack][5]);
                      }
                       if(MP3->RB_MP3_Mode()!=RX_BUF[ack][6])
                        {
                          MP3->RB_MP3_Set(RX_BUF[ack][6]);
                        }
                       MP3->RB_MP3_Star(RX_BUF[ack][7],RX_BUF[ack][8]);
                        MP3->RB_MP3_END();
                       delete MP3;
                       MP3 = NULL;
                       pinMode(RBPort[RX_BUF[ack][5]].clk,INPUT);
                       pinMode(RBPort[RX_BUF[ack][5]].dat,INPUT);
                       Set_Mode_Flag =1;
                       break;
        case UlSensorDistance_Read:  
                     if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Ultrasonic_Distance_Sensor)
                          {
                            break;
                          } 
                    if(Ultrasonic == NULL){
                         Ultrasonic = new RB_Ultrasonic(RX_BUF[ack][5]);
                     }
                     else if(Ultrasonic->GetPort() != RX_BUF[ack][5]){
                          delete Ultrasonic;
                          Ultrasonic = new RB_Ultrasonic(RX_BUF[ack][5]);  
                     }
                 distance = Ultrasonic->Uldistance();
                     distance_1  = (int)(distance *10);
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     TX_BUF[TX_Count++] = 0x01;  
                     TX_BUF[TX_Count++] = distance_1/256;
                     TX_BUF[TX_Count++] = distance_1%256; 
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                       {TX_CheckSum += TX_BUF[i];}
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
        case Button_Read:
                     switch(RX_BUF[ack][5])
                     {
                         case 0x00:    break;
                         case -7:      TX_BUF[TX_Count++] = 0x52;
                                       TX_BUF[TX_Count++] = 0X42;
                                       TX_BUF[TX_Count++] = 0X00;
                                       TX_BUF[TX_Count++] = RX_BUF[ack][3];
                                       TX_BUF[TX_Count++] = 0x01;
                                       if(digitalRead(USER_KEY_Pin) == HIGH)
                                           TX_BUF[TX_Count++] = 0x00;
                                        else   
                                            TX_BUF[TX_Count++] = 0x01;
                                       TX_BUF[2] = TX_Count+1;
                                       for(i=0;i<TX_Count;i++)
                                          {TX_CheckSum += TX_BUF[i];}
                                        TX_BUF[TX_Count++] = TX_CheckSum;
                                    RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                                    
                                break;
                         default:
                                break; 
                       }
                     break;
             case Power_Read:
                     switch(RX_BUF[ack][5])
                       {
                         case 0x00:
                                break;
                         case -3:
                                 V2  = analogRead(A7);
                                 V2 =V2*50/1023;
                                 TX_BUF[TX_Count++] = 0x52;
                                 TX_BUF[TX_Count++] = 0X42;
                                 TX_BUF[TX_Count++] = 0X00;
                                 TX_BUF[TX_Count++] = RX_BUF[ack][3];
                                 TX_BUF[TX_Count++] = 0x01; 
                                 TX_BUF[TX_Count++] = (V2%256)*3;
                                 TX_BUF[2] = TX_Count+1;                   
                                 for(i=0;i<TX_Count;i++)
                                          {TX_CheckSum += TX_BUF[i];}
                                 TX_BUF[TX_Count++] = TX_CheckSum;
                                 RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                                 break;
                         default:break;
                          }  
                         break;
           case  Line_Follower_Read :
                     if(LineFollower == NULL){
                         LineFollower = new RB_LineFollower(RX_BUF[ack][5]);
                     }
                     else if(LineFollower->GetPort() != RX_BUF[ack][5]){
                          delete LineFollower;
                          LineFollower = new RB_LineFollower(RX_BUF[ack][5]);  
                     } 
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] =  LineFollower->ReadSensors();;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
          case Humi_And_Temp_Read:
                    if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Temp_And_Humi_Sensor)
                          {
                            break;
                          }   
                   if(TempAndHumi == NULL){
                           TempAndHumi = new RB_TempAndHumi(RX_BUF[ack][5]);
                     }
                     else if(TempAndHumi->GetPort() != RX_BUF[ack][5]){
                          delete TempAndHumi;
                          TempAndHumi = new RB_TempAndHumi(RX_BUF[ack][5]);  
                     }
                      TempAndHumi->Update();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = TempAndHumi->GetHumidity();
                      TX_BUF[TX_Count++] = 0;
                      TX_BUF[TX_Count++] = TempAndHumi->GetTemperature();
                      TX_BUF[TX_Count++] = 0;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
        case Light_Read :
                     if(LightSensor == NULL){
                           LightSensor = new RB_LightSensor(RX_BUF[ack][5]);
                     }
                     else if(LightSensor->GetPort() != RX_BUF[ack][5]){
                          delete LightSensor;
                          LightSensor = new RB_LightSensor(RX_BUF[ack][5]);  
                     }
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = (uint8_t)(LightSensor->GetLightValue()/256);
                      TX_BUF[TX_Count++] = (uint8_t)(LightSensor->GetLightValue()%256);
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
        case Sound_Read:
                      if(SoundSensor == NULL){
                           SoundSensor = new RB_SoundSensor(RX_BUF[ack][5]);
                     }
                     else if(SoundSensor->GetPort() != RX_BUF[ack][5]){
                          delete SoundSensor;
                          SoundSensor = new RB_SoundSensor(RX_BUF[ack][5]);  
                     }
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = (uint8_t)(SoundSensor->GetSoundValue()/256);
                      TX_BUF[TX_Count++] = (uint8_t)(SoundSensor->GetSoundValue()%256);
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;                
           case PIRSensor_Read:
                      if(PirSensor == NULL){
                           PirSensor = new RB_PirSensor(RX_BUF[ack][5]);
                     }
                     else if(PirSensor->GetPort() != RX_BUF[ack][5]){
                          delete PirSensor;
                          PirSensor = new RB_PirSensor(RX_BUF[ack][5]);  
                     }
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = PirSensor->GetPirSensor();
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break;      
            case ColorSensor_Read:
                      if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Color_Sensor)
                          {
                            break;
                          }
                     unsigned short c,r,g,b;
                     if(COLORSENSOR == NULL){
                            if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Color_Sensor)
                            {
                           COLORSENSOR = new RB_COLORSENSOR(RX_BUF[ack][5],TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_1X);
                           COLORSENSOR->begin();
                            }
                     }
                     else if(COLORSENSOR->GetPort() != RX_BUF[ack][5]){
                            if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Color_Sensor){
                            delete COLORSENSOR;
                            COLORSENSOR = new RB_COLORSENSOR(RX_BUF[ack][5],TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_1X);
                            COLORSENSOR->begin();
                            }
                     }
                     if( RX_BUF[ack][6]==0x01)
                      {
                      COLORSENSOR->getRawData(&r,&g,&b,&c);
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = (unsigned char)(r/256);
                      TX_BUF[TX_Count++] = (unsigned char)(r%256);
                      TX_BUF[TX_Count++] = (unsigned char)(g/256);
                      TX_BUF[TX_Count++] = (unsigned char)(g%256);
                      TX_BUF[TX_Count++] = (unsigned char)(b/256);
                      TX_BUF[TX_Count++] = (unsigned char)(b%256);
                      TX_BUF[TX_Count++] = (unsigned char)(c/256);
                      TX_BUF[TX_Count++] = (unsigned char)(c%256);  
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      }
                      else if( RX_BUF[ack][6]==0x02)
                      {
                      b = COLORSENSOR->getRGBWvalue();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = 0x02;
                      TX_BUF[TX_Count++] = (unsigned char)(b/256);
                      TX_BUF[TX_Count++] = (unsigned char)(b%256);
                      TX_BUF[2] = TX_Count+1;
                       for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);   
                      } 
                      break;  
           case  GyroSensor_Read:
                      if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Gyro_Sensor)
                          {
                            break;
                          }  
               
                      if(GYRO == NULL){
                            if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor)
                             { 
                           GYRO = new RB_GYRO(RX_BUF[ack][5]);
                           GYRO->begin();
                             }
                     }
                     else if(GYRO->GetPort() != RX_BUF[ack][5]){
                           if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor)
                           {
                          delete GYRO;
                          GYRO = new RB_GYRO(RX_BUF[ack][5]);
                          GYRO->begin();
                           }
                     }
                      GYRO->Update();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = RX_BUF[ack][6];
                      switch(RX_BUF[ack][6])
                      {
                        case 0x01:int32_t  ax,ay,az;
                                  ax = int32_t(GYRO->getAngleX()*100);
                                  ay = int32_t(GYRO->getAngleY()*100);
                                  az = int32_t(GYRO->getAngleZ()*100);
                                  if(ax<=0)
                                    {  
                                       ax = abs(ax);
                                       TX_BUF[TX_Count++] = 0;
                                       TX_BUF[TX_Count++] = (unsigned char)(ax/256);
                                       TX_BUF[TX_Count++] = (unsigned char)(ax%256);
                                     }
                                   else 
                                     {   ax = abs(ax);
                                         TX_BUF[TX_Count++] = 1;
                                         TX_BUF[TX_Count++] = (unsigned char)(ax/256);
                                         TX_BUF[TX_Count++] = (unsigned char)(ax%256);
                                     }  
                                  if(ay<=0)
                                    {  
                                       ay = abs(ay);
                                       TX_BUF[TX_Count++] = 0;
                                       TX_BUF[TX_Count++] = (unsigned char)(ay/256);
                                       TX_BUF[TX_Count++] = (unsigned char)(ay%256);
                                     }
                                  else {
                                       ay = abs(ay);  
                                       TX_BUF[TX_Count++] = 1;
                                       TX_BUF[TX_Count++] = (unsigned char)(ay/256);
                                       TX_BUF[TX_Count++] = (unsigned char)(ay%256);
                                     }  
                                  if(az>=0)
                                    {  
                                       TX_BUF[TX_Count++] = 1;
                                       TX_BUF[TX_Count++] = (unsigned char)(az/256);
                                       TX_BUF[TX_Count++] = (unsigned char)(az%256);
                                     }
                                  else {
                                       az = abs(az);  
                                       TX_BUF[TX_Count++] = 0;
                                       TX_BUF[TX_Count++] = (unsigned char)(az/256);
                                       TX_BUF[TX_Count++] = (unsigned char)(az%256);
                                    } 
                            
                                  break;
                        default: 
                                  break;            
                      } 
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);   
                      break; 
           case TouchSensor_Read:
                     if(TouchSensor == NULL){
                           TouchSensor = new RB_TOUCHSENSOR(RX_BUF[ack][5]);
               
                     }
                     else if(TouchSensor->GetPort() != RX_BUF[ack][5]){
                          delete TouchSensor;
                          TouchSensor = new RB_TOUCHSENSOR(RX_BUF[ack][5]);  
                     }
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = (unsigned char)(TouchSensor->RB_TOUCHSENSOR_ReadValue()/256);
                      TX_BUF[TX_Count++] = (unsigned char)(TouchSensor->RB_TOUCHSENSOR_ReadValue()%256);  
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);   
                      break;         
           default : 
                   break;
        
       }
        if(1==Set_Mode_Flag )
         {            
                      Set_Mode_Flag = 0; 
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                          {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      if(TX_BUF[3]!=0)RB_Serial.Serial_SendString(TX_BUF,TX_Count);
         }
        
  }

void  SerialCheakEvent(void)
{    
     
    char   check_sum = 0;
    unsigned char   data_len  = 0;
     unsigned char   data_count = 0;
     char   Error_CheckSum = 0;
     bool   Error_Flag     = 0;
     
     if(RX_OK&0X01){
        RX_OK &=~0X01;
        data_len = (uint8_t)(RX_BUF[0][2]) ;
        for(data_count= 0 ; data_count<data_len-1;data_count++)
            check_sum += RX_BUF[0][data_count];
            if(check_sum==RX_BUF[0][data_len-1]){  
                SerialDataAnalysis(0);
                return;
        } 
       else { 
             RB_Serial.Serial_SendByte(0x52);
             RB_Serial.Serial_SendByte(0x42);
             RB_Serial.Serial_SendByte(0x07);
             RB_Serial.Serial_SendByte(RX_BUF[0][3]);
             RB_Serial.Serial_SendByte(RX_BUF[0][4]);
             RB_Serial.Serial_SendByte(0x00);
             Error_CheckSum=RX_BUF[0][3]+RX_BUF[0][4]+0x9A; 
             RB_Serial.Serial_SendByte(Error_CheckSum);
             return;
           }
      }
     if(RX_OK&0x02){
        RX_OK &=~0X02;
        data_len = (uint8_t)RX_BUF[1][2] ;
        for(data_count= 0 ; data_count<data_len-1;data_count++)
          check_sum += RX_BUF[1][data_count];
         if(check_sum==RX_BUF[1][data_len-1])
            {
             SerialDataAnalysis(1);
             return;
            } 
          else {
             RB_Serial.Serial_SendByte(0x52);
             RB_Serial.Serial_SendByte(0x42);
             RB_Serial.Serial_SendByte(0x07);
             RB_Serial.Serial_SendByte(RX_BUF[1][3]);
             RB_Serial.Serial_SendByte(RX_BUF[1][4]);
             RB_Serial.Serial_SendByte(0x00);
             Error_CheckSum=RX_BUF[1][3]+RX_BUF[1][4]+0x9A; 
             RB_Serial.Serial_SendByte(Error_CheckSum);
             return;
           }  
      }
}
/*
 * USART0_irq
*/

 ISR(USART_RX_vect)
 {
    uint8_t com_data=0;
    com_data = UDR0; 
    if(RX_COUNT == 0){
        if(com_data == 0X52){               //First
              RX_BUF[RX_ACT][0] = com_data;
              RX_COUNT = 1;
          }
        else RX_COUNT = 0;
    }
    else if(RX_COUNT == 1){               //Second
         if(com_data == 0X42){
              RX_BUF[RX_ACT][1] = com_data;
              RX_COUNT = 2;
         }
         else RX_COUNT = 0;
    } 
    else {
            RX_BUF[RX_ACT][RX_COUNT] = com_data;
            RX_COUNT++;
            if((RX_COUNT >=(unsigned char) (RX_BUF[RX_ACT][2]))||(RX_COUNT>=200))
              {  
                 
                 RX_COUNT = 0;    
                 if(RX_ACT ){
                     RX_ACT = 0;
                     RX_OK  |=0x02; 
                 }
                 else {
                     RX_ACT = 1;
                     RX_OK  |=0x01; 
                  }
              }
           
    }
 }

 /*
 * 
 */
void SendReset(void)
{   
    uint8_t check_sum =0;
    RB_Serial.Serial_SendByte(0x52);
    RB_Serial.Serial_SendByte(0x42);
    RB_Serial.Serial_SendByte(0x06);
    RB_Serial.Serial_SendByte(0x00);
    RB_Serial.Serial_SendByte(0x03);
    check_sum+=0x9D;
    RB_Serial.Serial_SendByte(check_sum);
}

/*
 *   低电压报警信号主动上报
 */
void SendLowPower(void)
{
    
    RB_Serial.Serial_SendByte(0x52);
    RB_Serial.Serial_SendByte(0x42);
    RB_Serial.Serial_SendByte(0x08);
    RB_Serial.Serial_SendByte(0x00);
    RB_Serial.Serial_SendByte(0x02);
    RB_Serial.Serial_SendByte(0xFD);
    RB_Serial.Serial_SendByte(0X01);
    RB_Serial.Serial_SendByte(0x9c);
}
/*
 *   按钮主动上报 
 */
void SendButton(char port,uint8_t button_information)
{   
    uint8_t check_sum =0;
    RB_Serial.Serial_SendByte(0x52);
    RB_Serial.Serial_SendByte(0x42);
    RB_Serial.Serial_SendByte(0x08);
    RB_Serial.Serial_SendByte(0x00);
    RB_Serial.Serial_SendByte(0x01);
    RB_Serial.Serial_SendByte(port);
    RB_Serial.Serial_SendByte(button_information);
    check_sum+=0x9D+port+button_information;
    RB_Serial.Serial_SendByte(check_sum); 
     RB_Serial.Serial_SendByte(check_sum); 
    RB_Serial.Serial_SendByte(check_sum); 


}

void LineFolloweMode(void)
{

      if(LineFollower == NULL)
     {
         LineFollower = new RB_LineFollower(Work_Mode_Port);
     }
     else if(LineFollower->GetPort() != Work_Mode_Port)
     {
           delete LineFollower;
           LineFollower = new RB_LineFollower(Work_Mode_Port);  
     }
    uint8_t  val =   LineFollower->ReadSensors();
     
    switch(val)
       {
         case S1_IN_S2_IN:    //全黑
              RB_DcmotorOnBoard.run(42,42);
              LineFollowFlag = 10;
              RGBled.setColor(0,0,200,0);
              RGBled.show();
              white_count = 0;
              break;
         case S1_IN_S2_OUT:   //黑白
              RB_DcmotorOnBoard.run(42,42);
              if(LineFollowFlag>1)LineFollowFlag--;
              RGBled.setColor(1,200,0,0);
              RGBled.show();
               white_count = 0;
              break;
         case S1_OUT_S2_IN:      //白黑  
              RB_DcmotorOnBoard.run(42,42);
              if(LineFollowFlag<20)LineFollowFlag++; 
              RGBled.setColor(2,200,0,0);
              RGBled.show();
              white_count = 0;
              break;
         case S1_OUT_S2_OUT:    // 全白
              if(LineFollowFlag==10)
                  {RB_DcmotorOnBoard.run(-50,-50); white_count +=1;}
              if(LineFollowFlag<10)
                  {RB_DcmotorOnBoard.run(-90,45);white_count += 1;}
              if(LineFollowFlag>10)
                  { RB_DcmotorOnBoard.run(45,-90); white_count += 1;}
              if(white_count>=50000){white_count=50000;
      RB_DcmotorOnBoard.run(0,0);
      RGBled.setColor(0,0,0,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,80,0,0);
      RGBled.show();
      delay(500);
      
      }
              break;  
         default:
              break; 
       }
}

void UltrCarProcess(void)
{    
    static   unsigned char flag  = 0;
    static   unsigned char last_flag = 0;
    uint8_t randnum;
     if(Ultrasonic == NULL) {
         Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);
     }
     else if(Ultrasonic->GetPort() != Work_Mode_Port){
           delete Ultrasonic;
           Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);  
     }
     double Distance = Ultrasonic->Uldistance();
     if(Distance>50)  { 
          flag = 0;
          last_flag = 0;
     }
     else if((Distance>=23)&&(Distance<=50))  { 
          if(flag==0) 
              {flag =1;}
          else 
              {last_flag  =1 ;flag = 1;}
          }    
    else 
         {       
            if(flag ==0)
               {flag =1;}
            else if(flag ==1)
               {
                  flag =2;
               }
          else {
              last_flag = 2;
              flag = 2;
              }
         }
       switch(last_flag) 
           {
             case 0:
                   RB_DcmotorOnBoard.run(55,55);
                   RGBled.setColor(0,0,0,50);
                   RGBled.show();
                   break;
             case 1:
                  RGBled.setColor(0,50,0,0);
                  RGBled.show();
                  randnum = random(200);
                  if(randnum>100){
                       RB_DcmotorOnBoard.run(-98,98);
                  }
                 else {
                  RB_DcmotorOnBoard.run(98,-98);
              }
               delay(300);
               wdt_reset();
   
                   break;
             case 2: 
                   RGBled.setColor(0,0,50,0);
                  RGBled.show();
                   RB_DcmotorOnBoard.run(-90,-90);
                   delay(400);
                   wdt_reset();
                   randnum = random(200);
                   if(randnum>100){
                       RB_DcmotorOnBoard.run(-98,98);
                   }
                  else {
                       RB_DcmotorOnBoard.run(98,-98);
                   }
                   delay(650);
                   wdt_reset();
                   break; 
             
           }       
}

/*
 *  按键查询
 */
void Button_Process(void)
{
       if(digitalRead(USER_KEY_Pin) == HIGH)
               Hardware_Button_flag = 1;
       if((digitalRead(USER_KEY_Pin) == LOW) && (Hardware_Button_flag == 1)){
               Button_Down = true;  Hardware_Button_flag = 0;
       }
       if(Button_Down==true){Button_Down = false;
          if(Button_flag==true){
            SendButton(-7,1);
          }   
       }
}

/*
 *  power adc
 */
void Power_Process(void)
{
          Power_ADC_count++;
          if(Power_ADC_count>=1000)
              {
                  Power_ADC_count = 0;
                  uint16_t  V1 = analogRead(A7);
                  float power = V1*(15.0/1024.0);
                  if((power>2.0)&&(power<6.8))
                    {
                      SendLowPower();
                      Power_Send_count++;
                   }        
              }
          if(Power_Send_count>=10){
               Power_Send_count= 0;
               Low_Porwer_flag = 0;
          }
}


void SensorType_Update(void)
{
   static unsigned char lastSensor[4]={0,0,0,0};
   static unsigned char nowSensor[4] ={0,0,0,0}; 
   static long          last_time    = 0;
   uint8_t              Sensor_count = 0;
   bool                 SensorTypeSendFlag =false;
   char                 TX_CheckSum;
   uint8_t              i = 0;
   if(( millis()-last_time)>800)
      {
        for(i=0;i<4;i++)
           {
            nowSensor[i] = RB_ADC.RB_ADC_Read(i+1);
            if(nowSensor[i]!=lastSensor[i])
            {
              SensorTypeSendFlag =true;
              
            switch(lastSensor[i])
                 { 
                case  Ultrasonic_Distance_Sensor:
                      if(Ultrasonic!=NULL)
                      {
                        delete Ultrasonic;
                        Ultrasonic = NULL;
                      } 
                case   LED_Matraix_Blue:
                      if(LEDMatrix!=NULL)
                      {
                        delete LEDMatrix;
                        LEDMatrix = NULL;
                      }
               case  Line_Follower_Sensor:
                     if(LineFollower!=NULL)
                     {
                       delete LineFollower;
                        LineFollower = NULL;
                     }
                    break;
               case Temp_And_Humi_Sensor:
                    if(TempAndHumi!=NULL)
                     {
                       delete TempAndHumi;
                       TempAndHumi = NULL;
                     }
                    break;
               case Light_Sensor:
                    if(LightSensor!=NULL) 
                    {
                      delete LightSensor;
                       LightSensor = NULL;
                    }
                    break;
               case Sound_Sensor:
                   if(SoundSensor!=NULL)
                    {
                       delete SoundSensor;
                       SoundSensor = NULL;
                    }
                    break;
               case DC_Motor_Device:
                      if(DCmotor!=NULL)
                      {
                        delete DCmotor;
                        DCmotor = NULL;
                       }
                      break;
                case Servo_Device:
                      if(Servo1!=NULL)
                      {
                        delete Servo1;
                        Servo1 = NULL;
                       }
                      break;
              case PIR_Sensor:
                      if(PirSensor!=NULL)
                      {
                        delete PirSensor;
                        PirSensor = NULL;
                      }
             case MP3_Sensor:
                     if(MP3!= NULL){
                          MP3->RB_MP3_END();
                          delete MP3;
                          MP3 = NULL;
                      }
                     break;
			 case RGBLED_Matraix:
                      if(RGBLEDMatrix!=NULL)
                      { 
                        delete RGBLEDMatrix;
                        RGBLEDMatrix = NULL;
                      }
                      break;
            case Color_Sensor: 
                      if(COLORSENSOR!=NULL)
                      { 
                        delete COLORSENSOR; 
                        COLORSENSOR = NULL;
                      }
                      break;
            case Gyro_Sensor:
                      if(GYRO!=NULL)
                      {
                        delete GYRO;
                        GYRO = NULL;
                      }
                      break;
              default:
                      break;
                 }
             
              if(nowSensor[i]==0)
                 {
                  pinMode(RBPort[i+1].clk,INPUT);
                  pinMode(RBPort[i+1].dat,INPUT);
                 }
                   
            }
            lastSensor[i] =  nowSensor[i];
           }
         if(SensorTypeSendFlag==true)
            {
              SensorTypeSendFlag = false;
              TX_BUF[Sensor_count++] = 0x52;
              TX_BUF[Sensor_count++] = 0X42;
              TX_BUF[Sensor_count++] = 0X00;
              TX_BUF[Sensor_count++] = 0x00;
              TX_BUF[Sensor_count++] = 0x05;
              TX_BUF[Sensor_count++] = lastSensor[0];
              TX_BUF[Sensor_count++] = lastSensor[1];
              TX_BUF[Sensor_count++] = lastSensor[2];
              TX_BUF[Sensor_count++] = lastSensor[3];
              TX_BUF[Sensor_count++] = 0x00;
              TX_BUF[Sensor_count++] = 0x00;
              TX_BUF[Sensor_count++] = 0x00;
              TX_BUF[Sensor_count++] = 0x00;
              TX_BUF[2] = Sensor_count+1;
              for(i=0;i<Sensor_count;i++)
                   {TX_CheckSum += TX_BUF[i];}
              TX_BUF[Sensor_count++] = TX_CheckSum;
              RB_Serial.Serial_SendString(TX_BUF,Sensor_count); 
            }
            last_time = millis();
      }  
}

