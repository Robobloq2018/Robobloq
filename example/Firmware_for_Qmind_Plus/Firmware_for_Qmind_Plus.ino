 /*******************************************
 *             Update Data :   18/01/2018          
 *             File   Name :   Firmware for Qmind Plus 
 *             Author      :   Allen.Feng 
 *             Updated     :   Allen.Feng
 *             http://www.robobloq.com/
 *             History:
 *             <Version>       <Time>        <Author>                    <Desrc>
 *             1.0.1           2017/09/14     Allen.Feng                 Build The New
 *             1.1.1           2017/10/10     Allen.Feng                 出厂固件
 *             1.2.1           2017/11/10     Allen.Feng                 更改硬件电路，修复BUG
 *             1.2.2           2017/11/30     Allen.Feng                 修复超声波避障算法
 *             1.2.3           2017/12/20     Allen.Feng                 新增加巡线传感器
 *             1.2.3           2017/12/25     Allen.Feng                 新增加舵机
 *             1.2.3           2017/12/30     Allen.Feng                 新增加温湿度传感器
 *             1.2.3           2018/01/15     Allen.Feng                 新增加电机驱动
 *             1.2.4           2018/01/18     Allen.Feng                 新增加光线、声音传感器
 *             1.2.4           2018/01/23     Allen.Feng                 修复电机驱动代码
 *             1.2.4           2018/03/12     Allen.Feng                 新增加人体红外传感器
 *             1.2.4           2018/03/14     Allen.Feng                 修复双路电机驱动代码
  *            1.2.5           2018/03/14     Allen.Feng                 修复双路电机驱动代码
 *             1.2.6           2018/04/11     Allen.Feng                 新增加MP3
 *             1.2.7           2018/05/07     Allen.Feng                 新增加颜色传感器
 *             1.2.7           2018/05/07     Allen.Feng                 新增加陀螺仪传感器
 *             1.2.8           2018/06/10     Allen.Feng                 修复陀螺仪传感器
 *             1.2.9           2018/06/29     Allen.Feng                 新增触摸传感器
 *             1.2.10          2018/08/10     Allen.Feng                 修复bug
 *             1.2.10          2018/09/12     Allen.Feng                 增加陀螺仪端口,支持两个同时工作,删除触摸传感器主动上报
 *             1.2.11          2019/07/15     Allen.Feng                 增加2路温度传感器、4段数码管、4个RGB-LED传感器固件
  ********************************************************/


#include "RB_QMIND_PLUS.h"



  uint8_t     Work_Mode      = 0;
  char        Work_Mode_Port = 0 ;
  char        Car_Style      = 0;

/*
 *   
 */
RB_RGBLed RGBled(37,2);
RB_Buzzer Buzzer(36);
RB_Serial RB_Serial;
RB_ADC           RB_ADC;
RB_EncoderMotor  RB_EncoderMotor_M1(1);
RB_EncoderMotor  RB_EncoderMotor_M2(2);
RB_Ultrasonic    *Ultrasonic   = NULL ;
RB_LEDMatrix     *LEDMatrix    = NULL ;
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
RB_GYRO          *GYRO1  = NULL;
RB_GYRO          *GYRO2  = NULL;
RB_TOUCHSENSOR   *TouchSensor  = NULL;


RB_RGBLed         *LED            = NULL;        
RB_DIGITALDISPLAY *DIGITALDISPLAY = NULL;
RB_TEMPERATURE    *TEMPERATURE    = NULL;



RB_JOYSTICK       *Joystick     = NULL;
RB_FLAMESENSOR    *FlameSensor  = NULL;
RB_GASSENSOR      *GasSensor    = NULL;
RB_POTENTIOMETER  *Pot          = NULL;



uint8_t  LineFollowFlag   = 0;
unsigned short RGB_LED_test[72];
unsigned short RGB_LED_test2[72];


/*
 * 
 */
char TX_Count =0 ;
bool Button_Down = 0;
bool Button_flag   = 0;
bool Hardware_Button_flag =0;

bool        Low_Porwer_flag = 0;

uint8_t     Power_Send_count =0;
uint32_t    Power_ADC_count = 0;

unsigned    char TouthSensor_Send_flag = 0;

unsigned short value ;
bool        M1_flag = false;
bool        M2_flag = false;
char        moveSpeed =   70;
char        Left_Speed =  70;
char        Right_Speed = 70;
bool        leftflag=false; 
bool        rightflag=false; 
int16_t     randnum = 0;
uint8_t     Button_text_flag = 0; 

int  address = 10;    //eeprom 地址
int  write_addr_data  = 10;


void _setup(void)
{     
      RB_Serial.Serial_begin(115200);
      uint16_t  V1 = analogRead(LowPowr_Pin);
      float power = V1*(15.0/1024.0);
      pinMode(MOTOR_Charge_Pin,OUTPUT);
      if((power>10.2)) {
           digitalWrite(MOTOR_Charge_Pin,LOW);  
      }
      else {
           digitalWrite(MOTOR_Charge_Pin,HIGH); 
      }  
      write_addr_data = EEPROM.read(address);
      if(write_addr_data==10)
         {
            RGBled.setpin(37);
            Buzzer.setPin(36);
         }
      else 
         {
            RGBled.setpin(3);
            Buzzer.setPin(44);
            pinMode(MOTOR_Sleep_Pin,OUTPUT);
            digitalWrite(MOTOR_Sleep_Pin,HIGH); 
         } 
      RGBled.setColor(0,200,0,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,0,200,0);
      RGBled.show();
      delay(500);
      RGBled.setColor(0,0,0,200);
      RGBled.show(); 
      delay(500);  
      Buzzer.tone(500,500);
      delay(500);
}

void setup() {
      
      _setup();
      pinMode(USER_KEY_Pin,INPUT);     
      RGBled.setColor(0,0,0,200);
      RGBled.show();
      RB_EncoderMotor_M1.SetMotionMode(DIRECT_MODE);
      RB_EncoderMotor_M2.SetMotionMode(DIRECT_MODE);
      attachInterrupt(RB_EncoderMotor_M1.GetIntterrruptNum(), isr_process_encoder1, RISING);
      attachInterrupt(RB_EncoderMotor_M2.GetIntterrruptNum(), isr_process_encoder2, RISING);
      wdt_enable(WDTO_2S);
      wdt_reset();
      SendReset();
      
     
}

void loop() {
       

       wdt_reset();
       Power_Check();
       SerialCheak_Process();
       RB_EncoderMotor_M1.Loop();
       RB_EncoderMotor_M2.Loop();
       Button_Process();
       if(Low_Porwer_flag){
          Power_Process();
       }
       switch(Work_Mode)
          {
             case Remote_Control_Mode:
                    SensorType_Update(); 
                    attachInterrupt(RB_EncoderMotor_M1.GetIntterrruptNum(), isr_process_encoder1, RISING);
                    attachInterrupt(RB_EncoderMotor_M2.GetIntterrruptNum(), isr_process_encoder2, RISING);
                    break;
             case   Ultrasonic_Mode:
                    detachInterrupt(RB_EncoderMotor_M1.GetIntterrruptNum());
                    detachInterrupt(RB_EncoderMotor_M2.GetIntterrruptNum());
                    RB_EncoderMotor_M1.SetMotionMode(DIRECT_MODE);
                    RB_EncoderMotor_M2.SetMotionMode(DIRECT_MODE);
                    UltrCarProcess(Car_Style);
                    break;
             case Line_Follower_Mode:
                    detachInterrupt(RB_EncoderMotor_M1.GetIntterrruptNum());
                    detachInterrupt(RB_EncoderMotor_M2.GetIntterrruptNum());
                    RB_EncoderMotor_M1.SetMotionMode(DIRECT_MODE);
                    RB_EncoderMotor_M2.SetMotionMode(DIRECT_MODE);
                    LineFolloweProcess(); 
                    break;
            
             default:
                    break;
          }
      
}
void Set_Action(bool ack)
{   
    
    char TX_CheckSum =0;
    bool Set_Mode_Flag = 0;
    uint8_t  i =0; 
    TX_Count = 0;
    switch((uint8_t)RX_BUF[ack][4])
       {
        case Hardware://获取版本信息
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     TX_BUF[TX_Count++] = DeviceType;
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
                        case -2: M2_flag = true;
                                 TX_BUF[TX_Count++] = 0;
                                 if(RB_ADC.RB_ADC_Read(0xfe)==0){  
                                      RB_EncoderMotor_M2.SetMotorPwm(99);
                                      delay(100);       
                                      RB_EncoderMotor_M2.SetMotorPwm(0);
                                } 
                                else { TX_BUF[TX_Count++] =1; }
                                 M2_flag = false;
                                break;
                        case -1:   M1_flag = true;
                                   TX_BUF[TX_Count++] = 0;
                                   if(RB_ADC.RB_ADC_Read(0xff)==0)
                                       {
                                         RB_EncoderMotor_M1.SetMotorPwm(99);
                                         delay(100);
                                         RB_EncoderMotor_M1.SetMotorPwm(0);
                                        } 
                                   else { TX_BUF[TX_Count++] =1;}
                                   M1_flag = false;
                                  break;
                        case 1:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(1);
                                  break;
                        case 2:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(2);
                                  break;
                        case 3:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(3);
                                  break;
                        case 4:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(4);
                                  break;
                        case 5:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(5);
                                  break;
                        case 6:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(6);
                                  break;
                        case 7:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(7);
                                  break;
                        case 8:TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(8);
                                  break;  
                    }
                     TX_BUF[2] = TX_Count+1;
                     for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                     TX_BUF[TX_Count++] = TX_CheckSum;
                     RB_Serial.Serial_SendString(TX_BUF,TX_Count);
                     break;
        case Device_All:
                    
                     TX_BUF[TX_Count++] = 0x52;
                     TX_BUF[TX_Count++] = 0X42;
                     TX_BUF[TX_Count++] = 0X00;
                     TX_BUF[TX_Count++] = RX_BUF[ack][3];
                     M2_flag = true;
                     TX_BUF[TX_Count++] = 0;
                     if(RB_ADC.RB_ADC_Read(0xfe)==0){  
                        RB_EncoderMotor_M2.SetMotorPwm(99);
                        delay(100);       
                        RB_EncoderMotor_M2.SetMotorPwm(0);
                     } 
                     else { TX_BUF[TX_Count++] =1; }
                     M2_flag = false;
                     M1_flag = true;
                     TX_BUF[TX_Count++] = 0;
                     if(RB_ADC.RB_ADC_Read(0xff)==0){
                        RB_EncoderMotor_M1.SetMotorPwm(99);
                        delay(100);
                        RB_EncoderMotor_M1.SetMotorPwm(0);
                     } 
                     else { TX_BUF[TX_Count++] =1;}
                      M1_flag = false;
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(1);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(2);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(3);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(4);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(5);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(6);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(7);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(8);
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
                     M2_flag = true;
                     TX_BUF[TX_Count++] = 0;
                     if(RB_ADC.RB_ADC_Read(0xfe)==0){  
                        RB_EncoderMotor_M2.SetMotorPwm(99);
                        delay(100);       
                        RB_EncoderMotor_M2.SetMotorPwm(0);
                     } 
                     else { TX_BUF[TX_Count++] =1; }
                     M2_flag = false;
                     M1_flag = true;
                     TX_BUF[TX_Count++] = 0;
                     if(RB_ADC.RB_ADC_Read(0xff)==0){
                        RB_EncoderMotor_M1.SetMotorPwm(99);
                        delay(100);
                        RB_EncoderMotor_M1.SetMotorPwm(0);
                     } 
                     else { TX_BUF[TX_Count++] =1;}
                      M1_flag = false;
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
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(5);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(6);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(7);
                     TX_BUF[TX_Count++] = RB_ADC.RB_ADC_Read(8);
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
                         case 0: RB_EncoderMotor_M1.SetTarPWM(RX_BUF[ack][6]);
                                 RB_EncoderMotor_M2.SetTarPWM(RX_BUF[ack][7]);
                                 if((RX_BUF[ack][7]==0)&&(RX_BUF[ack][6]==0))
                                     {
                                      Work_Mode_Port = 0;
                                      Work_Mode      = 0;
                                     }
                                 break;
                         case -1:RB_EncoderMotor_M1.SetTarPWM(RX_BUF[ack][6]);
                                 break;
                         case -2:RB_EncoderMotor_M2.SetTarPWM(RX_BUF[ack][6]);
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
                      Set_Mode_Flag =1;
                      break;
                      
        case Mode_Change:
                      Work_Mode_Port = RX_BUF[ack][5]; 
                      Work_Mode = RX_BUF[ack][6];
                      Car_Style  = RX_BUF[ack][7];
                      switch(RX_BUF[ack][7])
                          {
                            case 0x00:
                                   Left_Speed = 70;
                                   Right_Speed = 70;
                                   break;
                            case 0x01:
                                   Left_Speed = 60;
                                   Right_Speed = 60;
                                   Car_Style = 1;  
                                   break;
                            case 0x02:
                                   Left_Speed = 70;
                                   Right_Speed = 70;
                                   Car_Style =1; 
                                   break;
                            case 0x03:
                                   Left_Speed = 70;
                                   Right_Speed = 70;
                                   break;
                            case 0x04:
                                   Left_Speed = 65;
                                   Right_Speed = 65; 
                                   break;
                            case 0x05:
                                   Left_Speed = 45;
                                   Right_Speed = 45; 
                                   break;
                            case 0x06:
                                   Left_Speed = 55;
                                   Right_Speed = 55;
                                   break;
                            case 0x07:
                                   Left_Speed = 45;
                                   Right_Speed = 45;
                                   Car_Style =2; 
                                   break;
                             case 0x08:
                                   Left_Speed = 55;
                                   Right_Speed = 55;
                                   Car_Style =2; 
                                   break;
                            default:
                                   break;
                          }
                      break;
         case MoterTurs_Set:
                       switch(RX_BUF[ack][5])
                       {
                         case 0:   
                                   break;
                         case -1:  
                                   break;
                         case -2:  
                                   break;
                         default:
                                   break;
                      }
                      Set_Mode_Flag =1;
                      break;
        case Servo_Set:
                      if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=Servo_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                     if(RX_BUF[ack][5]==1||RX_BUF[ack][5]==8)
                        {
                              if(Servo1 == NULL){
                                    Servo1 = new RB_SERVO(RX_BUF[ack][5]);
                                    Servo1->RB_SERVO_INIT();
                                    
                              }
                              else if(Servo1->GetPort() != RX_BUF[ack][5]) {
                                        delete Servo1;
                                        Servo1 = new RB_SERVO(RX_BUF[ack][5]);
                                        Servo1->RB_SERVO_INIT();
                              }
                              
                              switch(RX_BUF[ack][6]){
                                        case 0: Servo1->RB_SERVO_Write((uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8]);
                                                break;
                                        case 1: Servo1->RB_SERVO_Write_Servo1((uint8_t)RX_BUF[ack][7]);
                                                 break;
                                        case 2:Servo1->RB_SERVO_Write_Servo2((uint8_t)RX_BUF[ack][7]);
                                                break;
                              }
                        }        
                      Set_Mode_Flag = 1;
                      break;
          case ExDCMotor_Set:
                        if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=DC_Motor_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                      if(RX_BUF[ack][5]==1||RX_BUF[ack][5]==8)
                        {  
                           if(Servo1->GetPort()== RX_BUF[ack][5])
                            {
                              Servo1->RB_SERVO_Detach();
                              delete Servo1;
                              Servo1 = NULL;
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
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X00,abs(RX_BUF[ack][7])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X01,abs(RX_BUF[ack][7])); }
                                         if(RX_BUF[ack][8]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X00,abs(RX_BUF[ack][8])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X01,abs(RX_BUF[ack][8])); }   
                                        break;
                                  case 1:if(RX_BUF[ack][7]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X00,abs(RX_BUF[ack][7])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x01,0X01,abs(RX_BUF[ack][7])); }
                                        break;
                                  case 2:if(RX_BUF[ack][8]<0)
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X00,abs(RX_BUF[ack][8])); }
                                         else 
                                            { DCmotor->RB_DCMotor_SetSpeed(0x50,0xA3,0x02,0X01,abs(RX_BUF[ack][8])); }   
                                        break;
                                }
                          
                      }
                      Set_Mode_Flag = 1;
                      break;
          case RGBLEDMatrix_Set:
                        volatile uint8_t d1[16],d2[16],d3[16],d4[16];
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
                          }
                         }
                     else if(RGBLEDMatrix->GetPort() != RX_BUF[ack][5]){
                           if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==RGBLED_Matraix){
                              delete RGBLEDMatrix;
                              RGBLEDMatrix = new RB_RGBLEDMATRIX(RX_BUF[ack][5]);
                              RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X77);
                              RGBLEDMatrix->RB_RGBLEDMATRIX_Init(0X74);
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
        case RGBLEDArray_Set:
                         if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=RGBLED_Array_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                         if(LED==NULL)
                         {
                           LED = new RB_RGBLed(RX_BUF[ack][5]);
                         }
                         else if(LED->GetPort() != RX_BUF[ack][5]){
                          delete LED;
                          LED = new RB_RGBLed(RX_BUF[ack][5]);
                         }
                        LED->setColor(RX_BUF[ack][6],(uint8_t)RX_BUF[ack][7],(uint8_t)RX_BUF[ack][8],(uint8_t)RX_BUF[ack][9]);
                        LED->show(); 
                        break;
        case DigitalDisplay_Set:
                         if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])!=DigitalDisplay_Device)
                          {
                            Set_Mode_Flag =1;
                            break;
                          }
                         if(DIGITALDISPLAY==NULL)
                         {
                           DIGITALDISPLAY = new RB_DIGITALDISPLAY(RX_BUF[ack][5]);
                           DIGITALDISPLAY->RB_DIGITALDISPLAY_Init();
                         }
                         else if(DIGITALDISPLAY->GetPort() != RX_BUF[ack][5]){
                          delete DIGITALDISPLAY;
                          DIGITALDISPLAY = new RB_DIGITALDISPLAY(RX_BUF[ack][5]);
                          DIGITALDISPLAY->RB_DIGITALDISPLAY_Init();
                         }
                        DIGITALDISPLAY->RB_DIGITALDISPLAY_SET(1,(uint8_t)RX_BUF[ack][6]);
                        DIGITALDISPLAY->RB_DIGITALDISPLAY_SET(2,(uint8_t)RX_BUF[ack][7]);
                        DIGITALDISPLAY->RB_DIGITALDISPLAY_SET(3,(uint8_t)RX_BUF[ack][8]);
                        DIGITALDISPLAY->RB_DIGITALDISPLAY_SET(4,(uint8_t)RX_BUF[ack][9]);
                        break;
      
        default : 
                     break;
        
       }
      if(1==Set_Mode_Flag )
         {            Set_Mode_Flag = 0; 
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




/*
 *    读取传感器指令
 *    :param  ack :   数组号 0/1
 *    :return     :   None
 */
void Read_Action(bool ack)
{   
    
    char TX_CheckSum =0;
    bool Set_Mode_Flag = 0;
    uint8_t  i =0; 
    double distance ; 
    int    distance_1 ; 
    int V2 = 0;
     TX_Count = 0;

    int16_t   joy_x =0;
    int16_t   joy_y =0;
    float temp_value = 0;
    switch((uint8_t)RX_BUF[ack][4])
       {
       
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
                           COLORSENSOR = new RB_COLORSENSOR(RX_BUF[ack][5],TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_4X);
                           COLORSENSOR->begin();
                            }
                     }
                     else if(COLORSENSOR->GetPort() != RX_BUF[ack][5]){
                            if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Color_Sensor){
                            delete COLORSENSOR;
                            COLORSENSOR = new RB_COLORSENSOR(RX_BUF[ack][5],TCS34725_INTEGRATIONTIME_50MS,TCS34725_GAIN_4X);
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
                      if(RX_BUF[ack][5]<=4) {  
                             if(GYRO1 == NULL){
                                 if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor){ 
                                      GYRO1 = new RB_GYRO(RX_BUF[ack][5]);
                                      GYRO1->begin();
                                 }
                            }
                           else if(GYRO1->GetPort() != RX_BUF[ack][5]){
                                 if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor) {
                                    delete GYRO1;
                                    GYRO1 = new RB_GYRO(RX_BUF[ack][5]);
                                    GYRO1->begin();
                                    
                                 }
                           }
                            GYRO1->Update();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = RX_BUF[ack][6];
                      switch(RX_BUF[ack][6])
                      {
                        case 0x01:int32_t  ax,ay,az;
                                  ax = int32_t(GYRO1->getAngleX()*100);
                                  ay = int32_t(GYRO1->getAngleY()*100);
                                  az = int32_t(GYRO1->getAngleZ()*100);
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
                      }
                      else 
                      {
                         if(GYRO2 == NULL){
                                 if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor){ 
                                      GYRO2 = new RB_GYRO(RX_BUF[ack][5]);
                                      GYRO2->begin();
                                 }
                            }
                           else if(GYRO2->GetPort() != RX_BUF[ack][5]){
                                 if(RB_ADC.RB_ADC_Read(RX_BUF[ack][5])==Gyro_Sensor) {
                                    delete GYRO2;
                                    GYRO2 = new RB_GYRO(RX_BUF[ack][5]);
                                    GYRO2->begin();
                                 }
                           }
                            GYRO2->Update();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = RX_BUF[ack][6];
                      switch(RX_BUF[ack][6])
                      {
                        case 0x01:int32_t  ax,ay,az;
                                  ax = int32_t(GYRO2->getAngleX()*100);
                                  ay = int32_t(GYRO2->getAngleY()*100);
                                  az = int32_t(GYRO2->getAngleZ()*100);
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
                        
                        
                      }  
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
                  
            case TemptureSensors_Read:
                     
                      if(TEMPERATURE == NULL){
                           TEMPERATURE = new RB_TEMPERATURE(RX_BUF[ack][5]);
                      }
                      else if(TEMPERATURE->GetPort() != RX_BUF[ack][5]){
                          delete TEMPERATURE;
                          TEMPERATURE = new RB_TEMPERATURE(RX_BUF[ack][5]);  
                      }
                      temp_value = TEMPERATURE->GET_TTEMPERATURE((uint8_t)(RX_BUF[ack][5]));
                      temp_value = temp_value*100;
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      if(temp_value>=0) 
                          TX_BUF[TX_Count++]=1;
                      else 
                          TX_BUF[TX_Count++]= 0;
                      TX_BUF[TX_Count++] = (short)(abs(temp_value))/256;
                      TX_BUF[TX_Count++] = (short)(abs(temp_value))%256;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break; 
                 
        
                 
           case JoyStickSensor_Read:

                      if(Joystick == NULL){
                           Joystick = new RB_JOYSTICK(RX_BUF[ack][5]);  
                      }
                      else if(Joystick->GetPort() != RX_BUF[ack][5]){
                          delete Joystick;
                          Joystick = new RB_JOYSTICK(RX_BUF[ack][5]);  
                      }
                      joy_x =  Joystick->ReadJoystickX();
                      joy_y =  Joystick->ReadJoystickY();
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      if(joy_x>=0) TX_BUF[TX_Count++]=1;
                      else         TX_BUF[TX_Count++]= 2;
                     
                      TX_BUF[TX_Count++] = (short)(abs(joy_x))/256;
                      TX_BUF[TX_Count++] = (short)(abs(joy_x))%256;
                      if(joy_y>=0) TX_BUF[TX_Count++]=1;
                      TX_BUF[TX_Count++] = (short)(abs(joy_y))/256;
                      TX_BUF[TX_Count++] = (short)(abs(joy_y))%256;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break;  
              
               case FlameSensor_Read:
                      if(FlameSensor == NULL){
                           FlameSensor = new RB_FLAMESENSOR(RX_BUF[ack][5]);  
                      }
                      else if(FlameSensor->GetPort() != RX_BUF[ack][5]){
                          delete FlameSensor;
                          FlameSensor = new RB_FLAMESENSOR(RX_BUF[ack][5]);  
                      }
                      
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = FlameSensor->GetFlameAnalog()/256;
                      TX_BUF[TX_Count++] = FlameSensor->GetFlameAnalog()%256;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break;  
              case GasSensor_Read:
                      if(GasSensor == NULL){
                           GasSensor = new RB_GASSENSOR(RX_BUF[ack][5]);  
                      }
                      else if(GasSensor->GetPort() != RX_BUF[ack][5]){
                          delete GasSensor;
                          GasSensor = new RB_GASSENSOR(RX_BUF[ack][5]);  
                      }
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = GasSensor->GetGas()/256;
                      TX_BUF[TX_Count++] = GasSensor->GetGas()%256;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break;  
            case Potentimeter_Read:
                      if(Pot == NULL){
                           Pot = new RB_POTENTIOMETER(RX_BUF[ack][5]);  
                      }
                      else if(Pot->GetPort() != RX_BUF[ack][5]){
                          delete Pot;
                          Pot = new RB_POTENTIOMETER(RX_BUF[ack][5]);  
                      }
                      
                      TX_BUF[TX_Count++] = 0x52;
                      TX_BUF[TX_Count++] = 0X42;
                      TX_BUF[TX_Count++] = 0X00;
                      TX_BUF[TX_Count++] = RX_BUF[ack][3];
                      TX_BUF[TX_Count++] = 0x01;
                      TX_BUF[TX_Count++] = Pot->GetPotentiometer()/256;
                      TX_BUF[TX_Count++] = Pot->GetPotentiometer()%256;
                      TX_BUF[2] = TX_Count+1;
                      for(i=0;i<TX_Count;i++)
                         {TX_CheckSum += TX_BUF[i];}
                      TX_BUF[TX_Count++] = TX_CheckSum;
                      RB_Serial.Serial_SendString(TX_BUF,TX_Count);    
                      break;    
                        
           default : 
                   break;
        
       }
       
        
  }






/*
 *      串口检测时间
 */
void  SerialCheak_Process(void)
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
                if((uint8_t)RX_BUF[0][4]<=0XA0){
                       Set_Action(0);
                       
                }  
                else {
                       RB_Serial.Serial_SendByte(RX_BUF[0][4]); 
                       Read_Action(0);
                }
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
                if((uint8_t)RX_BUF[1][4]<=0XA0){
                       Set_Action(1);
                       
                }  
                else { 
                       RB_Serial.Serial_SendByte(RX_BUF[1][4]); 
                       Read_Action(1);
                }
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
 *     发送复位信号
 */
void SendReset(void)
{   
    
    RB_Serial.Serial_SendByte(0x52);
    RB_Serial.Serial_SendByte(0x42);
    RB_Serial.Serial_SendByte(0x06);
    RB_Serial.Serial_SendByte(0x00);
    RB_Serial.Serial_SendByte(0x03);
    RB_Serial.Serial_SendByte(0x9d);
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
}
/*
 *    串口接收数据中断
 */
 ISR(USART0_RX_vect)
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
            if(RX_COUNT >=(unsigned char) (RX_BUF[RX_ACT][2])||(RX_COUNT>200))
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
    wdt_reset();
 }
/*
 *   编码电机M1中断
 */
 void isr_process_encoder1(void)
{
    
    if(digitalRead(RB_EncoderMotor_M1.GetPortB()) == 0)
        {
          RB_EncoderMotor_M1.PulsePosPlus();
          
        }
    else {
          RB_EncoderMotor_M1.PulsePosMinus(); 
          
      }
    if(M1_flag == true) {
         RB_EncoderMotor_M1.SetMotorPwm(0);
          TX_BUF[TX_Count-1] = 1;
    }    
}
/*
 *  编码电机M2中断
 */
 void isr_process_encoder2(void)
{  
  
  if(digitalRead(RB_EncoderMotor_M2.GetPortB()) == 0) {
         RB_EncoderMotor_M2.PulsePosMinus(); 
   }
  else {
          RB_EncoderMotor_M2.PulsePosPlus(); 
  }
  if(M2_flag == true){
        RB_EncoderMotor_M2.SetMotorPwm(0);
        TX_BUF[TX_Count-1] = 1;
  }
}
/*
 *   自动壁障模式
 */
void UltrCarProcess_1(void)
{   
   int i =0;
    if(Ultrasonic == NULL){
         Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);
    }
   else if(Ultrasonic->GetPort() != Work_Mode_Port){
         delete Ultrasonic;
         Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);  
   }
   uint32_t Distance = Ultrasonic->Uldistance();
   if((Distance>=18)&&(Distance<45)) 
   {      
          RGBled.setColor(0,200,0,0);
          RGBled.show();
          randnum = random(300);
          if((randnum > 190) && (!rightflag)) 
          { 
               leftflag=true; 
               TurnLeft1(); 
          } 
         else 
          { 
             rightflag=true; 
             TurnRight1();
                
          } 
          for(i= 0;i<200;i++)
             {
              wdt_reset();
              SerialCheak_Process();
              delay(2);
             }
   }
   if((Distance <18)&&(Distance>3))
   {
       RGBled.setColor(0,0,200,0);
       RGBled.show();
       randnum=random(300);
      if((randnum > 190) && (!rightflag)) 
         {
          leftflag=true; 
          BackwardAndTurnLeft();
         }
      else 
         {
          rightflag=true; 
          BackwardAndTurnRight();
         
         }
       for( i= 0;i<300;i++){
              wdt_reset();
              SerialCheak_Process();
              delay(2);
       }
      if((randnum > 190) && (!rightflag)) 
          { 
               leftflag=true; 
               TurnLeft1(); 
          } 
         else 
          { 
             rightflag=true; 
             TurnRight1();
                
          } 
          for(i= 0;i<100;i++)
             {
              wdt_reset();
              SerialCheak_Process();
              delay(2);
             }
   } 
   if(Distance>=45) 
    { 
       leftflag=false; 
       rightflag=false; 
       RGBled.setColor(0,0,0,200);
       RGBled.show();
       Forward(); 
     } 
}

void UltrCarProcess_2(void)
{   
   int i =0;
    if(Ultrasonic == NULL){
         Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);
    }
   else if(Ultrasonic->GetPort() != Work_Mode_Port){
         delete Ultrasonic;
         Ultrasonic = new RB_Ultrasonic(Work_Mode_Port);  
   }
   uint32_t Distance = Ultrasonic->Uldistance();
   if((Distance>=25)&&(Distance<45)) 
   {      
          RGBled.setColor(0,200,0,0);
          RGBled.show();
          randnum = random(300);
          if((randnum > 190) && (!rightflag)) 
          { 
               leftflag=true; 
               TurnLeft1(); 
          } 
         else 
          { 
             rightflag=true; 
             TurnRight1();   
          } 
       
      
   }
   if((Distance <25)&&(Distance>6))
   {
      
       RGBled.setColor(0,0,200,0);
       RGBled.show();
      randnum=random(300);
      if(randnum>190)
         {
          BackwardAndTurnLeft();
         }
      else 
         {
          BackwardAndTurnRight();
         }
       for( i= 0;i<30;i++){
              wdt_reset();
              SerialCheak_Process();
              delay(2);
       }
   } 
   if(Distance>=45) 
    { 
       leftflag=false; 
       rightflag=false; 
       RGBled.setColor(0,0,0,200);
       RGBled.show();
       Forward();
       for( i= 0;i<20;i++){
              wdt_reset();
              SerialCheak_Process();
              delay(2);
       }  
     }
 
 
}


void UltrCarProcess(uint8_t  style_mode)
{
     switch(style_mode)
        {
           case 1:UltrCarProcess_1();     //经典车形态
                  break;
           case 2:UltrCarProcess_2();     //蝎子形态
                  break;
           default:
                  break;
               
          
        } 
  
  
 }

void LineFolloweProcess(void)
{
    uint8_t val;
   
   if(LineFollower == NULL){
         LineFollower = new RB_LineFollower(Work_Mode_Port);
    }
   else if(LineFollower->GetPort() != Work_Mode_Port){
         delete LineFollower;
         LineFollower = new RB_LineFollower(Work_Mode_Port);  
   }
    val =   LineFollower->ReadSensors();
    moveSpeed = 65;
    Left_Speed = 65;
    Right_Speed = 65;
    switch(val)
       {
         case S1_IN_S2_IN:    //全黑
              LineFollowFlag = 10;
              Forward();
              break;
         case S1_IN_S2_OUT:   //黑白
              if(LineFollowFlag>1)LineFollowFlag--;
              Forward();
              break;
         case S1_OUT_S2_IN:      //白黑                
              if(LineFollowFlag<20)LineFollowFlag++;
              Forward();
              break;
         case S1_OUT_S2_OUT:    // 全白
              if(LineFollowFlag==10)Backward();
              if(LineFollowFlag<10)TurnLeft1();
              if(LineFollowFlag>10)TurnRight1();  
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
               Button_Down = 1;  Hardware_Button_flag = 0;
       }
       if(Button_Down){Button_Down = 0;
          if(Button_flag){
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
                  uint16_t  V1 = analogRead(LowPowr_Pin);
                  float power = V1*(15.0/1024.0);
                  if((power>2.0)&&(power<5.7))
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
/*
 *   
 */
void  Forward(void)
{
     RB_EncoderMotor_M1.SetMotorPwm(Left_Speed-10);
     RB_EncoderMotor_M2.SetMotorPwm(Right_Speed-10);
}
void Backward(void)
{ 
     RB_EncoderMotor_M1.SetMotorPwm(-Left_Speed);
     RB_EncoderMotor_M2.SetMotorPwm(-Right_Speed); 
}
void BackwardAndTurnLeft(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(-Left_Speed-30); 
   RB_EncoderMotor_M2.SetMotorPwm(-Right_Speed); 
 } 
void BackwardAndTurnRight(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(-Left_Speed); 
   RB_EncoderMotor_M2.SetMotorPwm(-Right_Speed-30); 
 } 
 void TurnLeft(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(Left_Speed); 
   RB_EncoderMotor_M2.SetMotorPwm(Right_Speed/2); 
 } 
void TurnRight(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(Left_Speed/2); 
   RB_EncoderMotor_M2.SetMotorPwm(Right_Speed); 
 } 
void TurnLeft1(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(-Left_Speed-20); 
   RB_EncoderMotor_M2.SetMotorPwm(Right_Speed+20); 
 } 
void TurnRight1(void) 
 { 
   RB_EncoderMotor_M1.SetMotorPwm(Left_Speed+20); 
   RB_EncoderMotor_M2.SetMotorPwm(-Right_Speed-20); 
 } 
 void Stop(void) 
 { 
     RB_EncoderMotor_M1.SetMotorPwm(0); 
     RB_EncoderMotor_M2.SetMotorPwm(0); 
  } 


 void Power_Check(void)
  {    
         float  V1 = analogRead(LowPowr_Pin);
         float power = V1*(15.0/1024.0);
         if((power>10.2)) {
           digitalWrite(MOTOR_Charge_Pin,LOW);  
         }
         else {
           digitalWrite(MOTOR_Charge_Pin,HIGH); 
         }
  }
void SensorType_Update(void)
{
   static unsigned char lastSensor[8]={0,0,0,0,0,0,0,0};
   static unsigned char nowSensor[8] ={0,0,0,0,0,0,0,0}; 
   static long          last_time =0;
   uint8_t              Sensor_count = 0;
   bool                 SensorTypeSendFlag =false;
   char                 TX_CheckSum;
   uint8_t              i = 0;
   
   if((millis() - last_time)>800)
      {
        for(i=0;i<8;i++)
           {
            nowSensor[i] = RB_ADC.RB_ADC_Read(i+1);
             if(nowSensor[i]==0)
                 {
                  pinMode(RBPort[i+1].clk,INPUT);
                  pinMode(RBPort[i+1].dat,INPUT);
                 }
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
                        Servo1->RB_SERVO_Detach();
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
                      if(i<=3){
                          if(GYRO1!=NULL) {
                               delete GYRO1;
                                GYRO1 = new RB_GYRO(0);
                          }
                      }
                      else {
                          if(GYRO2!=NULL) {
                               delete GYRO2;
                                GYRO2 = new RB_GYRO(0);
                          }
                      }
                      break;
                     case RGBLED_Array_Device:
                     if(LED!=NULL)
                     {
                        delete LED;
                        LED = NULL;
                     }
                     break;
            case DigitalDisplay_Device:
                    if(DIGITALDISPLAY!=NULL)
                    {
                      delete DIGITALDISPLAY;
                      DIGITALDISPLAY = NULL;
                    }
                    break;
            case Tempture_Sensors:
                    if(TEMPERATURE!=NULL)
                    {
                      delete TEMPERATURE;
                      TEMPERATURE = NULL;
                    }
                    break;
            case JoyStick_Sensor:
                     if(Joystick!=NULL)
                    {
                      delete Joystick;
                      Joystick = NULL;
                    }
                  break;
            case Flame_Sensor:
                     if(FlameSensor!=NULL)
                    {
                      delete FlameSensor;
                      FlameSensor = NULL;
                    }
                  break;
            case Gas_Sensor:
                    if(GasSensor!=NULL){
                      delete GasSensor;
                      GasSensor = NULL;
                    }  
                  break;
            case Potentimeter_Sensor:
                    if(Pot!=NULL) {
                      delete Pot;
                      Pot = NULL;
                    }
                  break;   
          default:
                      break;
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
              TX_BUF[Sensor_count++] = lastSensor[4];
              TX_BUF[Sensor_count++] = lastSensor[5];
              TX_BUF[Sensor_count++] = lastSensor[6];
              TX_BUF[Sensor_count++] = lastSensor[7];
              TX_BUF[2] = Sensor_count+1;
              for(i=0;i<Sensor_count;i++)
                   {TX_CheckSum += TX_BUF[i];}
              TX_BUF[Sensor_count++] = TX_CheckSum;
              RB_Serial.Serial_SendString(TX_BUF,Sensor_count); 
            }
            last_time= millis();
      }  
}
