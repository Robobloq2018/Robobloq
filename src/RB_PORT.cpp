
#include "RB_PORT.h" 


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
RB_Port_Sig  RBPort[9] =
{           
   {0.0}, {PWM2,PWM3}, {A6,A7},
   {A8,A9}, {A10,A11}, {A4,A5},
   {A2,A3}, {A0,A1},  {PWM0,PWM1}
 };
RB_Port_Encoder encoder_Port[3]
{
  {0    ,0    ,0         ,0         },
  {ENA_A,ENA_B,MOTOR_PWM2,MOTOR_PWM3},
  {ENB_A,ENB_B,MOTOR_PWM0,MOTOR_PWM1},
  } ;
  
#else  
 
 RB_Port_Sig   RBPort[5] = 
{
   {NC,NC},
   {A0,A1},
   {A3,A2},
   {A5,A4},
   {PWM0,PWM1}  
};
#endif
/*
 * 
 */
RB_Port::RB_Port(void)
{
  _clk =  RBPort[0].clk;
  _dat =  RBPort[0].dat;
  _port = 0 ; 
}
/*
 * 
 */
RB_Port::RB_Port(uint8_t port)
{
   _clk = RBPort[port].clk;
   _dat = RBPort[port].dat;
   _port = port ;  
}
/*
 * 
 */
uint8_t RB_Port :: GetPort(void)
{
   return(_port);  
}
/*
 * 
 */
uint8_t RB_Port :: GetClk(void)
{
    return(_clk);  
}
/*
 * 
 */
uint8_t RB_Port::GetDat(void)
{
    return(_dat);  
}

/*
 *  读取数字clk 端口数据
 */
bool RB_Port::DReadClk(void)
{
     bool val;
     pinMode(_clk,INPUT_PULLUP);
     val = digitalRead(_clk);
     return(val);   
}
/*
 * 
 */
bool RB_Port::DReadDat(void)
{  
   bool val ;
   pinMode(_dat,INPUT_PULLUP);
   val = digitalRead(_dat);
   return(val); 
}
/*
 * 
 */
void RB_Port::DWriteClk(bool value)
{  
   pinMode(_clk,OUTPUT);
   digitalWrite(_clk,value);     
}
/*
 * 
 */
 void RB_Port::DWriteDat(bool value )
 {
   pinMode(_dat,OUTPUT);
   digitalWrite(_dat,value);
 }

/*
 * 
 */
int16_t RB_Port::AReadClk(void)
{
  int16_t val;
  pinMode(_clk, INPUT);
  val = analogRead(_clk);
}
/*
 * 
 */
int16_t RB_Port::AReadDat(void)
{
   int16_t val;
  pinMode(_dat, INPUT);
  val = analogRead(_dat);  
}

void RB_Port::ResetPort(uint8_t port)
{ 
   _clk = RBPort[port].clk;
   _dat = RBPort[port].dat;
}
