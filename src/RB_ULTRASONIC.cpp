#include "RB_ULTRASONIC.h"
#include "avr/wdt.h"

RB_Ultrasonic::RB_Ultrasonic(void) :RB_SoftI2CMaster(0)
{
    RB_SoftI2CMaster::SetMode(0); 
}

RB_Ultrasonic::RB_Ultrasonic(uint8_t port):RB_SoftI2CMaster(port)
{
 _SignalPin = RBPort[port].clk;
 RB_SoftI2CMaster::SetMode(0); 
  
}

void RB_Ultrasonic::RB_Ultrasonic_SetRGB(uint8_t SlaveAddress,uint8_t RegisterAddress,uint8_t Red,uint8_t Green,uint8_t Blue)
{
   beginTransmission(SlaveAddress);
   send(RegisterAddress);
   send(Red);
   send(Green);
   send(Blue);
   endTransmission();
}
double RB_Ultrasonic::distanceCm(uint16_t MAXcm)
{
  long distance = measure(MAXcm * 100+ 200);
  if(distance <= 0)
  {
    distance = MAXcm * 58;
  }
  return( (double)distance / 58.0);
}
double RB_Ultrasonic::distanceInch(uint16_t MAXinch)
{
  long distance = measure(MAXinch * 145 + 200);
  if(distance == 0)
  {
    distance = MAXinch * 148;
  }
  return( (double)(distance / 148.0) );
}
long RB_Ultrasonic::measure(unsigned long timeout)
{
  long duration;
  if(millis() - _lastEnterTime > 16)
  {
    _measureFlag = true; 
  }

  if(_measureFlag == true)
  {
    _lastEnterTime = millis();
    _measureFlag = false;
    beginTransmission(0x40);
    send(0xA2);
    send(0xA2);
    send(0XA2);
    send(0X01);
    endTransmission();
    pinMode(_SignalPin, INPUT);
    duration = pulseIn(_SignalPin, LOW, timeout);
	 if(duration>=300)
       {
        duration -=300;
       }
    _measureValue = duration;
  }
  else
  {
    duration = _measureValue;
  }
  return(duration);
}

double RB_Ultrasonic:: KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_mid ;//= x_last
   static double x_now;
   static double p_mid ;
   static double p_now;
   static double kg;        
   static double x_last,p_last;
   x_mid=x_last;          
   p_mid=p_last+Q;        
   kg=p_mid/(p_mid+R);   
   x_now=x_mid+kg*(ResrcData-x_mid);           
   p_now=(1-kg)*p_mid;   
   p_last = p_now;       
   x_last = x_now;       
   return x_now;                
}


double RB_Ultrasonic::Uldistance(double max_distance)
{
	
	
	double  Distance[2];
  uint8_t i=0;
	for(i=0;i<2;i++)
	{
		
    Distance[i] = distanceCm();
    delay(18); 
	//   2018-10-26    delay(28)
	}
	if(Distance[0]>=Distance[1])
	   {return Distance[0];}
  else 
     {return Distance[1];}
}

void  RB_Ultrasonic:: Swap(double  A[], int i, int j)
{
    double temp = A[i];
    A[i] = A[j];
    A[j] = temp;
}

void RB_Ultrasonic:: BubbleSort(double A[], int n)
{
    for (int j = 0; j < n - 1; j++)         // 每次最大元素就像气泡一样"浮"到数组的最后
    {
        for (int i = 0; i < n - 1 - j; i++) // 依次比较相邻的两个元素,使较大的那个向后移
        {
            if (A[i] > A[i + 1])            // 如果条件改成A[i] >= A[i + 1],则变为不稳定的排序算法
            {
                Swap(A, i, i + 1);
            }
        }
    }
}



