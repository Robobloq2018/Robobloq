#include "RB_TEMPANDHUMI.h"


RB_TempAndHumi :: RB_TempAndHumi(void)
{
	
	
}

RB_TempAndHumi:: RB_TempAndHumi(uint8_t port):RB_Port(port)
{
	_DataPin = RBPort[port].dat;

}

RB_TempAndHumi::SetPin(uint8_t pin)
{
	
	 _DataPin = pin;
}

void RB_TempAndHumi::Update(void)
{
	uint8_t data[5] = {0};
  unsigned long Time, datatime;
  

  pinMode(_DataPin,OUTPUT);
  digitalWrite(_DataPin,HIGH);
  delay(250);

  digitalWrite(_DataPin,LOW);
  delay(20);
  
  digitalWrite(_DataPin,HIGH);
  delayMicroseconds(40);
  digitalWrite(_DataPin,LOW);

  delayMicroseconds(10);
  Time = millis();

  pinMode(_DataPin,INPUT_PULLUP);
  while(digitalRead(_DataPin) != HIGH)

  {
    if( ( millis() - Time ) > 4)
    {
      Humidity = 0;
      Temperature = 0;
      break;
    }
  }

  Time = millis();
  

  pinMode(_DataPin,INPUT_PULLUP);
  while(digitalRead(_DataPin) != LOW)

  {
    if( ( millis() - Time ) > 4)
    {
      break;
    }
  }

  for(int16_t i=0;i<40;i++)
  {
    Time = millis();

    pinMode(_DataPin,INPUT_PULLUP);
    while(digitalRead(_DataPin) == LOW)

    {
      if( ( millis() - Time ) > 4)
      {
        break;
      }
    }

    datatime = micros();
    Time = millis();

    pinMode(_DataPin,INPUT_PULLUP);
    while(digitalRead(_DataPin) == HIGH)

    {
      if( ( millis() - Time ) > 4 )
      {
        break;
      }
    }

    if ( micros() - datatime > 40 )
    {
      data[i/8] <<= 1;
      data[i/8] |= 0x01;
    }
    else
    {
      data[i/8] <<= 1;
    }
  }
   
  if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
  {
    Humidity = data[0];
    Temperature = data[2];
  }

	
	
}

uint8_t RB_TempAndHumi::GetHumidity(void)
{
	return Humidity;
	
}
uint8_t RB_TempAndHumi::GetHumidity_2(void)
{       
	Update();
	delay(400);
	return Humidity;
	
}

uint8_t RB_TempAndHumi::GetTemperature(void)
{
	return Temperature;
}


uint8_t RB_TempAndHumi::GetTemperature_2(void)
{        
	Update();
	delay(400);
	return Temperature;
}

uint8_t RB_TempAndHumi::GetValue(uint8_t index)
{
	
	if(index == 0)
	{
		return Humidity;
	}
	else
    {
		return Temperature;
	}
}

double RB_TempAndHumi::GetFahrenheit(void)
{
	
	return 1.8*Temperature+32;
}

double RB_TempAndHumi::GetKelvin(void)
{
	 
	return Temperature +273.15;
}

double RB_TempAndHumi::GetdewPoint(void)
{
    double A = 373.15/(273.15+Temperature);
	  double SUM = -7.90298*(A-1);
	  SUM += 5.02808 * log10(A); 
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ; 
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ; 
    SUM += log10(1013.246); 
    double VP = pow(10, SUM-3) * Humidity; 
    double T = log(VP/0.61078);   // temp var 
    return (241.88 * T) / (17.558-T); 

}

double RB_TempAndHumi::getPointFast(void) 
 { 
   double a = 17.271; 
   double b = 237.7; 
   double temp = (a * Temperature) / (b + Temperature) + log(Humidity/100); 
   //double Td = (b * temp) / (a - temp); 
   return ((b * temp) / (a - temp)); 
 } 


