#include "RB_ADC.h"




RB_ADC::RB_ADC(void)
{   
     pinMode(ADC_OUT,INPUT);
     pinMode(ADC_A,OUTPUT);
     pinMode(ADC_B,OUTPUT);
     pinMode(ADC_C,OUTPUT);
     digitalWrite(ADC_A, LOW);
     digitalWrite(ADC_B, LOW);
     digitalWrite(ADC_C, LOW);
}
uint16_t  RB_ADC::RB_ADC_Read(uint8_t port)
{      
       uint8_t a = 0;
       uint8_t b = 0;
	  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)	  
       switch(port)
          { 
            
            case 1: // 000 4 100  
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, HIGH);
                delay(15);
                return(ADC_Read()); 
                break;     
            case 2: //001 5 101
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, HIGH);
                delay(15);
                return(ADC_Read()); 
                break;   
            case 3: //010 6 110
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, HIGH);
                delay(15);
                return(ADC_Read()); 
                break;  
            case 4: //011 7 
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, HIGH);
                delay(15);
                return(ADC_Read()); 
                break;  
            case 5: //010 2
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, LOW);
                delay(15);
                return(ADC_Read()); 
                break;
            case 6: //110 3
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, LOW);
                delay(15);
                return(ADC_Read()); 
                break; 
            case 7: //001
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, LOW);
                delay(15);
                return(ADC_Read()); 
                break; 
            case 8: //111
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, LOW);
                delay(15);
                return(ADC_Read()); 
                break;
            case 0xff:
                if(digitalRead(ENA_A==0)&&digitalRead(ENA_B==0))
                   {  
                      delay(3);
                      if(digitalRead(ENA_A==0)&&digitalRead(ENA_B==0))
                           { return 0; }
                      else 
                           { return 1; }
                    }
                else  { return 1; }      
                break;
            case 0xfe:
                 if(digitalRead(ENB_A==0)&&digitalRead(ENB_B==0))
                   {  
                      delay(3);
                      if(digitalRead(ENB_A==0)&&digitalRead(ENB_B==0))
                         { return 0; }
                      else 
                         { return 1; }
                    }
                else   { return 1; } 
                break;   
            default:
                return 0; 
          }
#else
	       switch(port)
          { 
            
            case 4: // 000   A0
                 digitalWrite(ADC_C, LOW);      //ADC_C
                 digitalWrite(ADC_B, LOW);      //ADC_B
                 digitalWrite(ADC_A, LOW);      //ADC_A
                 delay(25);
                return(ADC_Read()); 
                break;     
            case 3: //001    A1
                 digitalWrite(ADC_C, LOW);      //ADC_C
                 digitalWrite(ADC_B, LOW);      //ADC_B
                 digitalWrite(ADC_A, HIGH);     //ADC_A
                delay(25);
                return(ADC_Read()); 
                break;   
            case 1: //4     A4
                digitalWrite(ADC_C, HIGH);      //ADC_C
                digitalWrite(ADC_B, LOW);       //ADC_B
                digitalWrite(ADC_A, LOW);       //ADC_A
                delay(25);
                return(ADC_Read()); 
                break;  
            case 2: //6     A6
                 digitalWrite(ADC_C, HIGH);      //ADC_C
                 digitalWrite(ADC_B, HIGH);      //ADC_B
                 digitalWrite(ADC_A, LOW);       //ADC_A
                delay(25);
                return(ADC_Read()); 
                break;  
           default:
                
                break; 
          }
#endif 		  
        
}
uint16_t  RB_ADC::ADC_Read(void)
{        

        uint32_t       adc_sum = 0;
        uint16_t       valuefilter[10];
		uint32_t       value ;
		uint16_t       valuemax = 0  ;
		uint16_t       valuemin = 1023  ;
		uint8_t        value_count = 0;
        float          device_type = 0;
     
		for(value_count = 0;value_count<10;value_count++) 
		{
			 valuefilter[value_count] = analogRead(ADC_OUT);
			 
			 if(valuefilter[value_count]>=valuemax)
                    valuemax = 	valuefilter[value_count];
			 if(valuefilter[value_count]<=valuemin) 	
			        valuemin = 	valuefilter[value_count];
				
			 adc_sum += valuefilter[value_count];	  
		} 
		adc_sum = adc_sum- valuemax-valuemin;
        value = adc_sum/8;
		if(value>1010) 
			 return 0;
        else if(value>970)
            return RGBLED_Array_Device;
        else if(value>950)
            return Other_Device;    
        else if(value>930)
            return RGBLED_Matraix;
        else if(value>900) 
            return PIR_Sensor;
        else if(value>875)
            return Servo_Device;
        else if(value>800)
            return DC_Motor_Device;
        else if(value>766)
            return 	MP3_Sensor;
        else if(value>720)
            return  Color_Sensor;
        else if(value>670)
            return Ultrasonic_Distance_Sensor;
        else if(value>620)
            return Tempture_Sensors;
        else if(value>590)
            return Gyro_Sensor;
        else if(value>530)
            return Sound_Sensor;
        else if(value>500)
            return LED_Matraix_Blue;
		else if(value>480)
			return JoyStick_Sensor;
        else if(value>465)
            return Other_Device;
        else if(value>422)
            return Potentimeter_Sensor;
		else if(value>405)
			 return Flame_Sensor;
        else if(value>379)
            return Light_Sensor;
        else if(value>345)
            return Other_Device;
        else if(value>305)
            return DigitalDisplay_Device;
        else if(value>270)
            return Temp_And_Humi_Sensor;
        else if(value>207)
            return Line_Follower_Sensor;
        else if(value>164)
            return Other_Device;
        else if(value>121)
            return Other_Device;
        else if(value>78)
            return Other_Device;
        else if(value>35)
            return Other_Device;
        else 
            return Other_Device;
}
