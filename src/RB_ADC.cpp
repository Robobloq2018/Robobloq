#include "RB_ADC.h"

volatile uint16_t adc[10];


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
       switch(port)
          { 
            
            case 1: // 000 4 100  
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, HIGH);
                delay(10);
                return(ADC_Read()); 
                break;     
            case 2: //001 5 101
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, HIGH);
                delay(10);
                return(ADC_Read()); 
                break;   
            case 3: //010 6 110
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, HIGH);
                delay(10);
                return(ADC_Read()); 
                break;  
            case 4: //011 7 
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, HIGH);
                delay(10);
                return(ADC_Read()); 
                break;  
            case 5: //010 2
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, LOW);
                delay(10);
                return(ADC_Read()); 
                break;
            case 6: //110 3
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, HIGH);
                digitalWrite(ADC_C, LOW);
                delay(10);
                return(ADC_Read()); 
                break; 
            case 7: //001
                digitalWrite(ADC_A, HIGH);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, LOW);
                delay(10);
                return(ADC_Read()); 
                break; 
            case 8: //111
                digitalWrite(ADC_A, LOW);
                digitalWrite(ADC_B, LOW);
                digitalWrite(ADC_C, LOW);
                delay(10);
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
        
}
uint16_t  RB_ADC::ADC_Read(void)
{        

        uint32_t       adc_sum = 0;
        uint32_t       value = 0;
        float          device_type = 0;

        value = analogRead(ADC_OUT);
 
        if(value>981)
            return None_Device;
        else if(value>938)
            return Other_Device;
        else if(value>895) 
            return Other_Device;
        else if(value>852)
            return Servo_Device;
        else if(value>809)
            return DC_Motor_Device;
        else if(value>766)
            return Temp_And_Humi_Sensor;
        else if(value>724)
            return Other_Device;
        else if(value>660)
            return Ultrasonic_Distance_Sensor;
        else if(value>637)
            return Other_Device;
        else if(value>594)
            return Other_Device;
        else if(value>551)
            return Sound_Sensor;
        else if(value>490)
            return LED_Matraix_Blue;
        else if(value>465)
            return Other_Device;
        else if(value>422)
            return Other_Device;
        else if(value>379)
            return Light_Sensor;
        else if(value>336)
            return Other_Device;
        else if(value>293)
            return Other_Device;
        else if(value>250)
            return Other_Device;
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


