#include "RB_SERIAL_TASK.h"

          /*
           *  Rx_buf
           */
          char       RX_BUF[2][200] ;
          char       TX_BUF[200]    ;
          char       RX_OK = 0     ;
        unsigned   char       RX_COUNT = 0  ;
          bool       RX_ACT = 0    ; 

RB_Serial::RB_Serial(void)
{
  
}

void RB_Serial :: Serial_begin(uint32_t BAUD)
{    

     uint16_t baud_setting = (F_CPU / 4 / BAUD - 1) / 2;
     UCSR0A |=0X02;
     if (((F_CPU == 16000000UL) && (BAUD == 57600)) || (baud_setting >4095))
     {
        UCSR0A &=~0X02;
        baud_setting = (F_CPU / 8 / BAUD - 1) / 2;
      }
     UBRR0H = baud_setting >> 8;
     UBRR0L = baud_setting ;
 
     bitSet(UCSR0B,RXCIE0);
     bitSet(UCSR0B,RXEN0);
     bitSet(UCSR0B,TXEN0);
     bitSet(UCSR0C,UCSZ01);
     bitSet(UCSR0C,UCSZ00);
  
 }
 void RB_Serial::Serial_SendByte(char _byte)
 {
     loop_until_bit_is_set(UCSR0A,UDRE0);
     UDR0 = _byte;
 }
 void RB_Serial::Serial_SendString(char *str ,char len)
 {
    while(len)
      {
        Serial_SendByte(*str);
        str++; 
        len--; 
      }
  }
