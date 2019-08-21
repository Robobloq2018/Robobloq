#ifndef  RB_SERIAL_TASK_H
#define  RB_SERIAL_TASK_H


#include <inttypes.h>
#include <Arduino.h>

/*
 *  Rx_buf
 */
 extern         char    RX_BUF[2][200] ;
 extern         char    TX_BUF[200];
 extern         char    RX_OK  ;
 extern      unsigned   char    RX_COUNT ;
 extern         bool    RX_ACT ; 




 class RB_Serial
 {
      public :
           RB_Serial::RB_Serial(void);
           void  Serial_begin(uint32_t BAUD);
           void  Serial_SendByte(char _byte);
           void  Serial_SendString(char *str ,char len);
         
           
  } ;
  
#endif 
