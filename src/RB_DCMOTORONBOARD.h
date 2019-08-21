#ifndef  RB_DcmotorOnBoard_H
#define   RB_DcmotorOnBoard_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "RB_PORT.h"

class RB_DcmotorOnBoard
{
    public :
             RB_DcmotorOnBoard_Init();
      void   RB_DcmotorOnBoardM1_Run(int32_t speed);
      void   RB_DcmotorOnBoardM2_Run(int32_t speed);
      void   run(int32_t speed1,int32_t speed2);
      void   stop(void);
      void  RB_DcmotorOnBoard::SetSpeed( short Left_Speed, short Right_Speed);
    private:
      int32_t   _M2_Speed  ;
      int32_t   _M1_Speed  ;
};

#endif 
