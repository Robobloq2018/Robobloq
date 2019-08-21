#include "RB_RGBLEDMATRIX.h"

#include <util/delay.h>

uint8_t   bI2C_MasTxData[100];

   const  uint8_t tabLED_Type3Vaf[64] = { //Reference SLED1735 Datasheet Type3 Circuit Map
                                    //Frame 1
                                    0x50, 0x55, 0x55, 0x55, //C1-A ~ C1-P
                                    0x00, 0x00, 0x00, 0x00, //C2-A ~ C2-P ,
                                    0x00, 0x00, 0x00, 0x00, //C3-A ~ C3-P  
                                    0x15, 0x54, 0x55, 0x55, //C4-A ~ C4-P 
                                    0x00, 0x00, 0x00, 0x00, //C5-A ~ C5-P  
                                    0x00, 0x00, 0x00, 0x00, //C6-A ~ C6-P 
                                    0x55, 0x05, 0x55, 0x55, //C7-A ~ C7-P  
                                    0x00, 0x00, 0x00, 0x00, //C8-A ~ C8-P
                                    //Frame 2
                                    0x00, 0x00, 0x00, 0x00, //C9-A ~ C9-P 
                                    0x55, 0x55, 0x41, 0x55, //C10-A ~ C10-P 
                                    0x00, 0x00, 0x00, 0x00, //C11-A ~ C11-P  
                                    0x00, 0x00, 0x00, 0x00, //C12-A ~ C12-P 
                                    0x55, 0x55, 0x55, 0x50, //C13-A ~ C13-P  
                                    0x00, 0x00, 0x00, 0x00, //C14-A ~ C14-P 
                                    0x00, 0x00, 0x00, 0x00, //C15-A ~ C15-P 
                                    0x00, 0x00, 0x00, 0x00, //C16-A ~ C16-P 
                                    };

uint8_t Frame1[16] = {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};
  uint8_t Frame2[16] =  {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};                     
  uint8_t Frame3[16] =  {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};
  uint8_t Frame4[16] =  {0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
                        0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00};    


                                
RB_RGBLEDMATRIX::RB_RGBLEDMATRIX(void):RB_SoftI2CMaster(0)
{
	RB_SoftI2CMaster::SetMode(1);
	
}

RB_RGBLEDMATRIX::RB_RGBLEDMATRIX(uint8_t port):RB_SoftI2CMaster(port)
{
    RB_SoftI2CMaster::SetMode(1);
}

void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_Write2Byte(uint8_t address,uint8_t rgaddress,uint8_t data)
{
  beginTransmission(address);
  send(rgaddress);
  send(data);
  endTransmission();

}

void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_WriteNByte(uint8_t address,uint8_t rgaddress,uint8_t *data,unsigned short datalen)
{
  
  RB_SoftI2CMaster::I2C_Star();
  RB_SoftI2CMaster::I2C_Write(address<<1|0x00);
  RB_SoftI2CMaster::I2C_GetAck();
  RB_SoftI2CMaster::I2C_Write(rgaddress);
  RB_SoftI2CMaster::I2C_GetAck();
  while(datalen--)
  {
   RB_SoftI2CMaster::I2C_Write(*data);
   RB_SoftI2CMaster::I2C_GetAck();
   data++;
  }
  RB_SoftI2CMaster::I2C_Stop();
}
void RB_RGBLEDMATRIX::LED_Type3ClearFrame1Page(unsigned char address)
{
     RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FRAME1_PAGE);
	 for(int  i = 0; i< TYPE3_FRAME1PAGE_LENGTH ; i++)
	{
		bI2C_MasTxData[i] = TYPE3_LED_FRAME_CLR_DATA;
	}
	//send 0xB3 bytes length Data From address 0x00 
	RB_RGBLEDMATRIX_WriteNByte(address,(mskLED_FRAME_REG_ADDR&0x00), bI2C_MasTxData,TYPE3_FRAME1PAGE_LENGTH-1);
}

void RB_RGBLEDMATRIX::LED_Type3ClearFrame2Page(unsigned char address)
{
     RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FRAME2_PAGE);
	for(int  i = 0; i< TYPE3_FRAME2PAGE_LENGTH ; i++)
	{
		bI2C_MasTxData[i] = TYPE3_LED_FRAME_CLR_DATA;
	}
	//send 0xB3 bytes length Data From address 0x00 
	RB_RGBLEDMATRIX_WriteNByte(address,(mskLED_FRAME_REG_ADDR&0x00),bI2C_MasTxData, TYPE3_FRAME2PAGE_LENGTH-1);
}


void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_Init(unsigned char address)
{   
  
	RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);
  RB_RGBLEDMATRIX_Write2Byte(address,0X0A, 0x00);	
	RB_RGBLEDMATRIX_Write2Byte(address,PICTURE_DISPLAY_REG, 0x10);	
	RB_RGBLEDMATRIX_Write2Byte(address,STAGGERED_DELAY_REG, ((mskSTD4 & CONST_STD_GROUP4)|(mskSTD3 & CONST_STD_GROUP3)|(mskSTD2 & CONST_STD_GROUP2)|(mskSTD1 & CONST_STD_GROUP1)));
	RB_RGBLEDMATRIX_Write2Byte(address,SLEW_RATE_CTL_REG, mskSLEW_RATE_CTL_EN);
	RB_RGBLEDMATRIX_Write2Byte(address,VAF_CTL_REG, (mskVAF2|mskVAF1));
	RB_RGBLEDMATRIX_Write2Byte(address,VAF_CTL_REG2, (mskFORCEVAFCTL_DISABLE|mskFORCEVAFTIME_CONST|mskVAF3));
	RB_RGBLEDMATRIX_Write2Byte(address,CURRENT_CTL_REG, (mskCURRENT_CTL_EN|CONST_CURRENT_STEP_20mA));
	LED_Type3ClearFrame1Page(address);	
	LED_Type3ClearFrame2Page(address);	
	RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, LED_VAF_PAGE);
	for(int i = 0;i < TYPE3_VAF_FRAME_LENGTH;i++)
	{
		bI2C_MasTxData[i] = tabLED_Type3Vaf[i];
	}
	RB_RGBLEDMATRIX_WriteNByte(address,TYPE3_VAF_FRAME_FIRST_ADDR,bI2C_MasTxData, TYPE3_VAF_FRAME_LENGTH-1);
	RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);
	RB_RGBLEDMATRIX_Write2Byte(address,SW_SHUT_DOWN_REG, mskSW_NORMAL_MODE);	
  LED_SledType3Fun1(address);
  LED_SledType3Fun2(address);

}
void RB_RGBLEDMATRIX::LED_SledType3Fun1(unsigned char address)
{
	uint32_t i;
		
	// System must go to SW shutdowm mode
	RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);//   __LED_SelectFunctionPage;
	RB_RGBLEDMATRIX_Write2Byte(address,SW_SHUT_DOWN_REG, mskSW_SHUT_DOWN_MODE);
		
	RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FRAME1_PAGE);//__LED_SelectFrame1Page;		
	for( i = 0; i< TYPE3_LED_FRAME_LENGTH ; i++)
	{
		bI2C_MasTxData[i] = 0X00;
	}
	RB_RGBLEDMATRIX_WriteNByte(address,TYPE3_LED_FRAME_FIRST_ADDR,bI2C_MasTxData, TYPE3_LED_FRAME_LENGTH-1);
	for( i = 0; i< TYPE3_PWM_FRAME_LENGTH ; i++)
	{
		bI2C_MasTxData[i] = 0x55;
	}
  RB_RGBLEDMATRIX_WriteNByte(address,TYPE3_PWM_FRAME_FIRST_ADDR,bI2C_MasTxData,TYPE3_PWM_FRAME_LENGTH-1);	

  RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);
	RB_RGBLEDMATRIX_Write2Byte(address,SW_SHUT_DOWN_REG, mskSW_NORMAL_MODE);	

}

void RB_RGBLEDMATRIX::LED_SledType3Fun2(unsigned char address)
{
  uint32_t i;
    
  RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);//   __LED_SelectFunctionPage;
  RB_RGBLEDMATRIX_Write2Byte(address,SW_SHUT_DOWN_REG, mskSW_SHUT_DOWN_MODE);
    
  RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FRAME2_PAGE);//__LED_SelectFrame1Page;  
  for( i = 0; i< TYPE3_LED_FRAME_LENGTH ; i++)
  {
    bI2C_MasTxData[i] = 0X00;
  }
   
  RB_RGBLEDMATRIX_WriteNByte(address,TYPE3_LED_FRAME_FIRST_ADDR,bI2C_MasTxData, TYPE3_LED_FRAME_LENGTH-1);
  

  for( i = 0; i< TYPE3_PWM_FRAME_LENGTH ; i++)
  {
    bI2C_MasTxData[i] = 0x55;
  }

  RB_RGBLEDMATRIX_WriteNByte(address,TYPE3_PWM_FRAME_FIRST_ADDR,bI2C_MasTxData,TYPE3_PWM_FRAME_LENGTH-1);  
  RB_RGBLEDMATRIX_Write2Byte(address,CONFIGURE_CMD_PAGE, FUNCTION_PAGE);
  RB_RGBLEDMATRIX_Write2Byte(address,SW_SHUT_DOWN_REG, mskSW_NORMAL_MODE);
}



void RB_RGBLEDMATRIX::RGBLEDMATRIX_DISPALY1(uint16_t *Display_data,uint16_t *Display_data1)
{
      uint16_t data;
      uint16_t i;
      data = 0;
      data  |= (((Display_data[0]&0x01)<<0))|(((Display_data[0]&0x02)>>1)<<1)|(((Display_data[0]&0x04)>>2)<<2);
      data  |= (((Display_data[1]&0x01)<<3))|(((Display_data[1]&0x02)>>1)<<4)|(((Display_data[1]&0x04)>>2)<<5);
      data  |= (((Display_data[2]&0x01)<<6))|(((Display_data[2]&0x02)>>1)<<7)|(((Display_data[2]&0x04)>>2)<<8);
      data  |= (((Display_data[3]&0x01)<<9))|(((Display_data[3]&0x02)>>1)<<10)|(((Display_data[3]&0x04)>>2)<<11);
      data  |= (((Display_data[4]&0x01)<<12))|(((Display_data[4]&0x02)>>1)<<13)|(((Display_data[4]&0x04)>>2)<<14);

      Frame1[0]  = (unsigned char)(data%256);
      Frame1[1]  = (unsigned char)(data/256);
    
      data = 0;
     
      data  |= (((Display_data[7]&0x01)<<3))|(((Display_data[7]&0x02)>>1)<<4)|(((Display_data[7]&0x04)>>2)<<5);
      data  |= (((Display_data[8]&0x01)<<6))|(((Display_data[8]&0x02)>>1)<<7)|(((Display_data[8]&0x04)>>2)<<8);
      data  |= (((Display_data[9]&0x01)<<9))|(((Display_data[9]&0x02)>>1)<<10)|(((Display_data[9]&0x04)>>2)<<11);
      data  |= (((Display_data[10]&0x01)<<12))|(((Display_data[10]&0x02)>>1)<<13)|(((Display_data[10]&0x04)>>2)<<14);

      Frame1[2]  = (unsigned char)(data%256);
      Frame1[3]  = (unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data[6]&0x01)<<1))|(((Display_data[6]&0x02)>>1)<<0)|(((Display_data[6]&0x04)>>2)<<2);
      data  |= (((Display_data[13]&0x01)<<3))|(((Display_data[13]&0x02)>>1)<<4)|(((Display_data[13]&0x04)>>2)<<5);
      data  |= (((Display_data[14]&0x01)<<6))|(((Display_data[14]&0x02)>>1)<<7)|(((Display_data[14]&0x04)>>2)<<8);
      data  |= (((Display_data[15]&0x01)<<9))|(((Display_data[15]&0x02)>>1)<<10)|(((Display_data[15]&0x04)>>2)<<11);
      data  |= (((Display_data[5]&0x01)<<12))|(((Display_data[5]&0x02)>>1)<<13)|(((Display_data[5]&0x04)>>2)<<14);

      Frame1[4]  = (unsigned char)(data%256);
      Frame1[5]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[12]&0x01)<<1))|(((Display_data[12]&0x02)>>1)<<2)|(((Display_data[12]&0x04)>>2)<<15);
      data  |= (((Display_data[19]&0x01)<<3))|(((Display_data[19]&0x02)>>1)<<4)|(((Display_data[19]&0x04)>>2)<<5);
      data  |= (((Display_data[20]&0x01)<<6))|(((Display_data[20]&0x02)>>1)<<7)|(((Display_data[20]&0x04)>>2)<<8);
      data  |= (((Display_data[21]&0x01)<<9))|(((Display_data[21]&0x02)>>1)<<10)|(((Display_data[21]&0x04)>>2)<<11);
      data  |= (((Display_data[11]&0x01)<<12))|(((Display_data[11]&0x02)>>1)<<13)|(((Display_data[11]&0x04)>>2)<<14);

      Frame1[6]  = (unsigned char)(data%256);
      Frame1[7]  = (unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data[18]&0x01)<<1))|(((Display_data[18]&0x02)>>1)<<2)|(((Display_data[18]&0x04)>>2)<<3);
      data  |= (((Display_data[26]&0x01)<<6))|(((Display_data[26]&0x02)>>1)<<7)|(((Display_data[26]&0x04)>>2)<<8);
      data  |= (((Display_data[16]&0x01)<<9))|(((Display_data[16]&0x02)>>1)<<10)|(((Display_data[16]&0x04)>>2)<<11);
      data  |= (((Display_data[17]&0x01)<<12))|(((Display_data[17]&0x02)>>1)<<13)|(((Display_data[17]&0x04)>>2)<<14);

      Frame1[8]  = (unsigned char)(data%256);
      Frame1[9]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[25]&0x01)<<4))|(((Display_data[25]&0x02)>>1))|(((Display_data[25]&0x04)>>2)<<5);
      data  |= (((Display_data[24]&0x01)<<1))|(((Display_data[24]&0x02)>>1)<<2)|(((Display_data[24]&0x04)>>2)<<3);
      data  |= (((Display_data[32]&0x01)<<6))|(((Display_data[32]&0x02)>>1)<<7)|(((Display_data[32]&0x04)>>2)<<8);
      data  |= (((Display_data[22]&0x01)<<9))|(((Display_data[22]&0x02)>>1)<<10)|(((Display_data[22]&0x04)>>2)<<11);
      data  |= (((Display_data[23]&0x01)<<12))|(((Display_data[23]&0x02)>>1)<<13)|(((Display_data[23]&0x04)>>2)<<14);

      Frame1[10]  = (unsigned char)(data%256);
      Frame1[11]  = (unsigned char)(data/256);
 

      data = 0;
      data  |= (((Display_data[30]&0x01)<<1))|(((Display_data[30]&0x02)>>1)<<2)|(((Display_data[30]&0x04)>>2)<<3);
      data  |= (((Display_data[31]&0x01)<<4))|(((Display_data[31]&0x02)>>1)<<5)|(((Display_data[31]&0x04)>>2)<<15);
      data  |= (((Display_data[27]&0x01)<<6))|(((Display_data[27]&0x02)>>1)<<7)|(((Display_data[27]&0x04)>>2)<<8);
      data  |= (((Display_data[28]&0x01)<<9))|(((Display_data[28]&0x02)>>1)<<10)|(((Display_data[28]&0x04)>>2)<<11);
      data  |= (((Display_data[29]&0x01)<<12))|(((Display_data[29]&0x02)>>1)<<13)|(((Display_data[29]&0x04)>>2)<<14);

      Frame1[12]  = (unsigned char)(data%256);
      Frame1[13]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[36]&0x01)<<1))|(((Display_data[36]&0x02)>>1)<<2)|(((Display_data[36]&0x04)>>2)<<3);
      data  |= (((Display_data[37]&0x01)<<4))|(((Display_data[37]&0x02)>>1)<<5)|(((Display_data[37]&0x04)>>2)<<6);
      data  |= (((Display_data[34]&0x01)<<9))|(((Display_data[34]&0x02)>>1)<<10)|(((Display_data[34]&0x04)>>2)<<11);
      data  |= (((Display_data[35]&0x01)<<12))|(((Display_data[35]&0x02)>>1)<<13)|(((Display_data[35]&0x04)>>2)<<14);

      Frame1[14]  = (unsigned char)(data%256);
      Frame1[15]  = (unsigned char)(data/256);
  

     
      data = 0;
      data  |= (((Display_data[42]&0x01)<<1))|(((Display_data[42]&0x02)>>1)<<2)|(((Display_data[42]&0x04)>>2)<<3);
      data  |= (((Display_data[43]&0x01)<<4))|(((Display_data[43]&0x02)>>1)<<5)|(((Display_data[43]&0x04)>>2)<<6);
      data  |= (((Display_data[33]&0x01)<<7))|(((Display_data[33]&0x02)>>1)<<0)|(((Display_data[33]&0x04)>>2)<<8);
      data  |= (((Display_data[40]&0x01)<<9))|(((Display_data[40]&0x02)>>1)<<10)|(((Display_data[40]&0x04)>>2)<<11);
      data  |= (((Display_data[41]&0x01)<<12))|(((Display_data[41]&0x02)>>1)<<13)|(((Display_data[41]&0x04)>>2)<<14);

      Frame2[0]  = (unsigned char)(data%256);
      Frame2[1]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[48]&0x01)<<1))|(((Display_data[48]&0x02)>>1)<<2)|(((Display_data[48]&0x04)>>2)<<3);
      data  |= (((Display_data[49]&0x01)<<4))|(((Display_data[49]&0x02)>>1)<<5)|(((Display_data[49]&0x04)>>2)<<6);
      data  |= (((Display_data[39]&0x01)<<7))|(((Display_data[39]&0x02)>>1)<<8)|(((Display_data[39]&0x04)>>2)<<15);
      data  |= (((Display_data[46]&0x01)<<9))|(((Display_data[46]&0x02)>>1)<<10)|(((Display_data[46]&0x04)>>2)<<11);
      data  |= (((Display_data[47]&0x01)<<12))|(((Display_data[47]&0x02)>>1)<<13)|(((Display_data[47]&0x04)>>2)<<14);

      Frame2[2]  = (unsigned char)(data%256);
      Frame2[3]  = (unsigned char)(data/256);
  
      data = 0;
      data  |= (((Display_data[54]&0x01)<<1))|(((Display_data[54]&0x02)>>1)<<2)|(((Display_data[54]&0x04)>>2)<<3);
      data  |= (((Display_data[38]&0x01)<<4))|(((Display_data[38]&0x02)>>1)<<5)|(((Display_data[38]&0x04)>>2)<<6);
      data  |= (((Display_data[45]&0x01)<<7))|(((Display_data[45]&0x02)>>1)<<8)|(((Display_data[45]&0x04)>>2)<<9);
      data  |= (((Display_data[53]&0x01)<<12))|(((Display_data[53]&0x02)>>1)<<13)|(((Display_data[53]&0x04)>>2)<<14);

      Frame2[4]  = (unsigned char)(data%256);
      Frame2[5]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data[60]&0x01)<<1))|(((Display_data[60]&0x02)>>1)<<2)|(((Display_data[60]&0x04)>>2)<<3);
      data  |= (((Display_data[44]&0x01)<<4))|(((Display_data[44]&0x02)>>1)<<5)|(((Display_data[44]&0x04)>>2)<<6);
      data  |= (((Display_data[51]&0x01)<<7))|(((Display_data[51]&0x02)>>1)<<8)|(((Display_data[51]&0x04)>>2)<<9);
      data  |= (((Display_data[52]&0x01)<<10))|(((Display_data[52]&0x02)>>1)<<0)|(((Display_data[52]&0x04)>>2)<<11);
      data  |= (((Display_data[59]&0x01)<<12))|(((Display_data[59]&0x02)>>1)<<13)|(((Display_data[59]&0x04)>>2)<<14);

      Frame2[6]  = (unsigned char)(data%256);
      Frame2[7]  = (unsigned char)(data/256);
 
      data = 0;
      data  |= (((Display_data[66]&0x01)<<1))|(((Display_data[66]&0x02)>>1)<<2)|(((Display_data[66]&0x04)>>2)<<3);
      data  |= (((Display_data[50]&0x01)<<4))|(((Display_data[50]&0x02)>>1)<<5)|(((Display_data[50]&0x04)>>2)<<6);
      data  |= (((Display_data[57]&0x01)<<7))|(((Display_data[57]&0x02)>>1)<<8)|(((Display_data[57]&0x04)>>2)<<9);
      data  |= (((Display_data[58]&0x01)<<10))|(((Display_data[58]&0x02)>>1)<<11)|(((Display_data[58]&0x04)>>2)<<15);
      data  |= (((Display_data[65]&0x01)<<12))|(((Display_data[65]&0x02)>>1)<<13)|(((Display_data[65]&0x04)>>2)<<14);

      Frame2[8]  = (unsigned char)(data%256);
      Frame2[9]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data[55]&0x01)<<1))|(((Display_data[55]&0x02)>>1)<<2)|(((Display_data[55]&0x04)>>2)<<3);
      data  |= (((Display_data[56]&0x01)<<4))|(((Display_data[56]&0x02)>>1)<<5)|(((Display_data[56]&0x04)>>2)<<6);
      data  |= (((Display_data[63]&0x01)<<7))|(((Display_data[63]&0x02)>>1)<<8)|(((Display_data[63]&0x04)>>2)<<9);
      data  |= (((Display_data[64]&0x01)<<10))|(((Display_data[64]&0x02)>>1)<<11)|(((Display_data[64]&0x04)>>2)<<12);


      Frame2[10]  = (unsigned char)(data%256);
      Frame2[11]  = (unsigned char)(data/256);
   
      data = 0;
      data  |= (((Display_data[61]&0x01)<<1))|(((Display_data[61]&0x02)>>1)<<2)|(((Display_data[61]&0x04)>>2)<<3);
      data  |= (((Display_data[62]&0x01)<<4))|(((Display_data[62]&0x02)>>1)<<5)|(((Display_data[62]&0x04)>>2)<<6);
      data  |= (((Display_data[69]&0x01)<<7))|(((Display_data[69]&0x02)>>1)<<8)|(((Display_data[69]&0x04)>>2)<<9);
      data  |= (((Display_data[70]&0x01)<<10))|(((Display_data[70]&0x02)>>1)<<11)|(((Display_data[70]&0x04)>>2)<<12);
      data  |= (((Display_data[71]&0x01)<<13))|(((Display_data[71]&0x02)>>1)<<0)|(((Display_data[71]&0x04)>>2)<<14);

      Frame2[12]  = (unsigned char)(data%256);
      Frame2[13]  = (unsigned char)(data/256);
    ;
      data = 0;
      data  |= (((Display_data[67]&0x01)<<1))|(((Display_data[67]&0x02)>>1)<<2)|(((Display_data[67]&0x04)>>2)<<3);
      data  |= (((Display_data[68]&0x01)<<4))|(((Display_data[68]&0x02)>>1)<<5)|(((Display_data[68]&0x04)>>2)<<6);
      Frame2[14]  = (unsigned char)(data%256);
      Frame2[15]  = (unsigned char)(data/256);



      
      data = 0;
      data  |= (((Display_data1[0]&0x01)<<0))|(((Display_data1[0]&0x02)>>1)<<1)|(((Display_data1[0]&0x04)>>2)<<2);
      data  |= (((Display_data1[1]&0x01)<<3))|(((Display_data1[1]&0x02)>>1)<<4)|(((Display_data1[1]&0x04)>>2)<<5);
      data  |= (((Display_data1[2]&0x01)<<6))|(((Display_data1[2]&0x02)>>1)<<7)|(((Display_data1[2]&0x04)>>2)<<8);
      data  |= (((Display_data1[3]&0x01)<<9))|(((Display_data1[3]&0x02)>>1)<<10)|(((Display_data1[3]&0x04)>>2)<<11);
      data  |= (((Display_data1[4]&0x01)<<12))|(((Display_data1[4]&0x02)>>1)<<13)|(((Display_data1[4]&0x04)>>2)<<14);

      Frame3[0]  = (unsigned char)(data%256);
      Frame3[1]  = (unsigned char)(data/256);
    
      data = 0;
     
      data  |= (((Display_data1[7]&0x01)<<3))|(((Display_data1[7]&0x02)>>1)<<4)|(((Display_data1[7]&0x04)>>2)<<5);
      data  |= (((Display_data1[8]&0x01)<<6))|(((Display_data1[8]&0x02)>>1)<<7)|(((Display_data1[8]&0x04)>>2)<<8);
      data  |= (((Display_data1[9]&0x01)<<9))|(((Display_data1[9]&0x02)>>1)<<10)|(((Display_data1[9]&0x04)>>2)<<11);
      data  |= (((Display_data1[10]&0x01)<<12))|(((Display_data1[10]&0x02)>>1)<<13)|(((Display_data1[10]&0x04)>>2)<<14);

      Frame3[2]  = (unsigned char)(data%256);
      Frame3[3]  = (unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data1[6]&0x01)<<1))|(((Display_data1[6]&0x02)>>1)<<0)|(((Display_data1[6]&0x04)>>2)<<2);
      data  |= (((Display_data1[13]&0x01)<<3))|(((Display_data1[13]&0x02)>>1)<<4)|(((Display_data1[13]&0x04)>>2)<<5);
      data  |= (((Display_data1[14]&0x01)<<6))|(((Display_data1[14]&0x02)>>1)<<7)|(((Display_data1[14]&0x04)>>2)<<8);
      data  |= (((Display_data1[15]&0x01)<<9))|(((Display_data1[15]&0x02)>>1)<<10)|(((Display_data1[15]&0x04)>>2)<<11);
      data  |= (((Display_data1[5]&0x01)<<12))|(((Display_data1[5]&0x02)>>1)<<13)|(((Display_data1[5]&0x04)>>2)<<14);

      Frame3[4]  = (unsigned char)(data%256);
      Frame3[5]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data1[12]&0x01)<<1))|(((Display_data1[12]&0x02)>>1)<<2)|(((Display_data1[12]&0x04)>>2)<<15);
      data  |= (((Display_data1[19]&0x01)<<3))|(((Display_data1[19]&0x02)>>1)<<4)|(((Display_data1[19]&0x04)>>2)<<5);
      data  |= (((Display_data1[20]&0x01)<<6))|(((Display_data1[20]&0x02)>>1)<<7)|(((Display_data1[20]&0x04)>>2)<<8);
      data  |= (((Display_data1[21]&0x01)<<9))|(((Display_data1[21]&0x02)>>1)<<10)|(((Display_data1[21]&0x04)>>2)<<11);
      data  |= (((Display_data1[11]&0x01)<<12))|(((Display_data1[11]&0x02)>>1)<<13)|(((Display_data1[11]&0x04)>>2)<<14);

      Frame3[6]  = (unsigned char)(data%256);
      Frame3[7]  = (unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data1[18]&0x01)<<1))|(((Display_data1[18]&0x02)>>1)<<2)|(((Display_data1[18]&0x04)>>2)<<3);
      data  |= (((Display_data1[26]&0x01)<<6))|(((Display_data1[26]&0x02)>>1)<<7)|(((Display_data1[26]&0x04)>>2)<<8);
      data  |= (((Display_data1[16]&0x01)<<9))|(((Display_data1[16]&0x02)>>1)<<10)|(((Display_data1[16]&0x04)>>2)<<11);
      data  |= (((Display_data1[17]&0x01)<<12))|(((Display_data1[17]&0x02)>>1)<<13)|(((Display_data1[17]&0x04)>>2)<<14);

      Frame3[8]  = (unsigned char)(data%256);
      Frame3[9]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data1[25]&0x01)<<4))|(((Display_data1[25]&0x02)>>1))|(((Display_data1[25]&0x04)>>2)<<5);
      data  |= (((Display_data1[24]&0x01)<<1))|(((Display_data1[24]&0x02)>>1)<<2)|(((Display_data1[24]&0x04)>>2)<<3);
      data  |= (((Display_data1[32]&0x01)<<6))|(((Display_data1[32]&0x02)>>1)<<7)|(((Display_data1[32]&0x04)>>2)<<8);
      data  |= (((Display_data1[22]&0x01)<<9))|(((Display_data1[22]&0x02)>>1)<<10)|(((Display_data1[22]&0x04)>>2)<<11);
      data  |= (((Display_data1[23]&0x01)<<12))|(((Display_data1[23]&0x02)>>1)<<13)|(((Display_data1[23]&0x04)>>2)<<14);

      Frame3[10]  = (unsigned char)(data%256);
      Frame3[11]  = (unsigned char)(data/256);
 

      data = 0;
      data  |= (((Display_data1[30]&0x01)<<1))|(((Display_data1[30]&0x02)>>1)<<2)|(((Display_data1[30]&0x04)>>2)<<3);
      data  |= (((Display_data1[31]&0x01)<<4))|(((Display_data1[31]&0x02)>>1)<<5)|(((Display_data1[31]&0x04)>>2)<<15);
      data  |= (((Display_data1[27]&0x01)<<6))|(((Display_data1[27]&0x02)>>1)<<7)|(((Display_data1[27]&0x04)>>2)<<8);
      data  |= (((Display_data1[28]&0x01)<<9))|(((Display_data1[28]&0x02)>>1)<<10)|(((Display_data1[28]&0x04)>>2)<<11);
      data  |= (((Display_data1[29]&0x01)<<12))|(((Display_data1[29]&0x02)>>1)<<13)|(((Display_data1[29]&0x04)>>2)<<14);

      Frame3[12]  = (unsigned char)(data%256);
      Frame3[13]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data1[36]&0x01)<<1))|(((Display_data1[36]&0x02)>>1)<<2)|(((Display_data1[36]&0x04)>>2)<<3);
      data  |= (((Display_data1[37]&0x01)<<4))|(((Display_data1[37]&0x02)>>1)<<5)|(((Display_data1[37]&0x04)>>2)<<6);
      data  |= (((Display_data1[34]&0x01)<<9))|(((Display_data1[34]&0x02)>>1)<<10)|(((Display_data1[34]&0x04)>>2)<<11);
      data  |= (((Display_data1[35]&0x01)<<12))|(((Display_data1[35]&0x02)>>1)<<13)|(((Display_data1[35]&0x04)>>2)<<14);

      Frame3[14]  = (unsigned char)(data%256);
      Frame3[15]  = (unsigned char)(data/256);
  

     
      data = 0;
      data  |= (((Display_data1[42]&0x01)<<1))|(((Display_data1[42]&0x02)>>1)<<2)|(((Display_data1[42]&0x04)>>2)<<3);
      data  |= (((Display_data1[43]&0x01)<<4))|(((Display_data1[43]&0x02)>>1)<<5)|(((Display_data1[43]&0x04)>>2)<<6);
      data  |= (((Display_data1[33]&0x01)<<7))|(((Display_data1[33]&0x02)>>1)<<0)|(((Display_data1[33]&0x04)>>2)<<8);
      data  |= (((Display_data1[40]&0x01)<<9))|(((Display_data1[40]&0x02)>>1)<<10)|(((Display_data1[40]&0x04)>>2)<<11);
      data  |= (((Display_data1[41]&0x01)<<12))|(((Display_data1[41]&0x02)>>1)<<13)|(((Display_data1[41]&0x04)>>2)<<14);

      Frame4[0]  = (unsigned char)(data%256);
      Frame4[1]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data1[48]&0x01)<<1))|(((Display_data1[48]&0x02)>>1)<<2)|(((Display_data1[48]&0x04)>>2)<<3);
      data  |= (((Display_data1[49]&0x01)<<4))|(((Display_data1[49]&0x02)>>1)<<5)|(((Display_data1[49]&0x04)>>2)<<6);
      data  |= (((Display_data1[39]&0x01)<<7))|(((Display_data1[39]&0x02)>>1)<<8)|(((Display_data1[39]&0x04)>>2)<<15);
      data  |= (((Display_data1[46]&0x01)<<9))|(((Display_data1[46]&0x02)>>1)<<10)|(((Display_data1[46]&0x04)>>2)<<11);
      data  |= (((Display_data1[47]&0x01)<<12))|(((Display_data1[47]&0x02)>>1)<<13)|(((Display_data1[47]&0x04)>>2)<<14);

      Frame4[2]  = (unsigned char)(data%256);
      Frame4[3]  = (unsigned char)(data/256);
  
      data = 0;
      data  |= (((Display_data1[54]&0x01)<<1))|(((Display_data1[54]&0x02)>>1)<<2)|(((Display_data1[54]&0x04)>>2)<<3);
      data  |= (((Display_data1[38]&0x01)<<4))|(((Display_data1[38]&0x02)>>1)<<5)|(((Display_data1[38]&0x04)>>2)<<6);
      data  |= (((Display_data1[45]&0x01)<<7))|(((Display_data1[45]&0x02)>>1)<<8)|(((Display_data1[45]&0x04)>>2)<<9);
      data  |= (((Display_data1[53]&0x01)<<12))|(((Display_data1[53]&0x02)>>1)<<13)|(((Display_data1[53]&0x04)>>2)<<14);

      Frame4[4]  = (unsigned char)(data%256);
      Frame4[5]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data1[60]&0x01)<<1))|(((Display_data1[60]&0x02)>>1)<<2)|(((Display_data1[60]&0x04)>>2)<<3);
      data  |= (((Display_data1[44]&0x01)<<4))|(((Display_data1[44]&0x02)>>1)<<5)|(((Display_data1[44]&0x04)>>2)<<6);
      data  |= (((Display_data1[51]&0x01)<<7))|(((Display_data1[51]&0x02)>>1)<<8)|(((Display_data1[51]&0x04)>>2)<<9);
      data  |= (((Display_data1[52]&0x01)<<10))|(((Display_data1[52]&0x02)>>1)<<0)|(((Display_data1[52]&0x04)>>2)<<11);
      data  |= (((Display_data1[59]&0x01)<<12))|(((Display_data1[59]&0x02)>>1)<<13)|(((Display_data1[59]&0x04)>>2)<<14);

      Frame4[6]  = (unsigned char)(data%256);
      Frame4[7]  = (unsigned char)(data/256);
 
      data = 0;
      data  |= (((Display_data1[66]&0x01)<<1))|(((Display_data1[66]&0x02)>>1)<<2)|(((Display_data1[66]&0x04)>>2)<<3);
      data  |= (((Display_data1[50]&0x01)<<4))|(((Display_data1[50]&0x02)>>1)<<5)|(((Display_data1[50]&0x04)>>2)<<6);
      data  |= (((Display_data1[57]&0x01)<<7))|(((Display_data1[57]&0x02)>>1)<<8)|(((Display_data1[57]&0x04)>>2)<<9);
      data  |= (((Display_data1[58]&0x01)<<10))|(((Display_data1[58]&0x02)>>1)<<11)|(((Display_data1[58]&0x04)>>2)<<15);
      data  |= (((Display_data1[65]&0x01)<<12))|(((Display_data1[65]&0x02)>>1)<<13)|(((Display_data1[65]&0x04)>>2)<<14);

      Frame4[8]  = (unsigned char)(data%256);
      Frame4[9]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data1[55]&0x01)<<1))|(((Display_data1[55]&0x02)>>1)<<2)|(((Display_data1[55]&0x04)>>2)<<3);
      data  |= (((Display_data1[56]&0x01)<<4))|(((Display_data1[56]&0x02)>>1)<<5)|(((Display_data1[56]&0x04)>>2)<<6);
      data  |= (((Display_data1[63]&0x01)<<7))|(((Display_data1[63]&0x02)>>1)<<8)|(((Display_data1[63]&0x04)>>2)<<9);
      data  |= (((Display_data1[64]&0x01)<<10))|(((Display_data1[64]&0x02)>>1)<<11)|(((Display_data1[64]&0x04)>>2)<<12);


      Frame4[10]  = (unsigned char)(data%256);
      Frame4[11]  = (unsigned char)(data/256);
   
      data = 0;
      data  |= (((Display_data1[61]&0x01)<<1))|(((Display_data1[61]&0x02)>>1)<<2)|(((Display_data1[61]&0x04)>>2)<<3);
      data  |= (((Display_data1[62]&0x01)<<4))|(((Display_data1[62]&0x02)>>1)<<5)|(((Display_data1[62]&0x04)>>2)<<6);
      data  |= (((Display_data1[69]&0x01)<<7))|(((Display_data1[69]&0x02)>>1)<<8)|(((Display_data1[69]&0x04)>>2)<<9);
      data  |= (((Display_data1[70]&0x01)<<10))|(((Display_data1[70]&0x02)>>1)<<11)|(((Display_data1[70]&0x04)>>2)<<12);
      data  |= (((Display_data1[71]&0x01)<<13))|(((Display_data1[71]&0x02)>>1)<<0)|(((Display_data1[71]&0x04)>>2)<<14);

      Frame4[12]  = (unsigned char)(data%256);
      Frame4[13]  = (unsigned char)(data/256);
      data = 0;
      data  |= (((Display_data1[67]&0x01)<<1))|(((Display_data1[67]&0x02)>>1)<<2)|(((Display_data1[67]&0x04)>>2)<<3);
      data  |= (((Display_data1[68]&0x01)<<4))|(((Display_data1[68]&0x02)>>1)<<5)|(((Display_data1[68]&0x04)>>2)<<6);
      Frame4[14]  = (unsigned char)(data%256);
      Frame4[15]  = (unsigned char)(data/256);

      
      RB_RGBLEDMATRIX_Write2Byte(0x77,CONFIGURE_CMD_PAGE, FRAME1_PAGE);//__LED_SelectFrame1Page;  
      RB_RGBLEDMATRIX_WriteNByte(0x77,0X00,Frame3, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x74,CONFIGURE_CMD_PAGE, FRAME1_PAGE);//__LED_SelectFrame1Page;  
      RB_RGBLEDMATRIX_WriteNByte(0x74,0X00,Frame1, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x77,CONFIGURE_CMD_PAGE, FRAME2_PAGE);//__LED_SelectFrame1Page;
      RB_RGBLEDMATRIX_WriteNByte(0x77,0X00,Frame4, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x74,CONFIGURE_CMD_PAGE, FRAME2_PAGE);//__LED_SelectFrame1Page;
      RB_RGBLEDMATRIX_WriteNByte(0x74,0X00,Frame2, TYPE3_LED_FRAME_LENGTH);
      
}


void RB_RGBLEDMATRIX::RGBLEDMATRIX_DATA(uint16_t *Display_data,uint8_t returndata1[16],uint8_t returndata2[16])
{

      uint16_t data;
      data = 0;
      data  |= (((Display_data[0]&0x01)<<0))|(((Display_data[0]&0x02)>>1)<<1)|(((Display_data[0]&0x04)>>2)<<2);
      data  |= (((Display_data[1]&0x01)<<3))|(((Display_data[1]&0x02)>>1)<<4)|(((Display_data[1]&0x04)>>2)<<5);
      data  |= (((Display_data[2]&0x01)<<6))|(((Display_data[2]&0x02)>>1)<<7)|(((Display_data[2]&0x04)>>2)<<8);
      data  |= (((Display_data[3]&0x01)<<9))|(((Display_data[3]&0x02)>>1)<<10)|(((Display_data[3]&0x04)>>2)<<11);
      data  |= (((Display_data[4]&0x01)<<12))|(((Display_data[4]&0x02)>>1)<<13)|(((Display_data[4]&0x04)>>2)<<14);

      returndata1[0]  = (unsigned char)(data%256);
      returndata1[1]  = (unsigned char)(data/256);
    
      data = 0;
     
      data  |= (((Display_data[7]&0x01)<<3))|(((Display_data[7]&0x02)>>1)<<4)|(((Display_data[7]&0x04)>>2)<<5);
      data  |= (((Display_data[8]&0x01)<<6))|(((Display_data[8]&0x02)>>1)<<7)|(((Display_data[8]&0x04)>>2)<<8);
      data  |= (((Display_data[9]&0x01)<<9))|(((Display_data[9]&0x02)>>1)<<10)|(((Display_data[9]&0x04)>>2)<<11);
      data  |= (((Display_data[10]&0x01)<<12))|(((Display_data[10]&0x02)>>1)<<13)|(((Display_data[10]&0x04)>>2)<<14);

      returndata1[2]  = (unsigned char)(data%256);
      returndata1[3]  =(unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data[6]&0x01)<<1))|(((Display_data[6]&0x02)>>1)<<0)|(((Display_data[6]&0x04)>>2)<<2);
      data  |= (((Display_data[13]&0x01)<<3))|(((Display_data[13]&0x02)>>1)<<4)|(((Display_data[13]&0x04)>>2)<<5);
      data  |= (((Display_data[14]&0x01)<<6))|(((Display_data[14]&0x02)>>1)<<7)|(((Display_data[14]&0x04)>>2)<<8);
      data  |= (((Display_data[15]&0x01)<<9))|(((Display_data[15]&0x02)>>1)<<10)|(((Display_data[15]&0x04)>>2)<<11);
      data  |= (((Display_data[5]&0x01)<<12))|(((Display_data[5]&0x02)>>1)<<13)|(((Display_data[5]&0x04)>>2)<<14);

      returndata1[4]  = (unsigned char)(data%256);
      returndata1[5]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[12]&0x01)<<1))|(((Display_data[12]&0x02)>>1)<<2)|(((Display_data[12]&0x04)>>2)<<15);
      data  |= (((Display_data[19]&0x01)<<3))|(((Display_data[19]&0x02)>>1)<<4)|(((Display_data[19]&0x04)>>2)<<5);
      data  |= (((Display_data[20]&0x01)<<6))|(((Display_data[20]&0x02)>>1)<<7)|(((Display_data[20]&0x04)>>2)<<8);
      data  |= (((Display_data[21]&0x01)<<9))|(((Display_data[21]&0x02)>>1)<<10)|(((Display_data[21]&0x04)>>2)<<11);
      data  |= (((Display_data[11]&0x01)<<12))|(((Display_data[11]&0x02)>>1)<<13)|(((Display_data[11]&0x04)>>2)<<14);

      returndata1[6]  = (unsigned char)(data%256);
      returndata1[7]  = (unsigned char)(data/256);
     
      data = 0;
      data  |= (((Display_data[18]&0x01)<<1))|(((Display_data[18]&0x02)>>1)<<2)|(((Display_data[18]&0x04)>>2)<<3);
      data  |= (((Display_data[26]&0x01)<<6))|(((Display_data[26]&0x02)>>1)<<7)|(((Display_data[26]&0x04)>>2)<<8);
      data  |= (((Display_data[16]&0x01)<<9))|(((Display_data[16]&0x02)>>1)<<10)|(((Display_data[16]&0x04)>>2)<<11);
      data  |= (((Display_data[17]&0x01)<<12))|(((Display_data[17]&0x02)>>1)<<13)|(((Display_data[17]&0x04)>>2)<<14);

      returndata1[8]  = (unsigned char)(data%256);
      returndata1[9]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[25]&0x01)<<4))|(((Display_data[25]&0x02)>>1))|(((Display_data[25]&0x04)>>2)<<5);
      data  |= (((Display_data[24]&0x01)<<1))|(((Display_data[24]&0x02)>>1)<<2)|(((Display_data[24]&0x04)>>2)<<3);
      data  |= (((Display_data[32]&0x01)<<6))|(((Display_data[32]&0x02)>>1)<<7)|(((Display_data[32]&0x04)>>2)<<8);
      data  |= (((Display_data[22]&0x01)<<9))|(((Display_data[22]&0x02)>>1)<<10)|(((Display_data[22]&0x04)>>2)<<11);
      data  |= (((Display_data[23]&0x01)<<12))|(((Display_data[23]&0x02)>>1)<<13)|(((Display_data[23]&0x04)>>2)<<14);

      returndata1[10]  = (unsigned char)(data%256);
      returndata1[11]  = (unsigned char)(data/256);
 

      data = 0;
      data  |= (((Display_data[30]&0x01)<<1))|(((Display_data[30]&0x02)>>1)<<2)|(((Display_data[30]&0x04)>>2)<<3);
      data  |= (((Display_data[31]&0x01)<<4))|(((Display_data[31]&0x02)>>1)<<5)|(((Display_data[31]&0x04)>>2)<<15);
      data  |= (((Display_data[27]&0x01)<<6))|(((Display_data[27]&0x02)>>1)<<7)|(((Display_data[27]&0x04)>>2)<<8);
      data  |= (((Display_data[28]&0x01)<<9))|(((Display_data[28]&0x02)>>1)<<10)|(((Display_data[28]&0x04)>>2)<<11);
      data  |= (((Display_data[29]&0x01)<<12))|(((Display_data[29]&0x02)>>1)<<13)|(((Display_data[29]&0x04)>>2)<<14);

      returndata1[12]  = (unsigned char)(data%256);
      returndata1[13]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[36]&0x01)<<1))|(((Display_data[36]&0x02)>>1)<<2)|(((Display_data[36]&0x04)>>2)<<3);
      data  |= (((Display_data[37]&0x01)<<4))|(((Display_data[37]&0x02)>>1)<<5)|(((Display_data[37]&0x04)>>2)<<6);
      data  |= (((Display_data[34]&0x01)<<9))|(((Display_data[34]&0x02)>>1)<<10)|(((Display_data[34]&0x04)>>2)<<11);
      data  |= (((Display_data[35]&0x01)<<12))|(((Display_data[35]&0x02)>>1)<<13)|(((Display_data[35]&0x04)>>2)<<14);

      returndata1[14]  = (unsigned char)(data%256);
      returndata1[15]  = (unsigned char)(data/256);
  

     
      data = 0;
      data  |= (((Display_data[42]&0x01)<<1))|(((Display_data[42]&0x02)>>1)<<2)|(((Display_data[42]&0x04)>>2)<<3);
      data  |= (((Display_data[43]&0x01)<<4))|(((Display_data[43]&0x02)>>1)<<5)|(((Display_data[43]&0x04)>>2)<<6);
      data  |= (((Display_data[33]&0x01)<<7))|(((Display_data[33]&0x02)>>1)<<0)|(((Display_data[33]&0x04)>>2)<<8);
      data  |= (((Display_data[40]&0x01)<<9))|(((Display_data[40]&0x02)>>1)<<10)|(((Display_data[40]&0x04)>>2)<<11);
      data  |= (((Display_data[41]&0x01)<<12))|(((Display_data[41]&0x02)>>1)<<13)|(((Display_data[41]&0x04)>>2)<<14);

      returndata2[0]  = (unsigned char)(data%256);
      returndata2[1]  = (unsigned char)(data/256);
     

      data = 0;
      data  |= (((Display_data[48]&0x01)<<1))|(((Display_data[48]&0x02)>>1)<<2)|(((Display_data[48]&0x04)>>2)<<3);
      data  |= (((Display_data[49]&0x01)<<4))|(((Display_data[49]&0x02)>>1)<<5)|(((Display_data[49]&0x04)>>2)<<6);
      data  |= (((Display_data[39]&0x01)<<7))|(((Display_data[39]&0x02)>>1)<<8)|(((Display_data[39]&0x04)>>2)<<15);
      data  |= (((Display_data[46]&0x01)<<9))|(((Display_data[46]&0x02)>>1)<<10)|(((Display_data[46]&0x04)>>2)<<11);
      data  |= (((Display_data[47]&0x01)<<12))|(((Display_data[47]&0x02)>>1)<<13)|(((Display_data[47]&0x04)>>2)<<14);

      returndata2[2]  = (unsigned char)(data%256);
      returndata2[3]  = (unsigned char)(data/256);
  
      data = 0;
      data  |= (((Display_data[54]&0x01)<<1))|(((Display_data[54]&0x02)>>1)<<2)|(((Display_data[54]&0x04)>>2)<<3);
      data  |= (((Display_data[38]&0x01)<<4))|(((Display_data[38]&0x02)>>1)<<5)|(((Display_data[38]&0x04)>>2)<<6);
      data  |= (((Display_data[45]&0x01)<<7))|(((Display_data[45]&0x02)>>1)<<8)|(((Display_data[45]&0x04)>>2)<<9);
      data  |= (((Display_data[53]&0x01)<<12))|(((Display_data[53]&0x02)>>1)<<13)|(((Display_data[53]&0x04)>>2)<<14);

      returndata2[4]  = (unsigned char)(data%256);
      returndata2[5]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data[60]&0x01)<<1))|(((Display_data[60]&0x02)>>1)<<2)|(((Display_data[60]&0x04)>>2)<<3);
      data  |= (((Display_data[44]&0x01)<<4))|(((Display_data[44]&0x02)>>1)<<5)|(((Display_data[44]&0x04)>>2)<<6);
      data  |= (((Display_data[51]&0x01)<<7))|(((Display_data[51]&0x02)>>1)<<8)|(((Display_data[51]&0x04)>>2)<<9);
      data  |= (((Display_data[52]&0x01)<<10))|(((Display_data[52]&0x02)>>1)<<0)|(((Display_data[52]&0x04)>>2)<<11);
      data  |= (((Display_data[59]&0x01)<<12))|(((Display_data[59]&0x02)>>1)<<13)|(((Display_data[59]&0x04)>>2)<<14);

      returndata2[6]  = (unsigned char)(data%256);
      returndata2[7]  = (unsigned char)(data/256);
 
      data = 0;
      data  |= (((Display_data[66]&0x01)<<1))|(((Display_data[66]&0x02)>>1)<<2)|(((Display_data[66]&0x04)>>2)<<3);
      data  |= (((Display_data[50]&0x01)<<4))|(((Display_data[50]&0x02)>>1)<<5)|(((Display_data[50]&0x04)>>2)<<6);
      data  |= (((Display_data[57]&0x01)<<7))|(((Display_data[57]&0x02)>>1)<<8)|(((Display_data[57]&0x04)>>2)<<9);
      data  |= (((Display_data[58]&0x01)<<10))|(((Display_data[58]&0x02)>>1)<<11)|(((Display_data[58]&0x04)>>2)<<15);
      data  |= (((Display_data[65]&0x01)<<12))|(((Display_data[65]&0x02)>>1)<<13)|(((Display_data[65]&0x04)>>2)<<14);

      returndata2[8]  = (unsigned char)(data%256);
      returndata2[9]  = (unsigned char)(data/256);
      
      data = 0;
      data  |= (((Display_data[55]&0x01)<<1))|(((Display_data[55]&0x02)>>1)<<2)|(((Display_data[55]&0x04)>>2)<<3);
      data  |= (((Display_data[56]&0x01)<<4))|(((Display_data[56]&0x02)>>1)<<5)|(((Display_data[56]&0x04)>>2)<<6);
      data  |= (((Display_data[63]&0x01)<<7))|(((Display_data[63]&0x02)>>1)<<8)|(((Display_data[63]&0x04)>>2)<<9);
      data  |= (((Display_data[64]&0x01)<<10))|(((Display_data[64]&0x02)>>1)<<11)|(((Display_data[64]&0x04)>>2)<<12);


      returndata2[10]  = (unsigned char)(data%256);
      returndata2[11]  = (unsigned char)(data/256);
   
      data = 0;
      data  |= (((Display_data[61]&0x01)<<1))|(((Display_data[61]&0x02)>>1)<<2)|(((Display_data[61]&0x04)>>2)<<3);
      data  |= (((Display_data[62]&0x01)<<4))|(((Display_data[62]&0x02)>>1)<<5)|(((Display_data[62]&0x04)>>2)<<6);
      data  |= (((Display_data[69]&0x01)<<7))|(((Display_data[69]&0x02)>>1)<<8)|(((Display_data[69]&0x04)>>2)<<9);
      data  |= (((Display_data[70]&0x01)<<10))|(((Display_data[70]&0x02)>>1)<<11)|(((Display_data[70]&0x04)>>2)<<12);
      data  |= (((Display_data[71]&0x01)<<13))|(((Display_data[71]&0x02)>>1)<<0)|(((Display_data[71]&0x04)>>2)<<14);

      returndata2[12]  = (unsigned char)(data%256);
      returndata2[13]  = (unsigned char)(data/256);

      data = 0;
      data  |= (((Display_data[67]&0x01)<<1))|(((Display_data[67]&0x02)>>1)<<2)|(((Display_data[67]&0x04)>>2)<<3);
      data  |= (((Display_data[68]&0x01)<<4))|(((Display_data[68]&0x02)>>1)<<5)|(((Display_data[68]&0x04)>>2)<<6);
      returndata2[14]  = (unsigned char)(data%256);
      returndata2[15]  = (unsigned char)(data/256);
      
}

void RB_RGBLEDMATRIX::RGBLEDMATRIX_Start(uint8_t d1[16],uint8_t d2[16],uint8_t d3[16],uint8_t d4[16])
{
      RB_RGBLEDMATRIX_Write2Byte(0x77,CONFIGURE_CMD_PAGE, FRAME1_PAGE);//__LED_SelectFrame1Page;  
      RB_RGBLEDMATRIX_WriteNByte(0x77,0X00,d3, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x74,CONFIGURE_CMD_PAGE, FRAME1_PAGE);//__LED_SelectFrame1Page;  
      RB_RGBLEDMATRIX_WriteNByte(0x74,0X00,d1, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x77,CONFIGURE_CMD_PAGE, FRAME2_PAGE);//__LED_SelectFrame1Page;
      RB_RGBLEDMATRIX_WriteNByte(0x77,0X00,d4, TYPE3_LED_FRAME_LENGTH);
      RB_RGBLEDMATRIX_Write2Byte(0x74,CONFIGURE_CMD_PAGE, FRAME2_PAGE);//__LED_SelectFrame1Page;
      RB_RGBLEDMATRIX_WriteNByte(0x74,0X00,d2, TYPE3_LED_FRAME_LENGTH);
}
