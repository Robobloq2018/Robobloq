#ifndef RB_RGBLEDMatrix_H
#define RB_RGBLEDMatrix_H

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"

#define   Slave_ADDRESSADGND    0X74
#define   Slave_ADDRESSADVCC    0X77


/*********************/
/*----------------Rehisters Page Define----------------*/
    #define	CONFIGURE_CMD_PAGE 0XFD
		#define	FRAME1_PAGE 0X00
		#define	FRAME2_PAGE 0X01
		#define	FUNCTION_PAGE 0X0B
		#define	DETECTION_PAGE 0X0C
		#define	LED_VAF_PAGE 0X0D
/*--------Registers Define------------*/		
		#define	CONFIGURATION_REG 0X00
		#define	PICTURE_DISPLAY_REG 0X01
		#define	DISPLAY_OPTION_REG 0X05
		#define	BREATH_CTL_REG 0X08
		#define	BREATH_CTL_REG2 0X09
		#define	SW_SHUT_DOWN_REG 0X0A	
		
		#define	AUDIO_GAIN_CTL_REG 0X0B
		#define	STAGGERED_DELAY_REG 0X0D
		#define	SLEW_RATE_CTL_REG 0X0E
		#define	CURRENT_CTL_REG 0X0F
		#define	OPEN_SHORT_REG 0X10				
		#define	OPEN_SHORT_REG2 0X11			
		#define	VAF_CTL_REG 0X14			
		#define	VAF_CTL_REG2 0X15	
		#define	TYPE4_VAF_REG1 0X18	
		#define	TYPE4_VAF_REG2 0X19
		#define	TYPE4_VAF_REG3 0X1A

/*--------Function Register: address 00H Configuration Reg------------*/	
		#define	mskDISPLAY_MODE_EN						(0X0<<3)		
		#define	mskDISPLAY_MODE_DIS					(0X1<<3)		//disable PWM output
		#define	mskSHOLD_EN									(0x1<<5)	
		#define	mskSHOLD_DIS								(0x0<<5)	
		#define	mskSYNC_HIGH_IMPEDANCE				(0x0<<6)
		#define	mskSYNC_MASTER_MODE					(0x1<<6)
		#define	mskSYNC_SLAVE_MODE						(0x2<<6)	
		
	/*--------Function Register: address 01H Picture Display Reg------------*/			
		#define	mskMATRIX_TYPE_TYPE1						(0x0<<3)	
		#define	mskMATRIX_TYPE_TYPE2						(0x1<<3)					
		#define	mskMATRIX_TYPE_TYPE3						(0x2<<3)				
		#define	mskMATRIX_TYPE_TYPE4						(0x3<<3)	

	/*--------Function Register: address 05H Display Option Reg------------*/
		#define	mskBLINK_PERIOD_TIME_CONST			(0x7<<0)	
		#define	mskBLINK_EN								(0x1<<3)
		#define	mskBLINK_DIS							(0x0<<3)			
		#define	mskBLINK_FRAME_300					(0x0<<6)			
		#define	mskBLINK_FRAME_600					(0x1<<6)	
		#define	mskBLINK_FRAME_75					(0x2<<6)	
		#define	mskBLINK_FRAME_150					(0x3<<6)	
		
	/*--------Function Register: address 08H Breath Control Reg------------*/
		#define	mskBC_FADE_IN_TIME_CONST					(0x7<<0)
		#define	mskBC_FADE_OUT_TIME_CONST				(0x7<<4)	
		
	/*--------Function Register: address 09H Breath Control Reg2------------*/	
		#define	mskBC_EXTINGUISH_TIME_CONST			(0x7<<0)		
		#define	mskBC_BREATH_EN								(0x1<<4)	
		#define	mskBC_BREATH_DIS								(0x0<<4)			
		#define	mskBC_CONTINUOUS_BREATH_EN			(0x1<<5)	
		#define	mskBC_CONTINUOUS_BREATH_DIS			(0x0<<5)	

	/*--------Function Register: address 0AH Shutdown Reg------------*/	
		#define	mskSW_SHUT_DOWN_MODE					(0x0<<0)
		#define	mskSW_NORMAL_MODE						(0x1<<0)	
	
	/*--------Function Register: address 0BH AGC Control Reg------------*/	
		#define	mskAGS_CONST							(0x7<<0)
		#define	mskAGC_EN									(0x0<<3)
		#define	mskAGC_DIS								(0x1<<3)	
		#define	mskAGC_MODE_SLOW						(0x0<<4)
		#define	mskAGC_MODE_FAST						(0x1<<4)	
		
	/*--------Function Register: address 0DH Staggered Delay Reg------------*/	
		#define	mskSTD1				(0x3<<0)	
		#define	mskSTD2				(0x3<<2)
		#define	mskSTD3				(0x3<<4)	
		#define	mskSTD4				(0x3<<6)
		
		#define	CONST_STD_GROUP1	0x00		
		#define	CONST_STD_GROUP2	0x55	
		#define	CONST_STD_GROUP3	0xAA		
		#define	CONST_STD_GROUP4	0xFF		
	
	/*--------Function Register: address 0EH Slew Rate Control Reg------------*/	
		#define	mskSLEW_RATE_CTL_EN				(0x1<<0)	
		#define	mskSLEW_RATE_CTL_DIS			(0x0<<0)	

	/*--------Function Register: address 0FH Current Control Reg------------*/
		#define	mskCURRENT_STEP_CONST		(0x3F<<0)		
		#define	CONST_CURRENT_STEP_10mA			(0x5<<0)
		#define	CONST_CURRENT_STEP_15mA			(0xF<<0)
		#define	CONST_CURRENT_STEP_20mA			(0x19<<0)
		#define	CONST_CURRENT_STEP_25mA			(0x23<<0)		
		#define	CONST_CURRENT_STEP_32mA			(0x31<<0)
		#define	CONST_CURRENT_STEP_40mA			(0x3F<<0)

		#define	mskCURRENT_CTL_EN				(0x1<<7)	
		#define	mskCURRENT_CTL_DIS				(0x0<<7)	

	/*--------Function Register: address 10H Open/Short Test Reg------------*/	
		#define	mskOPEN_SHORT_DETECT_DUTY	(0x3F<<0)	
		#define	mskSHORT_DETECT_START		(0x1<<6) // HW will clear this bits automatically	
		#define	mskOPEN_DETECT_START			(0x1<<7) // HW will clear this bits automatically		
		
	/*--------Function Register: address 11H Open/Short Test Reg2------------*/	
		#define	mskSHORT_DETECT_INT		(0x1<<6) //when open/short detect done, HW will set this bit as 1 automatically	
		#define	mskOPEN_DETECT_INT			(0x1<<7) //when open/short detect done, HW will set this bit as 1 automatically		

	/*--------Function Register: address 14H VAF Reg1 ------------*/
		#define	Pakeage_Type Pakeage_Type_EV
		//////////////////////////
		#define	Pakeage_Type_EV	0
		#define	Pakeage_Type_MP	1	
		//////////////////////////
	
		#if (Package_Type == Package_Type_MP)	
				#define	mskVAF1										(0x4<<0) 		
				#define	mskVAF2										(0x4<<4)		
		#else 		
				#define	mskVAF1										(0x0<<0) 	
				#define	mskVAF2										(0x0<<4) 		
		#endif
 		
	/*--------Function Register: address 15H VAF Reg2 ------------*/	
		#if (Package_Type == Package_Type_MP)	
				#define	mskVAF3										(0x4<<0) 		
		#else 		
				#define	mskVAF3										(0x7<<0) 				
		#endif
		
		#define	mskTP4VAF3								(0x0<<0) 			
		
		#define	mskFORCEVAFTIME_CONST			(0x0<<3) 			
		#define	mskFORCEVAFCTL_ALWAYSON		(0x0<<6) 	
		#define	mskFORCEVAFCTL_VAFTIMECTL	(0x1<<6) 	
		#define	mskFORCEVAFCTL_DISABLE		(0x2<<6) 	
		
	/*--------Function Register: address 18H Type 4 VAF Reg1 ------------*/	
		#define	mskTYPE4_CB4_VAF2									(0x0<<0) 
		#define	mskTYPE4_CB4_VAF1									(0x1<<0) 	
		#define	mskTYPE4_CB4_VAF3									(0x2<<0) 		
		#define	mskTYPE4_CB4_VAFFLOATING						(0x3<<0) 	
		
		#define	mskTYPE4_CB5_VAF2									(0x0<<2) 
		#define	mskTYPE4_CB5_VAF1									(0x1<<2) 	
		#define	mskTYPE4_CB5_VAF3									(0x2<<2) 		
		#define	mskTYPE4_CB5_VAFFLOATING						(0x3<<2) 	
		
		#define	mskTYPE4_CB6_VAF2									(0x0<<4) 
		#define	mskTYPE4_CB6_VAF1									(0x1<<4) 	
		#define	mskTYPE4_CB6_VAF3									(0x2<<4) 		
		#define	mskTYPE4_CB6_VAFFLOATING						(0x3<<4) 	
	
		#define	mskTYPE4_CB7_VAF2									(0x0<<6) 
		#define	mskTYPE4_CB7_VAF1									(0x1<<6) 	
		#define	mskTYPE4_CB7_VAF3									(0x2<<6) 		
		#define	mskTYPE4_CB7_VAFFLOATING						(0x3<<6) 
		
	/*--------Function Register: address 19H Type 4 VAF Reg2 ------------*/	
		#define	mskTYPE4_CB8_VAF2									(0x0<<0) 
		#define	mskTYPE4_CB8_VAF1									(0x1<<0) 	
		#define	mskTYPE4_CB8_VAF3									(0x2<<0) 		
		#define	mskTYPE4_CB8_VAFFLOATING						(0x3<<0) 	
		
		#define	mskTYPE4_CB9_VAF2									(0x0<<2) 
		#define	mskTYPE4_CB9_VAF1									(0x1<<2) 	
		#define	mskTYPE4_CB9_VAF3									(0x2<<2) 		
		#define	mskTYPE4_CB9_VAFFLOATING						(0x3<<2) 	
		
		#define	mskTYPE4_CC1_VAF2									(0x0<<4) 
		#define	mskTYPE4_CC1_VAF1									(0x1<<4) 	
		#define	mskTYPE4_CC1_VAF3									(0x2<<4) 		
		#define	mskTYPE4_CC1_VAFFLOATING						(0x3<<4) 	
	
		#define	mskTYPE4_CC2_VAF2									(0x0<<6) 
		#define	mskTYPE4_CC2_VAF1									(0x1<<6) 	
		#define	mskTYPE4_CC2_VAF3									(0x2<<6) 		
		#define	mskTYPE4_CC2_VAFFLOATING						(0x3<<6)
 		
	/*--------Function Register: address 1AH Type 4 VAF Reg3 ------------*/	
		#define	mskTYPE4_CC3_VAF2									(0x0<<0) 
		#define	mskTYPE4_CC3_VAF1									(0x1<<0) 	
		#define	mskTYPE4_CC3_VAF3									(0x2<<0) 		
		#define	mskTYPE4_CC3_VAFFLOATING					(0x3<<0) 	
		
		#define	mskTYPE4_CC4_VAF2									(0x0<<2) 
		#define	mskTYPE4_CC4_VAF1									(0x1<<2) 	
		#define	mskTYPE4_CC4_VAF3									(0x2<<2) 		
		#define	mskTYPE4_CC4_VAFFLOATING						(0x3<<2) 	
		
		#define	mskTYPE4_CC5_VAF2									(0x0<<4) 
		#define	mskTYPE4_CC5_VAF1									(0x1<<4) 	
		#define	mskTYPE4_CC5_VAF3									(0x2<<4) 		
		#define	mskTYPE4_CC5_VAFFLOATING						(0x3<<4) 	
	
		#define	mskTYPE4_CC6_VAF2									(0x0<<6) 
		#define	mskTYPE4_CC6_VAF1									(0x1<<6) 	
		#define	mskTYPE4_CC6_VAF3									(0x2<<6) 		
		#define	mskTYPE4_CC6_VAFFLOATING						(0x3<<6) 					
		
/*----------------------------------------------------------*/
/*											Frame1Page													*/
/*----------------------------------------------------------*/
		#define	mskLED_FRAME_REG_ADDR									(0xFF<<0) 
			#define	TYPE3_FRAME1PAGE_LENGTH							((TYPE3_PWM_FRAME_LAST_ADDR-TYPE3_LED_FRAME_FIRST_ADDR)+1)			
			#define	TYPE3_FRAME2PAGE_LENGTH							((TYPE3_PWM_FRAME2_LAST_ADDR-TYPE3_LED_FRAME2_FIRST_ADDR)+1)			
			#define	TYPE1_2_4_FRAME1PAGE_LENGTH					((TYPE1_PWM_FRAME_LAST_ADDR-TYPE1_LED_FRAME_FIRST_ADDR)+1)					
		
			//=======================//
			// LED Control Registers //
			//=======================//
			//-------Frame1----------//
			#define	TYPE1_LED_FRAME_FIRST_ADDR					0x00	
			#define	TYPE1_LED_FRAME_LAST_ADDR						0x11								 	
			#define	TYPE1_LED_FRAME_LENGTH					((TYPE1_LED_FRAME_LAST_ADDR-TYPE1_LED_FRAME_FIRST_ADDR)+1)

			#define	TYPE2_LED_FRAME_FIRST_ADDR					0x00	
			#define	TYPE2_LED_FRAME_LAST_ADDR						0x11								 	
			#define	TYPE2_LED_FRAME_LENGTH					((TYPE2_LED_FRAME_LAST_ADDR-TYPE2_LED_FRAME_FIRST_ADDR)+1)

			#define	TYPE3_LED_FRAME_FIRST_ADDR					0x00	
			#define	TYPE3_LED_FRAME_LAST_ADDR						0x0F								 	
			#define	TYPE3_LED_FRAME_LENGTH					((TYPE3_LED_FRAME_LAST_ADDR-TYPE3_LED_FRAME_FIRST_ADDR)+1)
			
			#define	TYPE4_LED_FRAME_FIRST_ADDR					0x00	
			#define	TYPE4_LED_FRAME_LAST_ADDR						0x11								 	
			#define	TYPE4_LED_FRAME_LENGTH					((TYPE4_LED_FRAME_LAST_ADDR-TYPE4_LED_FRAME_FIRST_ADDR)+1)
			
			//=========================//
			// Blink Control Registers //
			//=========================//	
			//-------Frame1----------//			
		#define	mskBLINK_FRAME_REG_ADDR								(0xFF<<0) 
			#define	TYPE1_BLINK_FRAME_FIRST_ADDR					0x12	
			#define	TYPE1_BLINK_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE1_BLINK_FRAME_LENGTH					((TYPE1_BLINK_FRAME_LAST_ADDR-TYPE1_BLINK_FRAME_FIRST_ADDR)+1)	

			#define	TYPE2_BLINK_FRAME_FIRST_ADDR					0x12	
			#define	TYPE2_BLINK_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE2_BLINK_FRAME_LENGTH					((TYPE2_BLINK_FRAME_LAST_ADDR-TYPE2_BLINK_FRAME_FIRST_ADDR)+1)

			#define	TYPE3_BLINK_FRAME_FIRST_ADDR					0x10	
			#define	TYPE3_BLINK_FRAME_LAST_ADDR						0x1F								 	
			#define	TYPE3_BLINK_FRAME_LENGTH					((TYPE3_BLINK_FRAME_LAST_ADDR-TYPE3_BLINK_FRAME_FIRST_ADDR)+1)	
			
			#define	TYPE4_BLINK_FRAME_FIRST_ADDR					0x12	
			#define	TYPE4_BLINK_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE4_BLINK_FRAME_LENGTH					((TYPE4_BLINK_FRAME_LAST_ADDR-TYPE4_BLINK_FRAME_FIRST_ADDR)+1)	
			
			//=======================//
			// PWM Control Registers //
			//=======================//	
			//-------Frame1----------//					
		#define	mskPWM_FRAME_REG_ADDR									(0xFF<<0) 
			#define	TYPE1_PWM_FRAME_FIRST_ADDR					0x24	
			#define	TYPE1_PWM_FRAME_LAST_ADDR						0xB3								 	
			#define	TYPE1_PWM_FRAME_LENGTH					((TYPE1_PWM_FRAME_LAST_ADDR-TYPE1_PWM_FRAME_FIRST_ADDR)+1)	
	
			#define	TYPE1_RGB_ANODE_PWM_LENGTH					0x24 //Total control 36 RGB LEDs  			
			#define	TYPE1_RGB_CATHODE_PWM_LENGTH				0x24 //Total control 36 RGB LEDs  
			
			#define	TYPE2_PWM_FRAME_FIRST_ADDR					0x24	
			#define	TYPE2_PWM_FRAME_LAST_ADDR						0xB3								 	
			#define	TYPE2_PWM_FRAME_LENGTH					((TYPE2_PWM_FRAME_LAST_ADDR-TYPE2_PWM_FRAME_FIRST_ADDR)+1)

			#define	TYPE2_RGB_ANODE_PWM_LENGTH					0x28 //Total control 40 RGB LEDs  			
			#define	TYPE2_RGB_CATHODE_PWM_LENGTH				0x29 //Total control 41 RGB LEDs  		
			
			#define	TYPE3_PWM_FRAME_FIRST_ADDR					0x20	
			#define	TYPE3_PWM_FRAME_LAST_ADDR						0x9F								 	
			#define	TYPE3_PWM_FRAME_LENGTH							((TYPE3_PWM_FRAME_LAST_ADDR-TYPE3_PWM_FRAME_FIRST_ADDR)+1)	

			#define	TYPE3_RGB_ANODE_PWM_LENGTH								0x46//Total control 70 RGB LEDs  	
			#define	TYPE3_RGB_ANODE_PWM_LENGTH_FRAME1					0x1C 
			#define	TYPE3_RGB_ANODE_PWM_LENGTH_FRAME1_2				0x2A 	  	
																		
			#define	TYPE3_RGB_CATHODE_PWM_FRAME1_LENGTH				0x25  	
			#define	TYPE3_RGB_CATHODE_PWM_FRAME2_LENGTH				0x4B //Total control 75 RGB LEDs  	
			
			#define	TYPE4_PWM_FRAME_FIRST_ADDR					0x24	
			#define	TYPE4_PWM_FRAME_LAST_ADDR						0xB3								 	
			#define	TYPE4_PWM_FRAME_LENGTH					((TYPE4_PWM_FRAME_LAST_ADDR-TYPE4_PWM_FRAME_FIRST_ADDR)+1)
			
			#define	TYPE4_RGB_ANODE_PWM_LENGTH					0x30 //Total control 48 RGB LEDs  			
			#define	TYPE4_RGB_CATHODE_PWM_LENGTH				0x30 //Total control 48 RGB LEDs  		
/*----------------------------------------------------------*/
/*											Frame2Page													*/
/*----------------------------------------------------------*/	
			//=======================//
			// LED Control Registers //
			//=======================//
			//-------Frame2----------//		
		#define	mskLED_FRAME2_REG_ADDR								(0xFF<<0) 		
			#define	TYPE3_LED_FRAME2_FIRST_ADDR					0x00	
			#define	TYPE3_LED_FRAME2_LAST_ADDR					0x0F								 	
			#define	TYPE3_LED_FRAME2_LENGTH				((TYPE3_LED_FRAME2_LAST_ADDR-TYPE3_LED_FRAME2_FIRST_ADDR)+1)
					
			//=========================//
			// Blink Control Registers //
			//=========================//	
			//-------Frame2------------//			
		#define	mskBLINK_FRAME2_REG_ADDR								(0xFF<<0) 	
			#define	TYPE3_BLINK_FRAME2_FIRST_ADDR					0x10	
			#define	TYPE3_BLINK_FRAME2_LAST_ADDR					0x1F								 	
			#define	TYPE3_BLINK_FRAME2_LENGTH				((TYPE3_BLINK_FRAME2_LAST_ADDR-TYPE3_BLINK_FRAME2_FIRST_ADDR)+1)		
			
			//=======================//
			// PWM Control Registers //
			//=======================//	
			//-------Frame2----------//				
		#define	mskPWM_FRAME2_REG_ADDR								(0xFF<<0) 		
			#define	TYPE3_PWM_FRAME2_FIRST_ADDR					0x20	
			#define	TYPE3_PWM_FRAME2_LAST_ADDR					0x9F								 	
			#define	TYPE3_PWM_FRAME2_LENGTH				((TYPE3_PWM_FRAME2_LAST_ADDR-TYPE3_PWM_FRAME2_FIRST_ADDR)+1)		
			
/*----------------------------------------------------------*/
/*											Frame Data 													*/
/*----------------------------------------------------------*/			
	/*--------Frame Register: Data ------------*/	
		#define	mskLED_FRAME_REG_DATA									(0xFF<<0) 
			#define	TYPE1_LED_FRAME_CLR_DATA						0x00	
			#define	TYPE1_LED_FRAME_MAX_DATA						0xFF 	
	
			#define	TYPE2_LED_FRAME_CLR_DATA						0x00	
			#define	TYPE2_LED_FRAME_MAX_DATA						0xFF 

			#define	TYPE3_LED_FRAME_CLR_DATA						0x00	
			#define	TYPE3_LED_FRAME_MAX_DATA						0xFF 	
			
			#define	TYPE4_LED_FRAME_CLR_DATA						0x00	
			#define	TYPE4_LED_FRAME_MAX_DATA						0xFF 
					
		#define	mskBLINK_FRAME_REG_DATA								(0xFF<<0) 	
			#define	TYPE1_BLINK_FRAME_CLR_DATA					0x00	
			#define	TYPE1_BLINK_FRAME_MAX_DATA					0xFF
	
			#define	TYPE2_BLINK_FRAME_CLR_DATA					0x00	
			#define	TYPE2_BLINK_FRAME_MAX_DATA					0xFF 	

			#define	TYPE3_BLINK_FRAME_CLR_DATA					0x00	
			#define	TYPE3_BLINK_FRAME_MAX_DATA					0xFF 
 			
			#define	TYPE4_BLINK_FRAME_CLR_DATA					0x00	
			#define	TYPE4_BLINK_FRAME_MAX_DATA					0xFF 	
						
		#define	mskPWM_FRAME_REG_DATA									(0xFF<<0)
			#define	TYPE1_PWM_FRAME_CLR_DATA						0x00	
			#define	TYPE1_PWM_FRAME_MAX_DATA						0xFF 
			
			#define	TYPE2_PWM_FRAME_CLR_DATA						0x00	
			#define	TYPE2_PWM_FRAME_MAX_DATA						0xFF 			
			
			#define	TYPE3_PWM_FRAME_CLR_DATA						0x00	
			#define	TYPE3_PWM_FRAME_MAX_DATA						0xFF 
			
			#define	TYPE4_PWM_FRAME_CLR_DATA						0x00	
			#define	TYPE4_PWM_FRAME_MAX_DATA						0xFF  		

/*----------------------------------------------------------*/
/*											FrameVAF1Page												*/
/*----------------------------------------------------------*/	
			//-------Frame1----------//
			#define	TYPE1_VAF_FRAME_FIRST_ADDR					0x00	
			#define	TYPE1_VAF_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE1_VAF_FRAME_LENGTH					((TYPE1_VAF_FRAME_LAST_ADDR-TYPE1_VAF_FRAME_FIRST_ADDR)+1)

			#define	TYPE2_VAF_FRAME_FIRST_ADDR					0x00	
			#define	TYPE2_VAF_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE2_VAF_FRAME_LENGTH					((TYPE2_VAF_FRAME_LAST_ADDR-TYPE2_VAF_FRAME_FIRST_ADDR)+1)

			#define	TYPE3_VAF_FRAME_FIRST_ADDR					0x00	
			#define	TYPE3_VAF_FRAME_LAST_ADDR						0x3F								 	
			#define	TYPE3_VAF_FRAME_LENGTH					((TYPE3_VAF_FRAME_LAST_ADDR-TYPE3_VAF_FRAME_FIRST_ADDR)+1)
			
			#define	TYPE4_VAF_FRAME_FIRST_ADDR					0x00	
			#define	TYPE4_VAF_FRAME_LAST_ADDR						0x23								 	
			#define	TYPE4_VAF_FRAME_LENGTH					((TYPE4_VAF_FRAME_LAST_ADDR-TYPE4_VAF_FRAME_FIRST_ADDR)+1)

/*_____ M A C R O S ________________________________________________________*/

/*********Marco function**************
	//Send Configure Command 0xFD and Select to Frame 1 Register(Page1, value = 0x00)
	#define __LED_SelectFrame1Page  (I2C_W_2BYTE(CONFIGURE_CMD_PAGE, FRAME1_PAGE))		
	//Send Configure Command 0xFD and Select to Frame 2 Register(Page2, value = 0x01)
	#define	__LED_SelectFrame2Page	(I2C_W_2BYTE(CONFIGURE_CMD_PAGE, FRAME2_PAGE))		
	//Send Configure Command 0xFD and Select to Function Register(Page9, value = 0x0B)
	#define	__LED_SelectFunctionPage	(I2C_W_2BYTE(CONFIGURE_CMD_PAGE, FUNCTION_PAGE))	
	//Send Configure Command 0xFD and Select to Function Register(Page10, value = 0x0C)
	#define	__LED_SelectDetectionPage		(I2C_W_2BYTE(CONFIGURE_CMD_PAGE, DETECTION_PAGE))	
	//Send Configure Command 0xFD and Select to LED VAF Register(Page11, value = 0x0D)
	#define	__LED_SelectLedVafPage		(I2C_W_2BYTE(CONFIGURE_CMD_PAGE, LED_VAF_PAGE))
	
******************************/   
  

class  RB_RGBLEDMATRIX:public RB_SoftI2CMaster
{
	 public :
	  RB_RGBLEDMATRIX(void);
		RB_RGBLEDMATRIX(uint8_t port);
		void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_Write2Byte(uint8_t address,uint8_t rgaddress,uint8_t data);
		void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_WriteNByte(uint8_t address,uint8_t rgaddress,uint8_t *data,unsigned short datalen);
		void RB_RGBLEDMATRIX::LED_Type3ClearFrame1Page(unsigned char address);
		void RB_RGBLEDMATRIX::LED_Type3ClearFrame2Page(unsigned char address);
		void RB_RGBLEDMATRIX::RB_RGBLEDMATRIX_Init(unsigned char address);
		void RB_RGBLEDMATRIX::LED_SledType3Fun1(unsigned char address);
        void RB_RGBLEDMATRIX::LED_SledType3Fun2(unsigned char address);
        void RB_RGBLEDMATRIX::RGBLEDMATRIX_DISPALY1(uint16_t *Display_data,uint16_t *Display_data1);
        void RB_RGBLEDMATRIX::RGBLEDMATRIX_DATA(uint16_t *Display_data,uint8_t returndata1[16],uint8_t returndata2[16]);
        void RB_RGBLEDMATRIX::RGBLEDMATRIX_Start(uint8_t d1[16],uint8_t d2[16],uint8_t d3[16],uint8_t d4[16]);
        
};





#endif
