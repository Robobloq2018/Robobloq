#ifndef RB_RGBLED_H_
#define RB_RGBLED_H_


#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 

#define RB_MAX_LED_NUMBER  (32)
#define LED_Pin    39

struct RB_RGB
{
      uint8_t g;
      uint8_t r;
      uint8_t b;
 };




class RB_RGBLed
{
   public:
       /*
        * 
        */
       RB_RGBLed(uint8_t port);
       /*
        * 
        */
       RB_RGBLed(uint8_t port,uint8_t led_number);
       /*
        * 
        */
       ~RB_RGBLed(void);
       /*
        * 
        */
        void setpin(uint8_t port);
        /*
         * 
         */
        setLedNumer(uint8_t led_number);
        /*
         * 
         */
        RB_RGB getColorAt(uint8_t index);
         /*
          * 
          */
        uint16_t getNumber(void);
          /*
           * 
           */
        bool setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
         /*
          * 
          */
        bool setColor(uint8_t red, uint8_t green, uint8_t blue);
        /*    
         *     
         */
        bool setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
        /*
         * 
         */
        bool RB_RGBLed::setColor(uint8_t index, long value);
        /*
         * 
         */
         void RB_RGBLed::rgbled_sendarray_mask(uint8_t *data, uint16_t datlen, uint8_t maskhi, uint8_t *port);
         /*
          * 
          */
         void show(void);
  private:
      const volatile uint8_t *ws2812_port;
      volatile uint8_t *ws2812_port_reg;
      uint8_t pinMask;  
      uint16_t count_led;
      uint8_t *pixels;
  
  
};
 


#endif 
