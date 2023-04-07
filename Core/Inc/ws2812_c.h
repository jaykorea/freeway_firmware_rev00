/*
 * ws2812_c.h
 *
 *  Created on: Jan 31, 2023
 *      Author: user
 */

#ifndef INC_WS2812_C_H_
#define INC_WS2812_C_H_

#include "stm32f4xx_hal.h"


#define WS2812_LED_NUMBER	(60u)

#define WS2812_PWM_VALUE_BIT_1	(54u)
#define WS2812_PWM_VALUE_BIT_0	(26u)


typedef enum
{
	WS2812_Color_Red							= ((uint32_t)0x00FF0000),
	WS2812_Color_Green						= ((uint32_t)0x0000FF00),
	WS2812_Color_Blue							= ((uint32_t)0x000000FF),
	WS2812_Color_Yellow						= ((uint32_t)0x00FFFF00),
	WS2812_Color_White						= ((uint32_t)0x00FFFFFF),
	WS2812_Color_Black						= ((uint32_t)0x00000000),
	WS2812_Color_Snow							= ((uint32_t)0x00FFFAFA),
	WS2812_Color_AliceBlue				= ((uint32_t)0x00F0F8FF),
	WS2812_Color_NavyBlue					= ((uint32_t)0x00000080),
	WS2812_Color_LightSkyBlue			= ((uint32_t)0x0087CEFA),
	WS2812_Color_DarkGreen				= ((uint32_t)0x00006400),
	WS2812_Color_Orange						= ((uint32_t)0x00FFA500),
	WS2812_Color_DeepPink					= ((uint32_t)0x00FF1493),
	WS2812_Color_DarkGrey					= ((uint32_t)0x00A9A9A9)
}	WS2812_Color;


extern uint16_t WS2812_Buffer[WS2812_LED_NUMBER*24 + 76];


void ws2812_init(void);
void ws2812_set_one_led_rgb(uint8_t red, uint8_t green, uint8_t blue, uint16_t number);
void ws2812_set_one_led_color(WS2812_Color color, uint16_t number);
void ws2812_set_all_led_rgb(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_set_all_led_color(WS2812_Color color);


#endif /* INC_WS2812_C_H_ */
