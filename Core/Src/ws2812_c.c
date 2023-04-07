/*
 * ws2812_c.c
 *
 *  Created on: Jan 31, 2023
 *      Author: user
 */


/**
  ******************************************************************************
  * @file           : ws2812.c
  * @brief          : ws2812����
  ******************************************************************************
  * @attention
  *		������������STM32F103C8T6 72MHZ ��Ƶ�£�GPIO��ת�ٶȸ���
  ******************************************************************************
  */

#include "ws2812_C.h"


uint16_t WS2812_Buffer[WS2812_LED_NUMBER*24 + 76] = {0};


/**
  * @brief  ws2812 init led buffer ram
  * @retval none
  */
void ws2812_init(void)
{
	uint16_t count;

	for(count = 0; count < sizeof(WS2812_Buffer)/sizeof(WS2812_Buffer[0]); count++)
	{
		WS2812_Buffer[count] = 0;
	}
}


/**
  * @brief  ws2812 set rgb of one led
	*	@param	red
	*	@param	green
	*	@param	blue
	*	@param	the number of led, start for 0
  * @retval none
  */
void ws2812_set_one_led_rgb(uint8_t red, uint8_t green, uint8_t blue, uint16_t number)
{
	uint16_t index = number*24;
	uint8_t offset = 0;

	if(number >= WS2812_LED_NUMBER)
		return;

	for(offset = 0; offset < 8; offset++)
	{
		if(green & (0x80 >> offset))
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_1;
		else
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_0;
	}

	for(offset = 0; offset < 8; offset++)
	{
		if(red & (0x80 >> offset))
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_1;
		else
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_0;
	}

	for(offset = 0; offset < 8; offset++)
	{
		if(blue & (0x80 >> offset))
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_1;
		else
			WS2812_Buffer[index++] = WS2812_PWM_VALUE_BIT_0;
	}
}


/**
  * @brief  ws2812 set color of one led
	*	@param	color
	*	@param	the number of led, start for 0
  * @retval none
  */
void ws2812_set_one_led_color(WS2812_Color color, uint16_t number)
{
	ws2812_set_one_led_rgb((uint8_t)((color >> 16)&0x000000FF), (uint8_t)((color >> 8)&0x000000FF), (uint8_t)((color >> 0)&0x000000FF), number);
}


/**
  * @brief  ws2812 set rgb of all led
	*	@param	red
	*	@param	green
	*	@param	blue
  * @retval none
  */
void ws2812_set_all_led_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	uint16_t number;

	for(number = 0; number < WS2812_LED_NUMBER; number++)
	{
		ws2812_set_one_led_rgb(red, green, blue, number);
	}
}


/**
  * @brief  ws2812 set color of all led
	*	@param	color
  * @retval none
  */
void ws2812_set_all_led_color(WS2812_Color color)
{
	uint16_t number;

	for(number = 0; number < WS2812_LED_NUMBER; number++)
	{
		ws2812_set_one_led_color(color, number);
	}
}
