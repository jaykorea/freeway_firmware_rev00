/**
 * @file    ws2812.h
 * @author  Austin Glaser <austin.glaser@gmail.com>
 * @brief   Interface to WS2812 LED driver
 *
 * Copyright (C) 2016 Austin Glaser
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 *
 * @todo    Put in names and descriptions of variables which need to be defined to use this file
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/*
 * Configure zone
 * */
#include "stm32f4xx_hal.h"					//"stm32fXxx_hal.h" change X to your device

extern TIM_HandleTypeDef htim1;				//Change to your timer
extern DMA_HandleTypeDef hdma_tim1_ch1;		//Change to your timer and channel DMA

#define NUMBEROFLED 60						//Max led available

#define LEDPERZONE 2						//For rainbow mode
/*
 * Configure if you know what to change
 * */
#define WSBOFF		33
#define WSOFF		29

#define BUFFERLED NUMBEROFLED+2
#define STARTBUFFERLED 2
#define ENDBUFFERLED BUFFERLED
#define ZONE (NUMBEROFLED/LEDPERZONE)

typedef enum
{
	HALT = 0,
	START,
}_mode_t;

typedef struct
{
	uint8_t red;
	uint8_t blue;
	uint8_t green;
}color;

typedef enum
{
	NOTDEFINE = 0,
	WS2812,
	WS2812B
}type_led;

extern _mode_t mode;
extern color allrgb[BUFFERLED];

void init_neopixel(type_led in_type_of_led);
void all_black_render(void);
void render_neopixel(void);
void one_color_render(uint8_t blue,uint8_t red,uint8_t green);
void render_falling_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
void render_raising_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
void render_rainbow_cycle_mode(uint16_t delay);
void render_breath_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);

#endif //ifndef INC_WS2812_H_
