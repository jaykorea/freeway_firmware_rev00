/*
 * mainpp.h
 *
 *  Created on: Sep 16, 2022
 *      Author: van
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

long map(uint32_t a, long b, long c, long d, long e);
void setup(void);
uint8_t loop(uint32_t x_val, uint32_t y_val, double r_val, double l_val);
#ifdef __cplusplus
}
#endif

#endif /* INC_MAINPP_H_ */
