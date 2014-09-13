/*
 * conversion.h
 *
 *  Created on: 31 août 2014
 *      Author: paule_000
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <stdint.h>
#include <stdbool.h>

uint32_t itoa2(int value, char buff[], bool AddEndingZero);
uint32_t itoa(int value, char buff[]);

uint32_t ftoa(float value, char buff[], uint8_t MaxDecimalCount);

#endif /* CONVERSION_H_ */
