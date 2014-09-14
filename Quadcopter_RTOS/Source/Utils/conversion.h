/*
 * conversion.h
 *
 *  Created on: 31 août 2014
 */

#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <stdint.h>
#include <stdbool.h>

//------------------------------------------
// itoa:
// Int to char* conversion function.
//------------------------------------------
uint32_t itoa(int32_t value, char* buff);

//------------------------------------------
// itoa2:
// Int to char* conversion function with
// optional ending '\0'.
//------------------------------------------
uint32_t itoa2(int32_t value, char* buff, bool AddEndingZero);

//------------------------------------------
// ftoa:
// Float to char* conversion function.
//------------------------------------------
uint32_t ftoa(float value, char* buff, uint8_t DecimalCount);

//------------------------------------------
// ftoa2:
// Float to char* conversion function with
// optional ending '\0'.
//------------------------------------------
uint32_t ftoa2(float value, char* buff, uint8_t DecimalCount, bool AddEndingZero);

#endif /* CONVERSION_H_ */
