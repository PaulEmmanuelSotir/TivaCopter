/*
 * utils.h
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stdbool.h>

//----------------------------------------
// Signed and unsigned values saturation
//----------------------------------------
#define U_SAT(val, max) val = (val > max ? max : (val < 0 ? 0 : val))
#define SAT(val, extremum) val = (val > extremum ? extremum : (val < -extremum ? -extremum : val))

//------------------------------------------
// Constants defines
//------------------------------------------
#define PI							3.14159265358979323846
#define G							9.80665

//----------------------------------------
// Enum allowing clearer data accesses
//----------------------------------------
enum { x, y, z };

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

//-----------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//-----------------------------------------------------------
float invSqrt(float x);

#endif /* UTILS_H_ */
