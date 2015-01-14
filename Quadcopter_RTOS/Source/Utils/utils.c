/*
 * utils.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "math.h"
#include "utils.h"

static uint32_t uitoaReal(int32_t value, char* buff, bool AddEndingZero)
{
	static char const digit[] = "0123456789";
	char* It = buff;

	// Move to where representation ends
	if(value == 0)
		It++;
	else
	{
		uint32_t shifter;
		for(shifter = value; shifter != 0; shifter /= 10)
			It++;
	}

	// Get final string length
	uint32_t length = It - buff;

	if(AddEndingZero)
		*It = '\0';

	// Move back, inserting digits as you go
	if(value == 0)
		*--It = '0';
	else
	{
		while(value)
		{
			*--It = digit[value % 10];
			value /= 10;
		}
	}

	return length;
}

//------------------------------------------
// itoa:
// Int to char* conversion function.
//------------------------------------------
uint32_t itoa(int32_t value, char* buff)
{
	return itoa2(value, buff, true);
}

//------------------------------------------
// itoa2:
// Int to char* conversion function with
// optional ending '\0'.
//------------------------------------------
uint32_t itoa2(int32_t value, char* buff, bool AddEndingZero)
{
	if(isnan(value))
	{
		buff[0] = 'N';
		buff[1] = 'a';
		buff[2] = 'N';
		return 3;
	}
	else if(isinf(value))
	{
		buff[0] = value < 0 ? '-' : '+';
		buff[1] = 'i';
		buff[2] = 'n';
		buff[3] = 'f';
		return 4;
	}

	if(value < 0)
	{
		*buff++ = '-';
		value  *= -1;

		return uitoaReal(value, buff, AddEndingZero) + 1;
	}

	return uitoaReal(value, buff, AddEndingZero);
}

//------------------------------------------
// ftoa:
// Float to char* conversion function.
//------------------------------------------
uint32_t ftoa(float value, char* buff, uint8_t DecimalCount)
{
	return ftoa2(value, buff, DecimalCount, true);
}

//------------------------------------------
// ftoa2:
// Float to char* conversion function with
// optional ending '\0'.
//------------------------------------------
uint32_t ftoa2(float value, char* buff, uint8_t DecimalCount, bool AddEndingZero)
{
	uint32_t length = 0;
	static char const digit[] = "0123456789";

	if(isnan(value))
	{
		buff[0] = 'N';
		buff[1] = 'a';
		buff[2] = 'N';
		return 3;
	}
	else if(isinf(value))
	{
		buff[0] = value < 0 ? '-' : '+';
		buff[1] = 'i';
		buff[2] = 'n';
		buff[3] = 'f';
		return 4;
	}

	if(value < 0)
	{
		buff[length++] = '-';
		value  *= -1;
	}

	uint32_t intValue;
	if(DecimalCount != 0)
		intValue = (uint32_t)value;
	else
		intValue = (uint32_t)(value + 0.5);

	length += uitoaReal(intValue, buff + length, false);

	if(DecimalCount != 0)
	{
		float factor = 1.0f;
		uint8_t DecCount;
		for(DecCount = 0; DecCount < DecimalCount; ++DecCount)
			factor *= 10;

		uint32_t decValue = (uint32_t)((value - intValue)*factor + 0.5);

		if(decValue != 0)
		{
			buff += length;
			*(buff++) = '.';

			char* It = buff + DecimalCount;

			if(AddEndingZero)
				*It = '\0';

			// Move back, inserting decimal part digits as you go
			while(It > buff)
			{
				*--It = digit[decValue % 10];
				decValue /= 10;
			}

			length += DecimalCount + 1;
		}
	}
	return length;
}

//-----------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//-----------------------------------------------------------
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
