/*
 * conversion.c
 *
 *  Created on: 31 août 2014
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "conversion.h"

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
    static char const digit[] = "0123456789";
    char* It = buff;

    if(value < 0)
    {
        *It++ = '-';
        value  *= -1;
    }

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

	length += itoa2(intValue, buff + length, false);

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
