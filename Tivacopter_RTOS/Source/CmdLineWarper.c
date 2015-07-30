/*
 * CmdLineWarper.c
 *
 *  Created on: 12 sept. 2014
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles

#include "driverlib/debug.h"
#include "Utils/utils.h"
#include "Utils/UARTConsole.h"
#include "Utils/I2CTransaction.h"
#include "Utils/UARTConsole.h"
#include "PinMap.h"

#include "CmdLineWarper.h"

//----------------------------------------
// UART console from 'main.c'
//----------------------------------------
extern UARTConsole Console;

static uint32_t SelectedI2CBase = IMU_I2C_BASE;
static uint8_t I2CBuffer[10];

//------------------------------------------
// Check I2C error code
//------------------------------------------
static bool CheckI2CErrorCode(uint32_t errorCode)
{
	switch(errorCode)
	{
	case TRANSAC_OK:
	case TRANSAC_UNDETERMINED:
		return true;
	case TRANSAC_MAX_QUEUEING_REACHED:
		Log_error0("ERROR: I2C transaction max queueing reached.");
		UARTwrite(&Console, "ERROR: I2C transaction max queueing reached.", 44);
		break;
	case TIMEOUT_REACHED:
		Log_error0("ERROR: I2C transaction waiting timeout reached.");
		UARTwrite(&Console, "ERROR: I2C transaction waiting timeout reached.", 47);
		break;
	default:
		Log_error0("ERROR: I2C transaction unknown error.");
		UARTwrite(&Console, "ERROR: I2C transaction unknown error.", 37);
	};

	return false;
}

//------------------------------------------
// I²C read transaction callback
//------------------------------------------
static void ReadTransactionCallback(uint32_t status, uint8_t* buffer, uint32_t length)
{
	if(CheckI2CErrorCode(status))
	{
		// TODO: post a semaphore and do it in lower priority task
		// Print result in hexadecymal byte format
		uint32_t i;
		for(i = 0; i < length; ++i)
		{
			UARTprintf(&Console, " 0x%x",*(buffer++));
		}
	}

	EnableCmdLineInterface(&Console);
}

//------------------------------------------
// I²C write transaction callback
//------------------------------------------
static void WriteTransactionCallback(uint32_t status, uint8_t* buffer, uint32_t length)
{
	if(CheckI2CErrorCode(status))
		UARTwrite(&Console, "Done.", 5);

	EnableCmdLineInterface(&Console);
}

static void CheckSuccess(bool success)
{
	if(!success)
	{
		Log_error0("Error (re)allocating memory for UART console Warper commands.");
		ASSERT(FALSE);
	}
}

//----------------------------------------
// Subscribe warper cmds
//----------------------------------------
void SubscribeWarperCmds(void)
{
	// I2C transaction API to UART command line interface warper
	CheckSuccess(SubscribeCmd(&Console, "i2cSelect", 	I2CSelect_cmd, 			"Detrmines which I2C peripheral will be used for next i2c command calls (Default is IMU_I2C_BASE). e.g. \"i2cSelect 3\""));
	CheckSuccess(SubscribeCmd(&Console, "i2cregr", 		I2CRegRead_cmd, 		"Performs an asynchronous I2C register read operation. First argument is slave decimal address, second one is the first I2C register decimal address and the last one is the number of bytes to be read."));
	CheckSuccess(SubscribeCmd(&Console, "i2cregw", 		I2CRegWrite_cmd, 		"Performs an asynchronous I2C register write operation. First argument is slave decimal address, second one is the I2C register decimal address and the other ones are bytes to be writen in decimal format."));
	CheckSuccess(SubscribeCmd(&Console, "i2cregrmw", 	I2CRegReadModifyWrite, 	"Performs an asynchronous I2C register read-modify-write operation. First argument is slave decimal address, second one is the first I2C register decimal address, the third one is the decimal bit mask and the last one is the decimal value."));
	CheckSuccess(SubscribeCmd(&Console, "i2cw", 		I2CWrite_cmd, 			"Performs an asynchronous I2C write operation. First argument is slave decimal address and the other ones are bytes to be writen in decimal format."));
}

//------------------------------------------
// I²C peripheral select
//------------------------------------------
void I2CSelect_cmd(int argc, char *argv[])
{
	static uint32_t I2CBases[10] = {I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2C4_BASE, I2C5_BASE, I2C6_BASE, I2C7_BASE, I2C8_BASE, I2C9_BASE};

	if(checkArgCount(&Console, argc, 2))
	{
		uint32_t I2CNum = atoi(argv[1]);
		if(I2CNum <= 10 && I2CNum > 0)
			SelectedI2CBase = I2CBases[I2CNum-1];
		else
			UARTwrite(&Console, "Wrong I2C peripheral number, select an I2C peripheral number from 1 to 10.", 74);
	}
}

//------------------------------------------
// I²C register read
//------------------------------------------
void I2CRegRead_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 4))
	{
		// TODO: verifier que ce sont bien des chiffre (sinon 'atoi' renvoi 0 !!)
		uint32_t SlaveAddress = atoi(argv[1]);
		uint32_t RegisterAddress = atoi(argv[2]);
		uint32_t ByteCount = atoi(argv[3]);

		if(ByteCount > 0 && ByteCount <= 10)
		{
			DisableCmdLineInterface(&Console);

			Async_I2CRegRead(SelectedI2CBase, SlaveAddress, RegisterAddress, I2CBuffer, ByteCount, ReadTransactionCallback);
		}
		else
			UARTwrite(&Console, "Can't read more than 10 bytes at once from command line interface.", 66);
	}
}

//------------------------------------------
// I²C register write
//------------------------------------------
void I2CRegWrite_cmd(int argc, char *argv[])
{
	if(checkArgRange(&Console, argc, 3, 13))
	{
		// TODO: verifier que ce sont bien des chiffre (sinon 'atoi' renvoi 0 !!)
		uint32_t SlaveAddress = atoi(argv[1]);
		uint32_t RegisterAddress = atoi(argv[2]);
		uint32_t ByteCount = argc-3;

		DisableCmdLineInterface(&Console);

		// Copy argument data bytes to I2C buffer
		uint32_t i;
		for(i = 0, argv += 3; i < ByteCount; ++i)
			I2CBuffer[i] = atoi(argv[i]);

		Async_I2CRegWrite(SelectedI2CBase, SlaveAddress, RegisterAddress, I2CBuffer, ByteCount, WriteTransactionCallback);

	}
	else if(argc > 13)
		UARTwrite(&Console, "Can't write more than 10 bytes at once from command line interface.", 67);
}

//------------------------------------------
// I²C register read-modify-write
//------------------------------------------
void I2CRegReadModifyWrite(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 5))
	{
		// TODO: verifier que ce sont bien des chiffre (sinon 'atoi' renvoi 0 !!)
		uint32_t SlaveAddress = atoi(argv[1]);
		uint32_t RegisterAddress = atoi(argv[2]);
		uint8_t mask = (uint8_t)atoi(argv[3]);
		I2CBuffer[0] = (uint8_t)atoi(argv[4]);

		DisableCmdLineInterface(&Console);

		Async_I2CRegReadModifyWrite(SelectedI2CBase, SlaveAddress, RegisterAddress, I2CBuffer, mask, WriteTransactionCallback);
	}
}

//------------------------------------------
// I²C write
//------------------------------------------
void I2CWrite_cmd(int argc, char *argv[])
{
	if(checkArgRange(&Console, argc, 2, 12))
	{
		// TODO: verifier que ce sont bien des chiffre (sinon 'atoi' renvoi 0 !!)
		uint32_t SlaveAddress = atoi(argv[1]);
		uint32_t ByteCount = argc-2;

		DisableCmdLineInterface(&Console);

		// Copy argument data bytes to I2C buffer
		uint32_t i;
		for(i = 0, argv += 2; i < ByteCount; ++i)
			I2CBuffer[i] = atoi(argv[i]);

		Async_I2CWrite(SelectedI2CBase, SlaveAddress, I2CBuffer, ByteCount, WriteTransactionCallback);
	}
	else if(argc > 12)
		UARTwrite(&Console, "Can't write more than 10 bytes at once from command line interface.", 67);
}
