/*
 * I2CTransaction.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"

#include "I2CTransaction.h"

//------------------------------------------
// Default I2C transaction, Last queued
// transaction and current transaction
//------------------------------------------
const static I2CTransaction DEFAULT_I2C_TRANSACTION = { I2C0_BASE, TRANSAC_DIR_READ, TRANSAC_TYPE_REG, NULL, 0x00, 1, 0x00, 0x00, STATE_IDLE, NULL, NULL };

//------------------------------------------------------------------------------------
// Variables handled by 'NextTransac()', 'AddTransac()' and 'WaitTransac()' functions:
// These variables shouldn't be modified anywhere else.
// > 'CurrentTransac' can be read but not modified.
// > Do not even read 'LastTransac' and 'AllocatedTransactionNumber'
// > TRANSACTION_SIZE is approx. 36 bytes.
//------------------------------------------------------------------------------------
static I2CTransaction*	CurrentTransac 				= NULL;
static I2CTransaction* 	LastTransac 				= NULL;
static uint32_t			AllocatedTransactionNumber	= 0;
static const uint32_t	TRANSACTION_SIZE 			= sizeof(struct I2CTransaction);

//-------------------------------------------
// User defined lock and unlock functions
// protecting I2C transactions creation from
// beeing corrupted by other threads accesses.
// For example, users using TI-RTOS could
// implement these function using GateMutexPri
//--------------------------------------------
extern intptr_t I2CTransactionsLock(void);
extern void I2CTransactionUnlock(intptr_t lock);

//------------------------------------------
// I2C interrupt state machine
//------------------------------------------
void I2CIntStateMachine()
{
	intptr_t lock = I2CTransactionsLock();

	if(CurrentTransac != NULL)
	{
		// Determine what to do based on the transaction state.
		switch(CurrentTransac->State)
		{
		// If transaction state have recently changed to Idle, we delete this transaction, call its callback function(if any) and begin the next transaction(if any).
		case STATE_IDLE:
		{
			// Store callback function and its parameters
			I2CTransacCallback callback = CurrentTransac->Callback;
			uint32_t dataCount = CurrentTransac->DataCount;
			uint8_t* data = CurrentTransac->pData - dataCount + 1;

			// Free done transaction an go to the next tranction
			NextTransac();

			// If user added a callback function, we call it
			if(callback != NULL)
				callback(TRANSAC_OK, data, dataCount);

			// If there is a queued transaction we begin it.
			if(CurrentTransac != NULL)
			{
				if(CurrentTransac->Direction == TRANSAC_DIR_WRITE)
					BeginWriteTransaction(CurrentTransac);
				else // Read or Read-modify-write
					BeginReadTransaction(CurrentTransac);
			}

			break;
		}

		// The state for the middle of a burst write.
		case STATE_WRITE_NEXT:
		{
			// Write the next byte to the data register.
			I2CMasterDataPut(CurrentTransac->I2CBase, *(CurrentTransac->pData++));
			CurrentTransac->RemainingDataCount--;

			// Continue the burst write.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_BURST_SEND_CONT);

			// If there is one byte left, set the next state to the final write
			// state.
			if(CurrentTransac->RemainingDataCount == 1)
			{
				CurrentTransac->State = STATE_WRITE_FINAL;
			}

			break;
		}

		// The state for the final write of a burst sequence.
		case STATE_WRITE_FINAL:
		{
			// Write the final byte to the data register.
			I2CMasterDataPut(CurrentTransac->I2CBase, *(CurrentTransac->pData));
			CurrentTransac->RemainingDataCount--;

			// Finish the burst write.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

			// The next state is to wait for the burst write to complete.
			CurrentTransac->State = STATE_IDLE;

			break;
		}

		case STATE_READ_ONE:
		{
			// Put the I2C master into receive mode.
			I2CMasterSlaveAddrSet(CurrentTransac->I2CBase, CurrentTransac->SlaveAddress, true);

			// Perform a single byte read.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

			// The next state is the wait for final read state.
			CurrentTransac->State = STATE_READ_WAIT;

			break;
		}

		// The state for the start of a burst read.
		case STATE_READ_FIRST:
		{
			// Put the I2C master into receive mode.
			I2CMasterSlaveAddrSet(CurrentTransac->I2CBase, CurrentTransac->SlaveAddress, true);

			// Start the burst receive.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_START);

			if(CurrentTransac->RemainingDataCount == 2)
				// If there are two characters left to be read, make the next state be the end of burst read state.
				CurrentTransac->State = STATE_READ_FINAL;
			else
				// The next state is the middle of the burst read.
				CurrentTransac->State = STATE_READ_NEXT;

			break;
		}

		// The state for the middle of a burst read.
		case STATE_READ_NEXT:
		{
			// Read the received character.
			*(CurrentTransac->pData++) = I2CMasterDataGet(CurrentTransac->I2CBase);
			CurrentTransac->RemainingDataCount--;

			// Continue the burst read.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

			// If there are two characters left to be read, make the next state be the end of burst read state.
			if(CurrentTransac->RemainingDataCount == 2)
			{
				CurrentTransac->State = STATE_READ_FINAL;
			}

			break;
		}

		// The state for the end of a burst read.
		case STATE_READ_FINAL:
		{
			// Read the received character.
			*(CurrentTransac->pData++) = I2CMasterDataGet(CurrentTransac->I2CBase);
			CurrentTransac->RemainingDataCount--;

			// Finish the burst read.
			I2CMasterControl(CurrentTransac->I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

			// The next state is the wait for final read state.
			CurrentTransac->State = STATE_READ_WAIT;

			break;
		}

		// This state is for the final read of a single or burst read.
		case STATE_READ_WAIT:
		{
			if(CurrentTransac->Direction == TRANSAC_DIR_READ)
			{
				// Read the received character.
				*(CurrentTransac->pData)  = I2CMasterDataGet(CurrentTransac->I2CBase);
				CurrentTransac->RemainingDataCount = 0; // TODO: verifier les variations de data count ici avant yavai count-- !

				// The state machine is now idle.
				CurrentTransac->State = STATE_IDLE;

				// Immediatly update state machine as I2C peripheral will not raise interrupt for this transaction anymore.
				I2CIntStateMachine();
			}
			else // CurrentTransac->Direction == TRANSAC_DIR_BOTH (Read-Modify-Write operation)
			{
				// Apply data modification
				*(CurrentTransac->pData) |= I2CMasterDataGet(CurrentTransac->I2CBase) & CurrentTransac->Mask;
				CurrentTransac->RemainingDataCount = 1;

				// Write modified data back to register
				BeginWriteTransaction(CurrentTransac);
			}
			break;
		}
		}
	}
	else
	{
		// TODO: error ??
	}

	I2CTransactionUnlock(lock);
}

//------------------------------------------
// Begin Write Transaction
//------------------------------------------
static void BeginWriteTransaction(I2CTransaction* transaction)
{
    uint32_t I2C_Base = transaction->I2CBase;
    // Set the slave address and setup for a transmit operation.
    I2CMasterSlaveAddrSet(I2C_Base, transaction->SlaveAddress, false);

    if(transaction->Type == TRANSAC_TYPE_REG)
    {
		if(transaction->RemainingDataCount != 1)
			transaction->State = STATE_WRITE_NEXT;
		else
			transaction->State = STATE_WRITE_FINAL;

		// We place the first register address to be accessed.
    	I2CMasterDataPut(I2C_Base, transaction->RegisterAddress);
        // Start the burst cycle, writing the register address as first byte.
        I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_SEND_START);
    }
    else
    {
    	if(--transaction->RemainingDataCount != 0)
		{
			if(transaction->RemainingDataCount != 1)
				transaction->State = STATE_WRITE_NEXT;
			else
				transaction->State = STATE_WRITE_FINAL;

			// Put the first data byte.
			I2CMasterDataPut(I2C_Base, *(CurrentTransac->pData++));
			// Start the burst cycle, writing the first data byte.
			I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_SEND_START);
		}
		else
		{
			transaction->State = STATE_IDLE;
			// Put the only data byte.
			I2CMasterDataPut(I2C_Base, *(CurrentTransac->pData++));
			// Start the single write.
			I2CMasterControl(I2C_Base, I2C_MASTER_CMD_SINGLE_SEND);
		}
    }
}

//------------------------------------------
// Begin Read Transaction
//------------------------------------------
static void BeginReadTransaction(I2CTransaction* transaction)
{
    // Only register reads are allowed
    if(transaction->Type == TRANSAC_TYPE_REG)
    {
        // Set the next state of the interrupt state machine based on the number of bytes to read.
        if(transaction->RemainingDataCount == 1)
            transaction->State = STATE_READ_ONE;
        else
            transaction->State = STATE_READ_FIRST;

        // Set the slave address and setup for a transmit operation.
        I2CMasterSlaveAddrSet(transaction->I2CBase, transaction->SlaveAddress, false);

        // We place the first register address to be accessed.
        I2CMasterDataPut(transaction->I2CBase, transaction->RegisterAddress);
        // Start the burst cycle, writing the address as the first byte.
        I2CMasterControl(transaction->I2CBase, I2C_MASTER_CMD_SINGLE_SEND);
    }
}

//------------------------------------------
// I2C Write:
// Write raw data to a slave device
//------------------------------------------
void Async_I2CWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback)
{
	intptr_t lock = I2CTransactionsLock();

	// Allocate a new transaction
	I2CTransaction *newTransac = AddTransac();

	newTransac->I2CBase = I2C_Base;
	newTransac->Direction = TRANSAC_DIR_WRITE;
	newTransac->Type = TRANSAC_TYPE_RAW;
	newTransac->pData = data;
	newTransac->DataCount = dataCount;
	newTransac->RemainingDataCount = dataCount;
	newTransac->SlaveAddress = slaveAddress;
	newTransac->RegisterAddress = NULL;
	newTransac->Callback = callback;

	I2CTransactionUnlock(lock);

	// If there isn't any other executing transactions we begin this transaction immediatly.
	if(newTransac == CurrentTransac)
		BeginWriteTransaction(newTransac);
}

//------------------------------------------
// Async I2C Register Write
//------------------------------------------
void Async_I2CRegWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback)
{
	intptr_t lock = I2CTransactionsLock();

	// Allocate a new transaction
	I2CTransaction *newTransac = AddTransac();

	newTransac->I2CBase = I2C_Base;
	newTransac->Direction = TRANSAC_DIR_WRITE;
	newTransac->pData = data;
	newTransac->DataCount = dataCount;
	newTransac->RemainingDataCount = dataCount;
	newTransac->SlaveAddress = slaveAddress;
	newTransac->RegisterAddress = registerAddress;
	newTransac->Callback = callback;

	I2CTransactionUnlock(lock);

	// If there isn't any other executing transactions we begin this transaction immediatly.
	if(newTransac == CurrentTransac)
		BeginWriteTransaction(newTransac);
}

//------------------------------------------
// Async I2C Register Read
//------------------------------------------
void Async_I2CRegRead(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback)
{
	intptr_t lock = I2CTransactionsLock();

	// Allocate a new transaction
	I2CTransaction *newTransac = AddTransac();

	newTransac->I2CBase = I2C_Base;
	newTransac->pData = data;
	newTransac->DataCount = dataCount;
	newTransac->RemainingDataCount = dataCount;
	newTransac->SlaveAddress = slaveAddress;
	newTransac->RegisterAddress = registerAddress;
	newTransac->Callback = callback;

	I2CTransactionUnlock(lock);

	// If there isn't any other executing transactions we begin this transaction immediatly.
	if(newTransac == CurrentTransac)
		BeginReadTransaction(newTransac);
}

//-----------------------------------------------
// Async I2C register Read-Modify-Write operation
// NOTE: after this operation, data will contain
// new register value.
//-----------------------------------------------
void Async_I2CRegReadModifyWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint8_t mask, I2CTransacCallback callback)
{
	intptr_t lock = I2CTransactionsLock();

	// Allocate a new transaction
	I2CTransaction *newTransac = AddTransac();

	newTransac->I2CBase = I2C_Base;
	newTransac->Direction = TRANSAC_DIR_BOTH;
	newTransac->pData = data;
	newTransac->Mask = mask;
	newTransac->DataCount = 1;
	newTransac->RemainingDataCount = 1;
	newTransac->SlaveAddress = slaveAddress;
	newTransac->RegisterAddress = registerAddress;
	newTransac->Callback = callback;

	I2CTransactionUnlock(lock);

	// If there isn't any other executing transactions we begin this transaction immediatly.
	if(newTransac == CurrentTransac)
		BeginReadTransaction(newTransac);
}

//------------------------------------------
// Default callback
// Callback function used by 'WaitTransac'
// to get I2C transactions error code.
//------------------------------------------
static uint32_t errorCode = TRANSAC_UNDETERMINED;
static void DefaultCallback(uint32_t status, uint8_t* buffer, uint32_t length)
{
	errorCode = status;
}

//------------------------------------------
// Wait I2C Transactions:
// Blocks untils all transactions are done.
// Usefull for Synchronous I2C comunication.
// Returns error code defined in header file
//------------------------------------------
uint32_t WaitI2CTransacs(uint32_t timeout)
{
	if(LastTransac != NULL)
	{
		errorCode = TRANSAC_UNDETERMINED;

		// If user didn't added a callback to the last transaction we register our own callback to get eventual error codes.
		if(LastTransac->Callback == NULL)
			LastTransac->Callback = &DefaultCallback;

		// Wait until last transaction is done, or timout is reached.
		if(timeout == 0)
			while(LastTransac != NULL);
		else {
			while(LastTransac != NULL) {
				if(!--timeout)
					return TIMEOUT_REACHED;
			}
		}

		// If we added callback, we have to wait until that this callback modifies errorCode
		//TODO: be sure it will never block here. We asume that error code will never be undetermined in an I2C
		// transaction callback function but does 'errorCode' modification is atomic ?
		if(LastTransac->Callback == &DefaultCallback)
			while(errorCode != TRANSAC_UNDETERMINED);

		return errorCode;
	}
	return TRANSAC_OK;
}

//--------------------------------------------
// Dynamically allocate a new I2C transaction.
//--------------------------------------------
static I2CTransaction* AddTransac(void)
{
	// Check if the transaction number isn't too high. (free all transactions if so)
	if(++AllocatedTransactionNumber > MAX_QUEUEING_TRANSACTIONS)
	{
		I2CTxFIFOFlush(CurrentTransac->I2CBase);
		I2CRxFIFOFlush(CurrentTransac->I2CBase);

		// Free all queued transaction so that new ones can be executed
		while(CurrentTransac != NULL)
		{
			// Call old transaction's user-defined callback with appropriate error code.
			if(CurrentTransac->Callback != NULL)
				CurrentTransac->Callback(TRANSAC_MAX_QUEUEING_REACHED, CurrentTransac->pData, CurrentTransac->DataCount);

			NextTransac();
		}
		// As an error in transaction queue may occur, make sure that 'AllocatedTransactionNumber' is 1 anyway for the next
		// transaction that will be created at the end of this function. (bad for heap if any error of that kind occurs)
		AllocatedTransactionNumber = 1;
	}

	// Store Last transaction
	I2CTransaction* previousTransac = LastTransac;

	// Dynamicaly create a new I2C transaction
	LastTransac = (I2CTransaction *)malloc(TRANSACTION_SIZE);
	if(LastTransac != NULL)
		memcpy(LastTransac, &DEFAULT_I2C_TRANSACTION, TRANSACTION_SIZE);
	else
		--AllocatedTransactionNumber;

	if(previousTransac == NULL)
		// The new transaction is the only transaction <=> previousTransac == NULL <=> CurrentTransac == NULL <=> AllocatedTransactionNumber = 1
		CurrentTransac = LastTransac;
	else
		// Append transation to the I2C transaction queue.
		previousTransac->NextTransaction = LastTransac;

	return LastTransac;
}

//---------------------------------------------
// Free current I2C transaction and make
// 'CurrentTransac' referencing to the next I2C
// transaction if any.
//---------------------------------------------
static void NextTransac(void)
{
	if(CurrentTransac != NULL)
	{
		I2CTransaction* nextTransaction = CurrentTransac->NextTransaction;

		free(CurrentTransac);

        if(nextTransaction == NULL)
        {
        	CurrentTransac = NULL;
            LastTransac = NULL;
        }
        else
        	CurrentTransac = nextTransaction;

		AllocatedTransactionNumber--;
	}
}
