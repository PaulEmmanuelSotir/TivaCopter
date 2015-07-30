/*
 * I2CTransaction.h
 * NOTES:
 * > This API asume that you are only using one I2C peripheral at once for the moment. (TODO: maybe not)
 * > This API asume that your are creating I2C transactions from a unique reading thread.
 * > 'I2CIntStateMachine' must be called from I2C peripheral interrupt or, if you use RTOS,
 *   from a task unblocked(semaphore) by an I2C Hwi and with a higher priority than tasks
 *   that request readings (calling Async_I2CWrite, Async_I2CRegWrite or Async_I2CRegRead).
 * > 'Async_I2CRegRead', 'Async_I2CWrite', 'Async_I2CRegWrite' and 'WaitTransac'
 * TODO: ajouter un define pour utiliser l'API sans allocations dynamiques
 */

#ifndef I2C_REG_TRANSACTION_H_
#define I2C_REG_TRANSACTION_H_

#include <stdint.h>
#include <stdbool.h>

//------------------------------------------
// I2C transaction states
//------------------------------------------
#define STATE_IDLE          0
#define STATE_WRITE_NEXT    1
#define STATE_WRITE_FINAL   2
#define STATE_READ_ONE      3
#define STATE_READ_FIRST    4
#define STATE_READ_NEXT     5
#define STATE_READ_FINAL    6
#define STATE_READ_WAIT     7

//------------------------------------------
// Callback error codes
//------------------------------------------
#define TRANSAC_OK						0
#define TRANSAC_MAX_QUEUEING_REACHED	1
#define TIMEOUT_REACHED					2
#define TRANSAC_UNDETERMINED			8
// Maximum I2c transaction queueing (10 = approximatelly 430 bytes allocated on heap)
#define MAX_QUEUEING_TRANSACTIONS		10

//------------------------------------------
// Defines transaction directions
//------------------------------------------
#define TRANSAC_DIR_READ    0
#define TRANSAC_DIR_WRITE   1
#define TRANSAC_DIR_BOTH	2		// Read-Modify-Write operation

//------------------------------------------
// Defines transaction types
//------------------------------------------
#define TRANSAC_TYPE_REG		0	// I2C register access
#define TRANSAC_TYPE_RAW		1	// Raw I2C communication

//------------------------------------------
// I2C transaction user-defined callback
// function prototype.
// NOTE: User couldn't block callback's thread as this is the I2C state machine's thread too. (it would block next I2C transactions if any)
// TODO: mieux gerer les erreurs (voir I2CMCSR : I2C pro said "I simply read the I2CMCSR to see if errors occurred before doing any data processing")
//------------------------------------------
typedef void (*I2CTransacCallback)(uint32_t status, uint8_t* buffer, uint32_t length);

//------------------------------------------
// The parameters that represents an I2C
// register transaction (Read/Write).
// NOTE: Do not change these parameters
// during transaction and never change State.
//------------------------------------------
typedef struct I2CTransaction
{
    // I2C Master base specific to an I2C peripheral.
    uint32_t I2CBase;
    // Transaction direction (TRANSAC_DIR_READ or TRANSAC_DIR_WRITE)
    uint8_t Direction;
    // Transaction type (see defines below)
    uint8_t Type;
    // Pointer to a data buffer of 'DataCount' length.
    uint8_t *pData;
    // Mask to apply to register (only used for read-modify-write I2C register operation)
    uint8_t Mask;
    // Number of bytes to be tranfered at the begining.
    uint32_t DataCount;
    // Number of bytes to be tranfered now.
    uint32_t RemainingDataCount;
    // I2C slave device address.
    uint32_t SlaveAddress;
    // I2C first register address. (if any)
    uint32_t RegisterAddress;
    // The current state of the interrupt handler state machine. (User don't have to change this)
    uint32_t State;
    // Next transaction to execute if any (I2C transaction queueing)
    struct I2CTransaction *NextTransaction;
    // User-defined callback function
    I2CTransacCallback Callback;
} I2CTransaction;

//------------------------------------------
// I2C interrupt state machine:
// Determines what to do based on the 
// current I2C transaction state.
// Have to be call from I2C interrupt
// handler by user.
//------------------------------------------
void I2CIntStateMachine();

//------------------------------------------
// Begin Write Transaction
//------------------------------------------
static void BeginWriteTransaction(I2CTransaction* transaction);

//------------------------------------------
// Begin Read Transaction
//------------------------------------------
static void BeginReadTransaction(I2CTransaction* transaction);

//------------------------------------------
// I2C Write:
// Write data to a slave device
//------------------------------------------
void Async_I2CWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback);

//------------------------------------------
// I2C Register Write:
// Write to a specified I2C registers for a
// given slave device.
//------------------------------------------
void Async_I2CRegWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback);

//------------------------------------------
// I2C Register Read:
// Read specified I2C registers from a given 
// slave device.
//------------------------------------------
void Async_I2CRegRead(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint32_t dataCount, I2CTransacCallback callback);

//-----------------------------------------------
// Async I2C register Read-Modify-Write operation
// NOTE: after this operation, data will contain
// new register value.
//-----------------------------------------------
void Async_I2CRegReadModifyWrite(uint32_t I2C_Base, uint32_t slaveAddress, uint32_t registerAddress, uint8_t *data, uint8_t mask, I2CTransacCallback callback);

//------------------------------------------
// Wait I2C Transactions:
// Blocks untils all transactions are done.
// Usefull for Synchronous I2C comunication.
// Returns error code defined in header file
//------------------------------------------
uint32_t WaitI2CTransacs(uint32_t status);

//--------------------------------------------
// Dynamically allocate a new I2C transation.
//--------------------------------------------
static I2CTransaction* AddTransac(void);

//---------------------------------------------
// Free current I2C transaction and make
// 'CurrentTransac' referencing to the next I2C
// transaction if any.
//---------------------------------------------
static void NextTransac(void);

#endif /* I2C_REG_TRANSACTION_H_ */
