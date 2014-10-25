/*
 * UARTConsole.h
 * IMPORTANT: Notice that commands callbacks thread are the same as UART console interrupt handler. Even if UART console
 * interrupt handler is called from an RTOS task or main loop rather than from hardware interrupt (recommended) you
 * shouldn't call 'UARTFlushRx', 'UARTFlushTx', 'UARTPeek', 'UARTgets' nor 'UARTgetc' and don't use 'UARTwrite',
 * 'UARTvprintf' or 'UARTprintf' to write more than UART_TX_BUFFER_SIZE.
 */

#ifndef UARTCONSOLE_H_
#define UARTCONSOLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//-----------------------------------------------
// If built for buffered operation, the following
// labels define the sizes of the transmit and
// receive buffers respectively.
//-----------------------------------------------
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE     4096
#endif

//------------------------------------------
// Defines the maximum number of arguments
// that can be parsed.
//------------------------------------------
#ifndef CMDLINE_MAX_ARGS
#define CMDLINE_MAX_ARGS        8
#endif

/*
//------------------------------------------
// Defines the maximum command number of
// command history.
//------------------------------------------
#ifndef COMMAND_HISTORY_SIZE
#define COMMAND_HISTORY_SIZE	8
#endif
*/

//------------------------------------------
// Command line function callback type.
//------------------------------------------
typedef void (*CmdApp)(int argc, char *argv[]);

//------------------------------------------
// Private structure typedef describing a
// command.
//------------------------------------------
typedef struct
{
    // A pointer to a string containing the name of the command.
    const char *cmdName;
    // A function pointer to the implementation of the command.
    CmdApp cmdApp;
    // A pointer to a string of brief help text for the command.
    const char *cmdHelp;
} CmdLineEntry;

//--------------------------------------------
// A structure gathering informations about an
// UART console.
// User should use API functions instead of
// modifing directly members of this struct.
//--------------------------------------------
typedef struct
{
	// The port number in use.
	uint32_t PortNum;
	// The UART base in use
	uint32_t UARTBase;

	// Dynamic command table provided by the user
	struct
	{
		CmdLineEntry *array;
		uint32_t used;
		uint32_t size;
	} CmdTable;

	// Output ring buffer. Buffer is full if UARTTxReadIndex is one ahead of
	// UARTTxWriteIndex. Buffer is empty if  the two indices are the same.
	unsigned char UARTTxBuffer[UART_TX_BUFFER_SIZE];
	volatile uint32_t UARTTxWriteIndex;
	volatile uint32_t UARTTxReadIndex;

	// Input ring buffer. Buffer is full if  UARTTxReadIndex is one ahead of
	// UARTTxWriteIndex. Buffer is empty if the two indices are the same.
	unsigned char UARTRxBuffer[UART_RX_BUFFER_SIZE];
	volatile uint32_t UARTRxWriteIndex;
	volatile uint32_t UARTRxReadIndex;

	// An array to hold the pointers to the command line arguments.
	char *Argv[CMDLINE_MAX_ARGS + 1];

	// This flag controls whether or not we are echoing characters back to the transmitter.
	volatile bool CmdLineInterfaceDisabled;
	// This flag indicates wether if current command execution abort is requested by user.
	volatile bool IsAbortRequested;
} UARTConsole;

//---------------------------------------------------------------------------
// UARTConsoleConfig:
// This function will configure the specified serial port to be used as a
// serial console. The serial parameters are set to the specified baud rate
// and use 8 bit, no parity, and 1 stop bit.
// PortNum is the number of UART port to use for the serial console
// This function must be called prior to using any of the other UART console
// functions. This function assumes that the caller has previously configured
// the relevant UART pins for operation as a UART rather than as GPIOs.
// TODO: ajouter un mots sur les cmd et help
//---------------------------------------------------------------------------
void UARTConsoleConfig(UARTConsole* console, uint32_t PortNum, uint32_t SrcClock, uint32_t BaudRate);

//---------------------------------------------------------------------------
// Suscribe command:
// Add a command line entry to a dynamic command table of specified console.
// Returns false if dynamic memory allocation failed.
//---------------------------------------------------------------------------
bool SubscribeCmd(UARTConsole* console, const char* name, CmdApp cmdApp, const char* cmdHelp);

//---------------------------------------------------------------------------
// Check argument count:
// This function must be called by user if argument count verification is
// needed. Notice that argument table (argv) includes command name so, for
// exemple, if you don't need any agument 'expected' must be 1.
// NOTE: UART console dont handle argument count in order to let user doing
// variable argument count commands.
//---------------------------------------------------------------------------
bool checkArgCount(UARTConsole* console, int argc, int expected);

//----------------------------------------------------------------------------
// Is abort requested:
// Indicates wether if current command execution need to be aborted.
// User can abort a command by typing 'CTRL+C' in the command line interface.
//----------------------------------------------------------------------------
bool IsAbortRequested(const UARTConsole* console);

//----------------------------------------------------------------------------
// Disable command line interface:
// This function is used to disable command line interface so that application
// -specific communication can be done without any echoing and command line
// process.
//----------------------------------------------------------------------------
void DisableCmdLineInterface(UARTConsole* console);

//----------------------------------------------------------------------------
// Enable command line interface
//----------------------------------------------------------------------------
void EnableCmdLineInterface(UARTConsole* console);

//---------------------------------------------------------------------------
// Handles UART interrupts.
// This function handles interrupts from the UART corresponding to specified
// console.
// It will copy data from the transmit buffer to the UART transmit FIFO if
// space is available, and it will copy data from the UART receive FIFO to
// the receive buffer if data is available.
// It will also trigger registered applications if appropriate commands are
// received and handle
// This function must be called for each UART console by the user in their
// respective UART interrupt handler.
// User must provide corresponding console structure and interrupt status.
// If you are using RTOS,
//---------------------------------------------------------------------------
void ConsoleUARTIntHandler(UARTConsole* console,  uint32_t IntStatus);

int UARTwrite(UARTConsole* console, const char *pcBuf, uint32_t ui32Len);
int UARTgets(UARTConsole* console, char *pcBuf, uint32_t ui32Len);
unsigned char UARTgetc(UARTConsole* console);
void UARTvprintf(UARTConsole* console, const char *pcString, va_list vaArgP);
void UARTprintf(UARTConsole* console, const char *pcString, ...);
int UARTRxBytesAvail(UARTConsole* console);
int UARTTxBytesFree(UARTConsole* console);
int UARTPeek(UARTConsole* console, unsigned char ucChar);
void UARTFlushRx(UARTConsole* console);
void UARTFlushTx(UARTConsole* console, bool bDiscard);

#endif /* UARTCONSOLE_H_ */
