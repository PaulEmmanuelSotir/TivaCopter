/*
 * UARTConsole.c
 * IMPORTANT: Notice that commands callbacks thread are the same as UART console interrupt handler. Even if UART console
 * interrupt handler is called from an RTOS task or main loop rather than from hardware interrupt (recommended) you
 * shouldn't call 'UARTFlushRx', 'UARTFlushTx', 'UARTPeek', 'UARTgets' nor 'UARTgetc' and don't use 'UARTwrite',
 * 'UARTvprintf' or 'UARTprintf' to write more than UART_TX_BUFFER_SIZE.
 * TODO: determiner si l'utilisation de intMasterDisable ne poserais pas problème maintenant que plusieur consoles simultanées sont possibles
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"

#include "UARTConsole.h"

//--------------------------------------------
// Macros to advance receive or tranmit buffer
// index
//--------------------------------------------
#define ADVANCE_TX_BUFFER_INDEX(Index)	(Index) = ((Index) + 1) % UART_TX_BUFFER_SIZE
#define ADVANCE_RX_BUFFER_INDEX(Index)	(Index) = ((Index) + 1) % UART_RX_BUFFER_SIZE

//-------------------------------------------
// A mapping from an integer between 0 and 15
// to its ASCII character equivalent
//-------------------------------------------
static const char* const ASCIIHexMap = "0123456789abcdef";

//------------------------------------------
// The list of possible base addresses for
// the console UART
//------------------------------------------
static const uint32_t UARTBases[4] = { UART0_BASE, UART1_BASE, UART2_BASE, UART3_BASE };

//------------------------------------------
// The list of possible interrupts for the
// console UART
//------------------------------------------
static const uint32_t UARTInts[4] = { INT_UART0, INT_UART1, INT_UART2, INT_UART3 };

//------------------------------------------
// The list of UART peripherals
//------------------------------------------
static const uint32_t UARTPeriphs[4] = { SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_UART2, SYSCTL_PERIPH_UART3 };

//------------------------------------------
// Static function forward declarations
//------------------------------------------
static void CmdLineProcess(UARTConsole* console, char *input, uint32_t length);
static void NotifyCharacterReceived(UARTConsole* console, char c);
static void UARTPrimeTransmit(UARTConsole* console);
static bool IsBufferEmpty(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write);
static bool IsBufferFull(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size);
static uint32_t GetBufferCount(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size);

//---------------------------------------------------------------------------
// UARTConsoleConfig:
// This function will configure the specified serial port to be used as a
// serial console. The serial parameters are set to the specified baud rate
// and use 8 bit, no parity, and 1 stop bit.
// PortNum is the number of UART port to use for the serial console
// This function must be called prior to using any of the other UART console
// functions. This function assumes that the caller has previously configured
// the relevant UART pins for operation as a UART rather than as GPIOs.
//---------------------------------------------------------------------------
void UARTConsoleConfig(UARTConsole* console, uint32_t PortNum, uint32_t SrcClock, uint32_t BaudRate)
{
	// Check the arguments.
	ASSERT(console != NULL);
	ASSERT((PortNum == 0) || (PortNum == 1) || (PortNum == 2) || (PortNum == 3));

	// Check to make sure the UART peripheral is present.
	if(!MAP_SysCtlPeripheralPresent(UARTPeriphs[PortNum]))
		return; // TODO: retourner plutôt un code d'erreur !? >.<

	// Select the base address of the UART and store data in UART console struct
	console->PortNum = PortNum;
	console->UARTBase = UARTBases[PortNum];
	console->CmdLineInterfaceDisabled = false; // Echo flag isn't raised until a command is received
	console->IsAbortRequested = false;

	// Enable the UART peripheral for use.
	MAP_SysCtlPeripheralEnable(UARTPeriphs[PortNum]);

	// Configure the UART for n, 8, 1 at specified baudrate
	MAP_UARTConfigSetExpClk(console->UARTBase, SrcClock, BaudRate, (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

	// Set the UART to interrupt whenever the TX FIFO is almost empty or when any character is received.
	MAP_UARTFIFOLevelSet(console->UARTBase, UART_FIFO_TX1_8, UART_FIFO_RX1_8);//UART_FIFO_RX1_8);

	// Flush both the buffers.
	UARTFlushRx(console);
	UARTFlushTx(console, true);

	// We are configured for buffered output so enable the master interrupt
	// for this UART and the receive interrupts.  We don't actually enable the
	// transmit interrupt in the UART itself until some data has been placed
	// in the transmit buffer.
	MAP_UARTIntDisable(console->UARTBase, 0xFFFFFFFF);
	MAP_UARTIntEnable(console->UARTBase, UART_INT_RX | UART_INT_RT);
	MAP_IntEnable(UARTInts[PortNum]);

	// Enable the UART operation.
	MAP_UARTEnable(console->UARTBase);
}

//---------------------------------------------------------------------------
// Suscribe command:
// Add a command line entry to a dynamic command table of specified console.
// Returns false if dynamic memory allocation failed.
//---------------------------------------------------------------------------
bool SubscribeCmd(UARTConsole* console, const char* name, CmdApp app, const char* help)
{
	ASSERT(console != NULL);
	ASSERT(app != NULL);
	ASSERT(name != "" && name != NULL);

	// If there is no more available space in dynamic command array, we allocate more memory.
	if (console->CmdTable.used == console->CmdTable.size)
	{
		console->CmdTable.size += 4;
		console->CmdTable.array = (CmdLineEntry *)realloc(console->CmdTable.array, console->CmdTable.size * sizeof(CmdLineEntry));

		// Verify wether if any error occured during memory allocation.
		if (console->CmdTable.array == NULL)
		{
			console->CmdTable.size = 0;
			console->CmdTable.used = 0;
			free(console->CmdTable.array);
			return false;
		}
	}

	CmdLineEntry* newCmd = &console->CmdTable.array[console->CmdTable.used++];
	newCmd->name = name;
	newCmd->app = app;
	newCmd->help = help;

	return true;
}

//---------------------------------------------------------------------------
// Suscribe listening command:
// Add a command line entry, which can listen for any received character
// among the given array, to a dynamic command table of specified console.
// Returns false if dynamic memory allocation failed.
//---------------------------------------------------------------------------
bool SubscribeListeningCmd(UARTConsole* console, const char* name, CmdApp app, const char* help, const char* interestingChars, ListeningCallback cb)
{
	ASSERT(console != NULL);
	ASSERT(app != NULL);
	ASSERT(name != "" && name != NULL);

	// If there is no more available space in dynamic command array, we allocate more memory.
	if (console->CmdTable.used == console->CmdTable.size)
	{
		console->CmdTable.size += 4;
		console->CmdTable.array = (CmdLineEntry *)realloc(console->CmdTable.array, console->CmdTable.size * sizeof(CmdLineEntry));

		// Verify wether if any error occured during memory allocation.
		if (console->CmdTable.array == NULL)
		{
			console->CmdTable.size = 0;
			console->CmdTable.used = 0;
			free(console->CmdTable.array);
			return false;
		}
	}

	CmdLineEntry* newCmd = &console->CmdTable.array[console->CmdTable.used++];
	newCmd->name = name;
	newCmd->app = app;
	newCmd->help = help;
	newCmd->interestingChars = interestingChars;
	newCmd->cb = cb;

	return true;
}

//---------------------------------------------------------------------------
// Check argument count:
// This function must be called by user if argument count verification is
// needed. Notice that argument table (argv) includes command name so, for
// exemple, if you don't need any agument 'expected' must be 1.
// NOTE: UART console dont handle argument count in order to let user doing
// variable argument count commands.
//---------------------------------------------------------------------------
bool checkArgCount(UARTConsole* console, int argc, int expected)
{
	if(expected > argc)
	{
		UARTwrite(console, "Too few arguments.", 18);
		return false;
	}
	else if(expected < argc)
	{
		UARTwrite(console, "Too many arguments.", 19);
		return false;
	}

	return true;
}

//----------------------------------------------------------------------------
// Disable command line interface:
// This function is used to disable command line interface so that application
// -specific communication can be done without any echoing and command line
// process.
//----------------------------------------------------------------------------
void DisableCmdLineInterface(UARTConsole* console)
{
	ASSERT(console != NULL);

	console->CmdLineInterfaceDisabled = true;
}

//----------------------------------------------------------------------------
// Enable command line interface
//----------------------------------------------------------------------------
void EnableCmdLineInterface(UARTConsole* console)
{
	ASSERT(console != NULL);

	if(console->CmdLineInterfaceDisabled)
	{
		console->CurrentlyRunningCmd = NULL;
		UARTFlushTx(console, true);
		UARTFlushRx(console);
		UARTwrite(console, "\n> ", 3);
		console->CmdLineInterfaceDisabled = false;
	}

	console->IsAbortRequested = false;
}

//---------------------------------------------------------------------------
// Console UART interrupt handler:
// This function handles interrupts from the UART corresponding to specified
// console.
// It will copy data from the transmit buffer to the UART transmit FIFO if
// space is available, and it will copy data from the UART receive FIFO to
// the receive buffer if data is available.
// It will also trigger registered applications if appropriate commands are
// received and handle console special characters like backspace.
// This function must be called for each UART console by the user in their
// respective UART interrupt handler.
// User must provide corresponding console structure and interrupt status.
//---------------------------------------------------------------------------
void ConsoleUARTIntHandler(UARTConsole* console, uint32_t IntStatus)
{
	int8_t cChar;
	int32_t i32Char;
	static bool bLastWasCR = false;
	static char buffer[UART_RX_BUFFER_SIZE+1];

	ASSERT(console != NULL);

	// Are we being interrupted because the TX FIFO has space available?
	if(IntStatus & UART_INT_TX)
	{
		// Move as many bytes as we can into the transmit FIFO.
		UARTPrimeTransmit(console);

		// If the output buffer is empty, turn off the transmit interrupt.
		if(IsBufferEmpty(&console->UARTTxReadIndex, &console->UARTTxWriteIndex))
			MAP_UARTIntDisable(console->UARTBase, UART_INT_TX);
	}

	// Are we being interrupted due to a received character?
	if(IntStatus & (UART_INT_RX | UART_INT_RT))
	{
		// Get all the available characters from the UART.
		while(MAP_UARTCharsAvail(console->UARTBase))
		{
			// Read a character
			i32Char = MAP_UARTCharGetNonBlocking(console->UARTBase);
			cChar = (unsigned char)(i32Char & 0xFF);

			// If command line interface is disabled, we skip the various text filtering  operations.
			if(!console->CmdLineInterfaceDisabled)
			{
				console->IsAbortRequested = false;

				// Handle backspace by erasing the last character in the buffer.
				if(cChar == '\b')
				{
					// If there are any characters already in the buffer, then delete the last.
					if(!IsBufferEmpty(&console->UARTRxReadIndex, &console->UARTRxWriteIndex))
					{
						// Rub out the previous character on the users terminal.
						UARTwrite(console, "\b \b", 3);

						// Decrement the number of characters in the buffer.
						if(console->UARTRxWriteIndex == 0)
							console->UARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
						else
							console->UARTRxWriteIndex--;
					}

					// Skip ahead to read the next character.
					continue;
				}

				// If this character is LF and last was CR, then just gobble up the character since we already
				// echoed the previous CR and we don't want to store 2 characters in the buffer if we don't need to.
				if((cChar == '\n') && bLastWasCR)
				{
					bLastWasCR = false;
					continue;
				}

				// See if a newline or escape character was received.
				if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
				{
					// If the character is a CR, then it may be followed by an  LF which should be paired with
					// the CR.  So remember that a CR was received.
					if(cChar == '\r')
						bLastWasCR = 1;

					// Regardless of the line termination character received, put a CR in the receive buffer as a
					// marker telling UARTgets() where the line ends. We also send an additional LF to ensure that
					// the local terminal echo receives both CR and LF.
					UARTwrite(console, "\n\r", 2);
					console->UARTRxBuffer[console->UARTRxWriteIndex] = '\r';
					ADVANCE_RX_BUFFER_INDEX(console->UARTRxWriteIndex);

					//TODO: pas super optimisé >.< et peut poser des problèmes si l'utilisateur s'amuse à utiliser les fonctions pour lire les buffers en même temps
					// Parse and execute received command
					uint32_t length = UARTgets(console, buffer, UART_RX_BUFFER_SIZE);
					CmdLineProcess(console, buffer, length);

					// Skip ahead to read the next character.
					continue;
				}
			}
			// else, if CTRL+C (ETX=0x03) character have been sent, we raise abrot requested flag
			else if(cChar == '\x03')
			{
				console->IsAbortRequested = true;
				continue;
			}


			// If there is space in the receive buffer, put the character there, otherwise throw it away.
			if(!IsBufferFull(&console->UARTRxReadIndex, &console->UARTRxWriteIndex, UART_RX_BUFFER_SIZE))
			{
				// Store the new character in the receive buffer
				console->UARTRxBuffer[console->UARTRxWriteIndex] = (unsigned char)(i32Char & 0xFF);
				ADVANCE_RX_BUFFER_INDEX(console->UARTRxWriteIndex);

				// If console is enabled, write the character to the transmit
				// buffer so that the user gets some immediate feedback (echo).
				if(!console->CmdLineInterfaceDisabled)
					UARTwrite(console, (const char *)&cChar, 1);
				// Else, notify the running command that we received a character if this command is listenning to this specific character
				else
					NotifyCharacterReceived(console, cChar);
			}
		}

		// If we wrote anything to the transmit buffer, make sure it actually gets transmitted.
		UARTPrimeTransmit(console);
		MAP_UARTIntEnable(console->UARTBase, UART_INT_TX);
	}
}

//----------------------------------------------------------------------------
// Notify character received:
// Notify the currently running command that we received a character if this
// command is listening to this specific character and if the command line
// interface is disabled.
//----------------------------------------------------------------------------
static void NotifyCharacterReceived(UARTConsole* console, char c)
{
	if(console->CurrentlyRunningCmd != NULL && console->CmdLineInterfaceDisabled)
		if(strchr(console->CurrentlyRunningCmd->interestingChars, c) != NULL)
			console->CurrentlyRunningCmd->cb(c);
}

//----------------------------------------------------------------------------
// Is abort requested:
// Indicates wether if current command execution need to be aborted.
// User can abort a command by typing 'CTRL+C' in the command line interface.
//----------------------------------------------------------------------------
bool IsAbortRequested(const UARTConsole* console)
{
	ASSERT(console != NULL);

	return console->IsAbortRequested;
}

//---------------------------------------------------------------------------
// IsBufferFull:
// Determines whether the ring buffer whose pointers and size are provided is
// full or not. pui32Read points to the read index for the buffer, pui32Write
// points to the write index for the buffer, and ui32Size is the size of the
// buffer in bytes.
// The structure of the code is specifically to ensure that we do not see
// warnings from the compiler related to the order of volatile accesses being
// undefined.
//---------------------------------------------------------------------------
static bool IsBufferFull(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size)
{
	const uint32_t ui32Write = *pui32Write;
	const uint32_t ui32Read = *pui32Read;

	return((((ui32Write + 1) % ui32Size) == ui32Read) ? true : false);
}

//---------------------------------------------------------------------------
// IsBufferEmpty:
// Determines whether the ring buffer whose pointers and size are provided
// is empty or not. pui32Read points to the read index for the buffer and
// pui32Write points to the write index for the buffer.
// The structure of the code is specifically to ensure that we do not see
// warnings from the compiler related to the order of volatile accesses being
// undefined.
//---------------------------------------------------------------------------
static bool IsBufferEmpty(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write)
{
	const uint32_t ui32Write = *pui32Write;
	const uint32_t ui32Read = *pui32Read;

	return((ui32Write == ui32Read) ? true : false);
}

//---------------------------------------------------------------------------
// GetBufferCount:
// Determines the number of bytes of data contained in a ring buffer.
// pui32Read points to the read index for the buffer, pui32Write points to
// the write index for the buffer, and ui32Size is the size of the buffer in
// bytes.
// The structure of the code is specifically to ensure that we do not see
// warnings from the compiler related to the order of volatile accesses being
// undefined.
//---------------------------------------------------------------------------
static uint32_t GetBufferCount(volatile uint32_t *pui32Read, volatile uint32_t *pui32Write, uint32_t ui32Size)
{
	const uint32_t ui32Write = *pui32Write;
	const uint32_t ui32Read = *pui32Read;

	return((ui32Write >= ui32Read) ? (ui32Write - ui32Read) : (ui32Size - (ui32Read - ui32Write)));
}

//--------------------------------------------
// UARTPrimeTransmit:
// Take as many bytes from the transmit buffer
// as we have space for and move them into the
// UART transmit FIFO.
//--------------------------------------------
static void UARTPrimeTransmit(UARTConsole* console)
{
	ASSERT(console != NULL);

	// Do we have any data to transmit?
	if(!IsBufferEmpty(&console->UARTTxReadIndex, &console->UARTTxWriteIndex))
	{
		// Disable the UART interrupt.  If we don't do this there is a race condition which can cause the read index to be corrupted.
		MAP_IntDisable(UARTInts[console->PortNum]);

		// Yes - take some characters out of the transmit buffer and feed them to the UART transmit FIFO.
		while(MAP_UARTSpaceAvail(console->UARTBase) && !IsBufferEmpty(&console->UARTTxReadIndex, &console->UARTTxWriteIndex))
		{
			MAP_UARTCharPutNonBlocking(console->UARTBase, console->UARTTxBuffer[console->UARTTxReadIndex]);
			ADVANCE_TX_BUFFER_INDEX(console->UARTTxReadIndex);
		}

		// Reenable the UART interrupt.
		MAP_IntEnable(UARTInts[console->PortNum]);
	}
}

//---------------------------------------------------------------------------
// Writes a string of characters to the UART output.
//
// \param pcBuf points to a buffer containing the string to transmit.
// \param ui32Len is the length of the string to transmit.
//
// This function will transmit the string to the UART output.  The number of
// characters transmitted is determined by the \e ui32Len parameter.  This
// function does no interpretation or translation of any characters.  Since
// the output is sent to a UART, any LF (/n) characters encountered will be
// replaced with a CRLF pair.
//
// Besides using the \e ui32Len parameter to stop transmitting the string, if
// a null character (0) is encountered, then no more characters will be
// transmitted and the function will return.
//
// In non-buffered mode, this function is blocking and will not return until
// all the characters have been written to the output FIFO.  In buffered mode,
// the characters are written to the UART transmit buffer and the call returns
// immediately.  If insufficient space remains in the transmit buffer,
// additional characters are discarded.
//
// \return Returns the count of characters written.
//---------------------------------------------------------------------------
int UARTwrite(UARTConsole* console, const char *pcBuf, uint32_t ui32Len)
{
	unsigned int uIdx;

	ASSERT(console != NULL);
	ASSERT(pcBuf != NULL);

	// Send the characters
	for(uIdx = 0; uIdx < ui32Len; uIdx++)
	{
		// If the character to the UART is \n, then add a \r before it so that \n is translated to \n\r in the output.
		if(pcBuf[uIdx] == '\n')
		{
			if(!IsBufferFull(&console->UARTTxReadIndex, &console->UARTTxWriteIndex, UART_TX_BUFFER_SIZE))
			{
				console->UARTTxBuffer[console->UARTTxWriteIndex] = '\r';
				ADVANCE_TX_BUFFER_INDEX(console->UARTTxWriteIndex);
			}
			else
			{
				// Buffer is full - discard remaining characters and return.
				break;
			}
		}

		// Send the character to the UART output.
		if(!IsBufferFull(&console->UARTTxReadIndex, &console->UARTTxWriteIndex, UART_TX_BUFFER_SIZE))
		{
			console->UARTTxBuffer[console->UARTTxWriteIndex] = pcBuf[uIdx];
			ADVANCE_TX_BUFFER_INDEX(console->UARTTxWriteIndex);
		}
		else
		{
			// Buffer is full - discard remaining characters and return.
			break;
		}
	}

	// If we have anything in the buffer, make sure that the UART is set
	// up to transmit it.
	if(!IsBufferEmpty(&console->UARTTxReadIndex, &console->UARTTxWriteIndex))
	{
		UARTPrimeTransmit(console);
		MAP_UARTIntEnable(console->UARTBase, UART_INT_TX);
	}

	// Return the number of characters written.
	return(uIdx);
}

//---------------------------------------------------------------------------
// A simple UART based get string function, with some line processing.
//
// \param pcBuf points to a buffer for the incoming string from the UART.
// \param ui32Len is the length of the buffer for storage of the string,
// including the trailing 0.
//
// This function will receive a string from the UART input and store the
// characters in the buffer pointed to by \e pcBuf. The characters will
// continue to be stored until a termination character is received.  The
// termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
// single termination character.  The termination characters are not stored in
// the string. The string will be terminated with a 0 and the function will
// return.
//
// In both buffered and unbuffered modes, this function will block until
// a termination character is received.  If non-blocking operation is required
// in buffered mode, a call to UARTPeek() may be made to determine whether
// a termination character already exists in the receive buffer prior to
// calling UARTgets().
//
// Since the string will be null terminated, the user must ensure that the
// buffer is sized to allow for the additional null character.
//
// \return Returns the count of characters that were stored, not including
// the trailing 0.
//---------------------------------------------------------------------------
int UARTgets(UARTConsole* console, char *pcBuf, uint32_t ui32Len)
{
	uint32_t ui32Count = 0;
	int8_t cChar;

	ASSERT(console != NULL);
	ASSERT(pcBuf != NULL);
	ASSERT(ui32Len != 0);

	// Adjust the length back by 1 to leave space for the trailing null terminator.
	ui32Len--;

	// Process characters until a newline is received.
	while(1)
	{
		// Read the next character from the receive buffer.
		if(!IsBufferEmpty(&console->UARTRxReadIndex, &console->UARTRxWriteIndex))
		{
			cChar = console->UARTRxBuffer[console->UARTRxReadIndex];
			ADVANCE_RX_BUFFER_INDEX(console->UARTRxReadIndex);

			// See if a newline or escape character was received.
			if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
			{
				// Stop processing the input and end the line.
				break;
			}

			// Process the received character as long as we are not at the end
			// of the buffer.  If the end of the buffer has been reached then
			// all additional characters are ignored until a newline is received.
			if(ui32Count < ui32Len)
			{
				// Store the character in the caller supplied buffer.
				pcBuf[ui32Count] = cChar;

				// Increment the count of characters received.
				ui32Count++;
			}
		}
	}

	// Add a null termination to the string.
	pcBuf[ui32Count] = 0;

	// Return the count of int8_ts in the buffer, not counting the trailing 0.
	return(ui32Count);
}

//---------------------------------------------------------------------------
// Read a single character from the UART, blocking if necessary.
//
// This function will receive a single character from the UART and store it at
// the supplied address.
//
// In both buffered and unbuffered modes, this function will block until a
// character is received. If non-blocking operation is required in buffered
// mode, a call to UARTRxAvail() may be made to determine whether any
// characters are currently available for reading.
//
// \return Returns the character read.
//---------------------------------------------------------------------------
unsigned char UARTgetc(UARTConsole* console)
{
	unsigned char cChar;

	ASSERT(console != NULL);

	// Wait for a character to be received (if the buffer is currently empty).
	while(IsBufferEmpty(&console->UARTRxReadIndex, &console->UARTRxWriteIndex)) { }

	// Read a character from the buffer.
	cChar = console->UARTRxBuffer[console->UARTRxReadIndex];
	ADVANCE_RX_BUFFER_INDEX(console->UARTRxReadIndex);

	// Return the character to the caller.
	return(cChar);
}

//----------------------------------------------------------------------------
// A simple UART based vprintf function supporting \%c, \%d, \%p, \%s, \%u,
// \%x, and \%X.
//
// \param pcString is the format string.
// \param vaArgP is a variable argument list pointer whose content will depend
// upon the format string passed in \e pcString.
//
// This function is very similar to the C library <tt>vprintf()</tt> function.
// All of its output will be sent to the UART.  Only the following formatting
// characters are supported:
//
// - \%c to print a character
// - \%d or \%i to print a decimal value
// - \%s to print a string
// - \%u to print an unsigned decimal value
// - \%x to print a hexadecimal value using lower case letters
// - \%X to print a hexadecimal value using lower case letters (not upper case
// letters as would typically be used)
// - \%p to print a pointer as a hexadecimal value
// - \%\% to print out a \% character
//
// For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
// between the \% and the format character, which specifies the minimum number
// of characters to use for that value; if preceded by a 0 then the extra
// characters will be filled with zeros instead of spaces.  For example,
// ``\%8d'' will use eight characters to print the decimal value with spaces
// added to reach eight; ``\%08d'' will use eight characters as well but will
// add zeroes instead of spaces.
//
// The type of the arguments in the variable arguments list must match the
// requirements of the format string.  For example, if an integer was passed
// where a string was expected, an error of some kind will most likely occur.
//----------------------------------------------------------------------------
void UARTvprintf(UARTConsole* console, const char *pcString, va_list vaArgP)
{
	uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
	char *pcStr, pcBuf[16], cFill;

	ASSERT(console != NULL);
	ASSERT(pcString != NULL);

	// Loop while there are more characters in the string.
	while(*pcString)
	{
		// Find the first non-% character, or the end of the string.
		for(ui32Idx = 0; (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0'); ui32Idx++) { }

		// Write this portion of the string.
		UARTwrite(console, pcString, ui32Idx);

		// Skip the portion of the string that was written.
		pcString += ui32Idx;

		// See if the next character is a %.
		if(*pcString == '%')
		{
			// Skip the %.
			pcString++;

			// Set the digit count to zero, and the fill character to space (in other words, to the defaults).
			ui32Count = 0;
			cFill = ' ';

			// It may be necessary to get back here to process more characters.  Goto's aren't pretty, but effective.
			// I feel extremely dirty for using not one but two of the beasts.
			again:

			// Determine how to handle the next character.
			switch(*pcString++)
			{
			// Handle the digit characters.
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			{
				// If this is a zero, and it is the first digit, then the fill character is a zero instead of a space.
				if((pcString[-1] == '0') && (ui32Count == 0))
				{
					cFill = '0';
				}

				// Update the digit count.
				ui32Count *= 10;
				ui32Count += pcString[-1] - '0';

				// Get the next character.
				goto again;
			}

			// Handle the %c command.
			case 'c':
			{
				// Get the value from the varargs.
				ui32Value = va_arg(vaArgP, uint32_t);

				// Print out the character.
				UARTwrite(console, (char *)&ui32Value, 1);

				// This command has been handled.
				break;
			}

			// Handle the %d and %i commands.
			case 'd':
			case 'i':
			{
				// Get the value from the varargs.
				ui32Value = va_arg(vaArgP, uint32_t);

				// Reset the buffer position.
				ui32Pos = 0;

				// If the value is negative, make it positive and indicate
				// that a minus sign is needed.
				if((int32_t)ui32Value < 0)
				{
					// Make the value positive.
					ui32Value = -(int32_t)ui32Value;

					// Indicate that the value is negative.
					ui32Neg = 1;
				}
				else
				{
					// Indicate that the value is positive so that a minus
					// sign isn't inserted.
					ui32Neg = 0;
				}

				// Set the base to 10.
				ui32Base = 10;

				// Convert the value to ASCII.
				goto convert;
			}

			// Handle the %s command.
			case 's':
			{
				// Get the string pointer from the varargs.
				pcStr = va_arg(vaArgP, char *);

				// Determine the length of the string.
				for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++) { }

				// Write the string.

				UARTwrite(console, pcStr, ui32Idx);

				// Write any required padding spaces
				if(ui32Count > ui32Idx)
				{
					ui32Count -= ui32Idx;
					while(ui32Count--)
					{
						UARTwrite(console, " ", 1);
					}
				}

				// This command has been handled.
				break;
			}

			// Handle the %u command.
			case 'u':
			{
				// Get the value from the varargs.
				ui32Value = va_arg(vaArgP, uint32_t);

				// Reset the buffer position.
				ui32Pos = 0;

				// Set the base to 10.
				ui32Base = 10;

				// Indicate that the value is positive so that a minus sign
				// isn't inserted.
				ui32Neg = 0;

				// Convert the value to ASCII.
				goto convert;
			}

			// Handle the %x and %X commands.  Note that they are treated identically; in other words,
			// %X will use lower case letters for a-f instead of the upper case letters it should use.
			// We also alias %p to %x.
			case 'x':
			case 'X':
			case 'p':
			{
				// Get the value from the varargs.
				ui32Value = va_arg(vaArgP, uint32_t);

				// Reset the buffer position.
				ui32Pos = 0;

				// Set the base to 16.
				ui32Base = 16;

				// Indicate that the value is positive so that a minus sign isn't inserted.
				ui32Neg = 0;

				// Determine the number of digits in the string version of the value.
				convert:
				for(ui32Idx = 1; (((ui32Idx * ui32Base) <= ui32Value) && (((ui32Idx * ui32Base) / ui32Base) == ui32Idx)); ui32Idx *= ui32Base, ui32Count--) { }

				// If the value is negative, reduce the count of padding characters needed.
				if(ui32Neg)
				{
					ui32Count--;
				}

				// If the value is negative and the value is padded with zeros, then place the minus sign before the padding.
				if(ui32Neg && (cFill == '0'))
				{
					// Place the minus sign in the output buffer.
					pcBuf[ui32Pos++] = '-';

					// The minus sign has been placed, so turn off the negative flag.
					ui32Neg = 0;
				}

				// Provide additional padding at the beginning of the string conversion if needed.
				if((ui32Count > 1) && (ui32Count < 16))
				{
					for(ui32Count--; ui32Count; ui32Count--)
					{
						pcBuf[ui32Pos++] = cFill;
					}
				}

				// If the value is negative, then place the minus sign before the number.
				if(ui32Neg)
				{
					// Place the minus sign in the output buffer.
					pcBuf[ui32Pos++] = '-';
				}

				// Convert the value into a string.
				for(; ui32Idx; ui32Idx /= ui32Base)
					pcBuf[ui32Pos++] = ASCIIHexMap[(ui32Value / ui32Idx) % ui32Base];

				// Write the string.
				UARTwrite(console, pcBuf, ui32Pos);

				// This command has been handled.
				break;
			}

			// Handle the %% command.
			case '%':
			{
				// Simply write a single %.
				UARTwrite(console, pcString - 1, 1);

				// This command has been handled.
				break;
			}

			// Handle all other commands.
			default:
			{
				// Indicate an error.
				UARTwrite(console, "ERROR", 5);

				// This command has been handled.
				break;
			}
			}
		}
	}
}

//---------------------------------------------------------------------------
// A simple UART based printf function supporting \%c, \%d, \%p, \%s, \%u,
// \%x, and \%X.
//
// \param pcString is the format string.
// \param ... are the optional arguments, which depend on the contents of the
// format string.
//
// This function is very similar to the C library <tt>fprintf()</tt> function.
// All of its output will be sent to the UART.  Only the following formatting
// characters are supported:
//
// - \%c to print a character
// - \%d or \%i to print a decimal value
// - \%s to print a string
// - \%u to print an unsigned decimal value
// - \%x to print a hexadecimal value using lower case letters
// - \%X to print a hexadecimal value using lower case letters (not upper case
// letters as would typically be used)
// - \%p to print a pointer as a hexadecimal value
// - \%\% to print out a \% character
//
// For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
// between the \% and the format character, which specifies the minimum number
// of characters to use for that value; if preceded by a 0 then the extra
// characters will be filled with zeros instead of spaces.  For example,
// ``\%8d'' will use eight characters to print the decimal value with spaces
// added to reach eight; ``\%08d'' will use eight characters as well but will
// add zeroes instead of spaces.
//
// The type of the arguments after \e pcString must match the requirements of
// the format string.  For example, if an integer was passed where a string
// was expected, an error of some kind will most likely occur.
//---------------------------------------------------------------------------
void UARTprintf(UARTConsole* console, const char *pcString, ...)
{
	va_list vaArgP;

	// Start the varargs processing.
	va_start(vaArgP, pcString);

	UARTvprintf(console, pcString, vaArgP);

	// We're finished with the varargs now.
	va_end(vaArgP);
}

//-------------------------------------------
// UARTRxBytesAvail:
// Determines the number of bytes of data
// currently available in the receive buffer.
//-------------------------------------------
int UARTRxBytesAvail(UARTConsole* console)
{
	ASSERT(console != NULL);

	return(GetBufferCount(&console->UARTRxReadIndex, &console->UARTRxWriteIndex, UART_RX_BUFFER_SIZE));
}

//-------------------------------------------
// UARTTxBytesFree:
// Determines the amount of space currently
// available in the transmit buffer.
//-------------------------------------------
int UARTTxBytesFree(UARTConsole* console)
{
	ASSERT(console != NULL);

	return UART_TX_BUFFER_SIZE - GetBufferCount(&console->UARTTxReadIndex, &console->UARTTxWriteIndex, UART_TX_BUFFER_SIZE);
}

//-------------------------------------------
// Looks ahead in the receive buffer for a particular character.
//
// \param ucChar is the character that is to be searched for.
//
// This function, available only when the module is built to operate in
// buffered mode using \b UART_BUFFERED, may be used to look ahead in the
// receive buffer for a particular character and report its position if found.
// It is typically used to determine whether a complete line of user input is
// available, in which case ucChar should be set to CR ('\\r') which is used
// as the line end marker in the receive buffer.
//
// \return Returns -1 to indicate that the requested character does not exist
// in the receive buffer.  Returns a non-negative number if the character was
// found in which case the value represents the position of the first instance
// of \e ucChar relative to the receive buffer read pointer.
//-------------------------------------------
int UARTPeek(UARTConsole* console, unsigned char ucChar)
{
	int iCount;
	int iAvail;
	uint32_t ui32ReadIndex;

	ASSERT(console != NULL);

	// How many characters are there in the receive buffer?
	iAvail = (int)(GetBufferCount(&console->UARTRxReadIndex, &console->UARTRxWriteIndex, UART_RX_BUFFER_SIZE));
	ui32ReadIndex = console->UARTRxReadIndex;

	// Check all the unread characters looking for the one passed.
	for(iCount = 0; iCount < iAvail; iCount++)
	{
		if(console->UARTRxBuffer[ui32ReadIndex] == ucChar)
		{
			// We found it so return the index
			return(iCount);
		}
		else
		{
			// This one didn't match so move on to the next character.
			ADVANCE_RX_BUFFER_INDEX(ui32ReadIndex);
		}
	}

	// If we drop out of the loop, we didn't find the character in the receive buffer.
	return(-1);
}

//-------------------------------------------
// Flushes the receive buffer.
//
// This function, available only when the module is built to operate in
// buffered mode using \b UART_BUFFERED, may be used to discard any data
// received from the UART but not yet read using UARTgets().
//-------------------------------------------
void UARTFlushRx(UARTConsole* console)
{
	uint32_t ui32Int;

	ASSERT(console != NULL);

	// Temporarily turn off interrupts.
	ui32Int = MAP_IntMasterDisable();

	// Flush the receive buffer.
	console->UARTRxReadIndex = 0;
	console->UARTRxWriteIndex = 0;

	// If interrupts were enabled when we turned them off, turn them
	// back on again.
	if(!ui32Int)
		MAP_IntMasterEnable();
}

//-------------------------------------------
// Flushes the transmit buffer.
//
// \param bDiscard indicates whether any remaining data in the buffer should
// be discarded (\b true) or transmitted (\b false).
//
// This function, available only when the module is built to operate in
// buffered mode using \b UART_BUFFERED, may be used to flush the transmit
// buffer, either discarding or transmitting any data received via calls to
// UARTprintf() that is waiting to be transmitted.  On return, the transmit
// buffer will be empty.
//-------------------------------------------
void UARTFlushTx(UARTConsole* console, bool bDiscard)
{
	uint32_t ui32Int;

	ASSERT(console != NULL);

	// Should the remaining data be discarded or transmitted?
	if(bDiscard)
	{
		// The remaining data should be discarded, so temporarily turn off interrupts.
		ui32Int = MAP_IntMasterDisable();

		// Flush the transmit buffer.
		console->UARTTxReadIndex = 0;
		console->UARTTxWriteIndex = 0;

		// If interrupts were enabled when we turned them off, turn them back on again.
		if(!ui32Int)
			MAP_IntMasterEnable();
	}
	else
	{
		// Wait for all remaining data to be transmitted before returning.
		while(!IsBufferEmpty(&console->UARTTxReadIndex, &console->UARTTxWriteIndex)) { }
	}
}

//----------------------------------------------------------------------------
// Command line process:
// Process a command line string into arguments and execute the command.
// This function will take the supplied command line string and break it up
// into individual arguments. The first argument is treated as a command and
// is searched for in the command table. If the command is found, then the
// command function is called and all of the command line arguments are passed
// in the normal argc, argv form.
// Returns CMDLINE_BAD_CMD if the command is not found, CMDLINE_TOO_MANY_ARGS
// if there are more arguments than can be parsed. Otherwise it returns the
// code that was returned by the command function.
//----------------------------------------------------------------------------
static void CmdLineProcess(UARTConsole* console, char *input, uint32_t length)
{
	char *pcChar = input;
    bool bFindArg = true;
    CmdLineEntry *psCmdEntry;
	// Argument counter
    uint_fast8_t ui8Argc = 0;
    // Counter used to iterate over command table.
    uint32_t cntr = 0;

    // Advance through the command line until a zero character is found.
    while(cntr < length)
    {
        // If there is a space, then replace it with a zero, and set the flag
        // to search for the next argument.
        if(*pcChar == ' ')
        {
            *pcChar = 0;
            bFindArg = true;
        }
        // Otherwise it is not a space, so it must be a character that is part of an argument.
        else
        {
            // If bFindArg is set, then that means we are looking for the start of the next argument.
            if(bFindArg)
            {
                // As long as the maximum number of arguments has not been
                // reached, then save the pointer to the start of this new arg
                // in the argv array, and increment the count of args, argc.
                if(ui8Argc < CMDLINE_MAX_ARGS)
                {
                	console->Argv[ui8Argc] = pcChar;
                    ui8Argc++;
                    bFindArg = false;
                }
                // The maximum number of arguments has been reached so send the error and return.
                else
                {
                	UARTwrite(console, "Too many arguments.\n> ", 22);
                	return;
                }
            }
        }

        // Advance to the next character in the command line.
        pcChar++;
        cntr++;
    }

    // If one or more arguments was found, then process the command.
    if(ui8Argc)
    {
    	cntr = console->CmdTable.used;

    	// If help is received, execute hard coded help command and return.
    	if(!strcmp(console->Argv[0], "help"))
    	{
    		UARTwrite(console, "############################ HELP ############################\nUser may be able to abort executing commands by typing CTRL+C.\n\nAVAILABLE COMMANDS:\n - help:		List all available commands with its description.\n", 207);

    		for(cntr = 0; cntr < console->CmdTable.used; ++cntr)
    		{
    			UARTprintf(console, " - %s:		%s\n", console->CmdTable.array[cntr].name, console->CmdTable.array[cntr].help);
    		}
    		UARTwrite(console, "\n> ", 3);
    		return;
    	}

    	// Start at the beginning of the command table, to look for a matching command.
    	psCmdEntry = &(console->CmdTable.array[0]);

    	// Search through the command table.
    	for(cntr = 0; cntr < console->CmdTable.used; ++cntr)
    	{
    		// If this command entry command string matches argv[0], then call
    		// the function for this command, passing the command line arguments.
    		if(!strcmp(console->Argv[0], psCmdEntry->name))
    		{
    			console->CurrentlyRunningCmd = psCmdEntry;
    			psCmdEntry->app(ui8Argc, console->Argv);

    			// If cmd app didn't disabled command line interface, we ask user for entering a new command ('> ')
    			if(!console->CmdLineInterfaceDisabled)
    			{
    				UARTwrite(console, "\n\n> ", 4);
        			console->CurrentlyRunningCmd = NULL;
    			}

    			return;
    		}

    		// Not found, so advance to the next entry.
    		psCmdEntry++;
    	}
    }

    // Fall through to here means that no matching command was found, so send an error.
    UARTwrite(console, "Bad command.\n> ", 15);
}

