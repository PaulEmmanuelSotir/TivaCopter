/*
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles

//------------------------------------------
// TivaWare Header Files
//------------------------------------------
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "string.h"

#include "Utils\UARTConsole.h"
#include "PinMap.h"
#include "CmdLineWarper.h"

#define MIN_BATTERY_LVL 50
#define MAX_BATTERY_LVL 1000

#ifdef DEBUG
//-------------------------------------------
// The error routine that is called if the
// driver library encounters an error.(DEBUG)
//-------------------------------------------
void __error__(char *pcFilename, uint32_t ui32Line)
{
	Log_error2("Error in \"%s\" at line %d", (xdc_IArg)pcFilename, ui32Line);
	while(1);
}
#endif

// Bluetooth UART command line interface
UARTConsole Console;
// UART interrupt status needed by UART console interrupt handler
static uint32_t IntStatus;

//------------------------------------------
// Main
//------------------------------------------
int main(void)
{
	// Run from the PLL at 120 MHz.
	uint32_t ClockFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), CLOCK_FREQ);
	ASSERT(CLOCK_FREQ == ClockFreq);

	// Enable floating-point unit except for interrupts as we dont use any floating-point instructions into interrupt handlers.
	FPUEnable();
	FPUStackingDisable();

	// Initialize peripherals
	PortFunctionInit();

	// Configure UART console
	UARTConsoleConfig(&Console, BLUETOOTH_UART_BASE_NUM, CLOCK_FREQ, BLUETOOTH_UART_BAUDRATE);

	// Add command line API warper commands to UART console
	SubscribeWarperCmds();

    BIOS_start();

    return(0);
}

//----------------------------------------
// Bluetooth UART console task:
// Handles command line interface via UART
//----------------------------------------
void UARTConsoleTask(void)
{
	while(1)
	{
		Semaphore_pend(UARTConsole_Sem, BIOS_WAIT_FOREVER);

		ConsoleUARTIntHandler(&Console, IntStatus);
	}
}

//----------------------------------------
// UART3 interrupt handler
//----------------------------------------
void UART3IntHandler(void)
{
	// Get and clear the current interrupt source(s)
	IntStatus = MAP_UARTIntStatus(BLUETOOTH_UART_BASE, true);
	MAP_UARTIntClear(BLUETOOTH_UART_BASE, IntStatus);

	// As bluetooth communication isn't critical, we handle it in a low priority task rather than in this software interrupt.
	Semaphore_post(UARTConsole_Sem);
}

//------------------------------------------
// Beeper state control
//------------------------------------------
void beep(bool state)
{
	if(state)
		MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
	else
		MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0x00);
}

//------------------------------------------
// Battery level periodic interrupt.
// TODO: le faire >.<
//------------------------------------------
void BatteryLevelSwi(void)
{
	static bool beepState = false;
	static uint8_t cool = 0x00;

	int32_t batteryLvl = -1;

	if(batteryLvl < 0)
	{
		switch(cool++)
		{
		case 0:
			GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);
			GPIOPinWrite(LED2_PORT, LED2_PIN, 0x00);
			GPIOPinWrite(LED3_PORT, LED3_PIN, 0x00);
			GPIOPinWrite(LED4_PORT, LED4_PIN, 0x00);
			break;
		case 1:
			GPIOPinWrite(LED1_PORT, LED1_PIN, 0x00);
			GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN);
			GPIOPinWrite(LED3_PORT, LED3_PIN, 0x00);
			GPIOPinWrite(LED4_PORT, LED4_PIN, 0x00);
			break;
		case 2:
			GPIOPinWrite(LED1_PORT, LED1_PIN, 0x00);
			GPIOPinWrite(LED2_PORT, LED2_PIN, 0x00);
			GPIOPinWrite(LED3_PORT, LED3_PIN, LED3_PIN);
			GPIOPinWrite(LED4_PORT, LED4_PIN, 0x00);
			break;
		case 3:
			GPIOPinWrite(LED1_PORT, LED1_PIN, 0x00);
			GPIOPinWrite(LED2_PORT, LED2_PIN, 0x00);
			GPIOPinWrite(LED3_PORT, LED3_PIN, 0x00);
			GPIOPinWrite(LED4_PORT, LED4_PIN, LED4_PIN);
			break;
		}
		cool %= 4;
	}
	else if(batteryLvl < MIN_BATTERY_LVL)
	{
		beep(beepState);
		beepState = !beepState;
	}
	else
		beep(false);
}

/*
 * GPIOPK_Hwi :
 * Interrupt number = 68
//----------------------------------------
// Bluetooth state flag:
// This is raised when bluetooth module is
// combined/connected with/to a master.
//----------------------------------------
uint32_t BluetoothState_Flag = 0;

//-----------------------------------------
// GPIO PK hardware interrupt handler (PK3)
//-----------------------------------------
GPIOPKHwiHandler(void)
{
	if(GPIOPinRead(BLUETOOTH_STATE_PORT, BLUETOOTH_STATE_PIN))
		BluetoothState_Flag = 1;
	else
		BluetoothState_Flag = 0;
}*/
