/*
 * PinMap.c
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"

#include "PinMap.h"

//------------------------------------------
// PortFunctionInit
// TODO: configure all unused pins as
// outputs so they are not floating. (they
// may pick up noise and toggle at a high
// frequency).
//------------------------------------------
void PortFunctionInit()
{
    // Enable Peripheral Clocks
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

    // Enable radio channels 1(PE0), 2(PE1), 3(PE2), 4(PE3), 5(PE5)
    MAP_GPIOPinTypeGPIOInput(RADIO_PORT, RADIO_PIN_MASK);
    MAP_GPIOIntTypeSet(RADIO_PORT, RADIO_PIN_MASK, GPIO_BOTH_EDGES);
    MAP_GPIOIntEnable(RADIO_PORT, RADIO_PIN_MASK);

    // Enable buzzer output (PE4)
    MAP_GPIOPinTypeGPIOOutput(BUZZER_PORT, BUZZER_PIN);
    MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0x00);

    // Enable Bluetooth STATE pin (PK3) for GPIOInput
//    MAP_GPIOPinTypeGPIOInput(BLUETOOTH_STATE_PORT, BLUETOOTH_STATE_PIN);
//    MAP_GPIOIntEnable(BLUETOOTH_STATE_PORT, BLUETOOTH_STATE_INT_PIN);
    // Enable Bluetooth UART3 pins PA4(U3RX) and PA5(U3TX)
    MAP_GPIOPinConfigure(GPIO_PA4_U3RX);
    MAP_GPIOPinTypeUART(BLUETOOTH_UART_PORT, BLUETOOTH_RX_PIN);
    MAP_GPIOPinConfigure(GPIO_PA5_U3TX);
    MAP_GPIOPinTypeUART(BLUETOOTH_UART_PORT, BLUETOOTH_TX_PIN);

    // Enable Battery level ADC pins as follow: cell1=PK0(AIN16), cell2=PK1(AIN17), cell3=PK2(AIN18)
 /*   MAP_GPIOPinTypeADC(BATTERY_PORT, BATTERY_CELL1_PIN | BATTERY_CELL2_PIN | BATTERY_CELL3_PIN);
    // Enable the first sample sequencer to capture the value of channel 0 when the processor trigger occurs.
    MAP_ADCSequenceConfigure(BATTERY_ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    MAP_ADCSequenceStepConfigure(BATTERY_ADC_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    MAP_ADCSequenceEnable(BATTERY_ADC_BASE, 0);
    // Trigger the sample sequence.
    MAP_ADCProcessorTrigger(BATTERY_ADC_BASE, 0);
    // Wait until the sample sequence has completed.
    while(!ADCIntStatus(BATTERY_ADC_BASE, 0, false)) { }
    // Read the value from the ADC.
    uint32_t ui32Value;
    ADCSequenceDataGet(BATTERY_ADC_BASE, 0, &ui32Value);*/
    // Enable LED1(PN1), LED2(PN0),  LED2(PN0) and LED4(PF0)
    MAP_GPIOPinTypeGPIOOutput(LED1_PORT, LED1_PIN);
    MAP_GPIOPinTypeGPIOOutput(LED2_PORT, LED2_PIN);
    MAP_GPIOPinTypeGPIOOutput(LED3_PORT, LED3_PIN);
    MAP_GPIOPinTypeGPIOOutput(LED4_PORT, LED4_PIN);

    // Enable MPU6050 and HMC5883L I²C pins PB3(I2C0SDA) and PB2(I2C0SCL)
    MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    MAP_GPIOPinTypeI2C(IMU_I2C_PORT, IMU_SDA_PIN);
    MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    MAP_GPIOPinTypeI2CSCL(IMU_I2C_PORT, IMU_SCL_PIN);
    MAP_I2CMasterInitExpClk(IMU_I2C_BASE, CLOCK_FREQ, true);	// Enable and initialize the I2C0 master module. (High speed)
    MAP_IntEnable(IMU_I2C_INT);									// Enable the I2C interrupt.
    MAP_I2CMasterIntEnable(IMU_I2C_BASE);						// Enable the I2C master interrupt.

    // Configure ESC PWM control pins (ESC1=TIMER2A=PM0, ESC2=TIMER2B=PM1, ESC3=TIMER3A=PM2, ESC4=TIMER3B=PM3)
    MAP_GPIOPinConfigure(ESC1_TIMER_GPIO);
    MAP_GPIOPinTypeTimer(ESC1_PORT, ESC1_PIN);
    MAP_GPIOPinConfigure(ESC2_TIMER_GPIO);
    MAP_GPIOPinTypeTimer(ESC2_PORT, ESC2_PIN);
    MAP_GPIOPinConfigure(ESC3_TIMER_GPIO);
    MAP_GPIOPinTypeTimer(ESC3_PORT, ESC3_PIN);
    MAP_GPIOPinConfigure(ESC4_TIMER_GPIO);
    MAP_GPIOPinTypeTimer(ESC4_PORT, ESC4_PIN);
    // Configure ESC PWM timers clock source to the precision internal oscillator wich permits lower frequencies
    // than the system clock (16MHz instead of 120MHz).
    MAP_TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_PIOSC);
    MAP_TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_PIOSC);
    // Configure ESC PWM timers
    MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    // Set the load for 400Hz ESC control.
    MAP_TimerLoadSet(ESC1_TIMER_BASE, TIMER_A, PIOSC_FREQ/400);
    MAP_TimerLoadSet(ESC2_TIMER_BASE, TIMER_B, PIOSC_FREQ/400);
    MAP_TimerLoadSet(ESC3_TIMER_BASE, TIMER_A, PIOSC_FREQ/400);
    MAP_TimerLoadSet(ESC4_TIMER_BASE, TIMER_B, PIOSC_FREQ/400);
    // Invert PWM output level so that timer’s output is made active low (simplifies TimerMatchSet calls)
    MAP_TimerControlLevel(ESC1_TIMER_BASE, TIMER_A, true);
    MAP_TimerControlLevel(ESC2_TIMER_BASE, TIMER_B, true);
    MAP_TimerControlLevel(ESC3_TIMER_BASE, TIMER_A, true);
    MAP_TimerControlLevel(ESC4_TIMER_BASE, TIMER_B, true);
    // Set match for 1ms pulses (duty cycle : 1ms/2.5ms)
    MAP_TimerMatchSet(ESC1_TIMER_BASE, TIMER_A, PIOSC_FREQ*0.001);
    MAP_TimerMatchSet(ESC2_TIMER_BASE, TIMER_B, PIOSC_FREQ*0.001);
    MAP_TimerMatchSet(ESC3_TIMER_BASE, TIMER_A, PIOSC_FREQ*0.001);
    MAP_TimerMatchSet(ESC4_TIMER_BASE, TIMER_B, PIOSC_FREQ*0.001);
    // Enable the timers.
    MAP_TimerEnable(ESC1_TIMER_BASE, TIMER_A);
    MAP_TimerEnable(ESC2_TIMER_BASE, TIMER_B);
    MAP_TimerEnable(ESC3_TIMER_BASE, TIMER_A);
    MAP_TimerEnable(ESC4_TIMER_BASE, TIMER_B);

	// Enable processor interrupts.
    MAP_IntMasterEnable();
}
