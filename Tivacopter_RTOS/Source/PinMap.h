/*
 * PinMap.h
 */

#ifndef PINMAP_H__
#define PINMAP_H__

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"

//------------------------------------------
// System clock and precision internal
// oscillator frequencies
//------------------------------------------
#define CLOCK_FREQ					120000000
#define PIOSC_FREQ					16000000
#define SYSTEM_CLOCK_PERIOD_US		2500					// 2500 us
#define SYSTEM_CLOCK_FREQ			1/SYSTEM_CLOCK_PERIOD 	// 400 Hz

//------------------------------------------
// Board's LEDs used for battery level
//------------------------------------------
//LED 1
#define LED1_PORT         		GPIO_PORTN_BASE
#define LED1_PIN          		GPIO_PIN_1
//LED 2
#define LED2_PORT         		GPIO_PORTN_BASE
#define LED2_PIN          		GPIO_PIN_0
//LED 3
#define LED3_PORT         		GPIO_PORTF_BASE
#define LED3_PIN          		GPIO_PIN_4
//LED 4
#define LED4_PORT         		GPIO_PORTF_BASE
#define LED4_PIN          		GPIO_PIN_0

//------------------------------------------
// Board user switches
//------------------------------------------
#define U_SW_PORT				GPIO_PORTJ_BASE
#define U_SW1_PIN				GPIO_PIN_0
#define U_SW2_PIN				GPIO_PIN_1

//------------------------------------------
// Bluetooth module (UART and State Pin)
//------------------------------------------
//#define BLUETOOTH_STATE_PORT	GPIO_PORTK_BASE
//#define BLUETOOTH_STATE_PIN		GPIO_PIN_3
//#define BLUETOOTH_STATE_INT_PIN	GPIO_INT_PIN_3
#define BLUETOOTH_UART_BASE_NUM	3
#define BLUETOOTH_UART_BAUDRATE	460800
#define BLUETOOTH_UART_BASE		UART3_BASE
#define BLUETOOTH_UART_PORT		GPIO_PORTA_BASE
#define BLUETOOTH_UART_INT		INT_UART3
#define BLUETOOTH_RX_PIN		GPIO_PIN_4
#define BLUETOOTH_TX_PIN		GPIO_PIN_5

//------------------------------------------
// MPU6050 and HMC5883L (I²C 0)
//------------------------------------------
#define IMU_I2C_PORT			GPIO_PORTB_BASE
#define IMU_I2C_BASE			I2C0_BASE
#define IMU_I2C_INT				INT_I2C0
#define IMU_SDA_PIN				GPIO_PIN_3
#define IMU_SCL_PIN				GPIO_PIN_2

//------------------------------------------
// ESCs (4xPWM)
//------------------------------------------
// ESC 1
#define ESC1_TIMER_BASE			TIMER2_BASE
#define ESC1_TIMER_INT			INT_TIMER2A
#define ESC1_PORT				GPIO_PORTM_BASE
#define ESC1_PIN				GPIO_PIN_0
#define ESC1_TIMER_GPIO			GPIO_PM0_T2CCP0
// ESC 2
#define ESC2_TIMER_BASE			TIMER2_BASE
#define ESC2_TIMER_INT			INT_TIMER2B
#define ESC2_TIMERB_INT			INT_TIMER3B
#define ESC2_PORT				GPIO_PORTM_BASE
#define ESC2_PIN				GPIO_PIN_1
#define ESC2_TIMER_GPIO			GPIO_PM1_T2CCP1
// ESC 3
#define ESC3_TIMER_BASE			TIMER3_BASE
#define ESC3_TIMER_INT			INT_TIMER3A
#define ESC3_PORT				GPIO_PORTM_BASE
#define ESC3_PIN				GPIO_PIN_2
#define ESC3_TIMER_GPIO			GPIO_PM2_T3CCP0
// ESC 4
#define ESC4_TIMER_BASE			TIMER3_BASE
#define ESC4_TIMER_INT			INT_TIMER3B
#define ESC4_PORT				GPIO_PORTA_BASE
#define ESC4_TIMER_GPIO			GPIO_PA7_T3CCP1
#define ESC4_PIN				GPIO_PIN_7

//------------------------------------------
// Radio (5xDigital IN)
//------------------------------------------
#define RADIO_PORT				GPIO_PORTE_BASE
#define RADIO_CH1_PIN			GPIO_PIN_0
#define RADIO_CH2_PIN			GPIO_PIN_1
#define RADIO_CH3_PIN			GPIO_PIN_2
#define RADIO_CH4_PIN			GPIO_PIN_3
#define RADIO_CH5_PIN			GPIO_PIN_5
#define RADIO_PIN_MASK			RADIO_CH1_PIN | RADIO_CH2_PIN | RADIO_CH3_PIN | RADIO_CH4_PIN | RADIO_CH5_PIN

//------------------------------------------
// Buzzer
//------------------------------------------
#define BUZZER_PORT				GPIO_PORTE_BASE
#define BUZZER_PIN				GPIO_PIN_4

//------------------------------------------
// Battery cells levels (ADC)
//------------------------------------------
#define BATTERY_PORT			GPIO_PORTK_BASE
#define BATTERY_ADC_BASE		ADC0_BASE
#define BATTERY_CELL1_PIN		GPIO_PIN_0
#define BATTERY_CELL2_PIN		GPIO_PIN_1
#define BATTERY_CELL3_PIN		GPIO_PIN_2

//------------------------------------------
// TODO: GPS module
//------------------------------------------

//------------------------------------------
// PortFunctionInit
//------------------------------------------
void PortFunctionInit();

#endif //  PINMAP_H__
