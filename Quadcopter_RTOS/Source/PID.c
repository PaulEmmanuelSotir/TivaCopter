/*
 * PID.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles

#include "inc/tm4c1294ncpdt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"

#include "PinMap.h"
#include "JSONCommunication.h"
#include "Utils/utils.h"
#include "IMU.h"
#include "PID.h"

//----------------------------------------
// IMU data structure from 'IMU.h'
//----------------------------------------
extern InertialMeasurementUnit IMU;

//----------------------------------------
// Motor data structures
//----------------------------------------
Motor Motors[4];

//----------------------------------------
// PID data structures
//----------------------------------------
PID YawPID, PitchPID, RollPID;

//-----------------------------------------
// String pointer array for PID data source
//-----------------------------------------
char* PIDStrPtrs[10] =  {	Motors[0].strPower, Motors[1].strPower, Motors[2].strPower, Motors[3].strPower,
							YawPID.strIn, PitchPID.strIn, RollPID.strIn,
							YawPID.strOut, PitchPID.strOut, RollPID.strOut	};

//----------------------------------------
// Data received from radio
//----------------------------------------
char* RadioIn[5] = { "0", "0", "0", "0", "0" };

//----------------------------------------
// GPIO Port E Hardware Interrupt (radio)
//----------------------------------------
void GPIOPEHwi(void)
{
	// Clear the GPIO interrupt.
	uint32_t intStatus = MAP_GPIOIntStatus(RADIO_PORT, true);
	MAP_GPIOIntClear(RADIO_PORT, intStatus);

	uint32_t data = GPIO_PORTE_AHB_DATA_R;

	RadioIn[0] = data & RADIO_CH1_PIN ? "1" : "0";
	RadioIn[1] = data & RADIO_CH2_PIN ? "1" : "0";
	RadioIn[2] = data & RADIO_CH3_PIN ? "1" : "0";
	RadioIn[3] = data & RADIO_CH4_PIN ? "1" : "0";
	RadioIn[4] = data & RADIO_CH5_PIN ? "1" : "0";
}

//----------------------------------------
// Radio data accessor:
// Accessor used by bluetooth to get data
// from Radio data source.
//----------------------------------------
char** RadioDataAccessor(void)
{
	return (char**)RadioIn;
}

//----------------------------------------
// PID data accessor:
// Accessor used by bluetooth to get data
// from PID data source.
//----------------------------------------
char** PIDDataAccessor(void)
{
	// Clear strings
	memset(YawPID.strOut, '\0', sizeof(YawPID.strOut));
	memset(PitchPID.strOut, '\0', sizeof(PitchPID.strOut));
	memset(RollPID.strOut, '\0', sizeof(RollPID.strOut));
	memset(YawPID.strIn, '\0', sizeof(YawPID.strIn));
	memset(PitchPID.strIn, '\0', sizeof(PitchPID.strIn));
	memset(RollPID.strIn, '\0', sizeof(RollPID.strIn));
	memset(Motors[0].strPower, '\0', sizeof(Motors[0].strPower));
	memset(Motors[1].strPower, '\0', sizeof(Motors[1].strPower));
	memset(Motors[2].strPower, '\0', sizeof(Motors[2].strPower));
	memset(Motors[3].strPower, '\0', sizeof(Motors[3].strPower));

	// Convert float values to strings
	ftoa(Motors[0].power , 	PIDStrPtrs[0], 4);
	ftoa(Motors[1].power , 	PIDStrPtrs[1], 4);
	ftoa(Motors[2].power , 	PIDStrPtrs[2], 4);
	ftoa(Motors[3].power , 	PIDStrPtrs[3], 4);
	ftoa(YawPID.in , 		PIDStrPtrs[4], 4);
	ftoa(PitchPID.in , 		PIDStrPtrs[5], 4);
	ftoa(RollPID.in , 		PIDStrPtrs[6], 4);
	ftoa(YawPID.out , 		PIDStrPtrs[7], 4);
	ftoa(PitchPID.out , 	PIDStrPtrs[8], 4);
	ftoa(RollPID.out , 		PIDStrPtrs[9], 4);

	return (char**)PIDStrPtrs;
}

//----------------------------------------
// PID task
//----------------------------------------
void PIDTask(void)
{
	// We just want the quadcopter to be horizontal when there isn't any remote control
	YawPID.in = 0.0f; PitchPID.in = 0.0f; RollPID.in = 0.0f;

	// Update radio inputs
	GPIOPEHwi();

	// Subscribe a bluetooth datasource to send periodically PID's data
	JSONDataSource* PID_ds = SubscribePeriodicJSONDataSource("PID", (const char*[]) {	"motor1", "motor2", "motor3", "motor4",
																						"YawIn", "PitchIn", "RollIn",
																						"YawOut", "PitchOut", "RollOut"}, 10, 20, PIDDataAccessor);

	// Subscribe a bluetooth datasource to send periodically Radio's data
	JSONDataSource* Radio_ds = SubscribePeriodicJSONDataSource("radio", (const char*[]) { "in0", "in1", "in2", "in3", "in4" }, 5, 40, RadioDataAccessor);

	while(1)
	{
		// TODO: savoir si il faudrais mettre ici un timout pour mettre la poussée des moteurs à 0.
		Semaphore_pend(PID_Sem, BIOS_WAIT_FOREVER);

		// Get error using euler angles from IMU
		YawPID.error = IMU.yaw - YawPID.in;
		PitchPID.error = IMU.pitch - PitchPID.in;
		RollPID.error = IMU.roll - RollPID.in;

		// If error is very small, we omit this
		if(YawPID.error < 0.0001 && YawPID.error > -0.0001)
			YawPID.error = 0;
		if(PitchPID.error < 0.0001 && PitchPID.error > -0.0001)
			PitchPID.error = 0;
		if(RollPID.error < 0.0001 && RollPID.error > -0.0001)
			RollPID.error = 0;

		// Integrate data and apply saturation
		YawPID.ITerm	+= 	YAW_KI		*	(YawPID.in + YawPID.lastIn)		*	(SAMPLE_PERIOD/2.0);
		PitchPID.ITerm	+=	PITCH_KI	*	(PitchPID.in + PitchPID.lastIn)	*	(SAMPLE_PERIOD/2.0);
		RollPID.ITerm	+= 	ROLL_KI		*	(RollPID.in + RollPID.lastIn)	*	(SAMPLE_PERIOD/2.0);
		if(YawPID.ITerm > YAW_I_LIMIT)				YawPID.ITerm	= 	YAW_I_LIMIT;
		else if(YawPID.ITerm < -YAW_I_LIMIT)		YawPID.ITerm	= 	-YAW_I_LIMIT;
		if(PitchPID.ITerm > PITCH_I_LIMIT)			PitchPID.ITerm	= 	PITCH_I_LIMIT;
		else if(PitchPID.ITerm < -PITCH_I_LIMIT)	PitchPID.ITerm	= 	-PITCH_I_LIMIT;
		if(RollPID.ITerm > ROLL_I_LIMIT)			RollPID.ITerm	= 	ROLL_I_LIMIT;
		else if(RollPID.ITerm < -ROLL_I_LIMIT)		RollPID.ITerm	= 	-ROLL_I_LIMIT;

		// Derivate data
		YawPID.DTerm	= 	YAW_KD		*	(YawPID.in - YawPID.lastIn)		*	SAMPLE_FREQ;
		PitchPID.DTerm	=	PITCH_KD	*	(PitchPID.in - PitchPID.lastIn)	*	SAMPLE_FREQ;
		RollPID.DTerm	= 	ROLL_KD		*	(RollPID.in - RollPID.lastIn)	*	SAMPLE_FREQ;

		// Sum each PID terms
		YawPID.out 		= (YAW_KP * YawPID.error) 		+ 	YawPID.ITerm 	+ 	YawPID.DTerm;
		PitchPID.out	= (PITCH_KP * PitchPID.error) 	+	PitchPID.ITerm 	+ 	PitchPID.DTerm;
		RollPID.out 	= (ROLL_KP * RollPID.error) 	+ 	RollPID.ITerm 	+ 	RollPID.DTerm;

		// Update last PID inputs
		YawPID.lastIn 	= YawPID.in;
		PitchPID.lastIn = PitchPID.in;
		RollPID.lastIn 	= RollPID.in;

		if(QuadControl.motorsOn)
		{
			// Convert euler angles to motors command
			Motors[0].power = - YawPID.out + PitchPID.out + RollPID.out + QuadControl.throttle;
			Motors[1].power =   YawPID.out - PitchPID.out + RollPID.out + QuadControl.throttle;
			Motors[2].power = - YawPID.out - PitchPID.out - RollPID.out + QuadControl.throttle;
			Motors[3].power =   YawPID.out + PitchPID.out - RollPID.out + QuadControl.throttle;

			// Limit output to its range (saturation)
			uint32_t i;
			for(i = 0; i < 4; ++i)
			{
				if(Motors[i].power > 1.0f) Motors[i].power = 1.0f;
				if(Motors[i].power < 0.0f) Motors[i].power = 0.0f;
			}
		}
		else
		{
			// Motors off
			Motors[0].power = 0;
			Motors[1].power = 0;
			Motors[2].power = 0;
			Motors[3].power = 0;
		}

		// Update PWM control of ESCs
		TimerMatchSet(TIMER2_BASE, TIMER_A, (Motors[0].power * (MAX_MOTOR - MIN_MOTOR))	+	MIN_MOTOR);
		TimerMatchSet(TIMER2_BASE, TIMER_B, (Motors[1].power * (MAX_MOTOR - MIN_MOTOR))	+	MIN_MOTOR);
		TimerMatchSet(TIMER3_BASE, TIMER_A, (Motors[2].power * (MAX_MOTOR - MIN_MOTOR))	+	MIN_MOTOR);
		TimerMatchSet(TIMER3_BASE, TIMER_B, (Motors[3].power * (MAX_MOTOR - MIN_MOTOR))	+	MIN_MOTOR);
	}
}
