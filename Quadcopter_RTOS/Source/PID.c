/*
 * PID.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

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

#include "Math.h"
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
static Motor Motors[4];
static QuadControl TivacopterControl = {.RadioControlEnabled = true};

//----------------------------------------
// PID data structures
// TODO: determine PIDs gains
//----------------------------------------
static PID YawPID =		{ .Kp = 0.035,	.Ki = 0.035,	.Kd = 0.0,		.ILimit = 0.30};
static PID PitchPID =	{ .Kp = 0.16,	.Ki = 0.48,		.Kd = 0.0004,	.ILimit = 1.20};//{ .Kp = 0.04,	.Ki = 0.12,		.Kd = 0.0001,	.ILimit = 0.30};
static PID RollPID =	{ .Kp = 0.16,	.Ki = 0.48,		.Kd = 0.0004,	.ILimit = 1.20};//{ .Kp = 0.04,	.Ki = 0.12,		.Kd = 0.0001,	.ILimit = 0.30};

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
static bool RadioInputUpdatedFlag = false;

//------------------------------------------
// Static function forward declarations
//------------------------------------------
static inline void ProcessPID(PID* pid);
static void TurnOffMotors(void);
static void MapRadioInputToQuadcopterControl(void);

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

	RadioInputUpdatedFlag = true;
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
	ftoa(Motors[0].power, 	PIDStrPtrs[0], 4);
	ftoa(Motors[1].power, 	PIDStrPtrs[1], 4);
	ftoa(Motors[2].power, 	PIDStrPtrs[2], 4);
	ftoa(Motors[3].power, 	PIDStrPtrs[3], 4);
	ftoa(YawPID.in, 		PIDStrPtrs[4], 4);
	ftoa(PitchPID.in, 		PIDStrPtrs[5], 4);
	ftoa(RollPID.in, 		PIDStrPtrs[6], 4);
	ftoa(YawPID.out, 		PIDStrPtrs[7], 4);
	ftoa(PitchPID.out, 		PIDStrPtrs[8], 4);
	ftoa(RollPID.out, 		PIDStrPtrs[9], 4);

	return (char**)PIDStrPtrs;
}

//----------------------------------------
// Remote control data set accessor:
// Accessor used by bluetooth to set data
// from remote control data input.
//----------------------------------------
void RemoteControlDataAccessor(char** RemoteCtrlKeys)
{
	TivacopterControl.throttle = atof(RemoteCtrlKeys[0]);
	U_SAT(TivacopterControl.throttle, 1.0f);

	TivacopterControl.direction[x] = atof(RemoteCtrlKeys[1]);
	U_SAT(TivacopterControl.direction[x], 1.0f);

	TivacopterControl.direction[y] = atof(RemoteCtrlKeys[2]);
	U_SAT(TivacopterControl.direction[y], 1.0f);

	TivacopterControl.yaw = atof(RemoteCtrlKeys[3]);
	U_SAT(TivacopterControl.yaw, 1.0f);

	TivacopterControl.beep = 't' == tolower(RemoteCtrlKeys[4][0])
					&& 'r' == tolower(RemoteCtrlKeys[4][1])
					&& 'u' == tolower(RemoteCtrlKeys[4][2])
					&& 'e' == tolower(RemoteCtrlKeys[4][3]);
}

//----------------------------------------
// PID task
//----------------------------------------
void PIDTask(void)
{
	// We just want the quadcopter to be horizontal (no radio control)
	YawPID.in = 0; PitchPID.in = 0; RollPID.in = 0;

	// Update radio inputs
	GPIOPEHwi();

	// Suscribe a bluetooth datasource to send periodically PID's data
	JSONDataSource* PID_ds = SuscribePeriodicJSONDataSource("PID", (const char*[]) {	"motor1", "motor2", "motor3", "motor4",
																						"YawIn", "PitchIn", "RollIn",
																						"YawOut", "PitchOut", "RollOut"}, 10, 20, PIDDataAccessor);

	// Suscribe a bluetooth datasource to send periodically Radio's data
	JSONDataSource* Radio_ds = SuscribePeriodicJSONDataSource("radio", (const char*[]) { "in0", "in1", "in2", "in3", "in4" }, 5, 40, RadioDataAccessor);

	// Subscribe a bluetooth datainput to receive remote control data
	JSONDataInput* RemoteControl_di = SubscribeJSONDataInput("RemoteControl", (const char*[]) { "throttle", "directionX", "directionY", "yaw", "beep" }, 5, RemoteControlDataAccessor);

	while(1)
	{
		// TODO: savoir si il faudrais mettre ici un timout pour mettre la poussée des moteurs à 0.
		Semaphore_pend(PID_Sem, BIOS_WAIT_FOREVER);

		if(TivacopterControl.ShutOffMotors)
		{
			// Turn off motors if any problem occurs
			TurnOffMotors();
			return;
		}

		if(TivacopterControl.RadioControlEnabled && RadioInputUpdatedFlag)
			MapRadioInputToQuadcopterControl();

		// Map TivacopterControl to PIDs input
		YawPID.in = TivacopterControl.yaw;
		PitchPID.in = PI/4 * TivacopterControl.direction[x];
		RollPID.in = PI/4 * TivacopterControl.direction[y];

		// Get error using euler angles from IMU
		PitchPID.error = IMU.pitch - PitchPID.in;
		RollPID.error = IMU.roll - RollPID.in;

		ProcessPID(&PitchPID);
		ProcessPID(&RollPID);

		// Convert euler angles to motors command
		Motors[0].power =   PitchPID.out + RollPID.out + TivacopterControl.throttle;
		Motors[1].power = - PitchPID.out + RollPID.out + TivacopterControl.throttle;
		Motors[2].power = - PitchPID.out - RollPID.out + TivacopterControl.throttle;
		Motors[3].power =   PitchPID.out - RollPID.out + TivacopterControl.throttle;

		if(TivacopterControl.yawRegulationEnabled)
		{
			YawPID.error = IMU.yaw - YawPID.in;
			ProcessPID(&YawPID);
			Motors[0].power -= YawPID.out;
			Motors[1].power += YawPID.out;
			Motors[2].power -= YawPID.out;
			Motors[3].power += YawPID.out;
		}

		// Limit motors power to its range
		U_SAT(Motors[0].power, 1.0f);
		U_SAT(Motors[1].power, 1.0f);
		U_SAT(Motors[2].power, 1.0f);
		U_SAT(Motors[3].power, 1.0f);

		// Map motors power into their real range (from measured minimum power to start each motors)
		Motors[0].power = Motors[0].power * (1.0f-MOTOR1_POWER_OFFSET) + MOTOR1_POWER_OFFSET;
		Motors[1].power = Motors[1].power * (1.0f-MOTOR2_POWER_OFFSET) + MOTOR2_POWER_OFFSET;
		Motors[2].power = Motors[2].power * (1.0f-MOTOR3_POWER_OFFSET) + MOTOR3_POWER_OFFSET;
		Motors[3].power = Motors[3].power * (1.0f-MOTOR4_POWER_OFFSET) + MOTOR4_POWER_OFFSET;

		// Update PWM control of ESCs
		TimerMatchSet(TIMER2_BASE, TIMER_A, (Motors[0].power * (MAX_MOTOR - MIN_MOTOR))	+ MIN_MOTOR);
		TimerMatchSet(TIMER2_BASE, TIMER_B, (Motors[1].power * (MAX_MOTOR - MIN_MOTOR))	+ MIN_MOTOR);
		TimerMatchSet(TIMER3_BASE, TIMER_A, (Motors[2].power * (MAX_MOTOR - MIN_MOTOR))	+ MIN_MOTOR);
		TimerMatchSet(TIMER3_BASE, TIMER_B, (Motors[3].power * (MAX_MOTOR - MIN_MOTOR))	+ MIN_MOTOR);
	}
}

//----------------------------------------
// Process PID:
// Process given PID structure's output
// from its error and input.
// Uses SAMPLE_FREQ to integrate and
// derive data.
// TODO: savoir si ce inline est utile
//----------------------------------------
static inline void ProcessPID(PID* pid)
{
	// If error is very small, we omit this
	if((*pid).error < 0.0001 && (*pid).error > -0.0001)
		(*pid).error = 0;

	// Integrate data and apply saturation
	(*pid).ITerm += (*pid).Ki * ((*pid).in + (*pid).lastIn) * (SAMPLE_PERIOD/2.0);
	SAT((*pid).ITerm, (*pid).ILimit);

	// Derivate data
	(*pid).DTerm = (*pid).Kd * ((*pid).in - (*pid).lastIn) * SAMPLE_FREQ;

	// Sum each PID terms
	(*pid).out = ((*pid).Kp * (*pid).error) + (*pid).ITerm + (*pid).DTerm;

	// Update last PID inputs
	(*pid).lastIn = (*pid).in;
}

//----------------------------------------
// Turn off motors
//----------------------------------------
static void TurnOffMotors(void)
{
	TimerMatchSet(TIMER2_BASE, TIMER_A, MIN_MOTOR);
	TimerMatchSet(TIMER2_BASE, TIMER_B, MIN_MOTOR);
	TimerMatchSet(TIMER3_BASE, TIMER_A, MIN_MOTOR);
	TimerMatchSet(TIMER3_BASE, TIMER_B, MIN_MOTOR);
}

//----------------------------------------
// Map radio input to quadcopter control
// TODO: find a safer and handy way to
// control quadcopter via 5CHs radio!
//----------------------------------------
static void MapRadioInputToQuadcopterControl(void)
{
	if(RadioIn[0] == "1")
	{
		TivacopterControl.throttle += 0.0005;
		U_SAT(TivacopterControl.throttle, 1.0f);
	}
	else
		TivacopterControl.throttle = 0;

	if(RadioIn[1] == "1")
	{
		TivacopterControl.direction[x] += 0.0005;
		SAT(TivacopterControl.direction[x], 1.0f);
	}
	else if(RadioIn[2] == "1")
	{
		TivacopterControl.direction[x] -= 0.0005;
		SAT(TivacopterControl.throttle, 1.0f);
	}
	else
		TivacopterControl.direction[x] = 0;

	if(RadioIn[3] == "1")
	{
		TivacopterControl.direction[y] += 0.0005;
		SAT(TivacopterControl.throttle, 1.0f);
	}
	else if(RadioIn[4] == "1")
	{
		TivacopterControl.direction[y] -= 0.0005;
		SAT(TivacopterControl.throttle, 1.0f);
	}
	else
		TivacopterControl.direction[y] = 0;

	// When we control quadcopter by radio, the quadcopter orientation is always ahead
	TivacopterControl.yaw = atan2(TivacopterControl.direction[y], TivacopterControl.direction[x]);

}
