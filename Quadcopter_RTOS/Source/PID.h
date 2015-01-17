/*
 * PID.h
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// We use 'SAMPLE_FREQ' define from 'IMU.h' as PID's integration and derivation is triggered by IMU task
#include "IMU.h"
#include "PinMap.h"

//----------------------------------------
// Maximum and minimum motor command
//----------------------------------------
#define MAX_MOTOR				PIOSC_FREQ*0.002
#define MIN_MOTOR				PIOSC_FREQ*0.001
#define MOTOR1_POWER_OFFSET		0.1845f
#define MOTOR2_POWER_OFFSET		0.1075f
#define MOTOR3_POWER_OFFSET		0.2330f
#define MOTOR4_POWER_OFFSET		0.1080f

//----------------------------------------
// PID data structure
//----------------------------------------
typedef struct PID
{
	const float Kp;
	const float Ki;
	const float Kd;

	float ITerm;
	const float ILimit;
	float DTerm;

	float in;
	char strIn[10];
	float lastIn;

	float out;
	char strOut[10];

	float error;
} PID;

//----------------------------------------
// Quadcopter control structure
//----------------------------------------
typedef struct
{
	// Global motor throttle control
	float throttle;
	// Moving direction vector in horizontal plane
	float direction[2];
	// The orientation of quadricopter around z axis in radians
	float yaw;
	bool yawRegulationEnabled;
	// Klaxon !
	bool beep;
	// Flag that must be raisezd if any problem occurs and motors must be stopped
	bool ShutOffMotors;
	// Boolean indicating wether if radio control is enabled.
	bool RadioControlEnabled;
}QuadControl;

//----------------------------------------
// Motor PWM control structure typedef
//----------------------------------------
typedef struct
{
	char strPower[10];
	float power;
}Motor;

//----------------------------------------
// GPIO Port E Hardware Interrupt (radio)
//----------------------------------------
void GPIOPEHwi(void);

//----------------------------------------
// Radio data accessor:
// Accessor used by bluetooth to get data
// from Radio data source.
//----------------------------------------
char** RadioDataAccessor(void);

//----------------------------------------
// PID data accessor:
// Accessor used by bluetooth to get data
// from PID data source.
//----------------------------------------
char** PIDDataAccessor(void);

//----------------------------------------
// PID task
//----------------------------------------
void PIDTask(void);

#endif /* PID_H_ */
