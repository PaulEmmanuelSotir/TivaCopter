/*
 * PID.h
 */

#ifndef PID_H_
#define PID_H_

// We use 'SAMPLE_FREQ' define from 'IMU.h' as PID's integration and derivation is triggered by IMU task
#include "IMU.h"
#include "PinMap.h"

//----------------------------------------
// Pitch/roll and yaw PID gains:
// We use the same gains for pitch
// and roll as quadcopter is symetric.
//----------------------------------------
// Yaw
#define	YAW_KP			0.03500
#define	YAW_KI			0.03500
#define	YAW_KD			0.000000
#define	YAW_I_LIMIT		0.300000
// Pitch
#define	PITCH_KP		0.04000
#define	PITCH_KI		0.12000
#define	PITCH_KD		0.00010
#define	PITCH_I_LIMIT	0.300000
// roll
#define	ROLL_KP			PITCH_KP
#define	ROLL_KI			PITCH_KI
#define	ROLL_KD			PITCH_KD
#define	ROLL_I_LIMIT	PITCH_I_LIMIT

//----------------------------------------
// Maximum absolute command euler angles
//----------------------------------------
#define MAX_YAW			PI
#define MAX_PITCH		PI
#define MAX_ROLL		PI

//----------------------------------------
// Maximum and minimum motor command
//----------------------------------------
#define MAX_MOTOR		PIOSC_FREQ*0.002
#define MIN_MOTOR		PIOSC_FREQ*0.001

//----------------------------------------
// PID data structure
//----------------------------------------
typedef struct PID
{
	float ITerm;
	float DTerm;

	float in;
	char strIn[10];
	float lastIn;

	float out;
	char strOut[10];

	float error;
} PID;

struct
{
	bool motorsOn;
	// Global motor throttle control
	float throttle;
	// Normalized moving direction in horizontal plane
	float direction[2];
	// Moving norm in horizontal plane
	float norm;
	// Minimal throttle or horizontal moving percentage to affect quadcopter's behavior
	float deadzone;
} QuadControl = {.motorsOn= true, .throttle = 0.25, .direction = {1, 0}, .norm = 0, .deadzone = 0.10};

//----------------------------------------
// Motor PWM control
//----------------------------------------
typedef struct
{
	char strPower[10];
	float power;
} Motor;

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
