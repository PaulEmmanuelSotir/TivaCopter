/*
 * IMU.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles
#include <ti/sysbios/gates/GateMutexPri.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "driverlib/debug.h"

#include "math.h"
#include "Utils/conversion.h"
#include "Utils/I2CTransaction.h"
#include "Utils/UARTConsole.h"
#include "JSONCommunication.h"
#include "PinMap.h"
#include "IMU.h"

//----------------------------------------
// UART console from 'main.c'
//----------------------------------------
extern UARTConsole Console;

//----------------------------------------
// Enum allowing clearer data accesses
//----------------------------------------
enum { x, y, z };

//----------------------------------------
// IMU data structures definition
//----------------------------------------
static Magnetometer Magn = {.range = _1300mGa, .xOffset = -27.034f, .yOffset = 59.649f, .zOffset = 149.464f,
							.M = {	{0.324, 	0, 		2.173},
									{-0.412, 	1.016, 	0.387},
									{-2.266, 	-0.043,	0.686}	}};
static Gyroscope Gyro = {.range = _250dps, .xOffset = 0.0f, .yOffset = 0.0f, .zOffset = 0.0f};
static Accelerometer Accel = {.range = _4g};
InertialMeasurementUnit IMU = {	.magn = &Magn, .accel = &Accel, . gyro = &Gyro,
								.q = {1.0, 0.0, 0.0, 0.0},
								.SensorsStrValues = {"0", "0", "0", "0", "0", "0", "0", "0", "0"},
								.IMUStrValues = {"1", "0", "0", "0"}};

//----------------------------------------
// Lock function used by I2C transaction
// API to protect its ressources from
// multithread accesses.
//----------------------------------------
intptr_t I2CTransactionsLock(void)
{
	intptr_t lock = GateMutexPri_enter(I2CTransactionsGateMutexPri);
	return lock;
}

//----------------------------------------
// Unlock function used by I2C transaction
// API to protect its ressources from
// multithread accesses.
//----------------------------------------
void I2CTransactionUnlock(intptr_t lock)
{
	GateMutexPri_leave(I2CTransactionsGateMutexPri, lock);
}

//----------------------------------------
// I²C0 Hardware Interrupt
//----------------------------------------
void I2C0HwiHandler(void)
{
	// Clear the I2C interrupt.
	I2CMasterIntClear(IMU_I2C_BASE);

	// Run I2C state machine task
	Semaphore_post(I2CStateMachine_Sem);
}

//----------------------------------------
// I2C State Machine Task
//----------------------------------------
void I2CStateMachineTask(void)
{
	while(1)
	{
		Semaphore_pend(I2CStateMachine_Sem, BIOS_WAIT_FOREVER);

		// Update I2C interrupt state machine.
		I2CIntStateMachine();
	}
}

//----------------------------------------
// MagnetoCompensate:
// Performs hard- and soft-iron
// compensation on magnetometer readings.
//----------------------------------------
static void MagnetoCompensate()
{
	// Center magnetometer data
	const float CntrMagnX = Magn.val[x] - Magn.xOffset;
	const float CntrMagnY = Magn.val[y] - Magn.yOffset;
	const float CntrMagnZ = Magn.val[z] - Magn.zOffset;

	// Apply transformation matrix
	Magn.val[x] = Magn.M[0][0]*CntrMagnX + Magn.M[0][1]*CntrMagnY + Magn.M[0][2]*CntrMagnZ;
	Magn.val[y] = Magn.M[1][0]*CntrMagnX + Magn.M[1][1]*CntrMagnY + Magn.M[1][2]*CntrMagnZ;
	Magn.val[z] = Magn.M[2][0]*CntrMagnX + Magn.M[2][1]*CntrMagnY + Magn.M[2][2]*CntrMagnZ;
}

//-----------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//-----------------------------------------------------------
static float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//----------------------------------------
// Sensors data accessor:
// Accessor used by JSON communication
// to get data from sensors data source.
//----------------------------------------
static char** SensorsDataAccessor(void)
{
	// Clear strings
	memset(IMU.SensorsStrValues, '\0', sizeof(IMU.SensorsStrValues));

	// Convert float values to strings
	ftoa(Accel.val[x], IMU.SensorsStrPtrs[0], 4);
	ftoa(Accel.val[y], IMU.SensorsStrPtrs[1], 4);
	ftoa(Accel.val[z], IMU.SensorsStrPtrs[2], 4);
	ftoa(Gyro.val[x], IMU.SensorsStrPtrs[3], 4);
	ftoa(Gyro.val[y], IMU.SensorsStrPtrs[4], 4);
	ftoa(Gyro.val[z], IMU.SensorsStrPtrs[5], 4);
	ftoa(Magn.val[x], IMU.SensorsStrPtrs[6], 4);
	ftoa(Magn.val[y], IMU.SensorsStrPtrs[7], 4);
	ftoa(Magn.val[z], IMU.SensorsStrPtrs[8], 4);

	return (char**)IMU.SensorsStrPtrs;
}

//----------------------------------------
// IMU data accessor:
// Accessor used by JSON communication to
// get data from IMU data source.
//----------------------------------------
static char** IMUDataAccessor(void)
{
	// Clear strings
	memset(IMU.IMUStrValues, '\0', sizeof(IMU.IMUStrValues));

	// Convert float values to strings
	ftoa(IMU.q[0], 	IMU.IMUStrPtrs[0], 5);
	ftoa(IMU.q[1], 	IMU.IMUStrPtrs[1], 5);
	ftoa(IMU.q[2], 	IMU.IMUStrPtrs[2], 5);
	ftoa(IMU.q[3], 	IMU.IMUStrPtrs[3], 5);
	ftoa(IMU.yaw, 	IMU.IMUStrPtrs[4], 4);
	ftoa(IMU.pitch, IMU.IMUStrPtrs[5], 4);
	ftoa(IMU.roll, 	IMU.IMUStrPtrs[6], 4);

	return (char**)IMU.IMUStrPtrs;
}

//----------------------------------------
// Send CSV magnetometer:
// Starts CSV magnetometer data sending.
//-----------------------------------------
static void SendCSVMagn_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 1))
	{
		DisableCmdLineInterface(&Console);

		Semaphore_post(Mag_Sem);
	}
}

//-----------------------------------------
// Send CSV magnetometer task
// Sends magnetometer data in CSV format
// (usefull for calibration).
//-----------------------------------------
void SendCSVMagnTask(void)
{
	while(1)
	{
		Semaphore_pend(Mag_Sem, BIOS_WAIT_FOREVER);

		while(!IsAbortRequested(&Console))
		{
			// As 'IMUProcessing_Sem' is a binary semaphore this task will be unblocked only if IMU processing task stay blocked
			// so that IMU processing task don't read nor compensate magnetometer data at the same time.
			// Be carefull to sleep enought time to avoid replacing to much IMU processings.
			Semaphore_pend(IMUProcessing_Sem, BIOS_WAIT_FOREVER);

			// Clear strings
			memset(IMU.SensorsStrPtrs[6], '\0', sizeof(char[3][10]));

			// Convert float values to strings
			ftoa(Magn.val[x], IMU.SensorsStrPtrs[6], 4);
			ftoa(Magn.val[y], IMU.SensorsStrPtrs[7], 4);
			ftoa(Magn.val[z], IMU.SensorsStrPtrs[8], 4);

			// Send uncompensated magnetometer data
			UARTprintf(&Console, "%s,%s,%s\r\n", IMU.SensorsStrPtrs[6], IMU.SensorsStrPtrs[7], IMU.SensorsStrPtrs[8]);

			// sleep for 50 000 us
			Task_sleep((uint32_t)50000/SYSTEM_CLOCK_PERIOD_US);
		}

		// Restart command line interface if user aborted
		EnableCmdLineInterface(&Console);
	}
}

//------------------------------------------
// IMU Task
//------------------------------------------
void IMUProcessingTask(void)
{
	// Madgwick AHRS algorithm variables
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _4q0, _4q1, _4q2 ,_8q1, _8q2;
	// Sensors values
	float gx, gy, gz, ax, ay, az, mx, my, mz;
	// Quaternion
	float q0, q1, q2, q3;

	// Configure and calibrates sensors
	ConfigureSensors();
	Log_info0("Inertial Measurement Unit initialized.");

	// Starts 'IMUSensors_Swi' periodic sofware interrupt
	Clock_start(IMU_Clock);

	// Add a command to allow user to receive uncompensated magetometer data for calibration
	if(!SubscribeCmd(&Console, "sendCSVMagn", SendCSVMagn_cmd, "Sends magnetometer data in CSV format (usefull for calibration)."))
	{
		Log_error0("Error (re)allocating memory for UART console command.");
		return;
	}

	// Fill 'SensorsStrPtrs' and 'IMUStrPtrs' string pointer arrays with pointers to 'SensorsStrValues' and 'IMUStrValues' strings
	uint32_t i;
	for(i = 0; i < 9; ++i)
		IMU.SensorsStrPtrs[i] = &IMU.SensorsStrValues[i][0];
	for(i = 0; i < 7; ++i)
		IMU.IMUStrPtrs[i] = &IMU.IMUStrValues[i][0];

	// Suscribe a bluetooth datasource to send periodically Sensors's data
	JSONDataSource* Sensorsds = SuscribePeriodicJSONDataSource("sensors", (const char*[]) {	"ax", "ay", "az",
																					"gx", "gy", "gz",
																					"mx", "my", "mz" }, 9, 20, SensorsDataAccessor);

	// Suscribe a bluetooth datasource to send periodically IMU's data
	JSONDataSource* IMUds = SuscribePeriodicJSONDataSource("IMU", (const char*[]){ "q0", "q1", "q2", "q3", "yaw", "pitch", "roll" }, 7, 20, IMUDataAccessor);

	while(1)
	{
		//Semaphore_pend(IMU_Sem, BIOS_WAIT_FOREVER);
		if (Semaphore_pend(IMUProcessing_Sem, 1000) == FALSE)
		{
			// Append error to logger if MPU6050 or magnetometer doesn't send informations anymore
			Log_error0("IMU TIMOUT : MPU6050 or HMC5883L doesn't send data anymore! Exit IMU task");
			// TODO: End IMU task ?
			break;
		}
		else
		{
			// Copy the gyroscope and accellerometer values. (global values cannot be cached)
			q0 = IMU.q[0];		q1 = IMU.q[1];		q2 = IMU.q[2];		q3 = IMU.q[3];
			gx = Gyro.val[x];	gy = Gyro.val[y];	gz = Gyro.val[z];
			ax = Accel.val[x];	ay = Accel.val[y];	az = Accel.val[z];

			// Rate of change of quaternion from gyroscope
			qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
			qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
			qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
			qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

			// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
			if((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
			{
				// Normalize accelerometer measurement
				recipNorm = invSqrt(ax * ax + ay * ay + az * az);
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;

				// Auxiliary quaternion variables to avoid repeated arithmetic
				_2q0 = 2.0f * q0;
				_2q1 = 2.0f * q1;
				_2q2 = 2.0f * q2;
				_2q3 = 2.0f * q3;
				_2q0q2 = 2.0f * q0 * q2;
				_2q2q3 = 2.0f * q2 * q3;
				q0q0 = q0 * q0;
				q0q1 = q0 * q1;
				q0q2 = q0 * q2;
				q0q3 = q0 * q3;
				q1q1 = q1 * q1;
				q1q2 = q1 * q2;
				q1q3 = q1 * q3;
				q2q2 = q2 * q2;
				q2q3 = q2 * q3;
				q3q3 = q3 * q3;

				// Use simplified algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
				// TODO: remetre la condition quand le magnetometre seras pret!
				if(false)//((Magn.val[x] != 0.0f) || (Magn.val[y] != 0.0f) || (Magn.val[z] != 0.0f))
				{
					// Correct magnetometer values.
					MagnetoCompensate();

					// Copy magnetometer values. (global values cannot be cached)
					mx = Magn.val[x];
					my = Magn.val[y];
					mz = Magn.val[z];

					// Normalize magnetometer measurement
					recipNorm = invSqrt(mx * mx + my * my + mz * mz);
					mx *= recipNorm;
					my *= recipNorm;
					mz *= recipNorm;

					// Auxiliary magnetic field variables to avoid repeated arithmetic
					_2q0mx = 2.0f * q0 * mx;
					_2q0my = 2.0f * q0 * my;
					_2q0mz = 2.0f * q0 * mz;
					_2q1mx = 2.0f * q1 * mx;

					// Reference direction of Earth's magnetic field
					hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
					hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
					_2bx = sqrt(hx * hx + hy * hy);
					_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
					_4bx = 2.0f * _2bx;
					_4bz = 2.0f * _2bz;

					// Gradient decent algorithm corrective step using magnetometer data
					s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
					recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
				}
				else
				{
		//			Log_error0("Wrong magnetometer values.");

					// Auxiliary simplified algorithm-specific variables to avoid repeated arithmetic
					_4q0 = 4.0f * q0;
					_4q1 = 4.0f * q1;
					_4q2 = 4.0f * q2;
					_8q1 = 8.0f * q1;
					_8q2 = 8.0f * q2;

					// Gradient decent algorithm corrective step without magnetometer data
					s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
					s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
					s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
					s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
					recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
				}

				// Normalize step magnitude
				s0 *= recipNorm;
				s1 *= recipNorm;
				s2 *= recipNorm;
				s3 *= recipNorm;

				// Apply feedback step
				qDot1 -= BETA * s0;
				qDot2 -= BETA * s1;
				qDot3 -= BETA * s2;
				qDot4 -= BETA * s3;
			}
			else
				Log_error0("Wrong accelerometer values.");

			// Integrate rate of change of quaternion to yield quaternion
			q0 += qDot1 * SAMPLE_PERIOD;
			q1 += qDot2 * SAMPLE_PERIOD;
			q2 += qDot3 * SAMPLE_PERIOD;
			q3 += qDot4 * SAMPLE_PERIOD;

			// Normalize quaternion
			recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
			q0 *= recipNorm;
			q1 *= recipNorm;
			q2 *= recipNorm;
			q3 *= recipNorm;

			// Convert quaternion to euler angles
			IMU.yaw 	=	atan2(2*(q0*q3 + q1*q2), 1 - 2*(pow(q2,2) + pow(q3,2)));
			IMU.pitch 	= 	asin(2*(q0*q2 - q3*q1));
			IMU.roll 	=	atan2(2*(q0*q1 + q2*q3), 1 - 2*(pow(q1,2) + pow(q2,2)));

		    // Return the quaternion values.
			IMU.q[0] = q0;
			IMU.q[1] = q1;
			IMU.q[2] = q2;
			IMU.q[3] = q3;

			// Unblock PID
			Semaphore_post(PID_Sem);
		}
	}

}

//----------------------------------------------------------------------------
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in meters per second squared.
// Values are obtained by taking the g conversion factors from the data sheet
// and multiplying by 9.80665 (1 g = 9.80665 m/s^2).
//----------------------------------------------------------------------------
static const float AccelFactors[] =
{
	5.9855042e-4f,	// Range = +/- 2 g (16384 lsb/g)
	1.1971008e-3f,	// Range = +/- 4 g (8192 lsb/g)
	2.3942017e-3f,	// Range = +/- 8 g (4096 lsb/g)
	4.7884033e-3f	// Range = +/- 16 g (2048 lsb/g)
};

//----------------------------------------------------------------------------
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in radians per second.
// Values are obtained by taking the degree per second conversion factors
// from the data sheet and then converting to radians per sec (1 degree =
// 0.0174532925 radians).
//----------------------------------------------------------------------------
static const float GyroFactors[] =
{
    1.3323124e-4f,   // Range = +/- 250 dps  (131.0 LSBs/DPS)
    2.6646248e-4f,   // Range = +/- 500 dps  (65.5 LSBs/DPS)
    5.3211258e-4f,   // Range = +/- 1000 dps (32.8 LSBs/DPS)
    0.0010642252f    // Range = +/- 2000 dps (16.4 LSBs/DPS)
};

//----------------------------------------------------------------------------
// The factors used to convert the magnetic field readings from the HMC5883L
// into floating point values in Gauss.
//----------------------------------------------------------------------------
static const float MagnFactors[] =
{
    7.2992701e-4f,   // Range = +/- 0.88 Gauss (1370 LSB/Gauss)
    9.1743119e-4f,   // Range = +/- 1.30 Gauss (1090 LSB/Gauss)
    0.0012195121f,   // Range = +/- 1.90 Gauss (820 LSB/Gauss)
    0.0015151515f,   // Range = +/- 2.50 Gauss (660 LSB/Gauss)
    0.0022727273f,   // Range = +/- 4.00 Gauss (440 LSB/Gauss)
    0.0025641026f,   // Range = +/- 4.70 Gauss (390 LSB/Gauss)
    0.0030303030f,   // Range = +/- 5.60 Gauss (330 LSB/Gauss)
    0.0043478261f    // Range = +/- 8.10 Gauss (230 LSB/Gauss)
};

//------------------------------------------
// ConvertRawData
//------------------------------------------
static void ConvertRawData(void)
{
	float factor;

	// Get real accelerometer value
	factor = AccelFactors[Accel.range];
	Accel.val[x] = (int16_t)((IMU.MPU6050RawData[0] << 8) | IMU.MPU6050RawData[1]) * factor;
	Accel.val[y] = (int16_t)((IMU.MPU6050RawData[2] << 8) | IMU.MPU6050RawData[3]) * factor;
	Accel.val[z] = (int16_t)((IMU.MPU6050RawData[4] << 8) | IMU.MPU6050RawData[5]) * factor;

	// Get real gyroscope values (with offset correction)
	factor = GyroFactors[Gyro.range];
	Gyro.val[x] = (int16_t)((IMU.MPU6050RawData[8] << 8)  | IMU.MPU6050RawData[9])  * factor - Gyro.xOffset;
	Gyro.val[y] = (int16_t)((IMU.MPU6050RawData[10] << 8) | IMU.MPU6050RawData[11]) * factor - Gyro.yOffset;
	Gyro.val[z] = (int16_t)((IMU.MPU6050RawData[12] << 8) | IMU.MPU6050RawData[13]) * factor - Gyro.zOffset;

	// Get real magnetometer values without transformation and centering (transformation matrix and offsets are applied later if data is valid)
	// We change magnetometer axes to fit MPU6050's landmark (newY = -realX, newX = realY)
	factor = MagnFactors[Magn.range];
	Magn.val[x] = (int16_t)((IMU.magnRawData[2] << 8) | IMU.magnRawData[3]) * factor;
	Magn.val[y] = -(int16_t)((IMU.magnRawData[0] << 8) | IMU.magnRawData[1]) * factor;
	Magn.val[z] = (int16_t)((IMU.magnRawData[4] << 8) | IMU.magnRawData[5]) * factor;
}

//------------------------------------------
// I²C transaction callback
//------------------------------------------
static void TransactionCallback(uint32_t status, uint8_t* buffer, uint32_t length)
{
	if(CheckI2CErrorCode(status, false))
	{
		// Raw data is now available in 'IMU.MPU6050RawData' but we have to convert it to meaningfull values before letting 'IMU_Task' process this data.
		ConvertRawData();

		// Unblock 'IMU_Task'
		Semaphore_post(IMUProcessing_Sem);
	}
}

//------------------------------------------
// IMU Reading Task
//------------------------------------------
void IMUReadingTask(void)
{
	while(1)
	{
		Semaphore_pend(IMUReading_Sem, BIOS_WAIT_FOREVER);

		// Read magnetometer's values
		Async_I2CRegRead(IMU_I2C_BASE, HMC5883L_I2C_ADDR, HMC5883L_DATA_REG_BEGIN, IMU.magnRawData, HMC5883L_DATA_REG_COUNT, NULL);

		// Read MPU6050 I²C accelerometer and gyroscope registers with a callback that unblocks IMU data processing thread.
		Async_I2CRegRead(IMU_I2C_BASE, MPU6050_I2C_ADDR, MPU6050_DATA_REG_BEGIN, IMU.MPU6050RawData, MPU6050_DATA_REG_COUNT, &TransactionCallback);
	}
}

//------------------------------------------
// IMUSensors Swi
// Sofware interrupt that reads IMU sensors
// through I²C.
//------------------------------------------
void IMUSensorsSwi(void)
{
	// Unblock IMUReadingTask periodically
	Semaphore_post(IMUReading_Sem);
}

//------------------------------------------
// Configure sensors (MPU6050 and HMC5883L)
// Initializes MPU6050 and HMC5883L and
// calibrates the MPU6050 gyroscope.
//------------------------------------------
static void ConfigureSensors(void)
{
	// Buffer used for reading and writing MPU6050 and HMC5883L configuration I2C registers
	uint8_t buffer[3];

    Task_sleep(200); // 500ms

	// Configure magnetometer to 75Hz sample rate, no averaged sample, 1090LSb/Gauss gain
	buffer[0] = HMC5883L_MEASUREMENT_FLOW_NORMAL | HMC5883L_SAMPLE_RATE_75HZ | HMC5883L_SAMPLE_AVERAGE_1;
	buffer[1] = HMC5883L_SCALE_1_3GAUSS | HMC5883L_MODE_HIGH_SPEED;
	Async_I2CRegWrite(IMU_I2C_BASE, HMC5883L_I2C_ADDR, HMC5883L_CONFIG_REG_A, buffer, 2, NULL);
	// Set magnetometer measurement mode
	buffer[2] = HMC5883L_MODE_CONTINUOUS;
	Async_I2CRegWrite(IMU_I2C_BASE, HMC5883L_I2C_ADDR, HMC5883L_MODE_REG, buffer+2, 1, NULL);
	CheckI2CErrorCode(WaitI2CTransacs(0), true); // Synchronize I2C transations with current thread.
	Log_info0("HMC5883L initialized.");

	// Perform MPU6050 device reset
    buffer[0] = MPU6050_PWR_MGMT_1_DEVICE_RESET;
    Async_I2CRegWrite(IMU_I2C_BASE, MPU6050_I2C_ADDR, MPU6050_O_PWR_MGMT_1, buffer, 1, NULL);
    CheckI2CErrorCode(WaitI2CTransacs(0), true);

    Task_sleep(200); // Wait 500ms for MPU6050 to reset its registers

	// Wake-up MPU6050 and set gyroscope Y axis PPL as clock source (improved stability)
    buffer[0] = MPU6050_PWR_MGMT_1_CLKSEL_YG;
    Async_I2CRegReadModifyWrite(IMU_I2C_BASE, MPU6050_I2C_ADDR, MPU6050_O_PWR_MGMT_1, buffer, ~MPU6050_PWR_MGMT_1_SLEEP & ~MPU6050_PWR_MGMT_1_CLKSEL_M, NULL);
    CheckI2CErrorCode(WaitI2CTransacs(0), true);

    Task_sleep(200); // Wait 500ms for MPU6050 to wake up

	// Configure the MPU6050 for +/- 4 g accelerometer range.
	buffer[0] = MPU6050_ACCEL_CONFIG_AFS_SEL_4G;
	Async_I2CRegReadModifyWrite(IMU_I2C_BASE, MPU6050_I2C_ADDR, MPU6050_O_ACCEL_CONFIG, buffer, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M, NULL);
	CheckI2CErrorCode(WaitI2CTransacs(0), true);

	Log_info0("MPU6050 initialized.");

	// Gyroscope calibration
	Log_info0("Gyroscope calibration ... (Measuring gyroscope offsets, do not move device)");
	float gyro_sum[3] = {0, 0, 0};
	uint32_t i = 0;
	for(i = 0; i < 512; i++)
	{
		// Read MPU6050 I²C accelerometer and gyroscope registers synchronously.
		Async_I2CRegRead(IMU_I2C_BASE, MPU6050_I2C_ADDR, MPU6050_DATA_REG_BEGIN, IMU.MPU6050RawData, MPU6050_DATA_REG_COUNT, NULL);
		CheckI2CErrorCode( WaitI2CTransacs(0), true);

		ConvertRawData();

		gyro_sum[x] += Gyro.val[x];
		gyro_sum[y] += Gyro.val[y];
		gyro_sum[z] += Gyro.val[z];
	}
	Gyro.xOffset = gyro_sum[x] / i;
	Gyro.yOffset = gyro_sum[y] / i;
	Gyro.zOffset = gyro_sum[z] / i;
	Log_info0("Gyroscope calibration done.");
}

//------------------------------------------
// Check I2C error code
//------------------------------------------
static bool CheckI2CErrorCode(uint32_t errorCode, bool IsFatal)
{
	switch(errorCode)
	{
	case TRANSAC_OK:
	case TRANSAC_UNDETERMINED:
		return TRUE;
	case TRANSAC_MAX_QUEUEING_REACHED:
		Log_error0("ERROR: I2C transaction max queueing reached.");
		break;
	case TIMEOUT_REACHED:
		Log_error0("ERROR: I2C transaction waiting timeout reached.");
		break;
	default:
		Log_error0("ERROR: I2C transaction unknown error.");
	};

	ASSERT(!IsFatal);
	return FALSE;
}
