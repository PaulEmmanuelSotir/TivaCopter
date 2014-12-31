/*
 * IMU.h
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <stdbool.h>

// MPU6050 registers and adresses
#include "Utils/hw_MPU6050.h"

//------------------------------------------
// Defines MPU6050 and HMC5883L I²C
// registers and adresses we use.
// The 14 bytes after MPU6050_O_ACCEL_XOUT_H
// corresponds to accelerometer, temperature
// and gyroscope data.
//------------------------------------------
#define MPU6050_I2C_ADDR			MPU6050_WHO_AM_I_MPU6050
#define MPU6050_DATA_REG_BEGIN		MPU6050_O_ACCEL_XOUT_H
#define MPU6050_DATA_REG_COUNT		14
#define HMC5883L_I2C_ADDR			0x1E
#define HMC5883L_CONFIG_REG_A		0x00			// HMC5883L CONFIG_REG_A
#define HMC5883L_CONFIG_REG_B		0x01			// HMC5883L CONFIG_REG_B
#define HMC5883L_MODE_REG			0x02			// HMC5883L Mode Register
#define HMC5883L_DATA_REG_BEGIN		0x03			// HMC5883L Data register
#define HMC5883L_DATA_REG_COUNT		6

//------------------------------------------
// Constants defines
//------------------------------------------
#define SAMPLE_FREQ					400.0f			// sample frequency in Hz, TODO: determine PERIOD at runtime (not frequency)
#define SAMPLE_PERIOD				1.0f/SAMPLE_FREQ
#define BETA						0.1f			// 2 *  Madgwick AHRS algorithm proportional gain
#define PI							3.14159265358979323846
#define G							9.80665

//-------------------------------------------------------------------------
// HMC5883L mesurement mode defines:
//> Mesurement mode I²C register :
//  ______________________________________________________________________
// |__MR7____|__MR6__|__MR5__|__MR4__|__MR3__|__MR2__|__MR1_____|__MR0____|
// |__HS(0)__|__(0)__|__(0)__|__(0)__|__(0)__|__(0)__|__MD1(0)__|__MD0(1)_|
//
// HS = High speed I2C (3400KHz)
//> MD1-MD0 Operating Modes:
// • 0-0
//  Continuous-Measurement Mode. In continuous-measurement mode,
//  the device continuously performs measurements and places the
//  result in the data register. RDY goes high when new data is placed
//  in all three registers. After a power-on or a write to the mode or
//  configuration register, the first measurement set is available from all
//  three data output registers after a period of 2/fDO and subsequent
//  measurements are available at a frequency of fDO, where fDO is the
//  frequency of data output.
// • 0-1
//  Single-Measurement Mode (Default). When single-measurement
//  mode is selected, device performs a single measurement, sets RDY
//  high and returned to idle mode. Mode register returns to idle mode
//  bit values. The measurement remains in the data output register and
//  RDY remains high until the data output register is read or another
//  measurement is performed.
// • 1-0
//  Idle Mode. Device is placed in idle mode.
// • 1-1
//  Idle Mode. Device is placed in idle mode.
//-------------------------------------------------------------------------
#define HMC5883L_MODE_HIGH_SPEED	0x80
#define HMC5883L_MODE_CONTINUOUS	0x00
#define HMC5883L_MODE_SINGLE		0x01 // DEFAULT
#define HMC5883L_MODE_IDLE			0x02 // or 0x03

//----------------------------------------
// HMC5883L measurment scales defines
//----------------------------------------
#define HMC5883L_SCALE_0_88GAUSS	0x00	// (0x00 << 5) Magnetic scale = 73 mG/LSb (± 0.88 Ga)
#define HMC5883L_SCALE_1_3GAUSS		0x20	// (0x01 << 5) Magnetic scale = 92 mG/LSb (± 1.3 Ga) DEFAULT
#define HMC5883L_SCALE_1_9GAUSS		0x40	// (0x02 << 5) Magnetic scale = 122 mG/LSb (± 1.9 Ga)
#define HMC5883L_SCALE_2_5GAUSS		0x60	// (0x03 << 5) Magnetic scale = 152 mG/LSb (± 2.5 Ga)
#define HMC5883L_SCALE_4GAUSS		0x80	// (0x04 << 5) Magnetic scale = 227 mG/LSb (± 4.0 Ga)
#define HMC5883L_SCALE_4_7GAUSS		0xA0	// (0x05 << 5) Magnetic scale = 25 6mG/LSb (± 4.7 Ga)
#define HMC5883L_SCALE_5_6GAUSS		0xC0	// (0x06 << 5) Magnetic scale = 303 mG/LSb (± 5.6 Ga)
#define HMC5883L_SCALE_8_1GAUSS		0xE0	// (0x07 << 5) Magnetic scale = 43 5mG/LSb (± 8.1 Ga)

//------------------------------------------
// HMC5883L configuration register A defines
//------------------------------------------
#define HMC5883L_MEASUREMENT_FLOW_NORMAL	0x00 // DEFAULT
#define HMC5883L_MEASUREMENT_POS_BIAS		0x01
#define HMC5883L_MEASUREMENT_NEG_BIAS		0x02
#define HMC5883L_SAMPLE_RATE_0_75HZ			0x00 // (0x00 << 2)
#define HMC5883L_SAMPLE_RATE_1_5HZ			0x04 // (0x01 << 2)
#define HMC5883L_SAMPLE_RATE_3HZ			0x08 // (0x02 << 2)
#define HMC5883L_SAMPLE_RATE_7_5HZ			0x0C // (0x03 << 2)
#define HMC5883L_SAMPLE_RATE_15HZ			0x10 // (0x04 << 2) DEFAULT
#define HMC5883L_SAMPLE_RATE_30HZ			0x14 // (0x05 << 2)
#define HMC5883L_SAMPLE_RATE_75HZ			0x18 // (0x06 << 2)
#define HMC5883L_SAMPLE_AVERAGE_1			0x00 // (0x00 << 5) DEFAULT
#define HMC5883L_SAMPLE_AVERAGE_2			0x20 // (0x01 << 5)
#define HMC5883L_SAMPLE_AVERAGE_4			0x40 // (0x02 << 5)
#define HMC5883L_SAMPLE_AVERAGE_8			0x60 // (0x03 << 5)

// TODO: load compensation parameters and scales from EEPROM

//------------------------------------------
// Gyroscope, accelerometer and magnetometer
// data range enum typedefs.
//------------------------------------------
typedef enum { _250dps, _500dps, _1000dps, _2000dps } GyroRange;
typedef enum { _2g, _4g, _8g, _16g } AccelRange;
typedef enum { _880mGa, _1300mGa, _1900mGa, _2500mGa, _4000mGa, _4700mGa, _5600mGa, _8100mGa } MagnRange;

//----------------------------------------
// Gyroscope data structure typedef
//----------------------------------------
typedef struct
{
	float val[3];
	GyroRange range;

	// Compensation data used to correct gyroscope data (offsets)
	float xOffset;
	float yOffset;
	float zOffset;
} Gyroscope;

//----------------------------------------
// Accelerometer data structure typedef
//----------------------------------------
typedef struct
{
	float val[3];
	AccelRange range;
} Accelerometer;

//----------------------------------------
// Magnetometer data structure typedef
//----------------------------------------
typedef struct
{
	float val[3];
	MagnRange range;

	// Compensation data used to correct magnetometer data (offset and transformation matrix)
	float xOffset;
	float yOffset;
	float zOffset;
	float M[3][3];
} Magnetometer;

//------------------------------------------
// IMU data structure typedef
//------------------------------------------
typedef struct
{
	// Raw I²C HMC5883L and MPU6050 register data
	uint8_t magnRawData[6];		// Magnetometer
	uint8_t MPU6050RawData[14];	// accelerometer, temperature and gyroscope

	// Meaningful data
	Magnetometer* magn;
	Gyroscope* gyro;
	Accelerometer* accel;

	// Quaternion
	float q[4];
	// Euler angles
	float yaw, pitch, roll;

	// String pointer arrays and string values used for JSON datasources
	char* SensorsStrPtrs[9];
	char SensorsStrValues[9][15];
	char* IMUStrPtrs[7];
	char IMUStrValues[7][10];
} InertialMeasurementUnit;

//-----------------------------------------
// Calibrate magnetometer task:
// Sends magnetometer data for calibration
// until user aborts.
//-----------------------------------------
void CalibrateMagnetometerTask(void);

//------------------------------------------
// IMU Task:
// Process Sensors data to deduce 3D
// rotation of quadcopter (quaternion)
// thanks to Madgwick AHRS algorithm.
//------------------------------------------
void IMUProcessingTask(void);

//----------------------------------------
// I2C0 State Machine Task
//----------------------------------------
void I2CStateMachineTask(void);

//----------------------------------------
// I²C0 Hardware Interrupt
//----------------------------------------
void I2C0HwiHandler(void);

//------------------------------------------
// IMU Reading Task
//------------------------------------------
void IMUReadingTask(void);

//------------------------------------------
// IMUSensors Swi
// Sofware interrupt that reads IMU sensors
// through I²C.
//------------------------------------------
void IMUSensorsSwi(void);

//------------------------------------------
// Configure sensors (MPU6050 and HMC5883L)
//------------------------------------------
static void ConfigureSensors(void);

//------------------------------------------
// Check I2C error code
//------------------------------------------
static bool CheckI2CErrorCode(uint32_t errorCode, bool IsFatal);

#endif /* IMU_H_ */
