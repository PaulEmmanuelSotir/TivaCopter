/*
 * quaternions.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "math.h"

#include "quaternions.h"

//-----------------------------------------------
// QuaternionToEuler:
// Computes a Euler angles in degrees from given
// quaternion.
//-----------------------------------------------
void QuaternionToEuler(float QIn[4], float* RollDegOut, float* PitchDegOut, float* YawDegOut)
{
	*YawDegOut 		=	atan2(2*(QIn[0]*QIn[3] + QIn[1]*QIn[2]), 1 - 2*(QIn[2]*QIn[2] + QIn[3]*QIn[3]));
	*PitchDegOut 	= 	asin (2*(QIn[0]*QIn[2] - QIn[3]*QIn[1]));
	*RollDegOut 	=	atan2(2*(QIn[0]*QIn[1] + QIn[2]*QIn[3]), 1 - 2*(QIn[1]*QIn[1] + QIn[2]*QIn[2]));
}

//-----------------------------------------------
// QuaternionFromEuler:
// Computes a quaternion from given Euler angles
// specified in degrees.
//-----------------------------------------------
void QuaternionFromEuler(float QOut[4], float RollDeg, float PitchDeg, float YawDeg)
{
	float Roll, Pitch, Yaw;
	float CosY, CosP, CosR;
	float SinY, SinP, SinR;

	// Convert roll, pitch, and yaw from degrees into radians
	Roll = RollDeg * PI / 180.0;
	Pitch = PitchDeg *  PI / 180.0;
	Yaw = YawDeg * PI / 180.0;

	// Pre-calculate the cos of (yaw, pitch, roll divided by 2)
	CosY = cosf(Yaw / 2.0);
	CosP = cosf(Pitch / 2.0);
	CosR = cosf(Roll / 2.0);

	// Pre-calculate the sin of (yaw, pitch, roll divided by 2)
	SinY = sinf(Yaw / 2.0);
	SinP = sinf(Pitch / 2.0);
	SinR = sinf(Roll / 2.0);

	// A component
	QOut[Q_A] = CosY * CosP * CosR - SinY * SinP * SinR;
	// B component
	QOut[Q_B] = SinY * SinP * CosR + CosY * CosP * SinR;
	// C component
	QOut[Q_C] = CosY * SinP * CosR - SinY * CosP * SinR;
	// D component
	QOut[Q_D] = SinY * CosP * CosR + CosY * SinP * SinR;
}

//-----------------------------------------------
// QuaternionMagnitude:
// Computes the magnitude of a quaternion by
// summing the square of each of the quaternion
// components.
//-----------------------------------------------
float QuaternionMagnitude(float QIn[4])
{
	return QIn[Q_A]*QIn[Q_A] + QIn[Q_B]*QIn[Q_B] + QIn[Q_C]*QIn[Q_C] + QIn[Q_D]*QIn[Q_D];
}

//------------------------------------------------
// QuaternionInverse:
// Computes the inverse of a quaternion.
// The inverse of a quaternion produces a rotation
// opposite to the source quaternion. This can be
// achieved by simply changing the signs of the
// imaginary components of a quaternion when the
// quatnerion is a unit quaternion. (conjugate)
// Note that you can use the same in and out
// quaternion here.
//-----------------------------------------------
void QuaternionInverse(float QOut[4], float QIn[4])
{
	float magnitude = QuaternionMagnitude(QIn);

	QOut[Q_A] = +QIn[Q_A] / magnitude;
	QOut[Q_B] = -QIn[Q_B] / magnitude;
	QOut[Q_C] = -QIn[Q_C] / magnitude;
	QOut[Q_D] = -QIn[Q_D] / magnitude;
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

//------------------------------------------------
// QuaternionNormalize:
// Computes the normalized quaternion of a given
// quaternion.
// You can also obtains a normalized quaternion
// faster by simply dividing by magnitude (instead
// of norm) when the quaternion is a unit
// quaternion.
// Note that you can use the same in and out
// quaternion here.
//------------------------------------------------
void QuaternionNormalize(float QOut[4], float QIn[4])
{
	float InvNorm = invSqrt(QuaternionMagnitude(QIn));

	QOut[Q_A] = QIn[Q_A] * InvNorm;
	QOut[Q_B] = QIn[Q_B] * InvNorm;
	QOut[Q_C] = QIn[Q_C] * InvNorm;
	QOut[Q_D] = QIn[Q_D] * InvNorm;
}

//------------------------------------------------
// QuaternionMultiply:
// Computes the cross product of two quaternions.
//------------------------------------------------
void QuaternionMultiply(float QOut[4], float QIn1[4], float QIn2[4])
{
	// Calculate the W term
	QOut[Q_A] = QIn2[Q_A]*QIn1[Q_A] - QIn2[Q_B]*QIn1[Q_B] - QIn2[Q_C]*QIn1[Q_C] - QIn2[Q_D]*QIn1[Q_D];

	// Calculate the X term
	QOut[Q_B] = QIn2[Q_B]*QIn1[Q_A] + QIn2[Q_A]*QIn1[Q_B] - QIn2[Q_C]*QIn1[Q_D] + QIn2[Q_D]*QIn1[Q_C];

	// Calculate the Y term
	QOut[Q_C] = QIn2[Q_A]*QIn1[Q_C] + QIn2[Q_B]*QIn1[Q_D] - QIn2[Q_C]*QIn1[Q_A] - QIn2[Q_D]*QIn1[Q_B];

	// Calculate the Z term
	QOut[Q_D] = QIn2[Q_A]*QIn1[Q_D] - QIn2[Q_B]*QIn1[Q_C] - QIn2[Q_C]*QIn1[Q_B] + QIn2[Q_D]*QIn1[Q_A];
}

//------------------------------------------------
// QuaternionRotateVector:
// Computes the rotation a vector by a quaternion.
//------------------------------------------------
void QuaternionRotateVector(float QIn[4], float VectorIn[3], float VectorOut[3])
{
	float qInv[4];
	float qProd[4];

	QuaternionInverse(qInv, QIn);

	// The rotated vector is : VectorOut = QIn * VectorIn * qInv
	// Here, a vector is considered as a pure quaternion (a quaternion with a zero A component)
	qProd[Q_A] = -VectorIn[Q_B-1]*QIn[Q_B] - VectorIn[Q_C-1]*QIn[Q_C] - VectorIn[Q_D-1]*QIn[Q_D];
	qProd[Q_B] = VectorIn[Q_B-1]*QIn[Q_A] - VectorIn[Q_C-1]*QIn[Q_D] + VectorIn[Q_D-1]*QIn[Q_C];
	qProd[Q_C] = VectorIn[Q_B-1]*QIn[Q_D] - VectorIn[Q_C-1]*QIn[Q_A] - VectorIn[Q_D-1]*QIn[Q_B];
	qProd[Q_D] = -VectorIn[Q_B-1]*QIn[Q_C] - VectorIn[Q_C-1]*QIn[Q_B] + VectorIn[Q_D-1]*QIn[Q_A];

	//VectorOut[Q_A] = qInv[Q_A]*qProd[Q_A] - qInv[Q_B]*qProd[Q_B] - qInv[Q_C]*qProd[Q_C] - qInv[Q_D]*qProd[Q_D];
	VectorOut[Q_B - 1] = qInv[Q_B]*qProd[Q_A] + qInv[Q_A]*qProd[Q_B] - qInv[Q_C]*qProd[Q_D] + qInv[Q_D]*qProd[Q_C];
	VectorOut[Q_C - 1] = qInv[Q_A]*qProd[Q_C] + qInv[Q_B]*qProd[Q_D] - qInv[Q_C]*qProd[Q_A] - qInv[Q_D]*qProd[Q_B];
	VectorOut[Q_D - 1] = qInv[Q_A]*qProd[Q_D] - qInv[Q_B]*qProd[Q_C] - qInv[Q_C]*qProd[Q_B] + qInv[Q_D]*qProd[Q_A];
}

//------------------------------------------------
// QuaternionAngle:
// Computes the angle between two quaternions.
//------------------------------------------------
float QuaternionAngle(float QIn1[4], float QIn2[4])
{
	float qInv[4], qProd[4];

	// Let Q1 and Q2 be two quaternions having components a,b,c,d.
	// The angle between the orientations represented by Q1 and Q2 can be calculated with:
	// angle = arccos( (Q2 * Q1^-1).w ) * 2.0

	QuaternionInverse(qInv, QIn1);

	QuaternionMultiply(qProd, QIn2, qInv);

	return(acosf(qProd[Q_A]) * 2.0);
}

