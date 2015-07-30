
#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

//-----------------------------------------------
// Quaternions components indexes
//-----------------------------------------------
#define Q_A				0
#define Q_B				1
#define Q_C				2
#define Q_D				3

//-----------------------------------------------
// Define PI costant if not defined yet
//-----------------------------------------------
#ifndef PI
#define PI                    3.14159265358979323846
#endif

//-----------------------------------------------
// QuaternionToEuler:
// Computes a Euler angles in degrees from given
// quaternion.
//-----------------------------------------------
void QuaternionToEuler(float QOut[4], float* RollDegOut, float* PitchDegOut, float* YawDegOut);

//-----------------------------------------------
// QuaternionFromEuler:
// Computes a quaternion from given Euler angles
// specified in degrees.
//-----------------------------------------------
void QuaternionFromEuler(float QOut[4], float RollDeg, float PitchDeg, float YawDeg);

//-----------------------------------------------
// QuaternionMagnitude:
// Computes the magnitude of a quaternion by
// summing the square of each of the quaternion
// components.
//-----------------------------------------------
float QuaternionMagnitude(float QIn[4]);

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
void QuaternionInverse(float QOut[4], float QIn[4]);

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
void QuaternionNormalize(float QOut[4], float QIn[4]);

//------------------------------------------------
// QuaternionMultiply:
// Computes the cross product of two quaternions.
//------------------------------------------------
void QuaternionMultiply(float QOut[4], float QIn1[4], float QIn2[4]);

//------------------------------------------------
// QuaternionRotateVector:
// Computes the rotation a vector by a quaternion.
//------------------------------------------------
void QuaternionRotateVector(float QIn[4], float VectorIn[3], float VectorOut[3]);

//------------------------------------------------
// QuaternionAngle:
// Computes the angle between two quaternions.
//------------------------------------------------
float QuaternionAngle(float QIn1[4], float QIn2[4]);

#endif /* QUATERNIONS_H_ */
