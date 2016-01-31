/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2013 by
 * Harrison Nordby and Ross Nordby
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "Vector3.hpp"
#include "Quaternion.hpp"
#include <cmath>

#ifdef DEBUG
#include <assert.h>
#endif
namespace BEPUmath
{

	/// <summary>
	/// Constructs a new Quaternion.
	/// </summary>
	/// <param name="x">X component of the quaternion.</param>
	/// <param name="y">Y component of the quaternion.</param>
	/// <param name="z">Z component of the quaternion.</param>
	/// <param name="w">W component of the quaternion.</param>
	Quaternion::Quaternion(float x, float y, float z, float w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	Quaternion::Quaternion() : X(0), Y(0), Z(0), W(0)
	{
	}

	/// <summary>
	/// Adds two quaternions together.
	/// </summary>
	/// <param name="a">First quaternion to add.</param>
	/// <param name="b">Second quaternion to add.</param>
	/// <param name="result">Sum of the addition.</param>
	void Quaternion::Add(Quaternion &a, Quaternion &b, Quaternion &result)
	{
		result.X = a.X + b.X;
		result.Y = a.Y + b.Y;
		result.Z = a.Z + b.Z;
		result.W = a.W + b.W;
	}
	
	bool Quaternion::IsNan()
	{
		return X!=X || Y!=Y || Z!=Z || W!=W;
			
	}

	/// <summary>
	/// Multiplies two quaternions.
	/// </summary>
	/// <param name="a">First quaternion to multiply.</param>
	/// <param name="b">Second quaternion to multiply.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Quaternion::Multiply(Quaternion &a, Quaternion &b, Quaternion &result)
	{
		float x = a.X;
		float y = a.Y;
		float z = a.Z;
		float w = a.W;
		float bX = b.X;
		float bY = b.Y;
		float bZ = b.Z;
		float bW = b.W;
		result.X = x * bW + bX * w + y * bZ - z * bY;
		result.Y = y * bW + bY * w + z * bX - x * bZ;
		result.Z = z * bW + bZ * w + x * bY - y * bX;
		result.W = w * bW - x * bX - y * bY - z * bZ;
	}

	/// <summary>
	/// Scales a quaternion.
	/// </summary>
	/// <param name="q">Quaternion to multiply.</param>
	/// <param name="scale">Amount to multiply each component of the quaternion by.</param>
	/// <param name="result">Scaled quaternion.</param>
	void Quaternion::Multiply(Quaternion &q, float scale, Quaternion &result)
	{
		result.X = q.X * scale;
		result.Y = q.Y * scale;
		result.Z = q.Z * scale;
		result.W = q.W * scale;
	}

	/// <summary>
	/// Multiplies two quaternions together in opposite order.
	/// </summary>
	/// <param name="a">First quaternion to multiply.</param>
	/// <param name="b">Second quaternion to multiply.</param>
	/// <param name="result">Product of the multiplication.</param>
	void Quaternion::Concatenate(Quaternion &a, Quaternion &b, Quaternion &result)
	{
		float aX = a.X;
		float aY = a.Y;
		float aZ = a.Z;
		float aW = a.W;
		float x = b.X;
		float y = b.Y;
		float z = b.Z;
		float w = b.W;
		result.X = x * aW + aX * w + y * aZ - z * aY;
		result.Y = y * aW + aY * w + z * aX - x * aZ;
		result.Z = z * aW + aZ * w + x * aY - y * aX;
		result.W = w * aW - x * aX - y * aY - z * aZ;
	}





	/// <summary>
	/// Ensures the quaternion has unit length.
	/// </summary>
	/// <param name="quaternion">Quaternion to normalize.</param>
	/// <param name="toReturn">Normalized quaternion.</param>
	void Quaternion::Normalize(Quaternion &quaternion, Quaternion &toReturn)
	{
		float inverse = (float)(1 / sqrt(quaternion.X * quaternion.X + quaternion.Y * quaternion.Y + quaternion.Z * quaternion.Z + quaternion.W * quaternion.W));
		toReturn.X = quaternion.X * inverse;
		toReturn.Y = quaternion.Y * inverse;
		toReturn.Z = quaternion.Z * inverse;
		toReturn.W = quaternion.W * inverse;
	}

	/// <summary>
	/// Scales the quaternion such that it has unit length.
	/// </summary>
	void Quaternion::Normalize()
	{
		float inverse = (float)(1 / sqrt(X * X + Y * Y + Z * Z + W * W));
#ifdef DEBUG
		assert(inverse==inverse);
#endif
		X *= inverse;
		Y *= inverse;
		Z *= inverse;
		W *= inverse;
#ifdef DEBUG
		assert(X==X);
		assert(Y==Y);
		assert(Z==Z);
		assert(W==W);
#endif
	}

	/// <summary>
	/// Blends two quaternions together to get an intermediate state.
	/// </summary>
	/// <param name="start">Starting point of the interpolation.</param>
	/// <param name="end">Ending point of the interpolation.</param>
	/// <param name="interpolationAmount">Amount of the end point to use.</param>
	/// <param name="result">Interpolated intermediate quaternion.</param>
	void Quaternion::Slerp(Quaternion &start, Quaternion &end, float interpolationAmount, Quaternion &result)
	{
		double cosHalfTheta = start.W * end.W + start.X * end.X + start.Y * end.Y + start.Z * end.Z;
		if (cosHalfTheta < 0)
		{
			//Negating a quaternion results in the same orientation,
			//but we need cosHalfTheta to be positive to get the shortest path.
			end.X = -end.X;
			end.Y = -end.Y;
			end.Z = -end.Z;
			end.W = -end.W;
			cosHalfTheta = -cosHalfTheta;
		}
		// If the orientations are similar enough, then just pick one of the inputs.
		if (cosHalfTheta > .999999)
		{
			result.W = start.W;
			result.X = start.X;
			result.Y = start.Y;
			result.Z = start.Z;
			return;
		}
		// Calculate temporary values.
		double halfTheta = acos(cosHalfTheta);
		double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
		//Check to see if we're 180 degrees away from the target.
		if (std::abs(sinHalfTheta) < 0.00001)
		{
			//Woops! There are an infinite number of ways to get to the goal.
			//Pick one.
			result.X = (start.X + end.X) * .5f;
			result.Y = (start.Y + end.Y) * .5f;
			result.Z = (start.Z + end.Z) * .5f;
			result.W = (start.W + end.W) * .5f;
			return;
		}
		double aFraction = sin((1 - interpolationAmount) * halfTheta) / sinHalfTheta;
		double bFraction = sin(interpolationAmount * halfTheta) / sinHalfTheta;

		//Blend the two quaternions to get the result!
		result.X = (float)(start.X * aFraction + end.X * bFraction);
		result.Y = (float)(start.Y * aFraction + end.Y * bFraction);
		result.Z = (float)(start.Z * aFraction + end.Z * bFraction);
		result.W = (float)(start.W * aFraction + end.W * bFraction);




	}


	/// <summary>
	/// Computes the conjugate of the quaternion.
	/// </summary>
	/// <param name="quaternion">Quaternion to conjugate.</param>
	/// <param name="result">Conjugated quaternion.</param>
	void Quaternion::Conjugate(Quaternion &quaternion, Quaternion &result)
	{
		result.X = -quaternion.X;
		result.Y = -quaternion.Y;
		result.Z = -quaternion.Z;
		result.W = quaternion.W;
	}

	/// <summary>
	/// Creates a quaternion from an axis and angle.
	/// </summary>
	/// <param name="axis">Axis of rotation.</param>
	/// <param name="angle">Angle to rotate around the axis.</param>
	/// <param name="p">Quaternion representing the axis and angle rotation.</param>
	void Quaternion::CreateFromAxisAngle(Vector3 &axis, float angle, Quaternion &q)
	{
		float halfAngle = angle * .5f;
		float s = sin(halfAngle);
		q.X = axis.X * s;
		q.Y = axis.Y * s;
		q.Z = axis.Z * s;
		q.W = cos(halfAngle);
	}

	/// <summary>
	/// Constructs a quaternion from yaw, pitch, and roll.
	/// </summary>
	/// <param name="yaw">Yaw of the rotation.</param>
	/// <param name="pitch">Pitch of the rotation.</param>
	/// <param name="roll">Roll of the rotation.</param>
	/// <param name="q">Quaternion representing the yaw, pitch, and roll.</param>
	void Quaternion::CreateFromYawPitchRoll(float yaw, float pitch, float roll, Quaternion &q)
	{
		double cosYaw = cos(yaw * .5f);
		double cosPitch = cos(pitch * .5f);
		double cosRoll = cos(roll * .5f);

		double sinYaw = sin(yaw * .5f);
		double sinPitch = sin(pitch * .5f);
		double sinRoll = sin(roll * .5f);

		double cosYawCosPitch = cosYaw * cosPitch;
		double cosYawSinPitch = cosYaw * sinPitch;
		double sinYawCosPitch = sinYaw * cosPitch;
		double sinYawSinPitch = sinYaw * sinPitch;

		q.W = (float)(cosYawCosPitch * cosRoll + sinYawSinPitch * sinRoll);
		q.X = (float)(sinYawCosPitch * cosRoll - cosYawSinPitch * sinRoll);
		q.Y = (float)(cosYawSinPitch * cosRoll + sinYawCosPitch * sinRoll);
		q.Z = (float)(cosYawCosPitch * sinRoll - sinYawSinPitch * cosRoll);
	}

	/// <summary>
	/// Transforms the vector using a quaternion.
	/// </summary>
	/// <param name="v">Vector to transform.</param>
	/// <param name="rotation">Rotation to apply to the vector.</param>
	/// <param name="result">Transformed vector.</param>
	void Quaternion::Transform(Vector3 &v, Quaternion &rotation, Vector3 &result)
	{
		//This operation is an optimized-down version of v' = q * v * q^-1.
		//The expanded form would be to treat v as an 'axis only' quaternion
		//and perform standard quaternion multiplication.  Assuming q is normalized,
		//q^-1 can be replaced by a conjugation.
		float x2 = rotation.X + rotation.X;
		float y2 = rotation.Y + rotation.Y;
		float z2 = rotation.Z + rotation.Z;
		float xx2 = rotation.X * x2;
		float xy2 = rotation.X * y2;
		float xz2 = rotation.X * z2;
		float yy2 = rotation.Y * y2;
		float yz2 = rotation.Y * z2;
		float zz2 = rotation.Z * z2;
		float wx2 = rotation.W * x2;
		float wy2 = rotation.W * y2;
		float wz2 = rotation.W * z2;
		//Defer the component setting since they're used in computation.
		float transformedX = v.X * ((1.0f - yy2) - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
		float transformedY = v.X * (xy2 + wz2) + v.Y * ((1.0f - xx2) - zz2) + v.Z * (yz2 - wx2);
		float transformedZ = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * ((1.0f - xx2) - yy2);
		result.X = transformedX;
		result.Y = transformedY;
		result.Z = transformedZ;

	}

	/// <summary>
	/// Transforms quaternion B into the local space of quaternion A.
	/// </summary>
	/// <param name="orientationA">Orientation A</param>
	/// <param name="orientationB">Orientation B</param>
	/// <param name="resultLocalB">Orientation B transformed into orientation A's local space.</param>
	void Quaternion::RelativeOrientation(Quaternion &orientationA, Quaternion &orientationB, Quaternion &resultLocalB)
	{
		Quaternion orientationAConjugate;
		Quaternion::Conjugate(orientationA, orientationAConjugate);
		Quaternion::Concatenate(orientationB, orientationAConjugate, resultLocalB);
	}
	
	bool Quaternion::IsUnit(float limit)
	{
		if (std::abs(W - 1.0f) < limit)
			if (std::abs(X) < limit)
				if (std::abs(Y) < limit)
					if (std::abs(Z) < limit)
						return true;
		
		return false;
	}

}
