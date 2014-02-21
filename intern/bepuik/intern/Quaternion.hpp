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

#pragma once
#include "Vector3.hpp"

namespace BEPUmath
{
	struct Quaternion
	{
		/// <summary>
		/// X component of the quaternion.
		/// </summary>
		float X;

		/// <summary>
		/// Y component of the quaternion.
		/// </summary>
		float Y;

		/// <summary>
		/// Z component of the quaternion.
		/// </summary>
		float Z;

		/// <summary>
		/// W component of the quaternion.
		/// </summary>
		float W;

		/// <summary>
		/// Constructs a new Quaternion.
		/// </summary>
		/// <param name="x">X component of the quaternion.</param>
		/// <param name="y">Y component of the quaternion.</param>
		/// <param name="z">Z component of the quaternion.</param>
		/// <param name="w">W component of the quaternion.</param>
		Quaternion(float x, float y, float z, float w);

		Quaternion();
		
		bool IsNan();

		/// <summary>
		/// Adds two quaternions together.
		/// </summary>
		/// <param name="a">First quaternion to add.</param>
		/// <param name="b">Second quaternion to add.</param>
		/// <param name="result">Sum of the addition.</param>
		static void Add(Quaternion &a, Quaternion &b, Quaternion &result);

		/// <summary>
		/// Multiplies two quaternions.
		/// </summary>
		/// <param name="a">First quaternion to multiply.</param>
		/// <param name="b">Second quaternion to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void Multiply(Quaternion &a, Quaternion &b, Quaternion &result);

		/// <summary>
		/// Scales a quaternion.
		/// </summary>
		/// <param name="q">Quaternion to multiply.</param>
		/// <param name="scale">Amount to multiply each component of the quaternion by.</param>
		/// <param name="result">Scaled quaternion.</param>
		static void Multiply(Quaternion &q, float scale, Quaternion &result);

		/// <summary>
		/// Multiplies two quaternions together in opposite order.
		/// </summary>
		/// <param name="a">First quaternion to multiply.</param>
		/// <param name="b">Second quaternion to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		static void Concatenate(Quaternion &a, Quaternion &b, Quaternion &result);





		/// <summary>
		/// Ensures the quaternion has unit length.
		/// </summary>
		/// <param name="quaternion">Quaternion to normalize.</param>
		/// <param name="toReturn">Normalized quaternion.</param>
		static void Normalize(Quaternion &quaternion, Quaternion &toReturn);

		/// <summary>
		/// Scales the quaternion such that it has unit length.
		/// </summary>
		void Normalize();
		
		/// <summary>
		/// Returns true if the quaternion is a unit quaternion within limit
		/// </summary>
		bool IsUnit(float limit);

		/// <summary>
		/// Blends two quaternions together to get an intermediate state.
		/// </summary>
		/// <param name="start">Starting point of the interpolation.</param>
		/// <param name="end">Ending point of the interpolation.</param>
		/// <param name="interpolationAmount">Amount of the end point to use.</param>
		/// <param name="result">Interpolated intermediate quaternion.</param>
		static void Slerp(Quaternion &start, Quaternion &end, float interpolationAmount, Quaternion &result);


		/// <summary>
		/// Computes the conjugate of the quaternion.
		/// </summary>
		/// <param name="quaternion">Quaternion to conjugate.</param>
		/// <param name="result">Conjugated quaternion.</param>
		static void Conjugate(Quaternion &quaternion, Quaternion &result);

		/// <summary>
		/// Creates a quaternion from an axis and angle.
		/// </summary>
		/// <param name="axis">Axis of rotation.</param>
		/// <param name="angle">Angle to rotate around the axis.</param>
		/// <param name="p">Quaternion representing the axis and angle rotation.</param>
		static void CreateFromAxisAngle(Vector3 &axis, float angle, Quaternion &q);

		/// <summary>
		/// Constructs a quaternion from yaw, pitch, and roll.
		/// </summary>
		/// <param name="yaw">Yaw of the rotation.</param>
		/// <param name="pitch">Pitch of the rotation.</param>
		/// <param name="roll">Roll of the rotation.</param>
		/// <param name="q">Quaternion representing the yaw, pitch, and roll.</param>
		static void CreateFromYawPitchRoll(float yaw, float pitch, float roll, Quaternion &q);

		/// <summary>
		/// Transforms the vector using a quaternion.
		/// </summary>
		/// <param name="v">Vector to transform.</param>
		/// <param name="rotation">Rotation to apply to the vector.</param>
		/// <param name="result">Transformed vector.</param>
		static void Transform(Vector3 &v, Quaternion &rotation, Vector3 &result);

		/// <summary>
		/// Gets the relative Orientation from A to B
		/// </summary>
		static void RelativeOrientation(Quaternion &orientationA, Quaternion &orientationB, Quaternion &relativeOrientation);
	};
}
