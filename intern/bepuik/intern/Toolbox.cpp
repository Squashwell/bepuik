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

#include <cmath>
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Toolbox.hpp"

namespace BEPUmath
{

	/// <summary>
	/// Threshold value used for floating point comparisons.
	/// </summary>
	float Toolbox::Epsilon = 1e-7f;

	/// <summary>
	/// Vector pointing in the up direction.
	/// </summary>
	Vector3 Toolbox::UpVector = Vector3(0, 1, 0);

	/// <summary>
	/// Vector pointing in the right direction.
	/// </summary>
	Vector3 Toolbox::RightVector = Vector3(1, 0, 0);

	/// <summary>
	/// Vector with all components equal to zero.
	/// </summary>
	Vector3 Toolbox::ZeroVector = Vector3(0,0,0);

	/// <summary>
	/// Computes the quaternion rotation between two normalized vectors.
	/// </summary>
	/// <param name="v1">First unit-length vector.</param>
	/// <param name="v2">Second unit-length vector.</param>
	/// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
	void Toolbox::GetQuaternionBetweenNormalizedVectors(Vector3 &v1, Vector3 &v2, Quaternion &q)
	{
		float dot;
		Vector3::Dot(v1, v2, dot);
		Vector3 axis;
		Vector3::Cross(v1, v2, axis);
		//For non-normal vectors, the multiplying the axes length squared would be necessary:
		//float w = dot + (float)Math.Sqrt(v1.LengthSquared() * v2.LengthSquared());
		if (dot < -0.9999f) //parallel, opposing direction
			q = Quaternion(-v1.Z, v1.Y, v1.X, 0);
		else
			q = Quaternion(axis.X, axis.Y, axis.Z, dot + 1);
		q.Normalize();
	}

	/// <summary>
	/// Computes the axis angle representation of a normalized quaternion.
	/// </summary>
	/// <param name="q">Quaternion to be converted.</param>
	/// <param name="axis">Axis represented by the quaternion.</param>
	/// <param name="angle">Angle around the axis represented by the quaternion.</param>
	void Toolbox::GetAxisAngleFromQuaternion(Quaternion &q, Vector3 &axis, float &angle)
	{
		float qx = q.X;
		float qy = q.Y;
		float qz = q.Z;
		float qw = q.W;
		if (qw < 0)
		{
			qx = -qx;
			qy = -qy;
			qz = -qz;
			qw = -qw;
		}
		if (qw > 1 - 1e-12)
		{
			axis = Toolbox::UpVector;
			angle = 0;
		}
		else
		{
			angle = 2 * acos(qw);
			float denominator = 1 / sqrt(1 - qw * qw);
			axis.X = qx * denominator;
			axis.Y = qy * denominator;
			axis.Z = qz * denominator;
		}
	}
}
