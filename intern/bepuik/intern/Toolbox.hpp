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
#include "Quaternion.hpp"


namespace BEPUmath
{
	/// <summary>
	/// Stores some helper math functions brought over from the BEPUphysics Toolbox.
	/// </summary>
	class Toolbox
	{
	public:
		/// <summary>
		/// Threshold value used for floating point comparisons.
		/// </summary>
		static float Epsilon;// = 1e-7f;

		/// <summary>
		/// Vector pointing in the up direction.
		/// </summary>
		static Vector3 UpVector;// = Vector3(0, 1, 0);

		/// <summary>
		/// Vector pointing in the right direction.
		/// </summary>
		static Vector3 RightVector;// = Vector3(1, 0, 0);

		/// <summary>
		/// Vector with all components equal to zero.
		/// </summary>
		static Vector3 ZeroVector;// = Vector3(0,0,0);

		/// <summary>
		/// Computes the quaternion rotation between two normalized vectors.
		/// </summary>
		/// <param name="v1">First unit-length vector.</param>
		/// <param name="v2">Second unit-length vector.</param>
		/// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
		static void GetQuaternionBetweenNormalizedVectors(Vector3 &v1, Vector3 &v2, Quaternion &q);

		/// <summary>
		/// Computes the axis angle representation of a normalized quaternion.
		/// </summary>
		/// <param name="q">Quaternion to be converted.</param>
		/// <param name="axis">Axis represented by the quaternion.</param>
		/// <param name="angle">Angle around the axis represented by the quaternion.</param>
		static void GetAxisAngleFromQuaternion(Quaternion &q, Vector3 &axis, float &angle);
	};
}
