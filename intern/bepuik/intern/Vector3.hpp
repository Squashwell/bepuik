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

namespace BEPUmath
{
	/// <summary>
	/// Provides XNA-like 3D vector math needed by the engine.
	/// </summary>
	struct Vector3
	{

		/// <summary>
		/// X component of the vector.
		/// </summary>
		float X;
		/// <summary>
		/// Y component of the vector.
		/// </summary>
		float Y;
		/// <summary>
		/// Z component of the vector.
		/// </summary>
		float Z;

		Vector3();

		/// <summary>
		/// Constructs a new 3d vector.
		/// </summary>
		/// <param name="x">X component of the vector.</param>
		/// <param name="y">Y component of the vector.</param>
		/// <param name="z">Z component of the vector.</param>
		Vector3(float x, float y, float z);

		/// <summary>
		/// Computes the squared length of the vector.
		/// </summary>
		/// <returns>Squared length of the vector.</returns>
		float LengthSquared();

		/// <summary>
		/// Computes the length of the vector.
		/// </summary>
		/// <returns>Length of the vector.</returns>
		float Length();

		/// <summary>
		/// Normalizes the vector.
		/// </summary>
		void Normalize();
		
		bool IsZero(float limit);
		
		bool IsNan();

		/// <summary>
		/// Computes the dot product of two vectors.
		/// </summary>
		/// <param name="a">First vector in the product.</param>
		/// <param name="b">Second vector in the product.</param>
		/// <param name="product">Resulting dot product.</param>
		static void Dot(Vector3 &a, Vector3 &b, float &product);

		/// <summary>
		/// Adds two vectors together.
		/// </summary>
		/// <param name="a">First vector to add.</param>
		/// <param name="b">Second vector to add.</param>
		/// <param name="sum">Sum of the two vectors.</param>
		static void Add(Vector3 &a, Vector3 &b, Vector3 &sum);

		/// <summary>
		/// Subtracts two vectors.
		/// </summary>
		/// <param name="a">Vector to subtract from.</param>
		/// <param name="b">Vector to subtract from the first vector.</param>
		/// <param name="difference">Result of the subtraction.</param>
		static void Subtract(Vector3 &a, Vector3 &b, Vector3 &difference);

		/// <summary>
		/// Scales a vector.
		/// </summary>
		/// <param name="v">Vector to scale.</param>
		/// <param name="scale">Amount to scale.</param>
		/// <param name="result">Scaled vector.</param>
		static void Multiply(Vector3 &v, float scale, Vector3 &result);

		/// <summary>
		/// Divides a vector's components by some amount.
		/// </summary>
		/// <param name="v">Vector to divide.</param>
		/// <param name="divisor">Value to divide the vector's components.</param>
		/// <param name="result">Result of the division.</param>
		static void Divide(Vector3 &v, float divisor, Vector3 &result);

		/// <summary>
		/// Computes the squared distance between two vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="distanceSquared">Squared distance between the two vectors.</param>
		static void DistanceSquared(Vector3 &a, Vector3 &b, float &distanceSquared);

		/// <summary>
		/// Computes the distance between two two vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="distance">Distance between the two vectors.</param>
		static void Distance(Vector3 &a, Vector3 &b, float &distance);

		/// <summary>
		/// Computes the cross product between two vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Cross product of the two vectors.</param>
		static void Cross(Vector3 &a, Vector3 &b, Vector3 &result);


		/// <summary>
		/// Normalizes the given vector.
		/// </summary>
		/// <param name="v">Vector to normalize.</param>
		/// <param name="result">Normalized vector.</param>
		static void Normalize(Vector3 &v, Vector3 &result);

		/// <summary>
		/// Negates a vector.
		/// </summary>
		/// <param name="v">Vector to negate.</param>
		/// <param name="negated">Negated vector.</param>
		static void Negate(Vector3 &v, Vector3 &negated);

		/// <summary>
		/// Computes a vector with the minimum components of the given vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Vector with the smaller components of each input vector.</param>
		static void Min(Vector3 &a, Vector3 &b, Vector3 &result);
		/// <summary>
		/// Computes a vector with the maximum components of the given vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Vector with the larger components of each input vector.</param>
		static void Max(Vector3 &a, Vector3 &b, Vector3 &result);

		/// <summary>
		/// Computes an interpolated state between two vectors.
		/// </summary>
		/// <param name="start">Starting location of the interpolation.</param>
		/// <param name="end">Ending location of the interpolation.</param>
		/// <param name="interpolationAmount">Amount of the end location to use.</param>
		/// <param name="result">Interpolated intermediate state.</param>
		static void Lerp(Vector3 &start, Vector3 &end, float interpolationAmount, Vector3 &result);

		/// <summary>
		/// Computes an intermediate location using hermite interpolation.
		/// </summary>
		/// <param name="value1">First position.</param>
		/// <param name="tangent1">Tangent associated with the first position.</param>
		/// <param name="value2">Second position.</param>
		/// <param name="tangent2">Tangent associated with the second position.</param>
		/// <param name="interpolationAmount">Amount of the second point to use.</param>
		/// <param name="result">Interpolated intermediate state.</param>
		static void Hermite(Vector3 &value1, Vector3 &tangent1, Vector3 &value2, Vector3 &tangent2, float interpolationAmount, Vector3 &result);
	};
}


