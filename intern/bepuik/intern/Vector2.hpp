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
#include <cmath>

namespace BEPUmath
{
	/// <summary>
	/// Provides XNA-like 2D vector math needed by the engine.
	/// </summary>
	struct Vector2
	{
		/// <summary>
		/// X component of the vector.
		/// </summary>
		float X;
		/// <summary>
		/// Y component of the vector.
		/// </summary>
		float Y;

		Vector2();

		/// <summary>
		/// Constructs a new two dimensional vector.
		/// </summary>
		/// <param name="x">X component of the vector.</param>
		/// <param name="y">Y component of the vector.</param>
		Vector2(float x, float y);

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
		/// Adds two vectors together.
		/// </summary>
		/// <param name="a">First vector to add.</param>
		/// <param name="b">Second vector to add.</param>
		/// <param name="sum">Sum of the two vectors.</param>
		static void Add(Vector2 &a, Vector2 &b, Vector2 &sum);

		/// <summary>
		/// Subtracts two vectors.
		/// </summary>
		/// <param name="a">Vector to subtract from.</param>
		/// <param name="b">Vector to subtract from the first vector.</param>
		/// <param name="difference">Result of the subtraction.</param>
		static void Subtract(Vector2 &a, Vector2 &b, Vector2 &difference);

		/// <summary>
		/// Scales a vector.
		/// </summary>
		/// <param name="v">Vector to scale.</param>
		/// <param name="scale">Amount to scale.</param>
		/// <param name="result">Scaled vector.</param>
		static void Multiply(Vector2 &v, float scale, Vector2 &result);

		/// <summary>
		/// Divides a vector's components by some amount.
		/// </summary>
		/// <param name="v">Vector to divide.</param>
		/// <param name="divisor">Value to divide the vector's components.</param>
		/// <param name="result">Result of the division.</param>
		static void Divide(Vector2 &v, float divisor, Vector2 &result);

		/// <summary>
		/// Normalizes the vector.
		/// </summary>
		/// <param name="v">Vector to normalize.</param>
		/// <param name="result">Normalized vector.</param>
		static void Normalize(Vector2 &v, Vector2 &result);

		/// <summary>
		/// Negates the vector.
		/// </summary>
		/// <param name="v">Vector to negate.</param>
		/// <param name="negated">Negated version of the vector.</param>
		static void Negate(Vector2 &v, Vector2 &negated);

		/// <summary>
		/// Normalizes the vector.
		/// </summary>
		void Normalize();
		
		bool IsZero(float limit);
	};
}
