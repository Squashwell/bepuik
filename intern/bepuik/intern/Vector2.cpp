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

#include "Vector2.hpp"

namespace BEPUmath
{

	Vector2::Vector2() : X(0), Y(0)
	{
	}

	/// <summary>
	/// Constructs a new two dimensional vector.
	/// </summary>
	/// <param name="x">X component of the vector.</param>
	/// <param name="y">Y component of the vector.</param>
	Vector2::Vector2(float x, float y)
	{
		X = x;
		Y = y;
	}

	/// <summary>
	/// Computes the squared length of the vector.
	/// </summary>
	/// <returns>Squared length of the vector.</returns>
	float Vector2::LengthSquared()
	{
		return X * X + Y * Y;
	}

	/// <summary>
	/// Computes the length of the vector.
	/// </summary>
	/// <returns>Length of the vector.</returns>
	float Vector2::Length()
	{
		return (float)sqrt(X * X + Y * Y);
	}

	/// <summary>
	/// Adds two vectors together.
	/// </summary>
	/// <param name="a">First vector to add.</param>
	/// <param name="b">Second vector to add.</param>
	/// <param name="sum">Sum of the two vectors.</param>
	void Vector2::Add(Vector2 &a, Vector2 &b, Vector2 &sum)
	{
		sum.X = a.X + b.X;
		sum.Y = a.Y + b.Y;
	}

	/// <summary>
	/// Subtracts two vectors.
	/// </summary>
	/// <param name="a">Vector to subtract from.</param>
	/// <param name="b">Vector to subtract from the first vector.</param>
	/// <param name="difference">Result of the subtraction.</param>
	void Vector2::Subtract(Vector2 &a, Vector2 &b, Vector2 &difference)
	{
		difference.X = a.X - b.X;
		difference.Y = a.Y - b.Y;
	}

	/// <summary>
	/// Scales a vector.
	/// </summary>
	/// <param name="v">Vector to scale.</param>
	/// <param name="scale">Amount to scale.</param>
	/// <param name="result">Scaled vector.</param>
	void Vector2::Multiply(Vector2 &v, float scale, Vector2 &result)
	{
		result.X = v.X * scale;
		result.Y = v.Y * scale;
	}

	/// <summary>
	/// Divides a vector's components by some amount.
	/// </summary>
	/// <param name="v">Vector to divide.</param>
	/// <param name="divisor">Value to divide the vector's components.</param>
	/// <param name="result">Result of the division.</param>
	void Vector2::Divide(Vector2 &v, float divisor, Vector2 &result)
	{
		float inverse = 1 / divisor;
		result.X = v.X * inverse;
		result.Y = v.Y * inverse;
	}

	/// <summary>
	/// Normalizes the vector.
	/// </summary>
	/// <param name="v">Vector to normalize.</param>
	/// <param name="result">Normalized vector.</param>
	void Vector2::Normalize(Vector2 &v, Vector2 &result)
	{
		float inverse = (float)(1 / sqrt(v.X * v.X + v.Y * v.Y));
		result.X = v.X * inverse;
		result.Y = v.Y * inverse;
	}

	/// <summary>
	/// Negates the vector.
	/// </summary>
	/// <param name="v">Vector to negate.</param>
	/// <param name="negated">Negated version of the vector.</param>
	void Vector2::Negate(Vector2 &v, Vector2 &negated)
	{
		negated.X = -v.X;
		negated.Y = -v.Y;
	}

	/// <summary>
	/// Normalizes the vector.
	/// </summary>
	void Vector2::Normalize()
	{
		float inverse = (float)(1 / sqrt(X * X + Y * Y));
		X *= inverse;
		Y *= inverse;
	}
	
	bool Vector2::IsZero(float limit)
	{
		if (std::abs(X) < limit)
			if (std::abs(Y) < limit)
				return true;
	
		return false;
	}
}
