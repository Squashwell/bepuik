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
#include <string>
#include "Vector3.hpp"

namespace BEPUmath
{

	Vector3::Vector3():X(0), Y(0), Z(0)
	{
	}

	/// <summary>
	/// Constructs a new 3d vector.
	/// </summary>
	/// <param name="x">X component of the vector.</param>
	/// <param name="y">Y component of the vector.</param>
	/// <param name="z">Z component of the vector.</param>
	Vector3::Vector3(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	/// <summary>
	/// Computes the squared length of the vector.
	/// </summary>
	/// <returns>Squared length of the vector.</returns>
	float Vector3::LengthSquared()
	{
		return X * X + Y * Y + Z * Z;
	}

	/// <summary>
	/// Computes the length of the vector.
	/// </summary>
	/// <returns>Length of the vector.</returns>
	float Vector3::Length()
	{
		return sqrt(X * X + Y * Y + Z * Z);
	}

	/// <summary>
	/// Normalizes the vector.
	/// </summary>
	void Vector3::Normalize()
	{
		float inverse = (float)(1 / sqrt(X * X + Y * Y + Z * Z));
		X *= inverse;
		Y *= inverse;
		Z *= inverse;
	}
	
	bool Vector3::IsZero(float limit)
	{
		if (std::abs(X) < limit)
			if (std::abs(Y) < limit)
				if (std::abs(Z) < limit)
					return true;
	
		return false;
	}

	/// <summary>
	/// Computes the dot product of two vectors.
	/// </summary>
	/// <param name="a">First vector in the product.</param>
	/// <param name="b">Second vector in the product.</param>
	/// <param name="product">Resulting dot product.</param>
	void Vector3::Dot(Vector3 &a, Vector3 &b, float &product)
	{
		product = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
	}
	/// <summary>
	/// Adds two vectors together.
	/// </summary>
	/// <param name="a">First vector to add.</param>
	/// <param name="b">Second vector to add.</param>
	/// <param name="sum">Sum of the two vectors.</param>
	void Vector3::Add(Vector3 &a, Vector3 &b, Vector3 &sum)
	{
		sum.X = a.X + b.X;
		sum.Y = a.Y + b.Y;
		sum.Z = a.Z + b.Z;
	}
	/// <summary>
	/// Subtracts two vectors.
	/// </summary>
	/// <param name="a">Vector to subtract from.</param>
	/// <param name="b">Vector to subtract from the first vector.</param>
	/// <param name="difference">Result of the subtraction.</param>
	void Vector3::Subtract(Vector3 &a, Vector3 &b, Vector3 &difference)
	{
		difference.X = a.X - b.X;
		difference.Y = a.Y - b.Y;
		difference.Z = a.Z - b.Z;
	}

	/// <summary>
	/// Scales a vector.
	/// </summary>
	/// <param name="v">Vector to scale.</param>
	/// <param name="scale">Amount to scale.</param>
	/// <param name="result">Scaled vector.</param>
	void Vector3::Multiply(Vector3 &v, float scale, Vector3 &result)
	{
		result.X = v.X * scale;
		result.Y = v.Y * scale;
		result.Z = v.Z * scale;
	}

	/// <summary>
	/// Divides a vector's components by some amount.
	/// </summary>
	/// <param name="v">Vector to divide.</param>
	/// <param name="divisor">Value to divide the vector's components.</param>
	/// <param name="result">Result of the division.</param>
	void Vector3::Divide(Vector3 &v, float divisor, Vector3 &result)
	{
		float inverse = 1 / divisor;
		result.X = v.X * inverse;
		result.Y = v.Y * inverse;
		result.Z = v.Z * inverse;
	}

	/// <summary>
	/// Computes the squared distance between two vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="distanceSquared">Squared distance between the two vectors.</param>
	void Vector3::DistanceSquared(Vector3 &a, Vector3 &b, float &distanceSquared)
	{
		float x = a.X - b.X;
		float y = a.Y - b.Y;
		float z = a.Z - b.Z;
		distanceSquared = x * x + y * y + z * z;
	}

	/// <summary>
	/// Computes the distance between two two vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="distance">Distance between the two vectors.</param>
	void Vector3::Distance(Vector3 &a, Vector3 &b, float &distance)
	{
		float x = a.X - b.X;
		float y = a.Y - b.Y;
		float z = a.Z - b.Z;
		distance = sqrt(x * x + y * y + z * z);
	}

	/// <summary>
	/// Computes the cross product between two vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="result">Cross product of the two vectors.</param>
	void Vector3::Cross(Vector3 &a, Vector3 &b, Vector3 &result)
	{
		float resultX = a.Y * b.Z - a.Z * b.Y;
		float resultY = a.Z * b.X - a.X * b.Z;
		float resultZ = a.X * b.Y - a.Y * b.X;
		result.X = resultX;
		result.Y = resultY;
		result.Z = resultZ;
	}


	/// <summary>
	/// Normalizes the given vector.
	/// </summary>
	/// <param name="v">Vector to normalize.</param>
	/// <param name="result">Normalized vector.</param>
	void Vector3::Normalize(Vector3 &v, Vector3 &result)
	{
		float inverse = (float)(1 / sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
		result.X = v.X * inverse;
		result.Y = v.Y * inverse;
		result.Z = v.Z * inverse;
	}

	/// <summary>
	/// Negates a vector.
	/// </summary>
	/// <param name="v">Vector to negate.</param>
	/// <param name="negated">Negated vector.</param>
	void Vector3::Negate(Vector3 &v, Vector3 &negated)
	{
		negated.X = -v.X;
		negated.Y = -v.Y;
		negated.Z = -v.Z;
	}


	/// <summary>
	/// Computes a vector with the minimum components of the given vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="result">Vector with the smaller components of each input vector.</param>
	void Vector3::Min(Vector3 &a, Vector3 &b, Vector3 &result)
	{
		result.X = a.X < b.X ? a.X : b.X;
		result.Y = a.Y < b.Y ? a.Y : b.Y;
		result.Z = a.Z < b.Z ? a.Z : b.Z;
	}
	/// <summary>
	/// Computes a vector with the maximum components of the given vectors.
	/// </summary>
	/// <param name="a">First vector.</param>
	/// <param name="b">Second vector.</param>
	/// <param name="result">Vector with the larger components of each input vector.</param>
	void Vector3::Max(Vector3 &a, Vector3 &b, Vector3 &result)
	{
		result.X = a.X > b.X ? a.X : b.X;
		result.Y = a.Y > b.Y ? a.Y : b.Y;
		result.Z = a.Z > b.Z ? a.Z : b.Z;
	}

	/// <summary>
	/// Computes an interpolated state between two vectors.
	/// </summary>
	/// <param name="start">Starting location of the interpolation.</param>
	/// <param name="end">Ending location of the interpolation.</param>
	/// <param name="interpolationAmount">Amount of the end location to use.</param>
	/// <param name="result">Interpolated intermediate state.</param>
	void Vector3::Lerp(Vector3 &start, Vector3 &end, float interpolationAmount, Vector3 &result)
	{
		float startAmount = 1 - interpolationAmount;
		result.X = start.X * startAmount + end.X * interpolationAmount;
		result.Y = start.Y * startAmount + end.Y * interpolationAmount;
		result.Z = start.Z * startAmount + end.Z * interpolationAmount;
	}

	/// <summary>
	/// Computes an intermediate location using hermite interpolation.
	/// </summary>
	/// <param name="value1">First position.</param>
	/// <param name="tangent1">Tangent associated with the first position.</param>
	/// <param name="value2">Second position.</param>
	/// <param name="tangent2">Tangent associated with the second position.</param>
	/// <param name="interpolationAmount">Amount of the second point to use.</param>
	/// <param name="result">Interpolated intermediate state.</param>
	void Vector3::Hermite(Vector3 &value1, Vector3 &tangent1, Vector3 &value2, Vector3 &tangent2, float interpolationAmount, Vector3 &result)
	{
		float weightSquared = interpolationAmount * interpolationAmount;
		float weightCubed = interpolationAmount * weightSquared;
		float value1Blend = 2 * weightCubed - 3 * weightSquared + 1;
		float tangent1Blend = weightCubed - 2 * weightSquared + interpolationAmount;
		float value2Blend = -2 * weightCubed + 3 * weightSquared;
		float tangent2Blend = weightCubed - weightSquared;
		result.X = value1.X * value1Blend + value2.X * value2Blend + tangent1.X * tangent1Blend + tangent2.X * tangent2Blend;
		result.Y = value1.Y * value1Blend + value2.Y * value2Blend + tangent1.Y * tangent1Blend + tangent2.Y * tangent2Blend;
		result.Z = value1.Z * value1Blend + value2.Z * value2Blend + tangent1.Z * tangent1Blend + tangent2.Z * tangent2Blend;
	}
	
	bool Vector3::IsNan()
	{
		return X!=X || Y!=Y || Z!=Z;
	}
}
