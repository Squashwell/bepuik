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

#include "MathHelper.hpp"

namespace BEPUmath
{
	/// <summary>
	/// Approximate value of Pi.
	/// </summary>
	const float MathHelper::Pi = 3.141592653589793239f;

	/// <summary>
	/// Approximate value of Pi multiplied by two.
	/// </summary>
	const float MathHelper::TwoPi = 6.283185307179586477f;

	/// <summary>
	/// Approximate value of Pi divided by two.
	/// </summary>
	const float MathHelper::PiOver2 = 1.570796326794896619f;

	/// <summary>
	/// Approximate value of Pi divided by four.
	/// </summary>
	const float MathHelper::PiOver4 = 0.785398163397448310f;

	/// <summary>
	/// Clamps a value between a minimum and maximum value.
	/// </summary>
	/// <param name="value">Value to clamp.</param>
	/// <param name="min">Minimum value.  If the value is less than this, the minimum is returned instead.</param>
	/// <param name="max">Maximum value.  If the value is more than this, the maximum is returned instead.</param>
	/// <returns>Clamped value.</returns>
	float MathHelper::Clamp(float value, float min, float max)
	{
		if (value < min)
			return min;
		else if (value > max)
			return max;
		return value;
	}

	/// <summary>
	/// Returns the higher value of the two parameters.
	/// </summary>
	/// <param name="a">First value.</param>
	/// <param name="b">Second value.</param>
	/// <returns>Higher value of the two parameters.</returns>
	float MathHelper::Max(float a, float b)
	{
		return a > b ? a : b;
	}

	/// <summary>
	/// Returns the lower value of the two parameters.
	/// </summary>
	/// <param name="a">First value.</param>
	/// <param name="b">Second value.</param>
	/// <returns>Lower value of the two parameters.</returns>
	float MathHelper::Min(float a, float b)
	{
		return a < b ? a : b;
	}

	/// <summary>
	/// Converts degrees to radians.
	/// </summary>
	/// <param name="degrees">Degrees to convert.</param>
	/// <returns>Radians equivalent to the input degrees.</returns>
	float MathHelper::ToRadians(float degrees)
	{
		return degrees * (Pi / 180);
	}

	/// <summary>
	/// Converts radians to degrees.
	/// </summary>
	/// <param name="radians">Radians to convert.</param>
	/// <returns>Degrees equivalent to the input radians.</returns>
	float MathHelper::ToDegrees(float radians)
	{
		return radians * (180 / Pi);
	}
}
