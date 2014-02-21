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
#include "IKLimit.hpp"

namespace BEPUik
{
	/// <summary>
	/// Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
	/// </summary>
	class IKSwingLimit : public IKLimit
	{
	private:
		float maximumAngle;
		float errorAngle;
	public:
		/// <summary>
		/// Gets or sets the axis attached to ConnectionA in its local space.
		/// </summary>
		Vector3 LocalAxisA;

		/// <summary>
		/// Gets or sets the axis attached to ConnectionB in its local space.
		/// </summary>
		Vector3 LocalAxisB;

		/// <summary>
		/// Gets the axis attached to ConnectionA in world space.
		/// </summary>
		Vector3 GetAxisA();

		/// <summary>
		/// Sets the axis attached to ConnectionA in world space.
		/// </summary>
		void SetAxisA(Vector3 axis);

		/// <summary>
		/// Gets the axis attached to connection B in world space.
		/// </summary>
		Vector3 GetAxisB();

		/// <summary>
		/// Sets the axis attached to connection B in world space.
		/// </summary>
		void SetAxisB(Vector3 axis);



		/// <summary>
		/// Gets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		float GetMaximumAngle();

		/// <summary>
		/// Sets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		void SetMaximumAngle(float angle);


		/// <summary>
		/// Builds a new swing limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
		/// </summary>
		/// <param name="connectionA">First connection of the limit.</param>
		/// <param name="connectionB">Second connection of the limit.</param>
		/// <param name="axisA">Axis attached to connectionA in world space.</param>
		/// <param name="axisB">Axis attached to connectionB in world space.</param>
		/// <param name="maximumAngle">Maximum angle allowed between connectionA's axis and connectionB's axis.</param>
		IKSwingLimit(IKBone* connectionA, IKBone* connectionB, Vector3 axisA, Vector3 axisB, float maximumAngle);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
