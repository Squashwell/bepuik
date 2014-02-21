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
	/// Prevents two bones from twisting beyond a certain angle away from each other as measured by the twist between two measurement axes.
	/// </summary>
	class IKTwistLimit : public IKLimit
	{
	private:
		float maximumAngle;
		float errorAngle;
	public:
		/// <summary>
		/// Gets or sets the axis attached to ConnectionA in its local space.
		/// Must be unit length and perpendicular to LocalMeasurementAxisA.
		/// </summary>
		Vector3 LocalAxisA;
		/// <summary>
		/// Gets or sets the axis attached to ConnectionB in its local space.
		/// Must be unit length and perpendicular to LocalMeasurementAxisB.
		/// </summary>
		Vector3 LocalAxisB;

		/// <summary>
		/// Gets or sets the measurement axis attached to connection A.
		/// Must be unit length and perpendicular to LocalAxisA.
		/// </summary>
		Vector3 LocalMeasurementAxisA;
		/// <summary>
		/// Gets or sets the measurement axis attached to connection B.
		/// Must be unit length and perpendicular to LocalAxisB.
		/// </summary>
		Vector3 LocalMeasurementAxisB;

		/// <summary>
		/// Gets the axis attached to ConnectionA in world space.
		/// Must be unit length and perpendicular to measurement axis A.
		/// </summary>
		Vector3 GetAxisA();

		/// <summary>
		/// Sets the axis attached to connection A in world space.
		/// Must be unit length and perpendicular to measurement axis A.
		/// </summary>
		void SetAxisA(Vector3 axis);

		/// <summary>
		/// Gets the axis attached to connection B in world space.
		/// Must be unit length and perpendicular to measurement axis B.
		/// </summary>
		Vector3 GetAxisB();

		/// <summary>
		/// Sets the axis attached to connection B in world space.
		/// Must be unit length and perpendicular to measurement axis B.
		/// </summary>
		void SetAxisB(Vector3 axis);

		/// <summary>
		/// Gets the measurement axis attached to connection A in world space.
		/// This axis is compared against the other connection's measurement axis to determine the twist.
		/// Must be unit length and perpendicular to axis A.
		/// </summary>
		Vector3 GetMeasurementAxisA();

		/// <summary>
		/// Sets the measurement axis attached to connection A in world space.
		/// This axis is compared against the other connection's measurement axis to determine the twist.
		/// Must be unit length and perpendicular to axis A.
		/// </summary>
		void SetMeasurementAxisA(Vector3 axis);


		/// <summary>
		/// Gets the measurement axis attached to connection B in world space.
		/// This axis is compared against the other connection's measurement axis to determine the twist.
		/// Must be unit length and perpendicular to axis B.
		/// </summary>
		Vector3 GetMeasurementAxisB();

		/// <summary>
		/// Sets the measurement axis attached to connection B in world space.
		/// This axis is compared against the other connection's measurement axis to determine the twist.
		/// Must be unit length and perpendicular to axis B.
		/// </summary>
		void SetMeasurementAxisB(Vector3 axis);




		/// <summary>
		/// Automatically computes the measurement axes for the current local axes.
		/// The current relative state of the entities will be considered 0 twist angle.
		/// </summary>
		void ComputeMeasurementAxes();

		/// <summary>
		/// Gets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		float GetMaximumAngle();

		/// <summary>
		/// Sets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		void SetMaximumAngle(float angle);

		/// <summary>
		/// Builds a new twist limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
		/// </summary>
		/// <param name="connectionA">First connection of the limit.</param>
		/// <param name="connectionB">Second connection of the limit.</param>
		/// <param name="axisA">Axis attached to connectionA in world space.</param>
		/// <param name="axisB">Axis attached to connectionB in world space.</param>
		/// <param name="maximumAngle">Maximum angle allowed between connectionA's axis and connectionB's axis.</param>
		IKTwistLimit(IKBone* connectionA, IKBone* connectionB, Vector3 axisA, Vector3 measurementAxisA, Vector3 axisB, Vector3 measurementAxisB, float maximumAngle);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
