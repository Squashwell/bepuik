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
#include "IKJoint.hpp"
#include "Vector2.hpp"
namespace BEPUik
{
	class IKRevoluteJoint : public IKJoint
	{
	private:
		Vector3 localFreeAxisA;
		Vector3 localFreeAxisB;

		Vector3 localConstrainedAxis1;
		Vector3 localConstrainedAxis2;
		
		Vector2 constraintSpaceError;
	public:
		/// <summary>
		/// Gets the free axis in connection A's local space.
		/// Must be unit length.
		/// </summary>
		Vector3 GetLocalFreeAxisA();
		/// <summary>
		/// Sets the free axis in connection A's local space.
		/// Must be unit length.
		/// </summary>
		void SetLocalFreeAxisA(Vector3 axis);

		/// <summary>
		/// Gets or sets the free axis in connection B's local space.
		/// Must be unit length.
		/// </summary>
		Vector3 GetLocalFreeAxisB();

		/// <summary>
		/// Gets or sets the free axis in connection B's local space.
		/// Must be unit length.
		/// </summary>
		void SetLocalFreeAxisB(Vector3 axis);



		/// <summary>
		/// Gets the free axis attached to connection A in world space.
		/// This does not change the other connection's free axis.
		/// </summary>
		Vector3 GetWorldFreeAxisA();

		/// <summary>
		/// Sets the free axis attached to connection A in world space.
		/// This does not change the other connection's free axis.
		/// </summary>
		void SetWorldFreeAxisA(Vector3 axis);

		/// <summary>
		/// Gets the free axis attached to connection B in world space.
		/// This does not change the other connection's free axis.
		/// </summary>
		Vector3 GetWorldFreeAxisB();

		/// <summary>
		/// Sets the free axis attached to connection B in world space.
		/// This does not change the other connection's free axis.
		/// </summary>
		void SetWorldFreeAxisB(Vector3 axis);


		void ComputeConstrainedAxes();

		/// <summary>
		/// Constructs a new orientation joint.
		/// Orientation joints can be used to simulate the angular portion of a hinge.
		/// Orientation joints allow rotation around only a single axis.
		/// </summary>
		/// <param name="connectionA">First entity connected in the orientation joint.</param>
		/// <param name="connectionB">Second entity connected in the orientation joint.</param>
		/// <param name="freeAxis">Axis allowed to rotate freely in world space.</param>
		IKRevoluteJoint(IKBone* connectionA, IKBone* connectionB, Vector3 freeAxis);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
