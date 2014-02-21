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

namespace BEPUik
{
	class IKSwivelHingeJoint : public IKJoint
	{
	private:
		float error;
	public:
		/// <summary>
		/// Gets or sets the free hinge axis attached to connection A in its local space.
		/// </summary>
		Vector3 LocalHingeAxis;
		/// <summary>
		/// Gets or sets the free twist axis attached to connection B in its local space.
		/// </summary>
		Vector3 LocalTwistAxis;


		/// <summary>
		/// Gets the free hinge axis attached to connection A in world space.
		/// </summary>
		Vector3 GetHingeAxis();

		/// <summary>
		/// Sets the free hinge axis attached to connection A in world space.
		/// </summary>
		void SetHingeAxis(Vector3 axis);

		/// <summary>
		/// Gets the free twist axis attached to connection B in world space.
		/// </summary>
		Vector3 GetTwistAxis();

		/// <summary>
		/// Sets the free twist axis attached to connection B in world space.
		/// </summary>
		void SetTwistAxis(Vector3 axis);


		/// <summary>
		/// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
		/// </summary>
		/// <param name="connectionA">First connection of the pair.</param>
		/// <param name="connectionB">Second connection of the pair.</param>
		/// <param name="worldHingeAxis">Hinge axis attached to connectionA.
		/// The connected bone will be able to rotate around this axis relative to each other.</param>
		/// <param name="worldTwistAxis">Twist axis attached to connectionB.
		/// The connected bones will be able to rotate around this axis relative to each other.</param>
		IKSwivelHingeJoint(IKBone* connectionA, IKBone* connectionB, Vector3 worldHingeAxis, Vector3 worldTwistAxis);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
