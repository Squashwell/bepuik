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
#include "Toolbox.hpp"

namespace BEPUik
{
	/// <summary>
	/// Keeps the anchor points on two bones at the same distance.
	/// </summary>
	class IKDistanceJoint : public IKJoint
	{
	private:
		float distance;
		float error;
	public:
		/// <summary>
		/// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
		/// </summary>
		Vector3 LocalAnchorA;
		/// <summary>
		/// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
		/// </summary>
		Vector3 LocalAnchorB;

		/// <summary>
		/// Gets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		Vector3 GetAnchorA();

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		void SetAnchorA(Vector3 anchor);

		/// <summary>
		/// Gets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		Vector3 GetAnchorB();

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		void SetAnchorB(Vector3 anchor);

		/// <summary>
		/// Gets the distance that the joint connections should be kept from each other.
		/// </summary>
		float GetDistance();
		/// <summary>
		/// Sets the distance that the joint connections should be kept from each other.
		/// </summary>
		void SetDistance(float newDistance);

		/// <summary>
		/// Constructs a new distance joint.
		/// </summary>
		/// <param name="connectionA">First bone connected by the joint.</param>
		/// <param name="connectionB">Second bone connected by the joint.</param>
		/// <param name="anchorA">Anchor point on the first bone in world space.</param>
		/// <param name="anchorB">Anchor point on the second bone in world space.</param>
		IKDistanceJoint(IKBone* connectionA, IKBone* connectionB, Vector3 anchorA, Vector3 anchorB);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
