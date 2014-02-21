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
	/// <summary>
	/// Keeps an anchor point on one bone on a plane defined by another bone.
	/// </summary>
	class IKPointOnPlaneJoint : public IKJoint
	{

	public:
		/// <summary>
		/// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
		/// </summary>
		Vector3 LocalPlaneAnchor;

		/// <summary>
		/// Gets or sets the direction of the line in connection A's local space.
		/// Must be unit length.
		/// </summary>
		Vector3 LocalPlaneNormal;

		/// <summary>
		/// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the plane.
		/// </summary>
		Vector3 LocalAnchorB;


		/// <summary>
		/// Gets the world space location of the line anchor attached to connection A.
		/// </summary>
		Vector3 GetPlaneAnchor();

		/// <summary>
		/// Sets the world space location of the line anchor attached to connection A.
		/// </summary>
		void SetPlaneAnchor(Vector3 anchor);

		/// <summary>
		/// Gets the world space normal of the plane attached to connection A.
		/// Must be unit length.
		/// </summary>
		Vector3 GetPlaneNormal();

		/// <summary>
		/// Sets the world space normal of the plane attached to connection A.
		/// Must be unit length.
		/// </summary>
		void SetPlaneNormal(Vector3 normal);

		/// <summary>
		/// Gets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		Vector3 GetAnchorB();

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		void SetAnchorB(Vector3 anchor);


		/// <summary>
		/// Constructs a new point on plane joint.
		/// </summary>
		/// <param name="connectionA">First bone connected by the joint.</param>
		/// <param name="connectionB">Second bone connected by the joint.</param>
		/// <param name="planeAnchor">Anchor point of the plane attached to the first bone in world space.</param>
		/// <param name="planeNormal">Normal of the plane attached to the first bone in world space. Must be unit length.</param>
		/// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
		IKPointOnPlaneJoint(IKBone* connectionA, IKBone* connectionB, Vector3 planeAnchor, Vector3 planeNormal, Vector3 anchorB);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	private:
		float currentDistanceError;
	};
}
