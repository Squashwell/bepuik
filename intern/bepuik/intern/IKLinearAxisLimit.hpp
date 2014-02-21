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
	/// Keeps an anchor point on one bone between planes defined by another bone.
	/// </summary>
	class IKLinearAxisLimit : public IKLimit
	{
	private:
		float minimumDistance;
		float maximumDistance;
		float error;
	public:
		/// <summary>
		/// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
		/// </summary>
		Vector3 LocalLineAnchor;

		/// <summary>
		/// Gets or sets the direction of the line in connection A's local space.
		/// Must be unit length.
		/// </summary>
		Vector3 LocalLineDirection;

		/// <summary>
		/// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the line.
		/// </summary>
		Vector3 LocalAnchorB;


		/// <summary>
		/// Gets the world space location of the line anchor attached to connection A.
		/// </summary>
		Vector3 GetLineAnchor();

		/// <summary>
		/// Sets the world space location of the line anchor attached to connection A.
		/// </summary>
		void SetLineAnchor(Vector3 lineAnchor);

		/// <summary>
		/// Gets the world space direction of the line attached to connection A.
		/// Must be unit length.
		/// </summary>
		Vector3 GetLineDirection();

		/// <summary>
		/// Sets the world space direction of the line attached to connection A.
		/// Must be unit length.
		/// </summary>
		void SetLineDirection(Vector3 lineDirection);

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
		float GetMinimumDistance();
		/// <summary>
		/// Sets the distance that the joint connections should be kept from each other.
		/// </summary>
		void SetMinimumDistance(float newDistance);

		/// <summary>
		/// Gets the distance that the joint connections should be kept from each other.
		/// </summary>
		float GetMaximumDistance();
		/// <summary>
		/// Sets the distance that the joint connections should be kept from each other.
		/// </summary>
		void SetMaximumDistance(float newDistance);

		/// <summary>
		/// Constructs a new axis limit.
		/// </summary>
		/// <param name="connectionA">First bone connected by the joint.</param>
		/// <param name="connectionB">Second bone connected by the joint.</param>
		/// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
		/// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
		/// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
		/// <param name="minimumDistance">Minimum distance that the joint connections should be kept from each other along the axis.</param>
		/// <param name="maximumDistance">Maximum distance that the joint connections should be kept from each other along the axis.</param>
		IKLinearAxisLimit(IKBone* connectionA, IKBone* connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB, float minimumDistance, float maximumDistance);

		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	};
}
