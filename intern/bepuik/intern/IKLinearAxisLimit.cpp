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

#include "IKLinearAxisLimit.hpp"
#include <cmath>

namespace BEPUik
{
	/// <summary>
	/// Gets the world space location of the line anchor attached to connection A.
	/// </summary>
	Vector3 IKLinearAxisLimit::GetLineAnchor()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalLineAnchor, connectionA->Orientation, toReturn);
		Vector3::Add(toReturn, connectionA->Position, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the world space location of the line anchor attached to connection A.
	/// </summary>
	void IKLinearAxisLimit::SetLineAnchor(Vector3 lineAnchor)
	{
		Vector3 offset;
		Vector3::Subtract(lineAnchor, connectionA->Position, offset);
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(offset, conjugate, LocalLineAnchor);
	}

	/// <summary>
	/// Gets the world space direction of the line attached to connection A.
	/// Must be unit length.
	/// </summary>
	Vector3 IKLinearAxisLimit::GetLineDirection()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalLineDirection, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the world space direction of the line attached to connection A.
	/// Must be unit length.
	/// </summary>
	void IKLinearAxisLimit::SetLineDirection(Vector3 lineDirection)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(lineDirection, conjugate, LocalLineDirection);
	}

	/// <summary>
	/// Gets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	Vector3 IKLinearAxisLimit::GetAnchorB()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, toReturn);
		Vector3::Add(connectionB->Position, toReturn, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	void IKLinearAxisLimit::SetAnchorB(Vector3 anchor)
	{
		Vector3::Subtract(anchor, connectionB->Position, anchor);
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(anchor, conjugate, anchor);
		LocalAnchorB = anchor;
	}

	/// <summary>
	/// Gets the distance that the joint connections should be kept from each other.
	/// </summary>
	float IKLinearAxisLimit::GetMinimumDistance()
	{
		return minimumDistance;
	}
	/// <summary>
	/// Sets the distance that the joint connections should be kept from each other.
	/// </summary>
	void IKLinearAxisLimit::SetMinimumDistance(float newDistance)
	{
		minimumDistance = newDistance;//MathHelper::Max(0, newDistance);
	}

	/// <summary>
	/// Gets the distance that the joint connections should be kept from each other.
	/// </summary>
	float IKLinearAxisLimit::GetMaximumDistance()
	{
		return maximumDistance;
	}
	/// <summary>
	/// Sets the distance that the joint connections should be kept from each other.
	/// </summary>
	void IKLinearAxisLimit::SetMaximumDistance(float newDistance)
	{
		maximumDistance = newDistance;//MathHelper::Max(0, newDistance);
	}

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
	IKLinearAxisLimit::IKLinearAxisLimit(IKBone* connectionA, IKBone* connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB, float minimumDistance, float maximumDistance)
		: IKLimit(connectionA, connectionB)
	{
		SetLineAnchor(lineAnchor);
		SetLineDirection(lineDirection);
		SetAnchorB(anchorB);
		SetMinimumDistance(minimumDistance);
		SetMaximumDistance(maximumDistance);
	}

	void IKLinearAxisLimit::UpdateJacobiansAndVelocityBias()
	{
		//Transform the anchors and offsets into world space.
		Vector3 offsetA, offsetB, lineDirection;
		Quaternion::Transform(LocalLineAnchor, connectionA->Orientation, offsetA);
		Quaternion::Transform(LocalLineDirection, connectionA->Orientation, lineDirection);
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, offsetB);
		Vector3 anchorA, anchorB;
		Vector3::Add(connectionA->Position, offsetA, anchorA);
		Vector3::Add(connectionB->Position, offsetB, anchorB);

		//Compute the distance.
		Vector3 separation;
		Vector3::Subtract(anchorB, anchorA, separation);
		//This entire constraint is very similar to the IKDistanceLimit, except the current distance is along an axis.
		float currentDistance;
		Vector3::Dot(separation, lineDirection, currentDistance);

		//Compute jacobians
		if (currentDistance > maximumDistance)
		{
			//We are exceeding the maximum limit.
			error = (currentDistance - maximumDistance);
			velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);
		}
		else if (currentDistance < minimumDistance)
		{
			//We are exceeding the minimum limit.
			error = (minimumDistance - currentDistance);
			velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);
			//The limit can only push in one direction. Flip the jacobian!
			Vector3::Negate(lineDirection, lineDirection);
		}
		else if (currentDistance - minimumDistance > (maximumDistance - minimumDistance) * 0.5f)
		{
			//The objects are closer to hitting the maximum limit.
			error = 0;
			velocityBias = Vector3(currentDistance - maximumDistance, 0, 0);
		}
		else
		{
			//The objects are closer to hitting the minimum limit.
			error = 0;
			velocityBias = Vector3(minimumDistance - currentDistance, 0, 0);
			//The limit can only push in one direction. Flip the jacobian!
			Vector3::Negate(lineDirection, lineDirection);
		}

		Vector3 angularA, angularB;
		//We can't just use the offset to anchor for A's jacobian- the 'collision' location is way out there at anchorB!
		Vector3 rA;
		Vector3::Subtract(anchorB, connectionA->Position, rA);
		Vector3::Cross(rA, lineDirection, angularA);
		//linearB = -linearA, so just swap the cross product order.
		Vector3::Cross(lineDirection, offsetB, angularB);

		//Put all the 1x3 jacobians into a 3x3 matrix representation.
		linearJacobianA = Matrix3X3();
		linearJacobianA.M11 = lineDirection.X;
		linearJacobianA.M12 = lineDirection.Y;
		linearJacobianA.M13 = lineDirection.Z;
		linearJacobianB = Matrix3X3();
		linearJacobianB.M11 = -lineDirection.X;
		linearJacobianB.M12 = -lineDirection.Y;
		linearJacobianB.M13 = -lineDirection.Z;
		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = angularA.X;
		angularJacobianA.M12 = angularA.Y;
		angularJacobianA.M13 = angularA.Z;
		angularJacobianB = Matrix3X3();
		angularJacobianB.M11 = angularB.X;
		angularJacobianB.M12 = angularB.Y;
		angularJacobianB.M13 = angularB.Z;

	}
	
	bool IKLinearAxisLimit::HasError()
	{
		return !(std::abs(error) < 0.0001f);
	}
}
