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

#include "IKPointOnPlaneJoint.hpp"
#include <cmath>

namespace BEPUik
{


	/// <summary>
	/// Gets the world space location of the line anchor attached to connection A.
	/// </summary>
	Vector3 IKPointOnPlaneJoint::GetPlaneAnchor()
	{
		Vector3 anchor;
		Quaternion::Transform(LocalPlaneAnchor, connectionA->Orientation, anchor);
		Vector3::Add(anchor, connectionA->Position, anchor);
		return anchor;
	}

	/// <summary>
	/// Sets the world space location of the line anchor attached to connection A.
	/// </summary>
	void IKPointOnPlaneJoint::SetPlaneAnchor(Vector3 anchor)
	{
		Vector3::Subtract(anchor, connectionA->Position, anchor);
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(anchor, conjugate, LocalPlaneAnchor);
	}

	/// <summary>
	/// Gets the world space normal of the plane attached to connection A.
	/// Must be unit length.
	/// </summary>
	Vector3 IKPointOnPlaneJoint::GetPlaneNormal()
	{
		Vector3 normal;
		Quaternion::Transform(LocalPlaneNormal, connectionA->Orientation, normal);
		return normal;
	}

	/// <summary>
	/// Sets the world space normal of the plane attached to connection A.
	/// Must be unit length.
	/// </summary>
	void IKPointOnPlaneJoint::SetPlaneNormal(Vector3 normal)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(normal, conjugate, LocalPlaneNormal);
	}

	/// <summary>
	/// Gets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	Vector3 IKPointOnPlaneJoint::GetAnchorB()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, toReturn);
		Vector3::Add(connectionB->Position, toReturn, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	void IKPointOnPlaneJoint::SetAnchorB(Vector3 anchor)
	{
		Vector3::Subtract(anchor, connectionB->Position, anchor);
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(anchor, conjugate, anchor);
		LocalAnchorB = anchor;
	}


	/// <summary>
	/// Constructs a new point on plane joint.
	/// </summary>
	/// <param name="connectionA">First bone connected by the joint.</param>
	/// <param name="connectionB">Second bone connected by the joint.</param>
	/// <param name="planeAnchor">Anchor point of the plane attached to the first bone in world space.</param>
	/// <param name="planeNormal">Normal of the plane attached to the first bone in world space. Must be unit length.</param>
	/// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
	IKPointOnPlaneJoint::IKPointOnPlaneJoint(IKBone* connectionA, IKBone* connectionB, Vector3 planeAnchor, Vector3 planeNormal, Vector3 anchorB)
		: IKJoint(connectionA, connectionB)
	{
		SetPlaneAnchor(planeAnchor);
		SetPlaneNormal(planeNormal);
		SetAnchorB(anchorB);
	}

	void IKPointOnPlaneJoint::UpdateJacobiansAndVelocityBias()
	{
		//Transform the anchors and offsets into world space.
		Vector3 offsetA, offsetB, lineDirection;
		Quaternion::Transform(LocalPlaneAnchor, connectionA->Orientation, offsetA);
		Quaternion::Transform(LocalPlaneNormal, connectionA->Orientation, lineDirection);
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, offsetB);
		Vector3 anchorA, anchorB;
		Vector3::Add(connectionA->Position, offsetA, anchorA);
		Vector3::Add(connectionB->Position, offsetB, anchorB);

		//Compute the distance.
		Vector3 separation;
		Vector3::Subtract(anchorB, anchorA, separation);
		//This entire constraint is very similar to the IKDistanceLimit, except the current distance is along an axis.
		Vector3::Dot(separation, lineDirection, currentDistanceError);
		velocityBias = Vector3(errorCorrectionFactor * currentDistanceError, 0, 0);

		//Compute jacobians
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
	
	bool IKPointOnPlaneJoint::HasError()
	{
		return !(std::abs(currentDistanceError) < 0.0001f);
	}
}
