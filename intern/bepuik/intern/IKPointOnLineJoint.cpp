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

#include "IKPointOnLineJoint.hpp"
#include "Toolbox.hpp"
#include "Vector2.hpp"

namespace BEPUik
{

	/// <summary>
	/// Gets the direction of the line in connection A's local space.
	/// Must be unit length.
	/// </summary>
	Vector3 IKPointOnLineJoint::GetLocalLineDirection()
	{
		return localLineDirection;
	}
	/// <summary>
	/// Sets the direction of the line in connection A's local space.
	/// Must be unit length.
	/// </summary>
	void IKPointOnLineJoint::SetLocalLineDirection(Vector3 lineDirection)
	{
		localLineDirection = lineDirection;
		ComputeRestrictedAxes();
	}


	/// <summary>
	/// Gets the world space location of the line anchor attached to connection A.
	/// </summary>
	Vector3 IKPointOnLineJoint::GetLineAnchor()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalLineAnchor, connectionA->Orientation, toReturn);
		Vector3::Add(toReturn, connectionA->Position, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the world space location of the line anchor attached to connection A.
	/// </summary>
	void IKPointOnLineJoint::SetLineAnchor(Vector3 lineAnchor)
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
	Vector3 IKPointOnLineJoint::GetLineDirection()
	{
		Vector3 toReturn;
		Quaternion::Transform(localLineDirection, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the world space direction of the line attached to connection A.
	/// Must be unit length.
	/// </summary>
	void IKPointOnLineJoint::SetLineDirection(Vector3 lineDirection)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(lineDirection, conjugate, localLineDirection);
		ComputeRestrictedAxes();
	}

	/// <summary>
	/// Gets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	Vector3 IKPointOnLineJoint::GetAnchorB()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, toReturn);
		Vector3::Add(connectionB->Position, toReturn, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the offset in world space from the center of mass of connection B to the anchor point.
	/// </summary>
	void IKPointOnLineJoint::SetAnchorB(Vector3 anchor)
	{
		Vector3::Subtract(anchor, connectionB->Position, anchor);
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(anchor, conjugate, anchor);
		LocalAnchorB = anchor;
	}

	void IKPointOnLineJoint::ComputeRestrictedAxes()
	{
		Vector3 cross;
		Vector3::Cross(localLineDirection, Toolbox::UpVector, cross);
		float lengthSquared = cross.LengthSquared();
		if (lengthSquared > Toolbox::Epsilon)
		{
			Vector3::Divide(cross, sqrt(lengthSquared), localRestrictedAxis1);
		}
		else
		{
			//Oops! The direction is aligned with the up vector.
			Vector3::Cross(localLineDirection, Toolbox::RightVector, cross);
			Vector3::Normalize(cross, localRestrictedAxis1);
		}
		//Don't need to normalize this; cross product of two unit length perpendicular vectors.
		Vector3::Cross(localRestrictedAxis1, localLineDirection, localRestrictedAxis2);
	}

	/// <summary>
	/// Constructs a new point on line joint.
	/// </summary>
	/// <param name="connectionA">First bone connected by the joint.</param>
	/// <param name="connectionB">Second bone connected by the joint.</param>
	/// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
	/// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
	/// <param name="anchorB">Anchor point on the second bone in world space which tries to stay on connection A's line.</param>
	IKPointOnLineJoint::IKPointOnLineJoint(IKBone* connectionA, IKBone* connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB)
		: IKJoint(connectionA, connectionB)
	{
		SetLineAnchor(lineAnchor);
		SetLineDirection(lineDirection);
		SetAnchorB(anchorB);

	}

	void IKPointOnLineJoint::UpdateJacobiansAndVelocityBias()
	{

		//Transform local stuff into world space
		Vector3 worldRestrictedAxis1, worldRestrictedAxis2;
		Quaternion::Transform(localRestrictedAxis1, connectionA->Orientation, worldRestrictedAxis1);
		Quaternion::Transform(localRestrictedAxis2, connectionA->Orientation, worldRestrictedAxis2);

		Vector3 worldLineAnchor;
		Quaternion::Transform(LocalLineAnchor, connectionA->Orientation, worldLineAnchor);
		Vector3::Add(worldLineAnchor, connectionA->Position, worldLineAnchor);
		Vector3 lineDirection;
		Quaternion::Transform(localLineDirection, connectionA->Orientation, lineDirection);

		Vector3 rB;
		Quaternion::Transform(LocalAnchorB, connectionB->Orientation, rB);
		Vector3 worldPoint;
		Vector3::Add(rB, connectionB->Position, worldPoint);

		//Find the point on the line closest to the world point.
		Vector3 offset;
		Vector3::Subtract(worldPoint, worldLineAnchor, offset);
		float distanceAlongAxis;
		Vector3::Dot(offset, lineDirection, distanceAlongAxis);

		Vector3 worldNearPoint;
		Vector3::Multiply(lineDirection, distanceAlongAxis, offset);
		Vector3::Add(worldLineAnchor, offset, worldNearPoint);
		Vector3 rA;
		Vector3::Subtract(worldNearPoint, connectionA->Position, rA);

		//Error
		Vector3 error3D;
		Vector3::Subtract(worldPoint, worldNearPoint, error3D);

		Vector3::Dot(error3D, worldRestrictedAxis1, error.X);
		Vector3::Dot(error3D, worldRestrictedAxis2, error.Y);

		velocityBias.X = errorCorrectionFactor * error.X;
		velocityBias.Y = errorCorrectionFactor * error.Y;


		//Set up the jacobians
		Vector3 angularA1, angularA2, angularB1, angularB2;
		Vector3::Cross(rA, worldRestrictedAxis1, angularA1);
		Vector3::Cross(rA, worldRestrictedAxis2, angularA2);
		Vector3::Cross(worldRestrictedAxis1, rB, angularB1);
		Vector3::Cross(worldRestrictedAxis2, rB, angularB2);

		//Put all the 1x3 jacobians into a 3x3 matrix representation.
		linearJacobianA = Matrix3X3();
		linearJacobianA.M11 = worldRestrictedAxis1.X;
		linearJacobianA.M12 = worldRestrictedAxis1.Y;
		linearJacobianA.M13 = worldRestrictedAxis1.Z;
		linearJacobianA.M21 = worldRestrictedAxis2.X;
		linearJacobianA.M22 = worldRestrictedAxis2.Y;
		linearJacobianA.M23 = worldRestrictedAxis2.Z;
		Matrix3X3::Negate(linearJacobianA, linearJacobianB);

		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = angularA1.X;
		angularJacobianA.M12 = angularA1.Y;
		angularJacobianA.M13 = angularA1.Z;
		angularJacobianA.M21 = angularA2.X;
		angularJacobianA.M22 = angularA2.Y;
		angularJacobianA.M23 = angularA2.Z;

		angularJacobianB = Matrix3X3();
		angularJacobianB.M11 = angularB1.X;
		angularJacobianB.M12 = angularB1.Y;
		angularJacobianB.M13 = angularB1.Z;
		angularJacobianB.M21 = angularB2.X;
		angularJacobianB.M22 = angularB2.Y;
		angularJacobianB.M23 = angularB2.Z;
	}
	
	bool IKPointOnLineJoint::HasError()
	{
		return !error.IsZero(0.0001f);
	}
}
