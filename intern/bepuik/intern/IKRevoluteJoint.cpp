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

#include "IKRevoluteJoint.hpp"
#include "Toolbox.hpp"
#include "Vector2.hpp"

namespace BEPUik
{

	/// <summary>
	/// Gets the free axis in connection A's local space.
	/// Must be unit length.
	/// </summary>
	Vector3 IKRevoluteJoint::GetLocalFreeAxisA()
	{
		return localFreeAxisA;
	}
	/// <summary>
	/// Sets the free axis in connection A's local space.
	/// Must be unit length.
	/// </summary>
	void IKRevoluteJoint::SetLocalFreeAxisA(Vector3 axis)
	{
		localFreeAxisA = axis;
		ComputeConstrainedAxes();
	}

	/// <summary>
	/// Gets or sets the free axis in connection B's local space.
	/// Must be unit length.
	/// </summary>
	Vector3 IKRevoluteJoint::GetLocalFreeAxisB()
	{
		return localFreeAxisB;
	}

	/// <summary>
	/// Gets or sets the free axis in connection B's local space.
	/// Must be unit length.
	/// </summary>
	void IKRevoluteJoint::SetLocalFreeAxisB(Vector3 axis)
	{
		localFreeAxisB = axis;
		ComputeConstrainedAxes();
	}



	/// <summary>
	/// Gets the free axis attached to connection A in world space.
	/// This does not change the other connection's free axis.
	/// </summary>
	Vector3 IKRevoluteJoint::GetWorldFreeAxisA()
	{
		Vector3 toReturn;
		Quaternion::Transform(localFreeAxisA, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the free axis attached to connection A in world space.
	/// This does not change the other connection's free axis.
	/// </summary>
	void IKRevoluteJoint::SetWorldFreeAxisA(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, localFreeAxisA);
		ComputeConstrainedAxes();
	}

	/// <summary>
	/// Gets the free axis attached to connection B in world space.
	/// This does not change the other connection's free axis.
	/// </summary>
	Vector3 IKRevoluteJoint::GetWorldFreeAxisB()
	{
		Vector3 toReturn;
		Quaternion::Transform(localFreeAxisB, connectionB->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the free axis attached to connection B in world space.
	/// This does not change the other connection's free axis.
	/// </summary>
	void IKRevoluteJoint::SetWorldFreeAxisB(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, localFreeAxisB);
		ComputeConstrainedAxes();
	}


	void IKRevoluteJoint::ComputeConstrainedAxes()
	{
		Vector3 worldAxisA = GetWorldFreeAxisA();
		Vector3 worldAxisB = GetWorldFreeAxisB();
		Vector3 error;
		Vector3::Cross(worldAxisA, worldAxisB, error);
		float lengthSquared = error.LengthSquared();
		Vector3 worldConstrainedAxis1;
		Vector3 worldConstrainedAxis2;
		//Find the first constrained axis.
		if (lengthSquared > Toolbox::Epsilon)
		{
			//The error direction can be used as the first axis!
			Vector3::Divide(error, sqrt(lengthSquared), worldConstrainedAxis1);
		}
		else
		{
			//There's not enough error for it to be a good constrained axis.
			//We'll need to create the constrained axes arbitrarily.
			Vector3::Cross(Toolbox::UpVector, worldAxisA, worldConstrainedAxis1);
			lengthSquared = worldConstrainedAxis1.LengthSquared();
			if (lengthSquared > Toolbox::Epsilon)
			{
				//The up vector worked!
				Vector3::Divide(worldConstrainedAxis1, sqrt(lengthSquared), worldConstrainedAxis1);
			}
			else
			{
				//The up vector didn't work. Just try the right vector.
				Vector3::Cross(Toolbox::RightVector, worldAxisA, worldConstrainedAxis1);
				worldConstrainedAxis1.Normalize();
			}
		}
		//Don't have to normalize the second constraint axis; it's the cross product of two perpendicular normalized vectors.
		Vector3::Cross(worldAxisA, worldConstrainedAxis1, worldConstrainedAxis2);

		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(worldConstrainedAxis1, conjugate, localConstrainedAxis1);
		Quaternion::Transform(worldConstrainedAxis2, conjugate, localConstrainedAxis2);
	}

	/// <summary>
	/// Constructs a new orientation joint.
	/// Orientation joints can be used to simulate the angular portion of a hinge.
	/// Orientation joints allow rotation around only a single axis.
	/// </summary>
	/// <param name="connectionA">First entity connected in the orientation joint.</param>
	/// <param name="connectionB">Second entity connected in the orientation joint.</param>
	/// <param name="freeAxis">Axis allowed to rotate freely in world space.</param>
	IKRevoluteJoint::IKRevoluteJoint(IKBone* connectionA, IKBone* connectionB, Vector3 freeAxis)
		: IKJoint(connectionA, connectionB)
	{
		SetWorldFreeAxisA(freeAxis);
		SetWorldFreeAxisB(freeAxis);
	}

	void IKRevoluteJoint::UpdateJacobiansAndVelocityBias()
	{
		linearJacobianA = linearJacobianB = Matrix3X3();

		//We know the one free axis. We need the two restricted axes. This amounts to completing the orthonormal basis.
		//We can grab one of the restricted axes using a cross product of the two world axes. This is not guaranteed
		//to be nonzero, so the normalization requires protection.

		Vector3 worldAxisA, worldAxisB;
		Quaternion::Transform(localFreeAxisA, connectionA->Orientation, worldAxisA);
		Quaternion::Transform(localFreeAxisB, connectionB->Orientation, worldAxisB);

		Vector3 error;
		Vector3::Cross(worldAxisA, worldAxisB, error);

		Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
		Quaternion::Transform(localConstrainedAxis1, connectionA->Orientation, worldConstrainedAxis1);
		Quaternion::Transform(localConstrainedAxis2, connectionA->Orientation, worldConstrainedAxis2);


		angularJacobianA = Matrix3X3();

		angularJacobianA.M11 = worldConstrainedAxis1.X;
		angularJacobianA.M12 = worldConstrainedAxis1.Y;
		angularJacobianA.M13 = worldConstrainedAxis1.Z;
		angularJacobianA.M21 = worldConstrainedAxis2.X;
		angularJacobianA.M22 = worldConstrainedAxis2.Y;
		angularJacobianA.M23 = worldConstrainedAxis2.Z;

		Matrix3X3::Negate(angularJacobianA, angularJacobianB);


		Vector3::Dot(error, worldConstrainedAxis1, constraintSpaceError.X);
		Vector3::Dot(error, worldConstrainedAxis2, constraintSpaceError.Y);
		velocityBias.X = errorCorrectionFactor * constraintSpaceError.X;
		velocityBias.Y = errorCorrectionFactor * constraintSpaceError.Y;


	}
	
	bool IKRevoluteJoint::HasError()
	{
		return !constraintSpaceError.IsZero(0.002f);
	}
}
