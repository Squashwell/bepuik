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

#include "IKSwivelHingeJoint.hpp"
#include "Toolbox.hpp"
#include <cmath>

namespace BEPUik
{

	/// <summary>
	/// Gets the free hinge axis attached to connection A in world space.
	/// </summary>
	Vector3 IKSwivelHingeJoint::GetHingeAxis()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalHingeAxis, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the free hinge axis attached to connection A in world space.
	/// </summary>
	void IKSwivelHingeJoint::SetHingeAxis(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalHingeAxis);
	}

	/// <summary>
	/// Gets the free twist axis attached to connection B in world space.
	/// </summary>
	Vector3 IKSwivelHingeJoint::GetTwistAxis()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalTwistAxis, connectionB->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the free twist axis attached to connection B in world space.
	/// </summary>
	void IKSwivelHingeJoint::SetTwistAxis(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalTwistAxis);
	}


	/// <summary>
	/// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
	/// </summary>
	/// <param name="connectionA">First connection of the pair.</param>
	/// <param name="connectionB">Second connection of the pair.</param>
	/// <param name="worldHingeAxis">Hinge axis attached to connectionA.
	/// The connected bone will be able to rotate around this axis relative to each other.</param>
	/// <param name="worldTwistAxis">Twist axis attached to connectionB.
	/// The connected bones will be able to rotate around this axis relative to each other.</param>
	IKSwivelHingeJoint::IKSwivelHingeJoint(IKBone* connectionA, IKBone* connectionB, Vector3 worldHingeAxis, Vector3 worldTwistAxis)
		: IKJoint(connectionA, connectionB)
	{
		SetHingeAxis(worldHingeAxis);
		SetTwistAxis(worldTwistAxis);
	}

	void IKSwivelHingeJoint::UpdateJacobiansAndVelocityBias()
	{
		linearJacobianA = linearJacobianB = Matrix3X3();


		//There are two free axes and one restricted axis.
		//The constraint attempts to keep the hinge axis attached to connection A and the twist axis attached to connection B perpendicular to each other.
		//The restricted axis is the cross product between the twist and hinge axes.

		Vector3 worldTwistAxis, worldHingeAxis;
		Quaternion::Transform(LocalHingeAxis, connectionA->Orientation, worldHingeAxis);
		Quaternion::Transform(LocalTwistAxis, connectionB->Orientation, worldTwistAxis);

		Vector3 restrictedAxis;
		Vector3::Cross(worldHingeAxis, worldTwistAxis, restrictedAxis);
		//Attempt to normalize the restricted axis.
		float lengthSquared = restrictedAxis.LengthSquared();
		if (lengthSquared > Toolbox::Epsilon)
		{
			Vector3::Divide(restrictedAxis, sqrt(lengthSquared), restrictedAxis);
		}
		else
		{
			restrictedAxis = Vector3();
		}


		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = restrictedAxis.X;
		angularJacobianA.M12 = restrictedAxis.Y;
		angularJacobianA.M13 = restrictedAxis.Z;
		Matrix3X3::Negate(angularJacobianA, angularJacobianB);

		Vector3::Dot(worldHingeAxis, worldTwistAxis, error);
		error = acos(MathHelper::Clamp(error, -1, 1)) - MathHelper::PiOver2;

		velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);


	}
	
	bool IKSwivelHingeJoint::HasError()
	{
		return !(std::abs(error) < 0.0001f);
	}
}
