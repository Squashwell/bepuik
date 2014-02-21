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

#include "IKSwingLimit.hpp"
#include <cmath>

namespace BEPUik
{

	/// <summary>
	/// Gets the axis attached to ConnectionA in world space.
	/// </summary>
	Vector3 IKSwingLimit::GetAxisA()
	{
		Vector3 axis;
		Quaternion::Transform(LocalAxisA, connectionA->Orientation, axis);
		return axis;
	}

	/// <summary>
	/// Sets the axis attached to ConnectionA in world space.
	/// </summary>
	void IKSwingLimit::SetAxisA(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalAxisA);
	}

	/// <summary>
	/// Gets the axis attached to connection B in world space.
	/// </summary>
	Vector3 IKSwingLimit::GetAxisB()
	{
		Vector3 axis;
		Quaternion::Transform(LocalAxisB, connectionB->Orientation, axis);
		return axis;
	}

	/// <summary>
	/// Sets the axis attached to connection B in world space.
	/// </summary>
	void IKSwingLimit::SetAxisB(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalAxisB);
	}



	/// <summary>
	/// Gets the maximum angle between the two axes allowed by the constraint.
	/// </summary>
	float IKSwingLimit::GetMaximumAngle()
	{
		return maximumAngle;
	}

	/// <summary>
	/// Sets the maximum angle between the two axes allowed by the constraint.
	/// </summary>
	void IKSwingLimit::SetMaximumAngle(float angle)
	{
		maximumAngle = MathHelper::Max(0, angle);
	}


	/// <summary>
	/// Builds a new swing limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
	/// </summary>
	/// <param name="connectionA">First connection of the limit.</param>
	/// <param name="connectionB">Second connection of the limit.</param>
	/// <param name="axisA">Axis attached to connectionA in world space.</param>
	/// <param name="axisB">Axis attached to connectionB in world space.</param>
	/// <param name="maximumAngle">Maximum angle allowed between connectionA's axis and connectionB's axis.</param>
	IKSwingLimit::IKSwingLimit(IKBone* connectionA, IKBone* connectionB, Vector3 axisA, Vector3 axisB, float maximumAngle)
		: IKLimit(connectionA, connectionB)
	{
		SetAxisA(axisA);
		SetAxisB(axisB);
		SetMaximumAngle(maximumAngle);
	}

	void IKSwingLimit::UpdateJacobiansAndVelocityBias()
	{

		//This constraint doesn't consider linear motion.
		linearJacobianA = linearJacobianB = Matrix3X3();

		//Compute the world axes.
		Vector3 axisA, axisB;
		Quaternion::Transform(LocalAxisA, connectionA->Orientation, axisA);
		Quaternion::Transform(LocalAxisB, connectionB->Orientation, axisB);

		float dot;
		Vector3::Dot(axisA, axisB, dot);

		//Yes, we could avoid this acos here. Performance is not the highest goal of this system; the less tricks used, the easier it is to understand.
		float angle = acos(MathHelper::Clamp(dot, -1, 1));

		//One angular DOF is constrained by this limit.
		Vector3 hingeAxis;
		Vector3::Cross(axisA, axisB, hingeAxis);

		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = hingeAxis.X;
		angularJacobianA.M12 = hingeAxis.Y;
		angularJacobianA.M13 = hingeAxis.Z;
		angularJacobianB = Matrix3X3();
		angularJacobianB.M11 = -hingeAxis.X;
		angularJacobianB.M12 = -hingeAxis.Y;
		angularJacobianB.M13 = -hingeAxis.Z;

		//Note how we've computed the jacobians despite the limit being potentially inactive.
		//This is to enable 'speculative' limits.
		if (angle >= maximumAngle)
		{
			errorAngle = (angle - maximumAngle);
			velocityBias = Vector3(errorCorrectionFactor * errorAngle, 0, 0);
		}
		else
		{
			//The constraint is not yet violated. But, it may be- allow only as much motion as could occur without violating the constraint.
			//Limits can't 'pull,' so this will not result in erroneous sticking.
			errorAngle = 0;
			velocityBias = Vector3(angle - maximumAngle, 0, 0);
		}


	}
	
	bool IKSwingLimit::HasError()
	{
		return !(std::abs(errorAngle) < 0.0001f);
	}	
}
