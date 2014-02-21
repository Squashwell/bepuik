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

#include "IKTwistJoint.hpp"
#include "Toolbox.hpp"
#include <cmath>

namespace BEPUik
{

	/// <summary>
	/// Gets the axis attached to ConnectionA in world space.
	/// Must be unit length and perpendicular to measurement axis A.
	/// </summary>
	Vector3 IKTwistJoint::GetAxisA()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalAxisA, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the axis attached to connection A in world space.
	/// Must be unit length and perpendicular to measurement axis A.
	/// </summary>
	void IKTwistJoint::SetAxisA(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalAxisA);
	}

	/// <summary>
	/// Gets the axis attached to connection B in world space.
	/// Must be unit length and perpendicular to measurement axis B.
	/// </summary>
	Vector3 IKTwistJoint::GetAxisB()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalAxisB, connectionB->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the axis attached to connection B in world space.
	/// Must be unit length and perpendicular to measurement axis B.
	/// </summary>
	void IKTwistJoint::SetAxisB(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalAxisB);
	}

	/// <summary>
	/// Gets the measurement axis attached to connection A in world space.
	/// This axis is compared against the other connection's measurement axis to determine the twist.
	/// Must be unit length and perpendicular to axis A.
	/// </summary>
	Vector3 IKTwistJoint::GetMeasurementAxisA()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalMeasurementAxisA, connectionA->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the measurement axis attached to connection A in world space.
	/// This axis is compared against the other connection's measurement axis to determine the twist.
	/// Must be unit length and perpendicular to axis A.
	/// </summary>
	void IKTwistJoint::SetMeasurementAxisA(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionA->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalMeasurementAxisA);
	}


	/// <summary>
	/// Gets the measurement axis attached to connection B in world space.
	/// This axis is compared against the other connection's measurement axis to determine the twist.
	/// Must be unit length and perpendicular to axis B.
	/// </summary>
	Vector3 IKTwistJoint::GetMeasurementAxisB()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalMeasurementAxisB, connectionB->Orientation, toReturn);
		return toReturn;
	}

	/// <summary>
	/// Sets the measurement axis attached to connection B in world space.
	/// This axis is compared against the other connection's measurement axis to determine the twist.
	/// Must be unit length and perpendicular to axis B.
	/// </summary>
	void IKTwistJoint::SetMeasurementAxisB(Vector3 axis)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(connectionB->Orientation, conjugate);
		Quaternion::Transform(axis, conjugate, LocalMeasurementAxisB);
	}




	/// <summary>
	/// Automatically computes the measurement axes for the current local axes.
	/// The current relative state of the entities will be considered 0 twist angle.
	/// </summary>
	void IKTwistJoint::ComputeMeasurementAxes()
	{
		Vector3 axisA, axisB;
		Quaternion::Transform(LocalAxisA, connectionA->Orientation, axisA);
		Quaternion::Transform(LocalAxisB, connectionB->Orientation, axisB);
		//Pick an axis perpendicular to axisA to use as the measurement axis.
		Vector3 worldMeasurementAxisA;
		Vector3::Cross(Toolbox::UpVector, axisA, worldMeasurementAxisA);
		float lengthSquared = worldMeasurementAxisA.LengthSquared();
		if (lengthSquared > Toolbox::Epsilon)
		{
			Vector3::Divide(worldMeasurementAxisA, sqrt(lengthSquared), worldMeasurementAxisA);
		}
		else
		{
			//Oops! It was parallel to the up vector. Just try again with the right vector.
			Vector3::Cross(Toolbox::RightVector, axisA, worldMeasurementAxisA);
			worldMeasurementAxisA.Normalize();
		}
		//Attach the measurement axis to entity B.
		//'Push' A's axis onto B by taking into account the swing transform.
		Quaternion alignmentRotation;
		Toolbox::GetQuaternionBetweenNormalizedVectors(axisA, axisB, alignmentRotation);
		Vector3 worldMeasurementAxisB;
		Quaternion::Transform(worldMeasurementAxisA, alignmentRotation, worldMeasurementAxisB);
		//Plop them on!
		SetMeasurementAxisA(worldMeasurementAxisA);
		SetMeasurementAxisB(worldMeasurementAxisB);

	}


	/// <summary>
	/// Builds a new twist limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
	/// </summary>
	/// <param name="connectionA">First connection of the limit.</param>
	/// <param name="connectionB">Second connection of the limit.</param>
	/// <param name="axisA">Axis attached to connectionA in world space.</param>
	/// <param name="axisB">Axis attached to connectionB in world space.</param>
	IKTwistJoint::IKTwistJoint(IKBone* connectionA, IKBone* connectionB, Vector3 axisA, Vector3 axisB)
		: IKJoint(connectionA, connectionB)
	{
		SetAxisA(axisA);
		SetAxisB(axisB);

		ComputeMeasurementAxes();
	}

	void IKTwistJoint::UpdateJacobiansAndVelocityBias()
	{

		//This constraint doesn't consider linear motion.
		linearJacobianA = linearJacobianB = Matrix3X3();

		//Compute the world axes.
		Vector3 axisA, axisB;
		Quaternion::Transform(LocalAxisA, connectionA->Orientation, axisA);
		Quaternion::Transform(LocalAxisB, connectionB->Orientation, axisB);

		Vector3 twistMeasureAxisA, twistMeasureAxisB;
		Quaternion::Transform(LocalMeasurementAxisA, connectionA->Orientation, twistMeasureAxisA);
		Quaternion::Transform(LocalMeasurementAxisB, connectionB->Orientation, twistMeasureAxisB);

		//Compute the shortest rotation to bring axisB into alignment with axisA.
		Quaternion alignmentRotation;
		Toolbox::GetQuaternionBetweenNormalizedVectors(axisB, axisA, alignmentRotation);

		//Transform the measurement axis on B by the alignment quaternion.
		Quaternion::Transform(twistMeasureAxisB, alignmentRotation, twistMeasureAxisB);

		//We can now compare the angle between the twist axes.
		Vector3::Dot(twistMeasureAxisA, twistMeasureAxisB, error);
		error = acos(MathHelper::Clamp(error, -1, 1));
		Vector3 cross;
		Vector3::Cross(twistMeasureAxisA, twistMeasureAxisB, cross);
		float dot;
		Vector3::Dot(cross, axisA, dot);
		if (dot < 0)
			error = -error;

		//Compute the bias based upon the error.
		velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);

		//We can't just use the axes directly as jacobians. Consider 'cranking' one object around the other.
		Vector3 jacobian;
		Vector3::Add(axisA, axisB, jacobian);
		float lengthSquared = jacobian.LengthSquared();
		if (lengthSquared > Toolbox::Epsilon)
		{
			Vector3::Divide(jacobian, sqrt(lengthSquared), jacobian);
		}
		else
		{
			//The constraint is in an invalid configuration. Just ignore it.
			jacobian = Vector3();
		}

		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = jacobian.X;
		angularJacobianA.M12 = jacobian.Y;
		angularJacobianA.M13 = jacobian.Z;
		angularJacobianB = Matrix3X3();
		angularJacobianB.M11 = -jacobian.X;
		angularJacobianB.M12 = -jacobian.Y;
		angularJacobianB.M13 = -jacobian.Z;

	}
	
	bool IKTwistJoint::HasError()
	{
		return !(std::abs(error) < 0.002f);
	}
}
