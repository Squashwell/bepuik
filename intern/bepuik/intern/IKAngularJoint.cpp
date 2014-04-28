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

#include "IKAngularJoint.hpp"

namespace BEPUik
{
	/// <summary>
	/// Constructs a 3DOF angular joint which tries to keep two bones in angular alignment.
	/// </summary>
	/// <param name="connectionA">First bone to connect to the joint.</param>
	/// <param name="connectionB">Second bone to connect to the joint.</param>
	IKAngularJoint::IKAngularJoint(IKBone* connectionA, IKBone* connectionB)
		: IKJoint(connectionA, connectionB)
	{
		Quaternion::RelativeOrientation(connectionA->Orientation,connectionB->Orientation, GoalRelativeOrientation);
	}
    
    IKAngularJoint::IKAngularJoint(IKBone*connectionA, IKBone*connectionB, Quaternion relativeOrientation)
        : IKJoint(connectionA,connectionB)
    {
        GoalRelativeOrientation = relativeOrientation;
    }

	void IKAngularJoint::UpdateJacobiansAndVelocityBias()
	{
		linearJacobianA = linearJacobianB = Matrix3X3();

		angularJacobianA = Matrix3X3();
		angularJacobianA.M11 = 1;
		angularJacobianA.M22 = 1;
		angularJacobianA.M33 = 1;

		angularJacobianB = Matrix3X3();
		angularJacobianB.M11 = -1;
		angularJacobianB.M22 = -1;
		angularJacobianB.M33 = -1;

		//The error is computed using this equation:
		//GoalRelativeOrientation * ConnectionA.Orientation * Error = ConnectionB.Orientation
		//GoalRelativeOrientation is the original rotation from A to B in A's local space.
		//Multiplying by A's orientation gives us where B *should* be.
		//Of course, B won't be exactly where it should be after initialization.
		//The Error component holds the difference between what is and what should be.
		//Error = (GoalRelativeOrientation * ConnectionA.Orientation)^-1 * ConnectionB.Orientation
		Quaternion bTarget;
		Quaternion::Concatenate(GoalRelativeOrientation,connectionA->Orientation,bTarget);
		Quaternion bTargetConjugate;
		Quaternion::Conjugate(bTarget,bTargetConjugate);
		
		Quaternion::Concatenate(bTargetConjugate,connectionB->Orientation,error);

		//Convert the error into an axis-angle vector usable for bias velocity.
		float angle;
		Vector3 axis;
		Toolbox::GetAxisAngleFromQuaternion(error,axis,angle);

		velocityBias.X = errorCorrectionFactor * axis.X * angle;
		velocityBias.Y = errorCorrectionFactor * axis.Y * angle;
		velocityBias.Z = errorCorrectionFactor * axis.Z * angle;
	}
	
	bool IKAngularJoint::HasError()
	{
		return !error.IsUnit(0.0001f);
	}



}
