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

#include "SingleBoneAngularPlaneConstraint.hpp"

namespace BEPUik
{
	void SingleBoneAngularPlaneConstraint::UpdateJacobiansAndVelocityBias()
	{

		linearJacobian.M11=0;
		linearJacobian.M12=0;
		linearJacobian.M13=0;
		linearJacobian.M21=0;
		linearJacobian.M22=0;
		linearJacobian.M23=0;
		linearJacobian.M31=0;
		linearJacobian.M32=0;
		linearJacobian.M33=0;


		Vector3 boneAxis;
		Quaternion::Transform(BoneLocalAxis,TargetBone->Orientation,boneAxis);

		Vector3 jacobian;
		Vector3::Cross(boneAxis,PlaneNormal,jacobian);

		angularJacobian.M11=jacobian.X;
		angularJacobian.M12=jacobian.Y;
		angularJacobian.M13=jacobian.Z;
		angularJacobian.M21=0;
		angularJacobian.M22=0;
		angularJacobian.M23=0;
		angularJacobian.M31=0;
		angularJacobian.M32=0;
		angularJacobian.M33=0;


		Vector3::Dot(boneAxis,PlaneNormal,velocityBias.X);
		velocityBias.X = -errorCorrectionFactor * velocityBias.X;


	}
}
