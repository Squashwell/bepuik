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

#include "SingleBoneLinearMotor.hpp"

using namespace BEPUmath;

namespace BEPUik
{

	Vector3 SingleBoneLinearMotor::GetOffset()
	{
		Vector3 toReturn;
		Quaternion::Transform(LocalOffset, TargetBone->Orientation, toReturn);
		return toReturn;
	}
	void SingleBoneLinearMotor::SetOffset(Vector3 worldOffset)
	{
		Quaternion conjugate;
		Quaternion::Conjugate(TargetBone->Orientation, conjugate);
		Quaternion::Transform(worldOffset, conjugate, LocalOffset);
	}

	void SingleBoneLinearMotor::UpdateJacobiansAndVelocityBias()
	{
		Matrix3X3::GetIdentity(linearJacobian);
		Vector3 r;
		Quaternion::Transform(LocalOffset, TargetBone->Orientation, r);
		Matrix3X3::CreateCrossProduct(r, angularJacobian);
		//Transposing a skew symmetric matrix is equivalent to negating it.
		Matrix3X3::Transpose(angularJacobian, angularJacobian);

		Vector3 worldPosition;
		Vector3::Add(TargetBone->Position, r, worldPosition);

		//Error is in world space.
		Vector3 linearError;
		Vector3::Subtract(TargetPosition, worldPosition, linearError);
		//This is equivalent to projecting the error onto the linear jacobian. The linear jacobian just happens to be the identity matrix!
		Vector3::Multiply(linearError, errorCorrectionFactor, velocityBias);
	}

}
