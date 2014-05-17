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



#include "AngularPlaneControl.hpp"
#include "SingleBoneAngularMotor.hpp"
#include "SingleBoneLinearMotor.hpp"
#include "SingleBoneAngularPlaneConstraint.hpp"
#include "float.h"

namespace BEPUik
{

	/// <summary>
	/// Gets the controlled bone.
	/// </summary>
	IKBone* AngularPlaneControl::GetTargetBone()
	{
		return angularMotor.TargetBone;
	}
	/// <summary>
	/// Sets the controlled bone.
	/// </summary>
	void AngularPlaneControl::SetTargetBone(IKBone* targetBone)
	{
		angularMotor.TargetBone = targetBone;
	}

	/// <summary>
	/// Gets the angular motor used by the control.
	/// </summary>
	SingleBoneAngularPlaneConstraint* AngularPlaneControl::GetAngularMotor()
	{
		return &angularMotor;
	}

	AngularPlaneControl::AngularPlaneControl() : angularMotor(SingleBoneAngularPlaneConstraint())
	{
		angularMotor.SetRigidity(1.0f);
	}

	void AngularPlaneControl::Preupdate(float dt, float updateRate)
	{
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.Preupdate(dt, updateRate);
	}

	void AngularPlaneControl::UpdateJacobiansAndVelocityBias()
	{
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.UpdateJacobiansAndVelocityBias();
	}

	void AngularPlaneControl::ComputeEffectiveMass()
	{

		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.ComputeEffectiveMass();
	}

	void AngularPlaneControl::WarmStart()
	{

		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.WarmStart();
	}

	void AngularPlaneControl::SolveVelocityIteration()
	{

		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.SolveVelocityIteration();
	}

	void AngularPlaneControl::ClearAccumulatedImpulses()
	{
		angularMotor.ClearAccumulatedImpulses();
	}

	void AngularPlaneControl::SetMaximumForce(float maximumForce)
	{
		angularMotor.SetMaximumForce(maximumForce);
	}
}
