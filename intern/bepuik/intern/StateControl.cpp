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

#include "Control.hpp"
#include "StateControl.hpp"
#include "SingleBoneAngularMotor.hpp"
#include "SingleBoneLinearMotor.hpp"
#include "float.h"

namespace BEPUik
{

	/// <summary>
	/// Gets the controlled bone.
	/// </summary>
	IKBone* StateControl::GetTargetBone()
	{
		return linearMotor.TargetBone;
	}
	/// <summary>
	/// Sets the controlled bone.
	/// </summary>
	void StateControl::SetTargetBone(IKBone* targetBone)
	{
		linearMotor.TargetBone = targetBone;
		angularMotor.TargetBone = targetBone;
		if (targetBone)
			angularMotor.TargetOrientation = targetBone->Orientation;
	}

	/// <summary>
	/// Gets the linear motor used by the control.
	/// </summary>
	SingleBoneLinearMotor* StateControl::GetLinearMotor()
	{
		return &linearMotor;
	}

	/// <summary>
	/// Gets the angular motor used by the control.
	/// </summary>
	SingleBoneAngularMotor* StateControl::GetAngularMotor()
	{
		return &angularMotor;
	}

	StateControl::StateControl() : linearMotor(SingleBoneLinearMotor()), angularMotor(SingleBoneAngularMotor())
	{
		linearMotor.SetRigidity(1.0f);
		angularMotor.SetRigidity(1.0f);
	}
	
	void StateControl::Preupdate(float dt, float updateRate)
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.Preupdate(dt, updateRate);
		
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.Preupdate(dt, updateRate);
	}

	void StateControl::UpdateJacobiansAndVelocityBias()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.UpdateJacobiansAndVelocityBias();
		
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.UpdateJacobiansAndVelocityBias();
	}

	void StateControl::ComputeEffectiveMass()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.ComputeEffectiveMass();
		
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.ComputeEffectiveMass();
	}

	void StateControl::WarmStart()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON) 
			linearMotor.WarmStart();
		
		if(angularMotor.GetRigidity() >= FLT_EPSILON) 
			angularMotor.WarmStart();
	}

	void StateControl::SolveVelocityIteration()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON) 
			linearMotor.SolveVelocityIteration();
		
		if(angularMotor.GetRigidity() >= FLT_EPSILON)
			angularMotor.SolveVelocityIteration();
	}

	void StateControl::ClearAccumulatedImpulses()
	{
		linearMotor.ClearAccumulatedImpulses();
		angularMotor.ClearAccumulatedImpulses();
	}

	void StateControl::SetMaximumForce(float maximumForce)
	{
		linearMotor.SetMaximumForce(maximumForce);
		angularMotor.SetMaximumForce(maximumForce);
	}

}
