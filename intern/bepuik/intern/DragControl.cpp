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

#include "DragControl.hpp"
#include "float.h"

namespace BEPUik
{
	/// <summary>
	/// Gets the controlled bone.
	/// </summary>
	IKBone* DragControl::GetTargetBone()
	{
		return linearMotor.TargetBone;
	}
	/// <summary>
	/// Sets the controlled bone.
	/// </summary>
	void DragControl::SetTargetBone(IKBone* targetBone)
	{
		linearMotor.TargetBone = targetBone;
	}

	/// <summary>
	/// Gets or sets the linear motor used by the control.
	/// </summary>
	SingleBoneLinearMotor* DragControl::GetLinearMotor()
	{
		return &linearMotor;
	}

	DragControl::DragControl() : linearMotor(SingleBoneLinearMotor())
	{
		linearMotor.SetRigidity(1.0f);
	}
	
	void DragControl::Preupdate(float dt, float updateRate)
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.Preupdate(dt, updateRate);
	}

	void DragControl::UpdateJacobiansAndVelocityBias()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.UpdateJacobiansAndVelocityBias();
	}

	void DragControl::ComputeEffectiveMass()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.ComputeEffectiveMass();
	}

	void DragControl::WarmStart()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.WarmStart();
	}

	void DragControl::SolveVelocityIteration()
	{
		if(linearMotor.GetRigidity() >= FLT_EPSILON)
			linearMotor.SolveVelocityIteration();
	}

	void DragControl::ClearAccumulatedImpulses()
	{
		linearMotor.ClearAccumulatedImpulses();
	}

	float DragControl::GetMaximumForce()
	{
		return linearMotor.GetMaximumForce();
	}

	void DragControl::SetMaximumForce(float maximumForce)
	{
		linearMotor.SetMaximumForce(maximumForce);
	}
}
