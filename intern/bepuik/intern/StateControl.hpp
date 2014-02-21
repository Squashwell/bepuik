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

#pragma once
#include "Control.hpp"
#include "SingleBoneAngularMotor.hpp"
#include "SingleBoneLinearMotor.hpp"

namespace BEPUik
{
	/// <summary>
	/// Constrains an individual bone in an attempt to reach some position and orientation goal.
	/// </summary>
	class StateControl : public Control
	{
	public:
		/// <summary>
		/// Gets the controlled bone.
		/// </summary>
		IKBone* GetTargetBone();
		/// <summary>
		/// Sets the controlled bone.
		/// </summary>
		void SetTargetBone(IKBone* targetBone);

		/// <summary>
		/// Gets the linear motor used by the control.
		/// </summary>
		SingleBoneLinearMotor* GetLinearMotor();

		/// <summary>
		/// Gets the angular motor used by the control.
		/// </summary>
		SingleBoneAngularMotor* GetAngularMotor();

		StateControl();

		void UpdateJacobiansAndVelocityBias();

		void ComputeEffectiveMass();

		void WarmStart();

		void SolveVelocityIteration();

		void ClearAccumulatedImpulses();

		void SetMaximumForce(float maximumLinearForce, float maximumAngularForce);

		void Preupdate(float dt, float updateRate);
		
		float GetRigidityMassMultiplier();
	private:
		SingleBoneLinearMotor linearMotor;
		SingleBoneAngularMotor angularMotor;
	};
}
