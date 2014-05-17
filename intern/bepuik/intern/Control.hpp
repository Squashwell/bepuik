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
#include "IKBone.hpp"

namespace BEPUik
{
	/// <summary>
	/// Constrains an individual bone in an attempt to reach some goal.
	/// Controls act as groups of single bone constraints. They are used
	/// by the solver to determine the active set of body constraints.
	/// </summary>
	class Control
	{
	public:
		/// <summary>
		/// Gets the controlled bone.
		/// </summary>
		virtual IKBone* GetTargetBone() = 0;

		/// <summary>
		/// Sets the controlled bone.
		/// </summary>
		virtual void SetTargetBone(IKBone* targetBone) = 0;
		
		virtual void Preupdate(float dt, float updateRate) = 0;

		virtual void UpdateJacobiansAndVelocityBias() = 0;

		virtual void ComputeEffectiveMass() = 0;

		virtual void WarmStart() = 0;

		virtual void SolveVelocityIteration() = 0;

		virtual void ClearAccumulatedImpulses() = 0;

		virtual void SetMaximumForce(float maximumForce) = 0;
		
		virtual ~Control();
	protected:
		Control();

	};
}
