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
#include "MathHelper.hpp"


namespace BEPUik
{

	class IKConstraint
	{

	public:


		/// <summary>
		/// Gets the rigidity of the constraint.
		/// </summary>
		float GetRigidity();

		/// <summary>
		/// Sets the rigidity of the constraint.
		/// </summary>
		void SetRigidity(float newRigidity);

		/// <summary>
		/// Gets the error correction factor of the constraint. Values range from 0 to 1. 0 means the constraint will not attempt to correct any error.
		/// 1 means the constraint will attempt to correct all error in a single iteration. This factor, combined with Softness, define the springlike behavior of a constraint.
		/// </summary>
		float GetStiffness();

		/// <summary>
		/// Sets the error correction factor of the constraint. Values range from 0 to 1. 0 means the constraint will not attempt to correct any error.
		/// 1 means the constraint will attempt to correct all error in a single iteration. This factor, combined with Softness, define the springlike behavior of a constraint.
		/// </summary>
		void SetStiffness(float newStiffnessConstant);

		/// <summary>
		/// Gets the maximum impulse that the constraint can apply.
		/// Velocity error requiring a greater impulse will result in the impulse being clamped to the maximum impulse.
		/// </summary>
		float GetMaximumForce();

		/// <summary>
		/// Sets the maximum impulse that the constraint can apply.
		/// Velocity error requiring a greater impulse will result in the impulse being clamped to the maximum impulse.
		/// </summary>
		void SetMaximumForce(float newMaximumImpulse);

		/// <summary>
		/// Update the jacobians for the latest position and orientation bone states and store a velocity bias based on the error.
		/// </summary>
		virtual void UpdateJacobiansAndVelocityBias() = 0;

		/// <summary>
		/// Computes the effective mass matrix for the constraint for the current jacobians.
		/// </summary>
		virtual void ComputeEffectiveMass() = 0;

		/// <summary>
		/// Applies the accumulated impulse to warm up the constraint solving process.
		/// </summary>
		virtual void WarmStart() = 0;

		/// <summary>
		/// Applies impulses to satisfy the velocity constraint.
		/// </summary>
		virtual void SolveVelocityIteration() = 0;

		/// <summary>
		/// Clears out the accumulated impulse.
		/// </summary>
		virtual void ClearAccumulatedImpulses() = 0;
		
		/// Updates the softness, bias factor, and maximum impulse based on the current time step.
		/// </summary>
		/// <param name="dt">Time step duration.</param>
		void Preupdate(float dt, float updateRate);

		//Make all constraints uncopyable. This prevents graph traversal annoyances due to bone joint listings.
		IKConstraint(const IKConstraint&);
		virtual ~IKConstraint();
		IKConstraint& operator=(const IKConstraint&);

	protected:
		float errorCorrectionFactor;
		float softness;

		float maximumImpulse;
		float maximumImpulseSquared;
		float maximumForce;
		IKConstraint();
		
	private:
		float rigidity;
		const float StiffnessOverDamping;
	};
}
