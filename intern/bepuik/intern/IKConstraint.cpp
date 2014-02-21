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

#include "MathHelper.hpp"
#include "IKConstraint.hpp"
#include "float.h"

using namespace BEPUmath;

namespace BEPUik
{
	/// <summary>
	/// Sets the rigidity of the constraint.
	/// </summary>
	void IKConstraint::SetRigidity(float newRigidity)
	{
		rigidity = MathHelper::Max(0,newRigidity);
	}
	
	/// <summary>
	/// Sets the rigidity of the constraint.
	/// </summary>
	float IKConstraint::GetRigidity()
	{
		return rigidity;
	}

	/// <summary>
	/// Get the maximum force that the constraint can apply.
	/// </summary>
	float IKConstraint::GetMaximumForce()
	{
		return maximumForce;
	}

	/// <summary>
	/// Sets the maximum force that the constraint can apply.
	/// </summary>
	void IKConstraint::SetMaximumForce(float newMaximumForce)
	{
		maximumForce = MathHelper::Max(newMaximumForce, 0);
	}
	
	/// <summary>
	/// Updates the softness, bias factor, and maximum impulse based on the current time step.
	/// </summary>
	/// <param name="dt">Time step duration.</param>
	void IKConstraint::Preupdate(float dt, float updateRate)
	{
//		if (stiffnessConstant == 0 && dampingConstant == 0)
//			printf("ERROR: Constraints cannot have both 0 stiffness and 0 damping.\n");
		
		float stiffness = StiffnessOverDamping * rigidity;
        float damping = rigidity;
        float multiplier = 1 / (dt * stiffness + damping);
        errorCorrectionFactor = stiffness * multiplier;
        softness = updateRate * multiplier;
        maximumImpulse = maximumForce * dt;
        maximumImpulseSquared = MathHelper::Min(FLT_MAX, maximumImpulse * maximumImpulse);

	}

	IKConstraint::IKConstraint() : maximumForce(FLT_MAX), rigidity(16.0f), StiffnessOverDamping(0.25f)
	{
	}

	IKConstraint::~IKConstraint()
	{
	}
}
