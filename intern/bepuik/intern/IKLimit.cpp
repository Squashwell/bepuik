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

#include "IKLimit.hpp"
#include "Toolbox.hpp"
#include <cmath>
using namespace BEPUmath;

namespace BEPUik
{
	IKLimit::IKLimit(IKBone* connectionA, IKBone* connectionB) : IKJoint(connectionA, connectionB)
	{
	}

	void IKLimit::SolveVelocityIteration()
	{
		//Compute the 'relative' linear and angular velocities. For single bone constraints, it's based entirely on the one bone's velocities!
		//They have to be pulled into constraint space first to compute the necessary impulse, though.
		Vector3 linearContributionA;
		Matrix3X3::TransformTranspose(connectionA->linearVelocity, linearJacobianA, linearContributionA);
		Vector3 angularContributionA;
		Matrix3X3::TransformTranspose(connectionA->angularVelocity, angularJacobianA, angularContributionA);
		Vector3 linearContributionB;
		Matrix3X3::TransformTranspose(connectionB->linearVelocity, linearJacobianB, linearContributionB);
		Vector3 angularContributionB;
		Matrix3X3::TransformTranspose(connectionB->angularVelocity, angularJacobianB, angularContributionB);

		//The constraint velocity error will be the velocity we try to remove.
		Vector3 constraintVelocityError;
		Vector3::Add(linearContributionA, angularContributionA, constraintVelocityError);
		Vector3::Add(constraintVelocityError, linearContributionB, constraintVelocityError);
		Vector3::Add(constraintVelocityError, angularContributionB, constraintVelocityError);
		//However, we need to take into account two extra sources of velocities which modify our target velocity away from zero.
		//First, the velocity bias from position correction:
		Vector3::Subtract(constraintVelocityError, velocityBias, constraintVelocityError);
		//And second, the bias from softness:
		Vector3 softnessBias;
		Vector3::Multiply(accumulatedImpulse, -softness, softnessBias);
		Vector3::Subtract(constraintVelocityError, softnessBias, constraintVelocityError);

		//By now, the constraint velocity error contains all the velocity we want to get rid of.
		//Convert it into an impulse using the effective mass matrix.
		Vector3 constraintSpaceImpulse;
		Matrix3X3::Transform(constraintVelocityError, effectiveMass, constraintSpaceImpulse);

		Vector3::Negate(constraintSpaceImpulse, constraintSpaceImpulse);

		//Add the constraint space impulse to the accumulated impulse so that warm starting and softness work properly.
		Vector3 preadd = accumulatedImpulse;
		Vector3::Add(constraintSpaceImpulse, accumulatedImpulse, accumulatedImpulse);
		//Limits can only apply positive impulses.
		Vector3::Max(Toolbox::ZeroVector, accumulatedImpulse, accumulatedImpulse);
		//But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
		float impulseSquared = accumulatedImpulse.LengthSquared();
		if (impulseSquared > maximumImpulseSquared)
		{
			//Oops! Clamp that down.
			Vector3::Multiply(accumulatedImpulse, maximumImpulse / sqrt(impulseSquared), accumulatedImpulse);
		}
		//Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
		Vector3::Subtract(accumulatedImpulse, preadd, constraintSpaceImpulse);

		//The constraint space impulse now represents the impulse we want to apply to the bone... but in constraint space.
		//Bring it out to world space using the transposed jacobian.
		if (!connectionA->Pinned)//Treat pinned elements as if they have infinite inertia.
		{
			Vector3 linearImpulseA;
			Matrix3X3::Transform(constraintSpaceImpulse, linearJacobianA, linearImpulseA);
			Vector3 angularImpulseA;
			Matrix3X3::Transform(constraintSpaceImpulse, angularJacobianA, angularImpulseA);

			//Apply them!
			connectionA->ApplyLinearImpulse(linearImpulseA);
			connectionA->ApplyAngularImpulse(angularImpulseA);
		}
		if (!connectionB->Pinned)//Treat pinned elements as if they have infinite inertia.
		{
			Vector3 linearImpulseB;
			Matrix3X3::Transform(constraintSpaceImpulse, linearJacobianB, linearImpulseB);
			Vector3 angularImpulseB;
			Matrix3X3::Transform(constraintSpaceImpulse, angularJacobianB, angularImpulseB);

			//Apply them!
			connectionB->ApplyLinearImpulse(linearImpulseB);
			connectionB->ApplyAngularImpulse(angularImpulseB);
		}

	}

}
