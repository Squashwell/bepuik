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

#include "IKBone.hpp"
#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "IKConstraint.hpp"
#include "SingleBoneConstraint.hpp"
#include <cmath>

#ifdef DEBUG
#include <stdio.h>
#include <assert.h>
#endif

namespace BEPUik
{
	SingleBoneConstraint::SingleBoneConstraint():
		accumulatedImpulse(Vector3(0,0,0)),
		TargetBone(NULL)
	{
	}

	void SingleBoneConstraint::ComputeEffectiveMass()
	{
		//For all constraints, the effective mass matrix is 1 / (J * M^-1 * JT).
		//For single bone constraints, J has 2 3x3 matrices. M^-1 (W below) is a 6x6 matrix with 2 3x3 block diagonal matrices.
		//To compute the whole denominator,
		Matrix3X3 linearW;
		Matrix3X3::CreateScale(TargetBone->InverseMass, linearW);
		Matrix3X3 linear;
		Matrix3X3::Multiply(linearJacobian, linearW, linear); //Compute J * M^-1 for linear component
		Matrix3X3::MultiplyByTransposed(linear, linearJacobian, linear); //Compute (J * M^-1) * JT for linear component

		Matrix3X3 angular;
		Matrix3X3::Multiply(angularJacobian, TargetBone->InertiaTensorInverse, angular); //Compute J * M^-1 for angular component
		Matrix3X3::MultiplyByTransposed(angular, angularJacobian, angular); //Compute (J * M^-1) * JT for angular component

		//A nice side effect of the block diagonal nature of M^-1 is that the above separated components are now combined into the complete denominator matrix by addition!
		Matrix3X3::Add(linear, angular, effectiveMass);

		//Incorporate the constraint softness into the effective mass denominator. This pushes the matrix away from singularity.
		//Softness will also be incorporated into the velocity solve iterations to complete the implementation.
		if (effectiveMass.M11 != 0)
			effectiveMass.M11 += softness;
		if (effectiveMass.M22 != 0)
			effectiveMass.M22 += softness;
		if (effectiveMass.M33 != 0)
			effectiveMass.M33 += softness;

		//Invert! Takes us from J * M^-1 * JT to 1 / (J * M^-1 * JT).
		Matrix3X3::AdaptiveInvert(effectiveMass, effectiveMass);
	}

	void SingleBoneConstraint::WarmStart()
	{
		//Take the accumulated impulse and transform it into world space impulses using the jacobians by P = JT * lambda
		//(where P is the impulse, JT is the transposed jacobian matrix, and lambda is the accumulated impulse).
		//Recall the jacobian takes impulses from world space into constraint space, and transpose takes them from constraint space into world space.
		//Compute and apply linear impulse.
		Vector3 impulse;
		Matrix3X3::Transform(accumulatedImpulse, linearJacobian, impulse);
		TargetBone->ApplyLinearImpulse(impulse);

		//Compute and apply angular impulse.
		Matrix3X3::Transform(accumulatedImpulse, angularJacobian, impulse);
		TargetBone->ApplyAngularImpulse(impulse);
	}

	void SingleBoneConstraint::SolveVelocityIteration()
	{
#ifdef DEBUG
		assert(!linearJacobian.IsNan());
		assert(!angularJacobian.IsNan());
#endif

		//Compute the 'relative' linear and angular velocities. For single bone constraints, it's based entirely on the one bone's velocities!
		//They have to be pulled into constraint space first to compute the necessary impulse, though.
		Vector3 linearContribution;
		Matrix3X3::TransformTranspose(TargetBone->linearVelocity, linearJacobian, linearContribution);
		Vector3 angularContribution;
		Matrix3X3::TransformTranspose(TargetBone->angularVelocity, angularJacobian, angularContribution);

		//The constraint velocity error will be the velocity we try to remove.
		Vector3 constraintVelocityError;
		Vector3::Add(linearContribution, angularContribution, constraintVelocityError);
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
		//But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
		float impulseSquared = accumulatedImpulse.LengthSquared();
		if (impulseSquared > maximumImpulseSquared)
		{
			//Oops! Clamp that down.
			Vector3::Multiply(accumulatedImpulse, maximumImpulse / sqrt(impulseSquared), accumulatedImpulse);
			//Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
			Vector3::Subtract(accumulatedImpulse, preadd, constraintSpaceImpulse);
		}

#ifdef DEBUG
		assert(!constraintSpaceImpulse.IsNan());
		assert(!linearJacobian.IsNan());
		assert(!angularJacobian.IsNan());
#endif

		//The constraint space impulse now represents the impulse we want to apply to the bone... but in constraint space.
		//Bring it out to world space using the transposed jacobian.
		Vector3 linearImpulse;
		Matrix3X3::Transform(constraintSpaceImpulse, linearJacobian, linearImpulse);
		Vector3 angularImpulse;
		Matrix3X3::Transform(constraintSpaceImpulse, angularJacobian, angularImpulse);

#ifdef DEBUG
		assert(!linearImpulse.IsNan());
		assert(!angularImpulse.IsNan());
#endif

		//Apply them!
		TargetBone->ApplyLinearImpulse(linearImpulse);
		TargetBone->ApplyAngularImpulse(angularImpulse);
	}

	void SingleBoneConstraint::ClearAccumulatedImpulses()
	{
		accumulatedImpulse = Vector3();
	}


}
