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
#include "IKJoint.hpp"
#include <cmath>
#include <algorithm>

#ifdef DEBUG
#include <assert.h>
#endif

namespace BEPUik
{
	/// <summary>
	/// Gets the first bone connected by this joint.
	/// </summary>
	IKBone* IKJoint::GetConnectionA()
	{
		return connectionA;
	}

	/// <summary>
	/// Gets the second bone connected by this joint.
	/// </summary>
	IKBone* IKJoint::GetConnectionB()
	{
		return connectionB;
	}

	/// <summary>
	/// Gets whether or not this joint is enabled. If set to true, this joint will be a part of
	/// the joint graph and will undergo solving. If set to false, this joint will be removed from the connected bones and will no longer be traversable.
	/// </summary>
	bool IKJoint::GetEnabled()
	{
		return enabled;
	}
	/// <summary>
	/// Sets whether or not this joint is enabled. If set to true, this joint will be a part of
	/// the joint graph and will undergo solving. If set to false, this joint will be removed from the connected bones and will no longer be traversable.
	/// </summary>
	void IKJoint::SetEnabled(bool newEnabled)
	{
		//The bones must know which joints they are associated with so that the bone-joint graph can be traversed.
		if (enabled && !newEnabled)
		{
			connectionA->Joints.erase(find(connectionA->Joints.begin(), connectionA->Joints.end(), this));
			connectionB->Joints.erase(find(connectionB->Joints.begin(), connectionB->Joints.end(), this));
		}
		else if (!enabled && newEnabled)
		{
			connectionA->Joints.push_back(this);
			connectionB->Joints.push_back(this);
		}
		enabled = newEnabled;
	}


	IKJoint::IKJoint(IKBone* connectionA, IKBone* connectionB) :
		connectionA(connectionA),
		connectionB(connectionB),
		IsActive(false),
		enabled(false),
		accumulatedImpulse(Vector3(0,0,0)),
		bConstraintType(0)
	{
		SetEnabled(true);
	}

	IKJoint::~IKJoint()
	{
		//Unlist the joint from the bones so that it won't keep bad pointers around.
		SetEnabled(false);
	}


	void IKJoint::ComputeEffectiveMass()
	{
		//For all constraints, the effective mass matrix is 1 / (J * M^-1 * JT).
		//For two bone constraints, J has 4 3x3 matrices. M^-1 (W below) is a 12x12 matrix with 4 3x3 block diagonal matrices.
		//To compute the whole denominator,
		Matrix3X3 linearW;
		Matrix3X3 linearA, angularA, linearB, angularB;

		if (!connectionA->Pinned)
		{
#ifdef DEBUG
			assert(connectionA->InverseMass==connectionA->InverseMass);
			assert(!linearJacobianA.IsNan());
			assert(!angularJacobianA.IsNan());
			assert(!connectionA->InertiaTensorInverse.IsNan());
#endif

			Matrix3X3::CreateScale(connectionA->InverseMass, linearW);
			Matrix3X3::Multiply(linearJacobianA, linearW, linearA); //Compute J * M^-1 for linear component
			Matrix3X3::MultiplyByTransposed( linearA,  linearJacobianA, linearA); //Compute (J * M^-1) * JT for linear component

			Matrix3X3::Multiply(angularJacobianA, connectionA->InertiaTensorInverse,  angularA); //Compute J * M^-1 for angular component
			Matrix3X3::MultiplyByTransposed(angularA, angularJacobianA, angularA); //Compute (J * M^-1) * JT for angular component
		}
		else
		{
			//Treat pinned bones as if they have infinite inertia.
			linearA = Matrix3X3();
			angularA = Matrix3X3();
		}

		if (!connectionB->Pinned)
		{
#ifdef DEBUG
			assert(connectionB->InverseMass==connectionB->InverseMass);
			assert(!linearJacobianB.IsNan());
			assert(!angularJacobianB.IsNan());
			assert(!connectionB->InertiaTensorInverse.IsNan());
#endif

			Matrix3X3::CreateScale(connectionB->InverseMass, linearW);
			Matrix3X3::Multiply(linearJacobianB, linearW, linearB); //Compute J * M^-1 for linear component
			Matrix3X3::MultiplyByTransposed( linearB,  linearJacobianB,  linearB); //Compute (J * M^-1) * JT for linear component

			Matrix3X3::Multiply(angularJacobianB, connectionB->InertiaTensorInverse,  angularB); //Compute J * M^-1 for angular component
			Matrix3X3::MultiplyByTransposed(angularB, angularJacobianB, angularB); //Compute (J * M^-1) * JT for angular component
		}
		else
		{
			//Treat pinned bones as if they have infinite inertia.
			linearB = Matrix3X3();
			angularB = Matrix3X3();
		}

		//A nice side effect of the block diagonal nature of M^-1 is that the above separated components are now combined into the complete denominator matrix by addition!
		Matrix3X3::Add(linearA, angularA, effectiveMass);
		Matrix3X3::Add(effectiveMass, linearB, effectiveMass);
		Matrix3X3::Add(effectiveMass, angularB, effectiveMass);

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

	void IKJoint::WarmStart()
	{
		//Take the accumulated impulse and transform it into world space impulses using the jacobians by P = JT * lambda
		//(where P is the impulse, JT is the transposed jacobian matrix, and lambda is the accumulated impulse).
		//Recall the jacobian takes impulses from world space into constraint space, and transpose takes them from constraint space into world space.
#ifdef DEBUG
		assert(!accumulatedImpulse.IsNan());
#endif

		Vector3 impulse;
		if (!connectionA->Pinned) //Treat pinned elements as if they have infinite inertia.
		{
#ifdef DEBUG
			assert(!linearJacobianA.IsNan());
			assert(!angularJacobianA.IsNan());
#endif
			//Compute and apply linear impulse for A.
			Matrix3X3::Transform(accumulatedImpulse, linearJacobianA, impulse);
			connectionA->ApplyLinearImpulse( impulse);

			//Compute and apply angular impulse for A.
			Matrix3X3::Transform(accumulatedImpulse, angularJacobianA, impulse);
			connectionA->ApplyAngularImpulse(impulse);
		}


		if (!connectionB->Pinned) //Treat pinned elements as if they have infinite inertia.
		{
#ifdef DEBUG
			assert(!linearJacobianB.IsNan());
			assert(!angularJacobianB.IsNan());
#endif
			//Compute and apply linear impulse for B.
			Matrix3X3::Transform(accumulatedImpulse, linearJacobianB, impulse);
			connectionB->ApplyLinearImpulse( impulse);

			//Compute and apply angular impulse for B.
			Matrix3X3::Transform(accumulatedImpulse, angularJacobianB, impulse);
			connectionB->ApplyAngularImpulse( impulse);
		}
	}

	void IKJoint::SolveVelocityIteration()
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
		//But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
		float impulseSquared = accumulatedImpulse.LengthSquared();
		if (impulseSquared > maximumImpulseSquared)
		{
			//Oops! Clamp that down.
			Vector3::Multiply(accumulatedImpulse, maximumImpulse / sqrt(impulseSquared), accumulatedImpulse);
			//Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
			Vector3::Subtract(accumulatedImpulse, preadd, constraintSpaceImpulse);
		}

		//The constraint space impulse now represents the impulse we want to apply to the bone... but in constraint space.
		//Bring it  to world space using the transposed jacobian.
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

	void IKJoint::ClearAccumulatedImpulses()
	{
		accumulatedImpulse = Vector3();
	}
}
