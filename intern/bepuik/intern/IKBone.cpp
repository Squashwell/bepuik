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

#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"
#include "IKBone.hpp"
#include "bepu.h"
#include <float.h>
#ifdef DEBUG
#include <cstdio>
#include <assert.h>
#endif

namespace BEPUik
{
	IKBone::IKBone() :
		Position(Vector3()),
		Orientation(Quaternion(0,0,0,1)),
		Joints(vector<IKJoint*>()),
		predecessors(vector<IKBone*>()),
		Pinned(false),
		IsActive(false),
		traversed(false),
		unstressedCycle(false),
		targetedByOtherControl(false),
		stressCount(0),
		pchan(0),

		setMassCalled(false),
		setLengthCalled(false),
		setRadiusCalled(false),
		setInertiaTensorScalingCalled(false),

		inertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN),
		radius(.2f),
		halfLength(.5f),
		InverseMass(1.0f),

		linearVelocity(Vector3()),
		angularVelocity(Vector3())
	{
	}

	/// <summary>
	/// Constructs a new bone.
	/// </summary>
	/// <param name="position">Initial position of the bone.</param>
	/// <param name="orientation">Initial orientation of the bone.</param>
	/// <param name="radius">Radius of the bone.</param>
	/// <param name="length">Length of the bone.</param>
	/// <param name="mass">Mass of the bone.</param>
	IKBone::IKBone(Vector3 position, Quaternion orientation, float radius, float length, float mass) :
		Position(position),
		Orientation(orientation),
		Joints(vector<IKJoint*>()),
		predecessors(vector<IKBone*>()),
		Pinned(false),
		IsActive(false),
		traversed(false),
		unstressedCycle(false),
		targetedByOtherControl(false),
		stressCount(0),
		pchan(0),

		setMassCalled(false),
		setLengthCalled(false),
		setRadiusCalled(false),
		setInertiaTensorScalingCalled(false),

		inertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN),
		radius(.2f),
		halfLength(.5f),
		InverseMass(1.0f),

		linearVelocity(Vector3()),
		angularVelocity(Vector3())
	{
		SetMass(mass);
		SetRadius(radius);
		SetLength(length);
	}

	/// <summary>
	/// Gets the mass of the bone.
	/// High mass bones resist motion more than those of small mass.
	/// Setting the mass updates the inertia tensor of the bone.
	/// </summary>
	float IKBone::GetMass()
	{
#ifdef DEBUG
		assert(InverseMass == InverseMass);
		assert(InverseMass!=0.0f);
		assert(setMassCalled);
#endif
		return 1 / InverseMass;
	}

	/// <summary>
	/// Sets the mass of the bone.
	/// High mass bones resist motion more than those of small mass.
	/// Setting the mass updates the inertia tensor of the bone.
	/// </summary>
	void IKBone::SetMass(float newMass)
	{
#ifdef DEBUG
		assert(newMass==newMass);
#endif

		if(newMass < FLT_EPSILON)
			newMass = FLT_EPSILON;
		
		InverseMass = 1 / newMass;
#ifdef DEBUG
		assert(InverseMass==InverseMass);
		assert(InverseMass!=0.0f);
#endif
		setMassCalled = true;
		ComputeLocalInertiaTensor();
	}


	/// <summary>
	/// Gets or sets the radius of the bone.
	/// Setting the radius changes the inertia tensor of the bone.
	/// </summary>
	float IKBone::GetRadius()
	{
		return radius;
	}

	/// <summary>
	/// Gets or sets the radius of the bone.
	/// Setting the radius changes the inertia tensor of the bone.
	/// </summary>
	void IKBone::SetRadius(float newRadius)
	{
		radius = newRadius;
#ifdef DEBUG
		assert(radius==radius);
		assert(radius >= FLT_EPSILON);
#endif
		setRadiusCalled = true;
		ComputeLocalInertiaTensor();

	}

	/// <summary>
	/// Gets the length of the bone.
	/// Setting the length changes the inertia tensor of the bone.
	/// </summary>
	float IKBone::GetLength()
	{
#ifdef DEBUG
		assert(halfLength == halfLength);
		assert(halfLength != 0.0f);
#endif
		return halfLength * 2;
	}

	/// <summary>
	/// Sets the length of the bone.
	/// Setting the length changes the inertia tensor of the bone.
	/// </summary>
	void IKBone::SetLength(float newLength)
	{
		halfLength = newLength / 2;
		setLengthCalled = true;
		ComputeLocalInertiaTensor();
	}

	void IKBone::SetInertiaTensorScaling(float newInertiaTensorScaling)
	{
#ifdef DEBUG
		assert(newInertiaTensorScaling!=0.0f);
		assert(newInertiaTensorScaling==newInertiaTensorScaling);
#endif
		inertiaTensorScaling = newInertiaTensorScaling;
		setInertiaTensorScalingCalled = true;
		ComputeLocalInertiaTensor();
	}


	void IKBone::ComputeLocalInertiaTensor()
	{
		if(!setLengthCalled || !setMassCalled || !setRadiusCalled || !setInertiaTensorScalingCalled) return;

		Matrix3X3 localInertiaTensor = Matrix3X3();
#ifdef DEBUG
		assert(inertiaTensorScaling == inertiaTensorScaling);
		assert(inertiaTensorScaling != 0.0f);
		assert(radius==radius);
		assert(radius!=0.0f);
#endif
		float multiplier = GetMass() * inertiaTensorScaling;
#ifdef DEBUG
		assert(multiplier==multiplier);
#endif
		float diagValue = (.0833333333f * GetLength() * GetLength() + .25f * radius * radius) * multiplier;
		localInertiaTensor.M11 = diagValue;
		localInertiaTensor.M22 = .5f * radius * radius * multiplier;
		localInertiaTensor.M33 = diagValue;
#ifdef DEBUG
		assert(!localInertiaTensor.IsNan());
#endif
		Matrix3X3::Invert(localInertiaTensor, LocalInertiaTensorInverse);
#ifdef DEBUG
		assert(!LocalInertiaTensorInverse.IsNan());
#endif
	}

	/// <summary>
	/// Updates the world inertia tensor based upon the local inertia tensor and current orientation.
	/// </summary>
	void IKBone::UpdateInertiaTensor()
	{
#ifdef DEBUG		
		assert(!Orientation.IsNan());
		assert(!LocalInertiaTensorInverse.IsNan());
#endif
		
		//This is separate from the position update because the orientation can change outside of our iteration loop, so this has to run first.
		//Iworld^-1 = RT * Ilocal^1 * R
		Matrix3X3 orientationMatrix;
		Matrix3X3::CreateFromQuaternion(Orientation, orientationMatrix);
		Matrix3X3::MultiplyTransposed(orientationMatrix, LocalInertiaTensorInverse, InertiaTensorInverse);
		Matrix3X3::Multiply(InertiaTensorInverse, orientationMatrix, InertiaTensorInverse);
	}

	/// <summary>
	/// Integrates the position and orientation of the bone forward based upon the current linear and angular velocity.
	/// </summary>
	void IKBone::UpdatePosition()
	{
#ifdef DEBUG
		assert(!linearVelocity.IsNan());
		assert(!Position.IsNan());
		assert(!angularVelocity.IsNan());
#endif
		
		//Update the position based on the linear velocity.
		Vector3::Add(linearVelocity, Position, Position);

#ifdef DEBUG
		assert(!Position.IsNan());
#endif		
		
		//Update the orientation based on the angular velocity.
		Vector3 increment;
		Vector3::Multiply(angularVelocity, .5f, increment);
		Quaternion multiplier = Quaternion(increment.X, increment.Y, increment.Z, 0);
		
#ifdef DEBUG
		assert(!multiplier.IsNan());
#endif
		
		Quaternion::Multiply(multiplier, Orientation, multiplier);
		Quaternion::Add(Orientation, multiplier, Orientation);
		Orientation.Normalize();

		//Eliminate any latent velocity in the bone to prevent unwanted simulation feedback.
		//This is the only thing conceptually separating this "IK" solver from the regular dynamics loop in BEPUphysics.
		//(Well, that and the whole lack of collision detection...)
		linearVelocity = Vector3();
		angularVelocity = Vector3();
	}

	void IKBone::ApplyLinearImpulse(Vector3 &impulse)
	{
#ifdef DEBUG
		assert(!impulse.IsNan());
		assert(!linearVelocity.IsNan());
		assert(InverseMass==InverseMass);
		assert(InverseMass!=0.0f);
#endif
		Vector3 velocityChange;
		Vector3::Multiply(impulse, InverseMass, velocityChange);
		Vector3::Add(linearVelocity, velocityChange, linearVelocity);
	}

	void IKBone::ApplyAngularImpulse(Vector3 &impulse)
	{
#ifdef DEBUG
		assert(!impulse.IsNan());
		assert(!InertiaTensorInverse.IsNan());
		assert(!angularVelocity.IsNan());

#endif
		
		Vector3 velocityChange;
		Matrix3X3::Transform(impulse, InertiaTensorInverse, velocityChange);
		Vector3::Add(velocityChange, angularVelocity, angularVelocity);
	}
}
