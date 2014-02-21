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

#ifdef DEBUG
#include <cstdio>
#include <cfloat>
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
		InertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_DEFAULT)
	{
		linearVelocity = Vector3();
		angularVelocity = Vector3();
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
		InertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_DEFAULT)
	{

		SetMass(mass);
		SetRadius(radius);
		SetLength(length);
		linearVelocity = Vector3();
		angularVelocity = Vector3();
	}

	/// <summary>
	/// Constructs a new bone. Assumes the mass will be set later.
	/// </summary>
	/// <param name="position">Initial position of the bone.</param>
	/// <param name="orientation">Initial orientation of the bone.</param>
	/// <param name="radius">Radius of the bone.</param>
	/// <param name="length">Length of the bone.</param>
	IKBone::IKBone(Vector3 position, Quaternion orientation, float radius, float length) :
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
	    InertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_DEFAULT)
	{
		SetMass(1);
		SetRadius(radius);
		SetLength(length);
		linearVelocity = Vector3();
		angularVelocity = Vector3();


	}


	/// <summary>
	/// Gets the mass of the bone.
	/// High mass bones resist motion more than those of small mass.
	/// Setting the mass updates the inertia tensor of the bone.
	/// </summary>
	float IKBone::GetMass()
	{
		return 1 / InverseMass;
	}

	/// <summary>
	/// Sets the mass of the bone.
	/// High mass bones resist motion more than those of small mass.
	/// Setting the mass updates the inertia tensor of the bone.
	/// </summary>
	void IKBone::SetMass(float newMass)
	{
		if(newMass < FLT_EPSILON)
			newMass = FLT_EPSILON;
		
		InverseMass = 1 / newMass;
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
		ComputeLocalInertiaTensor();

	}

	/// <summary>
	/// Gets the length of the bone.
	/// Setting the length changes the inertia tensor of the bone.
	/// </summary>
	float IKBone::GetLength()
	{
		return halfLength * 2;
	}

	/// <summary>
	/// Sets the length of the bone.
	/// Setting the length changes the inertia tensor of the bone.
	/// </summary>
	void IKBone::SetLength(float newLength)
	{
		halfLength = newLength / 2;
		ComputeLocalInertiaTensor();
	}


	void IKBone::ComputeLocalInertiaTensor()
	{
		Matrix3X3 localInertiaTensor = Matrix3X3();
		float multiplier = GetMass() * InertiaTensorScaling;
		float diagValue = (.0833333333f * GetLength() * GetLength() + .25f * radius * radius) * multiplier;
		localInertiaTensor.M11 = diagValue;
		localInertiaTensor.M22 = .5f * radius * radius * multiplier;
		localInertiaTensor.M33 = diagValue;
		Matrix3X3::Invert(localInertiaTensor, LocalInertiaTensorInverse);
	}

	/// <summary>
	/// Updates the world inertia tensor based upon the local inertia tensor and current orientation.
	/// </summary>
	void IKBone::UpdateInertiaTensor()
	{
#ifdef DEBUG		
		if(Orientation.IsNan())
			printf("BEPUik Orientation is NaN!\n");
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
		if(linearVelocity.IsNan())
			printf("BEPUik Linear velocity is NaN!\n");
		
		if(Position.IsNan())
			printf("BEPUik Position is NaN!\n");
#endif
		
		//Update the position based on the linear velocity.
		Vector3::Add(linearVelocity, Position, Position);

#ifdef DEBUG
		if(Position.IsNan())
			printf("BEPUik Position is NaN!\n");
#endif		
		
		//Update the orientation based on the angular velocity.
		Vector3 increment;
		Vector3::Multiply(angularVelocity, .5f, increment);
		Quaternion multiplier = Quaternion(increment.X, increment.Y, increment.Z, 0);
		
#ifdef DEBUG
		if(multiplier.IsNan())
			printf("BEPUik Update Position multiplier is NaN!\n");
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
		if(linearVelocity.IsNan())
			printf("IKBone::ApplyLinearImpulse linear velocity is Nan!\n");
#endif
		Vector3 velocityChange;
		Vector3::Multiply(impulse, InverseMass, velocityChange);
		Vector3::Add(linearVelocity, velocityChange, linearVelocity);
	}

	void IKBone::ApplyAngularImpulse(Vector3 &impulse)
	{
#ifdef DEBUG
		if(angularVelocity.IsNan())
			printf("IKBone::ApplyAngularImpulse angular velocity is Nan!\n");
#endif
		
		Vector3 velocityChange;
		Matrix3X3::Transform(impulse, InertiaTensorInverse, velocityChange);
		Vector3::Add(velocityChange, angularVelocity, angularVelocity);
	}

	void IKBone::GetHeadPosition(Vector3 &head)
	{
		Matrix worldMatrix = Matrix();
		Matrix::CreateFromQuaternion(Orientation,worldMatrix);

		Vector3 offsetIdentity = Vector3(0,-GetLength()/2,0);
		Vector3 offsetWorld = Vector3();
		Matrix::Transform(offsetIdentity,worldMatrix,offsetWorld);

		head.X = offsetWorld.X + Position.X;
		head.Y = offsetWorld.Y + Position.Y;
		head.Z = offsetWorld.Z + Position.Z;
	}

	void IKBone::GetTailPosition(Vector3 &tail)
	{
		Matrix worldMatrix = Matrix();
		Matrix::CreateFromQuaternion(Orientation,worldMatrix);

		Vector3 offsetIdentity = Vector3(0,GetLength()/2,0);
		Vector3 offsetWorld = Vector3();
		Matrix::Transform(offsetIdentity,worldMatrix,offsetWorld);

		tail.X = offsetWorld.X + Position.X;
		tail.Y = offsetWorld.Y + Position.Y;
		tail.Z = offsetWorld.Z + Position.Z;
	}

	void IKBone::GetHeadMatrix(Matrix &headMatrix)
	{
		Matrix::CreateFromQuaternion(Orientation,headMatrix);

		Vector3 headWorldPosition = Vector3();
		GetHeadPosition(headWorldPosition);

		headMatrix.M41 = headWorldPosition.X;
		headMatrix.M42 = headWorldPosition.Y;
		headMatrix.M43 = headWorldPosition.Z;
	}
    
    void IKBone::GetMatrix(Matrix &matrix)
    {
        Matrix::CreateFromQuaternion(Orientation,matrix);
		matrix.M41 = Position.X;
		matrix.M42 = Position.Y;
		matrix.M43 = Position.Z;        
    }

	void IKBone::GetTailMatrix(Matrix &tailMatrix)
	{
		Matrix::CreateFromQuaternion(Orientation,tailMatrix);

		Vector3 tailWorldPosition = Vector3();
		GetTailPosition(tailWorldPosition);

		tailMatrix.M41 = tailWorldPosition.X;
		tailMatrix.M42 = tailWorldPosition.Y;
		tailMatrix.M43 = tailWorldPosition.Z;
	}

}
