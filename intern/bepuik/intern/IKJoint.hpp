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
#include "IKConstraint.hpp"
#include "DNA_constraint_types.h"
namespace BEPUik
{
	/// <summary>
	/// Connects two bones together.
	/// </summary>
	class IKJoint : public IKConstraint
	{
	public:
		/// <summary>
		/// Gets the first bone connected by this joint.
		/// </summary>
		IKBone* GetConnectionA();

		/// <summary>
		/// Gets the second bone connected by this joint.
		/// </summary>
		IKBone* GetConnectionB();

		/// <summary>
		/// Gets whether or not the joint is a member of the active set as determined by the last IK solver execution.
		/// </summary>
		bool IsActive;

		/// <summary>
		/// Gets whether or not this joint is enabled. If set to true, this joint will be a part of
		/// the joint graph and will undergo solving. If set to false, this joint will be removed from the connected bones and will no longer be traversable.
		/// </summary>
		bool GetEnabled();

		/// <summary>
		/// Sets whether or not this joint is enabled. If set to true, this joint will be a part of
		/// the joint graph and will undergo solving. If set to false, this joint will be removed from the connected bones and will no longer be traversable.
		/// </summary>
		void SetEnabled(bool newEnabled);

		void ComputeEffectiveMass();

		void WarmStart();

		void SolveVelocityIteration();

		void ClearAccumulatedImpulses();
		
		virtual bool HasError() = 0;

		IKJoint(IKBone* connectionA, IKBone* connectionB);
		~IKJoint();
		
		int bConstraintType;
	protected:



		bool enabled;

		IKBone* connectionA;

		IKBone* connectionB;

		Vector3 velocityBias;
		Matrix3X3 linearJacobianA;
		Matrix3X3 angularJacobianA;
		Matrix3X3 linearJacobianB;
		Matrix3X3 angularJacobianB;
		Matrix3X3 effectiveMass;

		Vector3 accumulatedImpulse;

	};
}
