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
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"
#include <vector>
#include "DNA_action_types.h"

using namespace BEPUmath;
using namespace std;

namespace BEPUik
{
	class IKJoint;
	/// <summary>
	/// Piece of a character which is moved by constraints.
	/// </summary>
	class IKBone
	{
	public:
		IKBone();
		/// <summary>
		/// Constructs a new bone.
		/// </summary>
		/// <param name="position">Initial position of the bone.</param>
		/// <param name="orientation">Initial orientation of the bone.</param>
		/// <param name="radius">Radius of the bone.</param>
		/// <param name="length">Length of the bone.</param>
		/// <param name="mass">Mass of the bone.</param>
		IKBone(Vector3 position, Quaternion orientation, float radius, float length, float mass);

		/// <summary>
		/// Constructs a new bone. Assumes the mass will be set later.
		/// </summary>
		/// <param name="position">Initial position of the bone.</param>
		/// <param name="orientation">Initial orientation of the bone.</param>
		/// <param name="radius">Radius of the bone.</param>
		/// <param name="length">Length of the bone.</param>
		IKBone(Vector3 position, Quaternion orientation, float radius, float length);

		/// <summary>
		/// Gets the mass of the bone.
		/// High mass bones resist motion more than those of small mass.
		/// Setting the mass updates the inertia tensor of the bone.
		/// </summary>
		float GetMass();

		/// <summary>
		/// Sets the mass of the bone.
		/// High mass bones resist motion more than those of small mass.
		/// Setting the mass updates the inertia tensor of the bone.
		/// </summary>
		void SetMass(float newMass);


		void SetInertiaTensorScaling(float newInertiaTensorScaling);


		/// <summary>
		/// Gets or sets the radius of the bone.
		/// Setting the radius changes the inertia tensor of the bone.
		/// </summary>
		float GetRadius();

		/// <summary>
		/// Gets or sets the radius of the bone.
		/// Setting the radius changes the inertia tensor of the bone.
		/// </summary>
		void SetRadius(float newRadius);



		/// <summary>
		/// Gets the length of the bone.
		/// Setting the length changes the inertia tensor of the bone.
		/// </summary>
		float GetLength();

		/// <summary>
		/// Sets the length of the bone.
		/// Setting the length changes the inertia tensor of the bone.
		/// </summary>
		void SetLength(float newLength);




		void ComputeLocalInertiaTensor();

		/// <summary>
		/// Updates the world inertia tensor based upon the local inertia tensor and current orientation.
		/// </summary>
		void UpdateInertiaTensor();

		/// <summary>
		/// Integrates the position and orientation of the bone forward based upon the current linear and angular velocity.
		/// </summary>
		void UpdatePosition();

		void ApplyLinearImpulse(Vector3 &impulse);

		void ApplyAngularImpulse(Vector3 &impulse);

		vector<IKJoint*> Joints;

		/// <summary>
		/// Gets or sets the position of the bone.
		/// </summary>
		Vector3 Position;

		/// <summary>
		/// Gets or sets the orientation of the bone.
		/// </summary>
		Quaternion Orientation;

		float InverseMass;
		Matrix3X3 InertiaTensorInverse;
		Matrix3X3 LocalInertiaTensorInverse;

		/// <summary>
		/// Gets or sets whether or not this bone is pinned. Pinned bones cannot be moved by constraints.
		/// </summary>
		bool Pinned;

		/// <summary>
		/// Gets whether or not the bone is a member of the active set as determined by the last IK solver execution.
		/// </summary>
		bool IsActive;

		/// <summary>
		/// The mid-iteration angular velocity associated with the bone.
		/// This is computed during the velocity subiterations and then applied to the orientation at the end of each position iteration.
		/// </summary>
		Vector3 angularVelocity;

		/// <summary>
		/// The mid-iteration linear velocity associated with the bone.
		/// This is computed during the velocity subiterations and then applied to the position at the end of each position iteration.
		/// </summary>
		Vector3 linearVelocity;

		/// <summary>
		/// Used by the per-control traversals to find stressed paths.
		/// It has to be separate from the IsActive flag because the IsActive flag is used in the same traversal
		/// to denote all visited bones (including unstressed ones).
		/// Also used in the unstressed traversals; FindCycles uses the IsActive flag and the following DistributeMass phase uses the traversed flag.
		/// </summary>
		bool traversed;

		/// <summary>
		/// The number of stressed paths which use this bone. A stressed path is a possible path between a pin and a control.
		/// </summary>
		int stressCount;

		/// <summary>
		/// The set of parents of a given bone in a traversal. This is like a list of parents; there can be multiple incoming paths and they all need to be kept handy in order to perform some traversal details.
		/// </summary>
		vector<IKBone*> predecessors;

		/// <summary>
		/// True of the bone is a member of a cycle in an unstressed part of the graph or an unstressed predecessor of an unstressed cycle.
		/// Marking all the predecessors is conceptually simpler than attempting to mark the cycles in isolation.
		/// </summary>
		bool unstressedCycle;

		/// <summary>
		/// True if the bone is targeted by a control in the current stress cycle traversal that isn't the current source control.
		/// </summary>
		bool targetedByOtherControl;
		
		bPoseChannel * pchan;

	private:
		/// <summary>
		/// An arbitrary scaling factor is applied to the inertia tensor. This tends to improve stability.
		/// </summary>
		float inertiaTensorScaling;

		float radius;
		float halfLength;

		bool setMassCalled;
		bool setLengthCalled;
		bool setRadiusCalled;
		bool setInertiaTensorScalingCalled;
	};
}
