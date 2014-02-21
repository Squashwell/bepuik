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
#include "IKJoint.hpp"
#include "Control.hpp"
#include <queue>

using namespace std;

namespace BEPUik {
	/// <summary>
	/// Manages the subset of joints which potentially need solving.
	/// The active joint set contains connected components in the joint-bone graph which interact with control constraints.
	/// These connected components can be bounded by pinned bones which do not transfer any motion.
	/// </summary>
	class ActiveSet
	{


	public:

		/// <summary>
		/// Gets the list of joints in the active set.
		/// </summary>
		vector<IKJoint*> Joints;

		/// <summary>
		/// Gets the list of bones in the active set.
		/// </summary>
		vector<IKBone*> Bones;

		/// <summary>
		/// Gets or sets whether or not to automatically configure the masses of bones in the active set based upon their dependencies.
		/// Enabling this makes the solver more responsive and avoids some potential instability.
		/// This will overwrite any existing mass settings.
		/// </summary>
		bool UseAutomass;

		/// <summary>
		/// Gets or sets the multiplier applied to the mass of a bone before distributing it to the child bones.
		/// Used only when UseAutomass is set to true.
		/// </summary>
		float AutomassUnstressedFalloff;

		/// <summary>
		/// Gets or sets the mass that the heaviest bones will have when automass is enabled.
		/// </summary>
		float AutomassTarget;


		ActiveSet();
		~ActiveSet();

		/// <summary>
		/// Updates the ordered set of active joints.
		/// Joints are ordered according to their graph traversal distance from control constraints.
		/// Joints close to the control constraints are closer to index 0 than joints that are far from control constraints.
		/// The relative ordering of joints from different connected components is irrelevant; the only guarantee is that
		/// constraints further from the source in a particular connected component are later in the list than those that are close.
		/// </summary>
		/// <param name="controls">Currently active control constraints.</param>
		void UpdateActiveSet(vector<Control*>& controls);

		void UpdateActiveSet(vector<IKJoint*> &joints);

	private:
		queue<IKBone*> bonesToVisit;
		vector<IKBone*> uniqueChildren;

		void FindStressedPaths(vector<Control*>& controls);

		bool BonesHaveInteracted(IKBone* bone, IKBone* childBone);

		void NotifyPredecessorsOfStress(IKBone* bone);

		void FindStressedPaths(IKBone* bone);

		void NotifyPredecessorsOfCycle(IKBone* bone);

		void FindCycles(IKBone* bone);

		void DistributeMass(IKBone *bone);

		void DistributeMass(vector<Control*> &controls);

		void Clear();


	};
}
