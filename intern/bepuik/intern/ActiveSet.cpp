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

#include <algorithm>
#include "ActiveSet.hpp"
#include <stdio.h>
#include "float.h"

#ifdef DEBUG
#include <assert.h>
#endif

namespace BEPUik
{

	ActiveSet::ActiveSet() : Joints(vector<IKJoint*>()), Bones(vector<IKBone*>()), bonesToVisit(queue<IKBone*>()), uniqueChildren(vector<IKBone*>()), AutomassUnstressedFalloff(0.9f), AutomassTarget(1), UseAutomass(true)
	{
	}

	ActiveSet::~ActiveSet()
	{
		Clear();
	}


	void ActiveSet::Clear()
	{
		for (unsigned int i = 0; i < Bones.size(); ++i)
		{
			Bones[i]->IsActive = false;
			Bones[i]->stressCount = 0;
			Bones[i]->predecessors.clear();
			//This is not strictly required for solver correctness, but making it clear that old bones are out of consideration can be useful for debugging.
			Bones[i]->SetMass(.01f);
		}
		for (unsigned int i = 0; i < Joints.size(); ++i)
		{
			Joints[i]->IsActive = false;
		}
		Bones.clear();
		Joints.clear();
	}


	void ActiveSet::UpdateActiveSet(vector<IKJoint*> &joints)
	{
		//Clear out the previous active set to make way for the new active set.
		//Note that the below flag clearing and usage creates a requirement.
		//Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
		Clear();

		for (unsigned int i = 0; i < joints.size(); ++i)
		{
			if(joints[i]->GetEnabled())
			{
				if (!joints[i]->GetConnectionA()->IsActive)
				{
					joints[i]->GetConnectionA()->IsActive = true;
					Bones.push_back(joints[i]->GetConnectionA());
				}

				if (!joints[i]->GetConnectionB()->IsActive)
				{
					joints[i]->GetConnectionB()->IsActive = true;
					Bones.push_back(joints[i]->GetConnectionB());
				}

				Joints.push_back(joints[i]);
			}
		}

		//Use an arbitrary mass for the bones.
		//This could conceivably encounter issues with pathological cases, but we don't have controls to easily guide a better choice.
		if (UseAutomass)
		{
			for (unsigned int i = 0; i < Bones.size(); ++i)
			{
				Bones[i]->SetMass(AutomassTarget);
			}
		}




	}


	/// <summary>
	/// Updates the ordered set of active joints.
	/// Joints are ordered according to their graph traversal distance from control constraints.
	/// Joints close to the control constraints are closer to index 0 than joints that are far from control constraints.
	/// The relative ordering of joints from different connected components is irrelevant; the only guarantee is that
	/// constraints further from the source in a particular connected component are later in the list than those that are close.
	/// </summary>
	/// <param name="controls">Currently active control constraints.</param>
	void ActiveSet::UpdateActiveSet(vector<Control*>& controls)
	{
		//Clear out the previous active set to make way for the new active set.
		//Note that the below flag clearing and usage creates a requirement.
		//Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
		Clear();

		if (UseAutomass)
		{
			//Identify the stressed bones.
			FindStressedPaths(controls);

			//Compute the dependency graph for all the unstressed bones and assign masses.
			DistributeMass(controls);
		}

		//While we have traversed the whole active set in the previous stressed/unstressed searches, we do not yet have a proper breadth-first constraint ordering available.

		//Perform a breadth-first search through the graph starting at the bones targeted by each control.
		for (unsigned int i = 0; i < controls.size(); ++i)
		{
			Control * control = controls[i];
			IKBone* targetBone = control->GetTargetBone();
			if(targetBone->pchan->bepuikflag & BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET)
				targetBone->SetMass(100000);
//			targetBone->SetMass(control->GetRigidityMassMultiplier() * targetBone->GetMass());
			bonesToVisit.push(targetBone);
			//Note that a bone is added to the visited bone set before it is actually processed.
			//This prevents a bone from being put in the queue redundantly.
			targetBone->IsActive = true;
			Bones.push_back(targetBone);
		}

		//Note that it's technically possible for multiple controls to affect the same bone.
		//The containment tests will stop it from adding in any redundant constraints as a result.
		while (bonesToVisit.size() > 0)
		{
			IKBone* bone = bonesToVisit.front();
			bonesToVisit.pop();

			for (unsigned int i = 0; i < bone->Joints.size(); ++i)
			{
				IKJoint* joint = bone->Joints[i];
				if (!joint->IsActive)
				{
					joint->IsActive = true;
					//This is the first time the joint has been visited, so plop it into the list.
					Joints.push_back(joint);
				}
				IKBone* boneToAdd = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
				if (!boneToAdd->Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
					!boneToAdd->IsActive) //Don't try to add a bone if it's already active.
				{
					boneToAdd->IsActive = true;
					//The bone was not already present in the active set. We should visit it!
					//Note that a bone is added to the visited bone set before it is actually processed.
					//This prevents a bone from being put in the queue redundantly.
					bonesToVisit.push(boneToAdd);
					Bones.push_back(boneToAdd);
				}
			}
		}

	}

	void ActiveSet::FindStressedPaths(vector<Control*>& controls)
	{
		//Start a depth first search from each controlled bone to find any pinned bones.
		//All paths from the controlled bone to the pinned bones are 'stressed.'
		//Stressed bones are given greater mass later on.
		for (unsigned  int i =0; i < controls.size(); ++i)
		{
			Control* control = controls[i];
			//Paths connecting controls should be considered stressed just in case someone tries to pull things apart.
			//Mark bones affected by controls so we can find them in the traversal.
			for (unsigned  int j = 0; j < controls.size(); ++j)
			{
				if (i != j) //Don't include the current control; that could cause false positives for stress cycles.
					controls[j]->GetTargetBone()->targetedByOtherControl = true;
			}

			//The control.TargetBone.Parent is null; that's one of the terminating condition for the 'upwards' post-traversal
			//that happens after a pin or stressed path is found.
			FindStressedPaths(control->GetTargetBone());

			//We've analyzed the whole graph for this control. Clean up the bits we used.

			for (unsigned int i = 0; i < Bones.size(); ++i)
			{
				Bones[i]->traversed = false;
				Bones[i]->IsActive = false;
				Bones[i]->predecessors.clear();
			}
			Bones.clear();

			//Get rid of the targetedByOtherControl markings.



			for (unsigned int j = 0; j < controls.size(); ++j)
			{
				controls[j]->GetTargetBone()->targetedByOtherControl = false;
			}
		}

		//All bones in the active set now have their appropriate StressCount values.



	}

	bool ActiveSet::BonesHaveInteracted(IKBone* bone, IKBone* childBone)
	{
		//Two bones have interacted if one includes the other in its predecessor list.
		return find(bone->predecessors.begin(), bone->predecessors.end(), childBone) != bone->predecessors.end() ||//childBone is a parent of bone. Don't revisit them, that's where we came from!
			find(childBone->predecessors.begin(), childBone->predecessors.end(), bone) != childBone->predecessors.end(); //This bone already explored the childBone; don't do it again.
	}

	void ActiveSet::NotifyPredecessorsOfStress(IKBone* bone)
	{
		//We don't need to tell already-stressed bones about the fact that they are stressed.
		//Their predecessors are already stressed either by previous notifications like this or
		//through the predecessors being added on after the fact and seeing that the path was stressed.
		if (!bone->traversed)
		{
			bone->traversed = true;
			bone->stressCount++;
			for (unsigned int i = 0; i < bone->predecessors.size(); ++i)
			{
				NotifyPredecessorsOfStress(bone->predecessors[i]);
			}
		}
	}

	void ActiveSet::FindStressedPaths(IKBone* bone)
	{
		bone->IsActive = true; //We must keep track of which bones have been visited
		Bones.push_back(bone);
		for (unsigned int i = 0; i < bone->Joints.size(); ++i)
		{
			IKJoint* joint = bone->Joints[i];
			IKBone* boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
			if (BonesHaveInteracted(bone, boneToAnalyze))
				continue;

			if (!boneToAnalyze->Pinned)
			{
				//The boneToAnalyze is reached by following a path from bone. We record this regardless of whether or not we traverse further.
				//There is one exception: DO NOT create paths to pinned bones!
				boneToAnalyze->predecessors.push_back(bone);
			}

			if (boneToAnalyze->Pinned || boneToAnalyze->traversed)
			{
				//This bone is connected to a pinned bone (or a bone which is directly or indirectly connected to a pinned bone)!
				//This bone and all of its predecessors are a part of a 'stressed path.'
				//This backwards notification is necessary because this depth first search could attempt a deep branch which winds
				//its way back up to a part of the graph which SHOULD be marked as stressed, but is not yet marked because this path
				//has not popped its way all the way up the stack yet! Left untreated, this would lead to missed stressed paths.
				NotifyPredecessorsOfStress(bone);
				continue;
			}

			if (boneToAnalyze->targetedByOtherControl)
			{
				//We will consider other controls to be sources of stress. This prevents mass ratio issues from allowing multiple controls to tear a structure apart.
				//We do not, however, stop the traversal here. Allow it to continue.
				NotifyPredecessorsOfStress(bone);
			}
			if (boneToAnalyze->IsActive)
			{
				//The bone has already been visited. We should not proceed.
				//Any bone which is visited but not stressed is either A: not fully explored yet or B: fully explored.
				//Given that we followed an unexplored path to the bone, it must be not fully explored.
				//However, we do not attempt to perform exploration on the bone: any not-yet-fully-explored bones
				//must belong to one of our parents in the DFS! They will take care of it.
				continue;
			}

			//The search hasn't yet found a stressed path or pinned bone yet.
			//Keep on movin' on!
			FindStressedPaths(boneToAnalyze);
			//If a child finds a pin, we will be notified of that fact by the above while loop which traverses the parent pointers.
		}


	}

	void ActiveSet::NotifyPredecessorsOfCycle(IKBone* bone)
	{
		//Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
		if (!bone->unstressedCycle && bone->stressCount == 0)
		{
			bone->unstressedCycle = true;
			for (unsigned int i = 0; i < bone->predecessors.size(); ++i)
			{
				NotifyPredecessorsOfCycle(bone->predecessors[i]);
			}
		}
	}

	void ActiveSet::FindCycles(IKBone* bone)
	{
		//The current bone is known to not be stressed.
		for (unsigned int i = 0; i < bone->Joints.size(); ++i)
		{
			IKJoint* joint = bone->Joints[i];
			IKBone* boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();

			if (BonesHaveInteracted(bone, boneToAnalyze))
				continue;
			//We found this bone. Regardless of what happens after, make sure that the bone knows about this path.
			boneToAnalyze->predecessors.push_back(bone);

			if (boneToAnalyze->IsActive)
			{
				//This bone is butting up against a node which was previously visited.
				//Based on the previous stress path computation, there is only one entry point into an unstressed part of the graph.
				//by the previous condition, we know it's not our immediate parent. We hit an unstressed part of the graph.

				//In other words, this is an unstressed cycle.

				//Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
				NotifyPredecessorsOfCycle(bone);
				continue;
			}
			//Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
			//any unstressed bone is known to not be a path to any pinned bones.

			//The root bone is already added to the active set by the parent breadth-first search.
			//Children are added to the active set.
			boneToAnalyze->IsActive = true;
			Bones.push_back(boneToAnalyze);
			FindCycles(boneToAnalyze);
		}
	}

	void ActiveSet::DistributeMass(IKBone *bone)
	{
		//Accumulate the number of child joints which we are going to distribute mass to.
		for (unsigned int i =0; i < bone->Joints.size(); ++i)
		{
			IKJoint* joint = bone->Joints[i];
			IKBone* boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();

			if (boneToAnalyze->traversed || boneToAnalyze->unstressedCycle ||
				find(uniqueChildren.begin(), uniqueChildren.end(), boneToAnalyze) != uniqueChildren.end()) //There could exist multiple joints involved with the same pair of bones; don't continually double count.
			{
				//The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
				continue;
			}
			uniqueChildren.push_back(boneToAnalyze);
		}
		//We distribute a portion of the current bone's total mass to the child bones.
		//By applying a multiplier automassUnstressedFalloff, we guarantee that a chain has a certain maximum weight (excluding cycles).
		//This is thanks to the convergent geometric series sum(automassUnstressedFalloff^n, 1, infinity).
		float massPerChild = uniqueChildren.size() > 0 ? AutomassUnstressedFalloff * bone->GetMass() / uniqueChildren.size(): 0;

		uniqueChildren.clear();
		//(If the number of children is 0, then the only bones which can exist are either bones which were already traversed and will be skipped
		//or bones which are members of unstressed cycles and will inherit the full parent weight. Don't have to worry about the 0 mass.)

		//The current bone is known to not be stressed.
		for (unsigned int i =0; i < bone->Joints.size(); ++i)
		{
			IKJoint* joint = bone->Joints[i];
			IKBone* boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
			//Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
			//any unstressed bone is known to not be a path to any pinned bones.
			if (boneToAnalyze->traversed)// || bone.unstressedCycle)//bone.predecessors.Contains(boneToAnalyze))
			{
				//The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
				continue;
			}

			if (boneToAnalyze->unstressedCycle)
			{
				//This bone is part of a cycle! We cannot give it less mass; that would add in a potential instability.
				//Just give it the current node's full mass.
				boneToAnalyze->SetMass(bone->GetMass());
			}
			else
			{
				//This bone is not a part of a cycle; give it the allotted mass.
				boneToAnalyze->SetMass(massPerChild);
			}
			//The root bone is already added to the traversal set; add the children.
			boneToAnalyze->traversed = true;
			//Note that we do not need to add anything to the bones list here; the previous FindCycles DFS on this unstressed part of the graph did it for us.
			DistributeMass(boneToAnalyze);

		}

	}

	void ActiveSet::DistributeMass(vector<Control*> &controls)
	{
		//We assume that all stressed paths have already been marked with nonzero StressCounts.
		//Perform a multi-origin breadth-first search starting at every control. Look for any bones
		//which still have a StressCount of zero.

		//These zero-StressCount bones are the beginnings of isolated 'limbs' in the graph; there is only
		//one bone-to-bone connection (potentially made of multiple constraints, of course, but that does not affect graph connectivity)
		//between the stressed component of the graph and the isolated limb.
		//That means any traversal starting at that first bone and moving out away from the stressed graph will never return to the stressed graph
		//(no bone can be revisited).

		//Because these unstressed limbs are not critical weight-carrying paths, they do not need to be as heavy as the stressed paths.
		//In addition, to make the IK more responsive, unstressed bones further from the stressed component of the graph can be made less massive.

		//Care must be taken in determining the masses, though; if the root is light and its children, while individually lighter, are cumulatively much heavier,
		//there could be mass-ratio related instability.

		//To address this, cycles are found and given equal mass and each noncycle branch splits the current object's mass between all noncycle children.


		//Perform a breadth-first search through the graph starting at the bones targeted by each control.
		for (unsigned int i = 0; i < controls.size(); ++i)
		{
			Control* control = controls[i];
			bonesToVisit.push(control->GetTargetBone());
			//Note that a bone is added to the visited bone set before it is actually processed.
			//This prevents a bone from being put in the queue redundantly.
			control->GetTargetBone()->IsActive = true;
			//A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
			control->GetTargetBone()->traversed = true;
			Bones.push_back(control->GetTargetBone());
		}

		//Note that it's technically possible for multiple controls to affect the same bone.
		//The containment tests will stop it from adding in any redundant constraints as a result.
		while (bonesToVisit.size() > 0)
		{
			IKBone* bone = bonesToVisit.front();
			bonesToVisit.pop();
			if (bone->stressCount == 0)
			{
				bone->SetMass(AutomassUnstressedFalloff);
				//This is an unstressed bone. We should start a DFS to identify any cycles in the unstressed graph.
				FindCycles(bone);
				//Once the cycles are marked, we can proceed through the unstressed graph component and give child bones mass.
				DistributeMass(bone);
				//Do not continue the breadth-first search into the unstressed part of the graph.
				continue;
			}
			else
			{
				//The mass of stressed bones is a multiplier on the number of stressed paths overlapping the bone.
				bone->SetMass(bone->stressCount);
			}
			//This bone is not an unstressed branch root. Continue the breadth first search!
			for (unsigned int i =0; i < bone->Joints.size(); ++i)
			{
				IKJoint* joint = bone->Joints[i];
				IKBone* boneToAdd = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
				if (!boneToAdd->Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
					!boneToAdd->IsActive) //Don't try to add a bone if it's already active.
				{
					boneToAdd->IsActive = true;
					//A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
					boneToAdd->traversed = true;
					boneToAdd->predecessors.push_back(bone);
					//The bone was not already present in the active set. We should visit it!
					//Note that a bone is added to the visited bone set before it is actually processed.
					//This prevents a bone from being put in the queue redundantly.
					bonesToVisit.push(boneToAdd);
					Bones.push_back(boneToAdd);
				}
			}
		}

		//Normalize the masses of objects so that the heaviest bones have AutomassTarget mass.
		float lowestInverseMass = FLT_MAX;
		for (unsigned int i = 0; i < Bones.size(); ++i)
		{
			//Use this to visualize the flags for debugging purposes.
			//if (bones[i].stressCount > 0)
			//	bones[i].Mass = 2;
			//else if (bones[i].unstressedCycle)
			//	bones[i].Mass = .5f;
			//else
			//	bones[i].Mass = 0.01f;

			if (Bones[i]->InverseMass < lowestInverseMass)
				lowestInverseMass = Bones[i]->InverseMass;
		}
		
#ifdef DEBUG
			assert(AutomassTarget==AutomassTarget);
			assert(lowestInverseMass==lowestInverseMass);
#endif
		float inverseMassScale = 1 / (AutomassTarget * lowestInverseMass);
		
		for (unsigned int i = 0; i < Bones.size(); ++i)
		{
			IKBone * bone = Bones[i];
			
#ifdef DEBUG
			assert(bone->InverseMass==bone->InverseMass);
			assert(bone->InverseMass!=0.0f);
			assert(inverseMassScale==inverseMassScale);
			assert(inverseMassScale!=0.0f);
#endif
			//Normalize the mass to the AutomassTarget.
			bone->InverseMass *= inverseMassScale;
			
			//Clear the transversal flags while we're at it.
			bone->IsActive = false;
			bone->traversed = false;
			bone->stressCount = 0;
			bone->unstressedCycle = false;
			bone->predecessors.clear();
		}
		
		Bones.clear();
	}



}
