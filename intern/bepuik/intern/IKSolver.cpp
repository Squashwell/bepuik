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

#include "ActiveSet.hpp"
#include "IKSolver.hpp"
#include <boost/tr1/unordered_set.hpp>
using namespace boost;
#include <assert.h>
#include "float.h"

namespace BEPUik
{

		/// <summary>
		/// Constructs a new IKSolver.
		/// </summary>
		IKSolver::IKSolver() :
			activeSet(ActiveSet()),
			controls(vector<Control*>()),
			ControlIterationCount(50),
			FixerIterationCount(20),
			VelocitySubiterationCount(3),
			AutoscaleControlImpulses(true),
			AutoscaleControlMaximumForce(FLT_MAX),
            permutationMapper(PermutationMapper()),
			timeStepDuration(1.0f)

		{
		}

		IKSolver::~IKSolver()
		{

		}

		void IKSolver::Solve(vector<IKJoint*> &joints)
		{
			activeSet.UpdateActiveSet(joints);

            //Reset the permutation index; every solve should proceed in exactly the same order.
            permutationMapper.SetPermutationIndex(0);
            
			float updateRate = 1 / timeStepDuration;
			for (unsigned int j = 0; j < activeSet.Joints.size(); ++j)
			{
				activeSet.Joints[j]->Preupdate(timeStepDuration,updateRate);
			}
			
			for (int i = 0; i < FixerIterationCount; ++i)
			{

				//Update the world inertia tensors of objects for the latest position.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdateInertiaTensor();
				}


				//Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
				for (unsigned int j = 0; j < activeSet.Joints.size(); ++j)
				{

					IKJoint* joint = activeSet.Joints[j];
					joint->UpdateJacobiansAndVelocityBias();
					joint->ComputeEffectiveMass();
					joint->WarmStart();
				}

				for (int j = 0; j < VelocitySubiterationCount; ++j)
				{

                    //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
                    for (unsigned int k = 0; k < activeSet.Joints.size(); ++k)
					{
                        unsigned int remappedIndex = permutationMapper.GetMappedIndex(k,activeSet.Joints.size());
						activeSet.Joints[remappedIndex]->SolveVelocityIteration();
					}
                    //Increment to use the next permutation.
                    permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex()+1);
				}

				//Integrate the positions of the bones forward.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdatePosition();
				}
			}

			//Clear out accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
			for (unsigned int i = 0; i < activeSet.Joints.size(); ++i)
			{
				activeSet.Joints[i]->ClearAccumulatedImpulses();
			}
		}

		void IKSolver::Solve(vector<Control *> &controls, vector <IKJoint *> &joints)
		{
			Solve(controls);
			
			unordered_set<IKJoint *> unsolved_joints_set(joints.begin(), joints.end());
			vector <IKJoint *> unsolved_joints_vec = vector <IKJoint *>();
			
			for(unsigned int j =0; j < activeSet.Joints.size(); ++j)
			{
				unsolved_joints_set.erase(activeSet.Joints[j]);
			}
			
			for(unsigned int j =0; j < joints.size(); ++j)
			{
				if(unsolved_joints_set.find(joints[j]) != unsolved_joints_set.end())
					unsolved_joints_vec.push_back(joints[j]);
			}
				
			Solve(unsolved_joints_vec);
			
		}
		
		/// <summary>
		/// Updates the positions of bones acted upon by the controls given to this solver.
		/// </summary>
		/// <param name="controls">List of currently active controls.</param>
		void IKSolver::Solve(vector<Control*> &controls)
		{

			//Update the list of active joints.
			activeSet.UpdateActiveSet(controls);

			if (AutoscaleControlImpulses)
			{

				//Update the control strengths to match the mass of the target bones and the desired maximum force.
				for (unsigned int i = 0; i < controls.size(); ++i)
				{
					float mass = controls[i]->GetTargetBone()->GetMass();
					controls[i]->SetMaximumForce(mass * AutoscaleControlMaximumForce);
				}
			}

            //Reset the permutation index; every solve should proceed in exactly the same order.
            permutationMapper.SetPermutationIndex(0);
			
			float updateRate = 1 / timeStepDuration;
			for (unsigned int j = 0; j < activeSet.Joints.size(); ++j)
			{
				activeSet.Joints[j]->Preupdate(timeStepDuration, updateRate);
			}
			for (unsigned int j = 0; j < controls.size(); ++j)
			{
				controls[j]->Preupdate(timeStepDuration, updateRate);
			}
			
			//Go through the set of controls and active joints, updating the state of bones.
			for (int i = 0; i < ControlIterationCount; ++i)
			{
				//Update the world inertia tensors of objects for the latest position.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdateInertiaTensor();
				}

				//Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
				for (unsigned int j = 0; j < activeSet.Joints.size(); ++j)
				{
					IKJoint* joint = activeSet.Joints[j];
					joint->UpdateJacobiansAndVelocityBias();
					joint->ComputeEffectiveMass();
					joint->WarmStart();
				}

				for (unsigned int j = 0; j < controls.size(); ++j)
				{
					Control* control = controls[j];

					//Pinned objects should NEVER be moved by controls.
					assert(!control->GetTargetBone()->Pinned);
					control->UpdateJacobiansAndVelocityBias();
					control->ComputeEffectiveMass();
					control->WarmStart();
				}

				for (int j = 0; j < VelocitySubiterationCount; ++j)
				{
					//Controls are updated first.
					for (unsigned int k = 0; k < controls.size(); ++k)
					{
						controls[k]->SolveVelocityIteration();
					}
                    
                    //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
                    for (unsigned int k = 0; k < activeSet.Joints.size(); ++k)
					{
                        unsigned int remappedIndex = permutationMapper.GetMappedIndex(k,activeSet.Joints.size());
						activeSet.Joints[remappedIndex]->SolveVelocityIteration();
					}
                    //Increment to use the next permutation.
                    permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex()+1);
				}


				//Integrate the positions of the bones forward.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdatePosition();
				}
			}

			//Clear out accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
			for (unsigned int i = 0; i < activeSet.Joints.size(); ++i)
			{
				activeSet.Joints[i]->ClearAccumulatedImpulses();
			}
			
			//The previous loop may still have significant errors in the active joints due to
			//unreachable targets. Run a secondary pass without the influence of the controls to
			//fix the errors without interference from impossible goals
			//This can potentially cause the bones to move away from the control targets, but with a sufficient
			//number of control iterations, the result is generally a good approximation.
			for (int i = 0; i < FixerIterationCount; ++i)
			{

				//Update the world inertia tensors of objects for the latest position.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdateInertiaTensor();
				}


				//Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
				for (unsigned int j = 0; j < activeSet.Joints.size(); ++j)
				{

					IKJoint* joint = activeSet.Joints[j];
					joint->UpdateJacobiansAndVelocityBias();
					joint->ComputeEffectiveMass();
					joint->WarmStart();
				}

				for (int j = 0; j < VelocitySubiterationCount; ++j)
				{

                    //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
                    for (unsigned int k = 0; k < activeSet.Joints.size(); ++k)
					{
                        unsigned int remappedIndex = permutationMapper.GetMappedIndex(k,activeSet.Joints.size());
						activeSet.Joints[remappedIndex]->SolveVelocityIteration();
					}
                    //Increment to use the next permutation.
                    permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex()+1);
				}

				//Integrate the positions of the bones forward.
				for (unsigned int j = 0; j < activeSet.Bones.size(); ++j)
				{
					activeSet.Bones[j]->UpdatePosition();
				}
			}

			//Clear out accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
			for (unsigned int i = 0; i < activeSet.Joints.size(); ++i)
			{
				activeSet.Joints[i]->ClearAccumulatedImpulses();
			}

			for (unsigned int i = 0; i < controls.size(); ++i)
			{
				controls[i]->ClearAccumulatedImpulses();
			}
		}


//		/// <summary>
//		/// Adds a control constraint to the solver.
//		/// </summary>
//		/// <param name="control">Control to add.</param>
//		/// <returns>True if the control was added, or false otherwise.</returns>
//		bool IKSolver::Add(Control* control)
//		{
//			if (control->SolverIndex != -1)
//				return false;
//			control->SolverIndex = controls.size();
//			controls.push_back(control);
//			return true;
//		}

//		/// <summary>
//		/// Removes a control from the solver.
//		/// </summary>
//		/// <param name="control">Control to remove.</param>
//		/// <returns>True if the control was present, or false otherwise.</returns>
//		bool IKSolver::Remove(Control* control)
//		{
//			if (control->SolverIndex < 0 || control->SolverIndex >= controls.size() || controls[control->SolverIndex] != control)
//				return false;
//
//			if (control->SolverIndex == controls.size() - 1)
//				controls.erase(controls.begin() + controls.size() - 1);
//			else
//			{
//				Control* lastControl = controls[controls.size() - 1];
//				controls.erase(controls.begin() + controls.size() - 1);
//				controls[control->SolverIndex] = lastControl;
//				lastControl->SolverIndex = control->SolverIndex;
//			}
//			control->SolverIndex = -1;
//			return true;
//		}

		ActiveSet *IKSolver::GetActiveSet()
		{
			return &activeSet;
		}

		void IKSolver::SetTimeStepDuration(float newTimeStepDuration)
		{
			timeStepDuration = MathHelper::Max(0,newTimeStepDuration);
		}

}
