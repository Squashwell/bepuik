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
#include "ActiveSet.hpp"
#include "PermutationMapper.hpp"

namespace BEPUik
{
	class IKSolver
	{
	private:
		/// <summary>
		/// List of controls used by the solver.
		/// </summary>
		vector<Control*> controls;

		ActiveSet activeSet;
        
        PermutationMapper permutationMapper;
		float timeStepDuration;

	public:
		/// <summary>
		/// Gets the active joint set associated with the solver.
		/// </summary>
		ActiveSet* GetActiveSet();

		/// <summary>
		/// Gets or sets the number of solver iterations to perform in an attempt to reach specified goals.
		/// </summary>
		int ControlIterationCount;

		/// <summary>
		/// Gets or sets the number of solter iterations to perform after the control iterations in an attempt to minimize
		/// errors introduced by unreachable goals.
		/// </summary>
		int FixerIterationCount;

		/// <summary>
		/// Gets or sets the number of velocity iterations to perform per control or fixer iteration.
		/// </summary>
		int VelocitySubiterationCount;

		/// <summary>
		/// Gets or sets whether or not to scale control impulses such that they fit well with the mass of objects.
		/// </summary>
		bool AutoscaleControlImpulses;

		/// <summary>
		/// Gets or sets the maximum impulse the controls will try to push bones with when AutoscaleControlImpulses is enabled.
		/// </summary>
		float AutoscaleControlMaximumForce;

        
        
		/// <summary>
		/// Constructs a new IKSolver.
		/// </summary>
		IKSolver();

		IKSolver(int controlIterations, int fixerIterations, int subvelocityIterations);
		~IKSolver();

		/// <summary>
		/// Updates the positions of bones acted upon by the controls given to this solver.
		/// </summary>
		void Solve(vector<IKJoint*> &joints);

		void Solve(vector<Control*> &controls);
		
		void Solve(vector<Control *> &controls, vector <IKJoint *> &joints);
		
		void SetTimeStepDuration(float newTimeStepDuration);

		
		float GetTimeStepDuration();
		
	};
}

