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
#include "Toolbox.hpp"

namespace BEPUik
{
	/// <summary>
	/// Attempts to maintain the relative orientation between two bones.
	/// </summary>
	class IKAngularJoint : public IKJoint
	{
	public:
		/// <summary>
		/// Gets or sets the relative orientation between the connections to maintain.
		/// </summary>
		Quaternion GoalRelativeOrientation;

		/// <summary>
		/// Constructs a 3DOF angular joint which tries to keep two bones in angular alignment.
		/// </summary>
		/// <param name="connectionA">First bone to connect to the joint.</param>
		/// <param name="connectionB">Second bone to connect to the joint.</param>
		IKAngularJoint(IKBone* connectionA, IKBone* connectionB);
        IKAngularJoint(IKBone*connectionA, IKBone*connectionB, Quaternion relativeOrientation);
		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	private:
		Quaternion error;
	};
}
