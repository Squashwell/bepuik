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
#include "SingleBoneConstraint.hpp"

using namespace BEPUmath;

namespace BEPUik
{
	class SingleBoneLinearMotor : public SingleBoneConstraint
	{
	public:
		/// <summary>
		/// Gets or sets the target position to apply to the target bone.
		/// </summary>
		Vector3 TargetPosition;

		/// <summary>
		/// Gets or sets the offset in the bone's local space to the point which will be pulled towards the target position.
		/// </summary>
		Vector3 LocalOffset;


		Vector3 GetOffset();
		void SetOffset(Vector3 worldOffset);

		void UpdateJacobiansAndVelocityBias();
	};
}
