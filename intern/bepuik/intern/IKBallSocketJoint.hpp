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

namespace BEPUik
{
	//Keeps the anchors from two connections near each other.
	class IKBallSocketJoint : public IKJoint
	{
	public:
		/// <summary>
		/// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
		/// </summary>
		Vector3 LocalOffsetA;
		/// <summary>
		/// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
		/// </summary>
		Vector3 LocalOffsetB;

		/// <summary>
		/// Gets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		Vector3 GetOffsetA();

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		void SetOffsetA(Vector3 offset);
//        void SetOffsetA(Quaternion orientation, Vector3 offset);
        
        
		/// <summary>
		/// Gets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		Vector3 GetOffsetB();

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		void SetOffsetB(Vector3 offset);
//        void SetOffsetB(Quaternion orientation, Vector3 offset);

		/// <summary>
		/// Builds a ball socket joint.
		/// </summary>
		/// <param name="connectionA">First connection in the pair.</param>
		/// <param name="connectionB">Second connection in the pair.</param>
		/// <param name="anchor">World space anchor location used to initialize the local anchors.</param>
		IKBallSocketJoint(IKBone* connectionA, IKBone* connectionB, Vector3 anchor);
//        IKBallSocketJoint(IKBone *connectionA, IKBone *connectionB, IKBone *offsetReferenceA, IKBone *offsetReferenceB, Vector3 anchor);
        
        void CalculateOffsets(IKBone * connectionA, IKBone * connectionB, Vector3 anchor);
        
        
		void UpdateJacobiansAndVelocityBias();
		bool HasError();
	private:
		Vector3 linearError;
	};
}
