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

#include "IKBallSocketJoint.hpp"

namespace BEPUik
{
		/// <summary>
		/// Gets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		Vector3 IKBallSocketJoint::GetOffsetA()
		{
			Vector3 toReturn;
			Quaternion::Transform(LocalOffsetA, connectionA->Orientation, toReturn);
			return toReturn;
		}

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection A to the anchor point.
		/// </summary>
		void IKBallSocketJoint::SetOffsetA(Vector3 offset)
		{
			Quaternion conjugate;
			Quaternion::Conjugate(connectionA->Orientation, conjugate);
			Quaternion::Transform(offset, conjugate, LocalOffsetA);
		}
        
		/// <summary>
		/// Gets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		Vector3 IKBallSocketJoint::GetOffsetB()
		{
			Vector3 toReturn;
			Quaternion::Transform(LocalOffsetB, connectionB->Orientation, toReturn);
			return toReturn;
		}

		/// <summary>
		/// Sets the offset in world space from the center of mass of connection B to the anchor point.
		/// </summary>
		void IKBallSocketJoint::SetOffsetB(Vector3 offset)
		{
			Quaternion conjugate;
			Quaternion::Conjugate(connectionB->Orientation, conjugate);
			Quaternion::Transform(offset, conjugate, LocalOffsetB);
		}
        
		/// <summary>
		/// Builds a ball socket joint.
		/// </summary>
		/// <param name="connectionA">First connection in the pair.</param>
		/// <param name="connectionB">Second connection in the pair.</param>
		/// <param name="anchor">World space anchor location used to initialize the local anchors.</param>
		IKBallSocketJoint::IKBallSocketJoint(IKBone* connectionA, IKBone* connectionB, Vector3 anchor)
			: IKJoint(connectionA, connectionB)
		{
            CalculateOffsets(connectionA,connectionB,anchor);
            
		}

		void IKBallSocketJoint::UpdateJacobiansAndVelocityBias()
		{
			Matrix3X3::GetIdentity(linearJacobianA);
			//The jacobian entries are is [ La, Aa, -Lb, -Ab ] because the relative velocity is computed using A-B. So, negate B's jacobians!
			linearJacobianB = Matrix3X3();
			linearJacobianB.M11 = -1;
			linearJacobianB.M22 = -1;
			linearJacobianB.M33 = -1;
			Vector3 rA;
			Quaternion::Transform(LocalOffsetA, connectionA->Orientation, rA);
			Matrix3X3::CreateCrossProduct(rA, angularJacobianA);
			//Transposing a skew-symmetric matrix is equivalent to negating it.
			Matrix3X3::Transpose(angularJacobianA, angularJacobianA);

			Vector3 worldPositionA;
			Vector3::Add(connectionA->Position, rA, worldPositionA);

			Vector3 rB;
			Quaternion::Transform(LocalOffsetB, connectionB->Orientation, rB);
			Matrix3X3::CreateCrossProduct(rB, angularJacobianB);

			Vector3 worldPositionB;
			Vector3::Add(connectionB->Position, rB, worldPositionB);

			Vector3::Subtract(worldPositionB, worldPositionA, linearError);
			Vector3::Multiply(linearError, errorCorrectionFactor, velocityBias);

		}
        
        void IKBallSocketJoint::CalculateOffsets(IKBone * connectionA, IKBone * connectionB, Vector3 anchor)
        {
            Vector3 offset;
            Vector3::Subtract(anchor, connectionA->Position, offset);
            SetOffsetA(offset);
            Vector3::Subtract(anchor, connectionB->Position, offset);
            SetOffsetB(offset);            
        }
		
		bool IKBallSocketJoint::HasError()
		{
			return !linearError.IsZero(0.0001f);
		}
}
