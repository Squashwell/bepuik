#====================== BEGIN GPL LICENSE BLOCK ======================
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#  The Original Code is Copyright (C) 2013 by:
#  Harrison Nordby and Ross Nordby
#  All rights reserved.
#
#  The Original Code is: all of this file
#
#  Contributor(s): none yet.
#
#
#======================= END GPL LICENSE BLOCK ========================

set(INC
	.
        intern
        ../../source/blender/makesdna
        ../../source/blender/makesrna
        ../../source/blender/blenlib
        ../../source/blender/blenkernel
        ../../source/blender/blenloader
	)
	
	
list(APPEND INC_SYS
        ${BOOST_INCLUDE_DIR}
	)
	
add_definitions(${BOOST_DEFINITIONS})
	
set(SRC
	bepu.cpp
	bepu.h
	
	
	intern/ActiveSet.cpp
	intern/Control.cpp
	intern/DragControl.cpp
	intern/IKAngularJoint.cpp
	intern/IKBallSocketJoint.cpp
	intern/IKBone.cpp
	intern/IKConstraint.cpp
	intern/IKDistanceJoint.cpp
	intern/IKDistanceLimit.cpp
	intern/IKJoint.cpp
	intern/IKLimit.cpp
	intern/IKLinearAxisLimit.cpp
	intern/IKPointOnLineJoint.cpp
	intern/IKPointOnPlaneJoint.cpp
	intern/IKRevoluteJoint.cpp
	intern/IKSolver.cpp
	intern/IKSwingLimit.cpp
	intern/IKSwivelHingeJoint.cpp
	intern/IKTwistJoint.cpp
	intern/IKTwistLimit.cpp
	intern/MathHelper.cpp
	intern/Matrix.cpp
	intern/Matrix3x3.cpp
	intern/Quaternion.cpp
	intern/SingleBoneAngularMotor.cpp
	intern/SingleBoneConstraint.cpp
	intern/SingleBoneLinearMotor.cpp
	intern/StateControl.cpp
	intern/Toolbox.cpp
	intern/Vector2.cpp
	intern/Vector3.cpp
	intern/PermutationMapper.cpp
	
	
	intern/ActiveSet.hpp
	intern/Control.hpp
	intern/DragControl.hpp
	intern/IKAngularJoint.hpp
	intern/IKBallSocketJoint.hpp
	intern/IKBone.hpp
	intern/IKConstraint.hpp
	intern/IKDistanceJoint.hpp
	intern/IKDistanceLimit.hpp
	intern/IKJoint.hpp
	intern/IKLimit.hpp
	intern/IKLinearAxisLimit.hpp
	intern/IKPointOnLineJoint.hpp
	intern/IKPointOnPlaneJoint.hpp
	intern/IKRevoluteJoint.hpp
	intern/IKSolver.hpp
	intern/IKSwingLimit.hpp
	intern/IKSwivelHingeJoint.hpp
	intern/IKTwistJoint.hpp
	intern/IKTwistLimit.hpp
	intern/MathHelper.hpp
	intern/Matrix.hpp
	intern/Matrix3x3.hpp
	intern/Quaternion.hpp
	intern/SingleBoneAngularMotor.hpp
	intern/SingleBoneConstraint.hpp
	intern/SingleBoneLinearMotor.hpp
	intern/StateControl.hpp
	intern/Toolbox.hpp
	intern/Vector2.hpp
	intern/Vector3.hpp
	intern/PermutationMapper.hpp
	
	)

blender_add_lib(bf_intern_bepuik "${SRC}" "${INC}" "${INC_SYS}")





