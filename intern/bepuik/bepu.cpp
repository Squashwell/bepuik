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

/*
 * Generic bepuik todos:
 *
 * perhaps change bepuik constraints (except bepuik target) so they dont use targets and just update the bone rename function
 *
 * blender constraints could be solved before or after bepuik.  The best way to do this would be to have a node
 * interface with bepuik/blender constraints
 *
 * bepuik targets could have defined offset (otherwise defined by armature offset)
 * it would make targetting two palms together easier
 *
 */
#include <string.h>
#include <vector>

#ifdef DEBUG
#include <assert.h>
#endif

#include "Matrix.hpp"
#include "Vector3.hpp"
#include "SingleBoneLinearMotor.hpp"
#include "SingleBoneAngularMotor.hpp"
#include "DragControl.hpp"
#include "StateControl.hpp"
#include "IKBone.hpp"
#include "IKSolver.hpp"
#include "IKAngularJoint.hpp"
#include "IKBallSocketJoint.hpp"
#include "IKConstraint.hpp"
#include "IKDistanceJoint.hpp"
#include "IKDistanceLimit.hpp"
#include "IKLinearAxisLimit.hpp"
#include "IKPointOnLineJoint.hpp"
#include "IKPointOnPlaneJoint.hpp"
#include "IKSwingLimit.hpp"
#include "IKSwivelHingeJoint.hpp"
#include "IKTwistJoint.hpp"
#include "IKTwistLimit.hpp"
#include "IKRevoluteJoint.hpp"

using namespace BEPUik;
#include "bepu.h"

#include <boost/foreach.hpp>
#ifdef __cplusplus
extern "C"
{
#endif
#include "BLI_utildefines.h"
#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "BKE_armature.h"
#include "BKE_action.h"
#include "DNA_constraint_types.h"
#include "DNA_object_types.h"
#include "RNA_access.h"
#include "DNA_ID.h"
#include "BKE_idprop.h"
#include "DNA_curve_types.h"
#include "BLI_math.h"
#include "BLI_string.h"
#include "DNA_anim_types.h"
#include "BKE_main.h"
#include "BKE_global.h"
#include "BKE_constraint.h"
#include "DNA_object_types.h"
#include "BKE_object.h"

static void qt_bepuqt(float * quat, const Quaternion & quaternion)
{
	quat[0] = quaternion.W;
	quat[1] = quaternion.X;
	quat[2] = quaternion.Y;
	quat[3] = quaternion.Z;
}

static void bepuqt_qt(Quaternion & quaternion, float * quat)
{
	quaternion.W = quat[0];
	quaternion.X = quat[1];
	quaternion.Y = quat[2];
	quaternion.Z = quat[3];
}

static void v3_bepuv3(float *v3, const Vector3 & vector3)
{
	v3[0] = vector3.X;
	v3[1] = vector3.Y;
	v3[2] = vector3.Z;
}

static void bepuv3_v3(Vector3 & vector3, float * v3)
{
	vector3.X=v3[0];
	vector3.Y=v3[1];
	vector3.Z=v3[2];
}

static void get_offset_along_quat(float rpos[3],float qt[4],float offset)
{
 rpos[0] = 0;
 rpos[1] = offset;
 rpos[2] = 0;
 
 mul_qt_v3(qt,rpos);
}

static void pchan_bepuik_position_to_internal_bepuik_position(Vector3 &internal_bepuik_position, float pchan_bepuik_loc[3], float pchan_bepuik_quat[4], float pchan_bepuik_length)
{
	float internal_bepuik_loc[3];
	get_offset_along_quat(internal_bepuik_loc,pchan_bepuik_quat,pchan_bepuik_length*.5);
	add_v3_v3(internal_bepuik_loc,pchan_bepuik_loc);
	
	bepuv3_v3(internal_bepuik_position,internal_bepuik_loc);	
}

static void internal_bepuik_position_to_pchan_bepuik_position(float pchan_bepuik_position[3], const Vector3 &internal_bepuik_position, const Quaternion &internal_bepuik_orientation, float length)
{
	float internal_bepuik_loc[3];
	float internal_bepuik_quat[4];
	v3_bepuv3(internal_bepuik_loc,internal_bepuik_position);
	qt_bepuqt(internal_bepuik_quat,internal_bepuik_orientation);
	
	get_offset_along_quat(pchan_bepuik_position,internal_bepuik_quat,length*-0.5f);
	add_v3_v3(pchan_bepuik_position,internal_bepuik_loc);	
}

static bool string_ends_with(const char * s, const char * suffix)
{
	int s_length = strlen(s);
	int suffix_length = strlen(suffix);
	
	if (s_length > (suffix_length-1) && !strcmp(s + s_length - suffix_length, suffix))
	{
		return true;
	}
	
	return false;
}

static void ikbone_position_orientation_match_mat(IKBone * ikbone, float mat[4][4])
{
	float quat[4];
	mat4_to_quat(quat,mat);
	bepuqt_qt(ikbone->Orientation,quat);
	
	pchan_bepuik_position_to_internal_bepuik_position(ikbone->Position,mat[3],quat,ikbone->GetLength());
}

#define BEPUIK_MIN_BONE_LENGTH .001
static void ikbone_match_mat_length(IKBone * ikbone, float mat[4][4], float bone_rest_length)
{	
	float vec[3];
	copy_v3_v3(vec, mat[1]);
	mul_v3_fl(vec, bone_rest_length);
	
	float bepuik_length = len_v3(vec);

	bepuik_length = MAX2(bepuik_length,BEPUIK_MIN_BONE_LENGTH);

	float bepuik_radius = MAX2(BEPUIK_BONE_LENGTH_TO_RADIUS(bepuik_length),BEPUIK_MIN_BONE_LENGTH);

	ikbone->SetLength(bepuik_length);
	ikbone->SetRadius(bepuik_radius);
	
	ikbone_position_orientation_match_mat(ikbone,mat);
}

static void mat_get_axis(float result_axis[3], short axis_id, float mat[4][4])
{
	//Axis ids:
	//X = 0
	//Y = 1
	//Z = 2
	//-X = 3
	//-Y = 4
	//-Z = 5
	
	if(axis_id > 2)
	{
		copy_v3_v3(result_axis,mat[axis_id-3]);
		result_axis[0] *= -1.0f;
		result_axis[1] *= -1.0f;
		result_axis[2] *= -1.0f;
	}
	else
		copy_v3_v3(result_axis,mat[axis_id]);

	normalize_v3(result_axis);	
}

struct ControlledToTarget
{
	bPoseChannel * pchan_controlled;
	float controlled_to_target_mat[4][4];
};
struct BEPUikTempSolvingData
{
	IKBone * ikbone;
	vector <ControlledToTarget *> controlled_to_target_info;
	
	float loc[3];
	float size[3];

	/* rotations - written in by actions or transform (but only one representation gets used at any time) */
	float eul[3];                       /* euler rotation */
	float quat[4];                      /* quaternion rotation */
	float rotAxis[3], rotAngle;         /* axis-angle rotation */

	/* bepuik only temp information */
	float rest_tail[3];
	float rest_pose_size[3];

	float rest_pose_mat[4][4];

	float hard_controlled_position[3];
	float hard_controlled_orientation[4];

	float prev_pose_mat[4][4];
};

#define BEPUIK_COPY_IMPORTANT_CHAN_DATA(destination,source) \
copy_v3_v3(destination->loc,source->loc); \
copy_v3_v3(destination->eul,source->eul); \
copy_qt_qt(destination->quat,source->quat); \
copy_v3_v3(destination->rotAxis,source->rotAxis); \
copy_v3_v3(destination->size,source->size); \
destination->rotAngle = source->rotAngle;

static void bepu_store_locrotsize(bPoseChannel * pchan)
{
	BEPUikTempSolvingData * bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
	BEPUIK_COPY_IMPORTANT_CHAN_DATA(bepuik,pchan);
}

static void bepu_restore_locrotsize(bPoseChannel * pchan)
{
	BEPUikTempSolvingData * bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
	BEPUIK_COPY_IMPORTANT_CHAN_DATA(pchan,bepuik);
}

static IKBone * pchan_get_bepuik_bone(bPoseChannel * pchan)
{
	if(pchan)
		if(pchan->bepuik)
		{
			BEPUikTempSolvingData * tempsolvingdata = (BEPUikTempSolvingData *)(pchan->bepuik);
			if(tempsolvingdata->ikbone)
			{
				return tempsolvingdata->ikbone;
			}
		}
	
	return NULL;
}

#define BEPUIK_CONSTRAINT_HAS_VALID_POSEOB(con,ob) ((con) && (ob) && ((ob)->pose))

#define CONNECTION_VARS(struct_name) struct_name * bjoint = (struct_name *)constraint->data; \
if(!BEPUIK_CONSTRAINT_HAS_VALID_POSEOB(bjoint,ob)) break; \
if(constraint->bepuik_rigidity < FLT_EPSILON) break; \
bPoseChannel * pchan_connection = BKE_pose_channel_find_name(ob->pose,bjoint->connection_subtarget); \
if(!pchan_connection) break; \
IKBone * ikbone_connection = pchan_get_bepuik_bone(pchan_connection); \
if(!ikbone_connection) break;

#define BEPUIK_DATA(pchan) ((BEPUikTempSolvingData *)((pchan)->bepuik))

#define LOC(IDENTIFIER) bPoseChannel * pchan_##IDENTIFIER = BKE_pose_channel_find_name(ob->pose,bjoint->IDENTIFIER##_subtarget); \
Vector3 bepuv3_##IDENTIFIER; \
if(pchan_##IDENTIFIER)	\
{ \
float v3_##IDENTIFIER[3]; \
interp_v3_v3v3(v3_##IDENTIFIER,BEPUIK_DATA(pchan_##IDENTIFIER)->rest_pose_mat[3],BEPUIK_DATA(pchan_##IDENTIFIER)->rest_tail,bjoint->IDENTIFIER##_head_tail); \
bepuv3_v3(bepuv3_##IDENTIFIER,v3_##IDENTIFIER); \
} else { break; }


//Can't use pose_mat for axis, because it causes instability when feedback is in use
#define AXIS(IDENTIFIER) bPoseChannel * pchan_##IDENTIFIER = BKE_pose_channel_find_name(ob->pose,bjoint->IDENTIFIER##_subtarget); \
Vector3 bepuv3_##IDENTIFIER; \
if(pchan_##IDENTIFIER) \
{ \
float v3_##IDENTIFIER[3]; \
mat_get_axis(v3_##IDENTIFIER,bjoint->IDENTIFIER,BEPUIK_DATA(pchan_##IDENTIFIER)->rest_pose_mat); \
bepuv3_v3(bepuv3_##IDENTIFIER,v3_##IDENTIFIER); \
} else { break; }

#define PCHAN_BEPUIK_BONE_LENGTH(pchan) (((BEPUikTempSolvingData *)((pchan)->bepuik))->ikbone->GetLength())

static float get_bone_length_normalized_orientation_rigidity(IKBone * ikbone, float orientation_rigidity)
{
	float length = ikbone->GetLength();
	return length * length * orientation_rigidity;
}

static void get_scale_applied_bone_local_offsets(float r_scale_applied_bone_local_offset[3], float r_scale_applied_bone_local_offset_bepuik_internal[3], float length, float size[3], float bone_local_offset[3])
{
	zero_v3(r_scale_applied_bone_local_offset);
	
	
	//TODO:BEPUIK don't think length should be applied on x or z... or perhaps visually it makes sense because of how bones are drawn?
	r_scale_applied_bone_local_offset[0] += (length * size[0]) * bone_local_offset[0];
	r_scale_applied_bone_local_offset[1] += (length * size[1]) * bone_local_offset[1];
	r_scale_applied_bone_local_offset[2] += (length * size[2]) * bone_local_offset[2];
	
	copy_v3_v3(r_scale_applied_bone_local_offset_bepuik_internal,r_scale_applied_bone_local_offset);
	
	//subtract half the length in pose space because bepuik uses the center of bones as default while
	//blender uses the head of bones as default
	r_scale_applied_bone_local_offset_bepuik_internal[1] -= (length * size[1])*.5f;
}



static void flag_child_to_root_as_core(bPoseChannel * pchan)
{
	if(!(pchan->bepuikflag & BONE_BEPUIK_CORE))
	{
		pchan->bepuikflag |= BONE_BEPUIK_CORE;
		if(pchan->parent)
		{
			flag_child_to_root_as_core(pchan->parent);
		}
	}
		
}

static StateControl * new_statecontrol(IKBone * ikbone, float local_offset_bepuik_internal[3], float target_position[3], float target_orientation[4], float position_rigidity, float orientation_rigidity)
{
	StateControl * statecontrol = new StateControl();
	statecontrol->SetTargetBone(ikbone);
	
	bepuv3_v3(statecontrol->GetLinearMotor()->LocalOffset,local_offset_bepuik_internal);
	
	bepuv3_v3(statecontrol->GetLinearMotor()->TargetPosition,target_position);
	bepuqt_qt(statecontrol->GetAngularMotor()->TargetOrientation,target_orientation);
	
	statecontrol->GetAngularMotor()->SetRigidity(get_bone_length_normalized_orientation_rigidity(ikbone,orientation_rigidity));
	statecontrol->GetLinearMotor()->SetRigidity(position_rigidity);	
	
	return statecontrol;
}

static void setup_bepuik_control_drawinfo(bConstraint * con, bPoseChannel * pchan_controlled, float destination_mat[4][4], float string_start[3], float string_end[3], float offset_start[3], float offset_end[3])
{
	bBEPUikControl * bepuik_control = (bBEPUikControl *)con->data;
	BEPUikTempSolvingData * pchan_controlled_bepuik = (BEPUikTempSolvingData *)pchan_controlled->bepuik;

	copy_m4_m4(bepuik_control->destination_mat,destination_mat);

	//figure out the size for the bepuik target visualization mat
	float size_mat[4][4];
	size_to_mat4(size_mat,pchan_controlled_bepuik->rest_pose_size);
	mul_m4_m4m4(bepuik_control->destination_mat,bepuik_control->destination_mat,size_mat);

	copy_v3_v3(bepuik_control->string_start,string_start);
	copy_v3_v3(bepuik_control->string_end,string_end);

	con->flag |= CONSTRAINT_BEPUIK_DRAWABLE;

	copy_v3_v3(bepuik_control->offset_start,offset_start);
	copy_v3_v3(bepuik_control->offset_end,offset_end);
}

static void setup_bepuik_control(Object * ob, bConstraint * constraint, IKBone * ikbone, vector <Control *> &controls)
{	
	bBEPUikControl * bepuik_control = (bBEPUikControl *)constraint->data;
	if(!bepuik_control->connection_target) return;
	bPoseChannel * pchan_controlled = ikbone->pchan;
	BEPUikTempSolvingData * pchan_controlled_bepuik = (BEPUikTempSolvingData *)pchan_controlled->bepuik;
		
	float scale_applied_local_offset[3];
	float scale_applied_local_offset_bepuik_internal[3];
	get_scale_applied_bone_local_offsets(scale_applied_local_offset,scale_applied_local_offset_bepuik_internal,pchan_controlled->bone->length,BEPUIK_DATA(pchan_controlled)->rest_pose_size,bepuik_control->pulled_point);
	
	float effective_orientation_rigidity = bepuik_control->orientation_rigidity;
	float effective_position_rigidity = constraint->bepuik_rigidity;
	
	if(bepuik_control->connection_target->type == OB_ARMATURE)
	{
		bPoseChannel * pchan_target = BKE_pose_channel_find_name(bepuik_control->connection_target->pose,bepuik_control->connection_subtarget);
		if(!pchan_target) return;
		
		if(bepuik_control->connection_target == ob) //the target pchan is in the same armature
		{			
			float mat[4][4];
	
			copy_m4_m4(mat,pchan_controlled->bone->arm_mat);
			invert_m4(mat);
			
			float arm_controlled_to_arm_target[4][4];
			mul_m4_m4m4(arm_controlled_to_arm_target,mat,pchan_target->bone->arm_mat);
			
			float target_no_scale_bepuik_rest[4][4];
			float target_inverse_no_scale_bepuik_rest[4][4];
			unit_m4(target_no_scale_bepuik_rest);
			mul_m4_m4m4(target_no_scale_bepuik_rest,pchan_controlled_bepuik->rest_pose_mat,arm_controlled_to_arm_target);
			normalize_m4(target_no_scale_bepuik_rest);
			invert_m4_m4(target_inverse_no_scale_bepuik_rest,target_no_scale_bepuik_rest);
			
			float controlled_no_scale_bepuik_rest_mat[4][4];
			normalize_m4_m4(controlled_no_scale_bepuik_rest_mat,pchan_controlled_bepuik->rest_pose_mat);
			
			ControlledToTarget * controlled_to_target = new ControlledToTarget;
			controlled_to_target->pchan_controlled = pchan_controlled;
			
			float controlled_no_scale_inverse_bepuik_rest_mat[4][4];
			invert_m4_m4(controlled_no_scale_inverse_bepuik_rest_mat,controlled_no_scale_bepuik_rest_mat);
			mul_m4_m4m4(controlled_to_target->controlled_to_target_mat,controlled_no_scale_inverse_bepuik_rest_mat,target_no_scale_bepuik_rest);
			
			BEPUikTempSolvingData * pchan_target_bepuik = (BEPUikTempSolvingData *)pchan_target->bepuik;
			pchan_target_bepuik->controlled_to_target_info.push_back(controlled_to_target);
			
			float target_to_control_bepuik_rest[4][4];
			mul_m4_m4m4(target_to_control_bepuik_rest,target_inverse_no_scale_bepuik_rest,controlled_no_scale_bepuik_rest_mat);
			
			float bone_destination_mat[4][4];//bone destination mat is the mat of the bone assuming it reaches its ideal destination

			if(bepuik_control->bepuikflag & BEPUIK_CONSTRAINT_REST_OFFSET) {
				float normalized_target_pose_mat[4][4];
				normalize_m4_m4(normalized_target_pose_mat,pchan_target->pose_mat);
				mul_m4_m4m4(bone_destination_mat,normalized_target_pose_mat,target_to_control_bepuik_rest);
			}
			else
			{
				normalize_m4_m4(bone_destination_mat,pchan_target->pose_mat);
			}

			
			float scale_applied_local_offset_mat[4][4];
			unit_m4(scale_applied_local_offset_mat);
			copy_v3_v3(scale_applied_local_offset_mat[3],scale_applied_local_offset);

			float motor_target_mat[4][4]; //motor target mat is the mat that gives statecontrols their goal target
			mul_m4_m4m4(motor_target_mat,bone_destination_mat,scale_applied_local_offset_mat);

			float motor_target_position[3];
			float motor_target_orientation[4];
			mat4_to_loc_quat(motor_target_position,motor_target_orientation,motor_target_mat);
			
			setup_bepuik_control_drawinfo(constraint,pchan_controlled,bone_destination_mat,scale_applied_local_offset,motor_target_position,pchan_target->pose_mat[3],motor_target_position);

			//if an absolute target was previously created for this bone, then we dont need to create any other targets
			if(pchan_controlled->bepuikflag & BONE_BEPUIK_AFFECTED_BY_HARD_CONTROL) return;
			
			if(bepuik_control->bepuikflag & BEPUIK_CONSTRAINT_HARD)
			{
				pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_HARD_CONTROL;
				
				copy_v3_v3(pchan_controlled_bepuik->hard_controlled_position,bone_destination_mat[3]);
				mat4_to_quat(pchan_controlled_bepuik->hard_controlled_orientation,bone_destination_mat);
				
				effective_orientation_rigidity += 1000.0f;
				effective_position_rigidity += 1000.0f;
			}
			else
			{
				if(ob->pose->bepuikflag & POSE_BEPUIK_IGNORE_SOFT_CONTROLS) return;
			}
			
			if(effective_orientation_rigidity >= FLT_EPSILON)
			{
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
			}
			
			if((effective_position_rigidity >= FLT_EPSILON) || (effective_orientation_rigidity >= FLT_EPSILON))
			{
				pchan_target->bepuikflag |= BONE_BEPUIK_IS_ACTIVE_BEPUIK_TARGET;
				pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_CONTROL;
				
				StateControl * statecontrol = new_statecontrol(ikbone,scale_applied_local_offset_bepuik_internal,motor_target_position,motor_target_orientation,effective_position_rigidity,effective_orientation_rigidity);
				
				controls.push_back(statecontrol);
			}

		}

	}
	else //targeting a different object completely, use objects worldspace to drive the target
	{
		
		float destination_mat[4][4];
		float destination_position[3];
		float destination_orientation[4];
//		BKE_armature_mat_world_to_pose(ob,bepuik_control->connection_target->obmat,destination_mat); this function is exactly opposite of what it says it does???
		
		float imat[4][4];
		invert_m4_m4(imat, ob->obmat);
		mul_m4_m4m4(destination_mat, imat, bepuik_control->connection_target->obmat);
		
		normalize_m4(destination_mat);

		mat4_to_loc_quat(destination_position,destination_orientation,destination_mat);

		setup_bepuik_control_drawinfo(constraint,pchan_controlled,destination_mat,scale_applied_local_offset,destination_position,destination_position,destination_position);


		//if an absolute target was previously created for this bone, then we dont need to create any other targets
		if(pchan_controlled->bepuikflag & BONE_BEPUIK_AFFECTED_BY_HARD_CONTROL) return;

		
		if(bepuik_control->bepuikflag & BEPUIK_CONSTRAINT_HARD)
		{
			pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_HARD_CONTROL;
			
			copy_v3_v3(pchan_controlled_bepuik->hard_controlled_position,destination_position);
			copy_qt_qt(pchan_controlled_bepuik->hard_controlled_orientation,destination_orientation);
			
			effective_orientation_rigidity += 1000.0f;
			effective_position_rigidity += 1000.0f;
		}
		else
		{
			if(ob->pose->bepuikflag & POSE_BEPUIK_IGNORE_SOFT_CONTROLS) return;
		}
		
		if(effective_orientation_rigidity >= FLT_EPSILON)
		{
			ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
		}
		
		if((effective_position_rigidity >= FLT_EPSILON) || (effective_orientation_rigidity >= FLT_EPSILON))
		{
			pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_CONTROL;
			StateControl * statecontrol = new_statecontrol(ikbone,scale_applied_local_offset_bepuik_internal,destination_position,destination_orientation,effective_position_rigidity,effective_orientation_rigidity);
			
			controls.push_back(statecontrol);
		}
		
	}
}

static void rest_space_mat(float r_mat[4][4], bPoseChannel * pchan_a, bPoseChannel * pchan_b)
{
	//calculate the rest space delta from pchan to pchan_connection
	copy_m4_m4(r_mat,pchan_a->bone->arm_mat);
	invert_m4(r_mat);
	mul_m4_m4m4(r_mat,r_mat,pchan_b->bone->arm_mat);
}

void bepu_solve(Object * ob)
{
	if(ob->type != OB_ARMATURE) return;
	if(!ob->pose) return;

	vector <IKJoint *> joints = vector<IKJoint *>();
	vector <Control *> controls = vector<Control *>();
	vector <IKBone *> ikbones = vector<IKBone *>();
	
	// Sweep through the pchans and create ikbones matching the arbitrary bepuik "rest" pose, which
	// only includes the scale of the pchans.  Essentially, it's the edit bone rest pose with pchan scales applied
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{	
		BEPUikTempSolvingData * pchan_bepuik = new BEPUikTempSolvingData;
		pchan->bepuik = pchan_bepuik;
		bepu_store_locrotsize(pchan);

		size_to_mat4(pchan->chan_mat, pchan->size);
		BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
		
		copy_v3_v3(pchan->pose_head, pchan->pose_mat[3]);
		BKE_pose_where_is_bone_tail(pchan);
		
		copy_m4_m4(pchan_bepuik->rest_pose_mat,pchan->pose_mat);
		copy_v3_v3(pchan_bepuik->rest_tail,pchan->pose_tail);
		mat4_to_size(pchan_bepuik->rest_pose_size,pchan_bepuik->rest_pose_mat);
		
		if(pchan->bepuikflag & BONE_BEPUIK)
		{
			IKBone * ikbone = new IKBone();
			ikbones.push_back(ikbone);
			pchan_bepuik->ikbone = ikbone;
			ikbone->pchan = pchan;

			float effective_rotational_heaviness = pchan->bepuik_rotational_heaviness;
			CLAMP(effective_rotational_heaviness,BEPUIK_INTERTIA_TENSOR_SCALING_MIN,BEPUIK_INTERTIA_TENSOR_SCALING_MAX);
			ikbone->SetInertiaTensorScaling(effective_rotational_heaviness);
			
			// matches an ikbone's position, orientation and length to that of a pchan.
			// Note that a pchan's y axis length, including scale, will influence the length
			// of the bepuik bone itself.
			ikbone_match_mat_length(ikbone,pchan_bepuik->rest_pose_mat,pchan->bone->length);
		}
		else
		{
			pchan_bepuik->ikbone = NULL;
		}
	}
	
	//calculate the posemats again. This time with position, orientation and scale!
	//This needs to be done so that any BEPUik Target constraint has the correct pose info on which to base
	//its various offsets and visualization data
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		BKE_pchan_calc_mat(pchan);
		BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
		copy_m4_m4(pchan->bepuik_prepose_mat,pchan->pose_mat);
	}
	
	//Build all bepuik constraints!
	//WARNING: nothing in this loop should change positions or orientations of the bepuik bones themselves!
	//BEPUik uses the position and orientations (currently at the "bepuik rest pose") of the bepuik bones
	//to determine various bepuik constraint bases
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{	
		BEPUikTempSolvingData * pchan_bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
		IKBone * ikbone = pchan_bepuik->ikbone;
		
		//all bepuik constraints must exist on a bepuik bone
		if(!ikbone)
			continue;
		
		//find any absolute targets first
		for(bConstraint * constraint = (bConstraint *)pchan->constraints.first; constraint; constraint = constraint->next)
		{
			if(constraint->type != CONSTRAINT_TYPE_BEPUIK_CONTROL) continue;
			if(constraint->flag & (CONSTRAINT_DISABLE|CONSTRAINT_OFF)) continue;
			
			bBEPUikControl * bepuik_control = (bBEPUikControl *)constraint->data;
			if (!(bepuik_control->bepuikflag & BEPUIK_CONSTRAINT_HARD)) continue;
			
			setup_bepuik_control(ob,constraint,ikbone,controls);
		}
		
		//create constraints but exclude hard bepuik controls
		for(bConstraint * constraint = (bConstraint *)pchan->constraints.first; constraint; constraint = constraint->next)
		{
			if(constraint->flag & (CONSTRAINT_DISABLE|CONSTRAINT_OFF))
				continue;
			
			IKJoint * ikjoint = NULL;
			
			//only create a constraint if the constraint is a bepuik type
			switch(constraint->type)
			{
			case CONSTRAINT_TYPE_BEPUIK_ANGULAR_JOINT:
			{
				CONNECTION_VARS(bBEPUikAngularJoint)
				float goal_mat[4][4];

				bPoseChannel * pchan_relative_orientation = BKE_pose_channel_find_name(ob->pose,bjoint->relative_orientation_subtarget);
				Quaternion goal_relative_orientation;
				if(pchan_relative_orientation)
				{
					BKE_pchan_to_mat4(pchan_relative_orientation,goal_mat);

					if(bjoint->flag & BEPUIK_CONSTRAINT_REST_OFFSET)
					{
						float rest_mat[4][4];
						rest_space_mat(rest_mat,pchan,pchan_connection);
						mul_m4_m4m4(goal_mat,rest_mat,goal_mat);
					}
				}
				else
				{
					rest_space_mat(goal_mat,pchan,pchan_connection);
				}

				float quat[4];
				mat4_to_quat(quat,goal_mat);
				bepuqt_qt(goal_relative_orientation,quat);
				ikjoint = new IKAngularJoint(ikbone,ikbone_connection,goal_relative_orientation);

				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_BALL_SOCKET_JOINT:
			{
				CONNECTION_VARS(bBEPUikBallSocketJoint)
				LOC(anchor)
						
				ikjoint = new IKBallSocketJoint(ikbone,ikbone_connection,bepuv3_anchor);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_DISTANCE_JOINT:
			{
				CONNECTION_VARS(bBEPUikDistanceJoint)
				LOC(anchor_a)
				LOC(anchor_b)

				ikjoint = new IKDistanceJoint(ikbone,ikbone_connection,bepuv3_anchor_a,bepuv3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_DISTANCE_LIMIT:
			{
				CONNECTION_VARS(bBEPUikDistanceLimit)
				LOC(anchor_a)
				LOC(anchor_b)
				
				ikjoint = new IKDistanceLimit(ikbone,ikbone_connection,bepuv3_anchor_a,bepuv3_anchor_b,bjoint->min_distance,bjoint->max_distance);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_LINEAR_AXIS_LIMIT:
			{
				CONNECTION_VARS(bBEPUikLinearAxisLimit)
				LOC(line_anchor)
				AXIS(line_direction)
				LOC(anchor_b)
				
				ikjoint = new IKLinearAxisLimit(ikbone,ikbone_connection,bepuv3_line_anchor,bepuv3_line_direction,bepuv3_anchor_b,bjoint->min_distance,bjoint->max_distance);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_POINT_ON_LINE_JOINT:
			{
				CONNECTION_VARS(bBEPUikPointOnLineJoint)
				LOC(line_anchor)
				AXIS(line_direction)
				LOC(anchor_b)
				
				ikjoint = new IKPointOnLineJoint(ikbone,ikbone_connection,bepuv3_line_anchor,bepuv3_line_direction,bepuv3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_POINT_ON_PLANE_JOINT:
			{
				CONNECTION_VARS(bBEPUikPointOnPlaneJoint)
				LOC(plane_anchor)
				LOC(anchor_b)
				AXIS(plane_normal)
				
				ikjoint = new IKPointOnPlaneJoint(ikbone,ikbone_connection,bepuv3_plane_anchor,bepuv3_plane_normal,bepuv3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_REVOLUTE_JOINT:
			{
				CONNECTION_VARS(bBEPUikRevoluteJoint)
				AXIS(free_axis)
				
				ikjoint = new IKRevoluteJoint(ikbone,ikbone_connection,bepuv3_free_axis);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_SWING_LIMIT:
			{
				CONNECTION_VARS(bBEPUikSwingLimit)
				AXIS(axis_a)
				AXIS(axis_b)
				
				ikjoint = new IKSwingLimit(ikbone,ikbone_connection,bepuv3_axis_a,bepuv3_axis_b,bjoint->max_swing);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_SWIVEL_HINGE_JOINT:
			{
				CONNECTION_VARS(bBEPUikSwivelHingeJoint)
				AXIS(hinge_axis)
				AXIS(twist_axis)
				
				ikjoint = new IKSwivelHingeJoint(ikbone,ikbone_connection,bepuv3_hinge_axis,bepuv3_twist_axis);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_TWIST_JOINT:
			{
				CONNECTION_VARS(bBEPUikTwistJoint)
				AXIS(axis_a)
				AXIS(axis_b)
				
				ikjoint = new IKTwistJoint(ikbone,ikbone_connection,bepuv3_axis_a,bepuv3_axis_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_TWIST_LIMIT:
			{
				CONNECTION_VARS(bBEPUikTwistLimit)
				AXIS(axis_a)
				AXIS(axis_b)
				AXIS(measurement_axis_a)
				AXIS(measurement_axis_b)
				
				ikjoint = new IKTwistLimit(ikbone,ikbone_connection,bepuv3_axis_a,bepuv3_measurement_axis_a,bepuv3_axis_b,bepuv3_measurement_axis_b,bjoint->max_twist);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_CONTROL:
			{
				bBEPUikControl * bepuik_control = (bBEPUikControl *)constraint->data;
				if (bepuik_control->bepuikflag & BEPUIK_CONSTRAINT_HARD) break;
				
				setup_bepuik_control(ob,constraint,ikbone,controls);
				break;
			}
			}
	
			
			if(ikjoint)
			{
				ikjoint->SetRigidity(constraint->bepuik_rigidity);
				ikjoint->bConstraintType = constraint->type;
				joints.push_back(ikjoint);
			}
		}
		
		//check for auto ball socket joint
		if(pchan->parent && pchan_get_bepuik_bone(pchan->parent) && pchan->bepuik_ball_socket_rigidity >= FLT_EPSILON)
		{
			IKBone * child_connection = ikbone;
			IKBone * parent_connection = pchan_get_bepuik_bone(pchan->parent);
			
			Vector3 bepuv3 = Vector3();
			bepuv3_v3(bepuv3,BEPUIK_DATA(pchan)->rest_pose_mat[3]);
			IKJoint * auto_ballsocket = new IKBallSocketJoint(child_connection,parent_connection,bepuv3);
			auto_ballsocket->bConstraintType = CONSTRAINT_TYPE_BEPUIK_BALL_SOCKET_JOINT;
			auto_ballsocket->SetRigidity(pchan->bepuik_ball_socket_rigidity);
			joints.push_back(auto_ballsocket);
		}
		
		if(pchan->bone->flag & BONE_TRANSFORM && !(ob->pose->bepuikflag & POSE_BEPUIK_IGNORE_SOFT_CONTROLS))
		{
			if(ob->pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_DRAGCONTROL) 
			{
				DragControl * dragControl = new DragControl();
				dragControl->SetTargetBone(ikbone);
				dragControl->GetLinearMotor()->SetRigidity(ob->bepuik_dynamic_position_rigidity);
				
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
				
				//compensate for bepuik internal offset
				float local_offset[3];
				local_offset[0] = pchan->bepuik_transform_local_offset[0];
				local_offset[1] = pchan->bepuik_transform_local_offset[1] - (PCHAN_BEPUIK_BONE_LENGTH(pchan) * .5);
				local_offset[2] = pchan->bepuik_transform_local_offset[2];
				
				bepuv3_v3(dragControl->GetLinearMotor()->LocalOffset,local_offset);
				bepuv3_v3(dragControl->GetLinearMotor()->TargetPosition,pchan->bepuik_transform_position);
				controls.push_back(dragControl);
				pchan->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_CONTROL;
			}
			else if(ob->pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_STATECONTROL) 
			{
				StateControl * statecontrol = new StateControl();
				statecontrol->SetTargetBone(ikbone);
				statecontrol->GetLinearMotor()->SetRigidity(ob->bepuik_dynamic_position_rigidity);
				statecontrol->GetAngularMotor()->SetRigidity(get_bone_length_normalized_orientation_rigidity(ikbone,ob->bepuik_dynamic_orientation_rigidity));
				
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
				
				float quat[4];
				
				normalize_qt_qt(quat,pchan->bepuik_transform_orientation);
				bepuqt_qt(statecontrol->GetAngularMotor()->TargetOrientation,quat);
				pchan_bepuik_position_to_internal_bepuik_position(statecontrol->GetLinearMotor()->TargetPosition,pchan->bepuik_transform_position,quat,PCHAN_BEPUIK_BONE_LENGTH(pchan));
				controls.push_back(statecontrol);
				pchan->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_CONTROL;
			}
		}
	}

	//this is a gross hack to always solve some bones... blehgahhhhh
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		if((pchan->bepuikflag & BONE_BEPUIK_ALWAYS_SOLVE) && !(pchan->bepuikflag & BONE_BEPUIK_AFFECTED_BY_CONTROL))
		{
			IKBone * ikbone = pchan_get_bepuik_bone(pchan);
			if(ikbone)
			{
				DragControl * dragControl = new DragControl();
				dragControl->SetTargetBone(ikbone);
				dragControl->GetLinearMotor()->SetRigidity(0.0f);
				dragControl->GetLinearMotor()->LocalOffset = Vector3(0,0,0);
				dragControl->GetLinearMotor()->TargetPosition = Vector3(0,0,0);
				controls.push_back(dragControl);
			}
		}
	}

	BOOST_FOREACH(Control * control, controls)
	{
		IKBone * target_ikbone = (IKBone *)control->GetTargetBone();
		flag_child_to_root_as_core(target_ikbone->pchan);
	}

	if((ob->bepuik_dynamic_peripheral_stiffness >= FLT_EPSILON) && (ob->pose->bepuikflag & POSE_BEPUIK_DYNAMIC))
	{
		Quaternion dynamic_stiffness_orientation;
		Quaternion a;
		Quaternion b;
		
		float qa[4];
		float qb[4];
		
		for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
		{
			if(pchan->parent && !(pchan->bepuikflag & BONE_BEPUIK_CORE)) {
				IKBone * child = pchan_get_bepuik_bone(pchan);
				if(!child) continue;
				IKBone * parent = pchan_get_bepuik_bone(pchan->parent);
				if(!parent) continue;
				
				if(ob->pose->bepuikflag & POSE_BEPUIK_UPDATE_DYNAMIC_STIFFNESS_MAT)
				{										
					mat4_to_quat(qa,pchan->parent->bepuik_prepose_mat);
					mat4_to_quat(qb,pchan->bepuik_prepose_mat);
					
					bepuqt_qt(b,qb);
					bepuqt_qt(a,qa);
					
					Quaternion::RelativeOrientation(a,b,dynamic_stiffness_orientation);
					qt_bepuqt(pchan->bepuik_dynamic_stiffness_orientation,dynamic_stiffness_orientation);
				}
				else
				{
					bepuqt_qt(dynamic_stiffness_orientation,pchan->bepuik_dynamic_stiffness_orientation);
				}
								
				IKAngularJoint * ikangularjoint = new IKAngularJoint(parent,child,dynamic_stiffness_orientation);
				ikangularjoint->SetRigidity(get_bone_length_normalized_orientation_rigidity(child,ob->bepuik_dynamic_peripheral_stiffness));
				ikangularjoint->bConstraintType=CONSTRAINT_TYPE_BEPUIK_ANGULAR_JOINT;
				joints.push_back(ikangularjoint);
			}
		}
	}

#ifdef DEBUG
	BOOST_FOREACH(IKJoint * ikjoint, joints)
	{
		assert(ikjoint->bConstraintType!=0);
	}
#endif
	ob->pose->bepuikflag &= ~POSE_BEPUIK_UPDATE_DYNAMIC_STIFFNESS_MAT;
	
	if(!(ob->bepuikflag & OB_BEPUIK_SOLVE_PERIPHERAL_BONES))
	{
		//enable only ikjoints connectect to core bones
		BOOST_FOREACH(IKJoint * ikjoint, joints)
		{
			if((ikjoint->GetConnectionA()->pchan->bepuikflag & BONE_BEPUIK_CORE) && (ikjoint->GetConnectionB()->pchan->bepuikflag & BONE_BEPUIK_CORE))
				ikjoint->SetEnabled(true);
			else
				ikjoint->SetEnabled(false);
		}
	}
	
	//Finally, after all constraints have been built based on the ikbones at their "bepuik rest pose," 
	//teleport the bones to the positions from which they will be solved.
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{					
		IKBone * ikbone = pchan_get_bepuik_bone(pchan);
		BEPUikTempSolvingData * pchan_bepuik = BEPUIK_DATA(pchan);
		if(ikbone)
		{
			if(pchan->bepuikflag & BONE_BEPUIK_AFFECTED_BY_HARD_CONTROL)
			{
				pchan_bepuik_position_to_internal_bepuik_position(ikbone->Position,pchan_bepuik->hard_controlled_position,pchan_bepuik->hard_controlled_orientation,ikbone->GetLength());
				bepuqt_qt(ikbone->Orientation,pchan_bepuik->hard_controlled_orientation);
			}
			else
			{
				ikbone_position_orientation_match_mat(ikbone,pchan->bepuik_prepose_mat);
			}
		}
	}
	
	//setup iksolver parameters
	IKSolver * iksolver = new IKSolver();

	float bepuik_solve_length = (float)((ob->pose->bepuikflag & POSE_BEPUIK_DYNAMIC) ? ob->bepuik_dynamic_solve_length : ob->bepuik_solve_length);
	float bepuik_solve_quality = ob->bepuik_solve_quality;

	CLAMP(bepuik_solve_length,BEPUIK_SOLVE_LENGTH_MIN,BEPUIK_SOLVE_LENGTH_MAX);
	CLAMP(bepuik_solve_quality,BEPUIK_SOLVE_QUALITY_MIN,BEPUIK_SOLVE_QUALITY_MAX);

	float timestep_duration = 1 / (float)bepuik_solve_quality;
	iksolver->ControlIterationCount = bepuik_solve_length / timestep_duration;
	iksolver->SetTimeStepDuration(timestep_duration);
	iksolver->FixerIterationCount = ob->bepuik_fixer_iterations;
	iksolver->AutoscaleControlMaximumForce = FLT_MAX;

	int velocity_subiteration_count = ob->bepuik_solve_quality * 3;
	CLAMP(velocity_subiteration_count,1,10);
	iksolver->VelocitySubiterationCount = velocity_subiteration_count;

	if(controls.size()>0)
	{
		iksolver->Solve(controls);

		// match pchans to their solved state
		for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
		{
			IKBone * ikbone = pchan_get_bepuik_bone(pchan);
			BEPUikTempSolvingData * pchan_bepuik = (BEPUikTempSolvingData *)pchan->bepuik;

			if(ikbone && ikbone->IsActive)
			{
				float loc[3];
				float quat[4];
				qt_bepuqt(quat,ikbone->Orientation);
				internal_bepuik_position_to_pchan_bepuik_position(loc,ikbone->Position,ikbone->Orientation,ikbone->GetLength());

				loc_quat_size_to_mat4(pchan->pose_mat,loc,quat,pchan_bepuik->rest_pose_size);
				BKE_armature_mat_pose_to_bone(pchan,pchan->pose_mat,pchan->chan_mat);
				BKE_pchan_apply_mat4(pchan,pchan->chan_mat,true);

				if(ob->pose->bepuikflag & POSE_BEPUIK_DYNAMIC)
				{
					pchan->bepuikflag |= BONE_BEPUIK_FEEDBACK;
				}

			}
			else
			{
				BKE_pchan_calc_mat(pchan);
				BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
			}
		}
	}

	//match targets to their solved state
	if(ob->pose->bepuikflag & POSE_BEPUIK_INACTIVE_TARGETS_FOLLOW)
	{
		for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
		{
			BEPUikTempSolvingData * pchan_bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
			size_t num_controlled_to_target_info = pchan_bepuik->controlled_to_target_info.size();
			bool do_feedback = true;
			if (pchan->bepuikflag & BONE_BEPUIK_IS_ACTIVE_BEPUIK_TARGET) do_feedback = false;
			else if (num_controlled_to_target_info == 0) do_feedback = false;
			else if ((pchan->bone->flag & BONE_TRANSFORM) && !(ob->pose->bepuikflag & POSE_BEPUIK_IGNORE_SOFT_CONTROLS)) do_feedback = false;
			else if (pchan->bone->flag & BONE_CONNECTED) do_feedback = false;
			
			if(do_feedback)
			{
				float average_position[3];
				zero_v3(average_position);

				float average_orientation[4];
				zero_v4(average_orientation);

				BOOST_FOREACH(ControlledToTarget * controlled_to_target, pchan_bepuik->controlled_to_target_info)
				{
					float target_new_pose_mat[4][4];
					float quat[4];
					float normalized_controlled_pose_mat[4][4];
					normalize_m4_m4(normalized_controlled_pose_mat,controlled_to_target->pchan_controlled->pose_mat);
					mul_m4_m4m4(target_new_pose_mat,normalized_controlled_pose_mat,controlled_to_target->controlled_to_target_mat);
					mat4_to_quat(quat,target_new_pose_mat);


					add_v4_v4(average_orientation,quat);

					add_v3_v3(average_position,target_new_pose_mat[3]);
				}

				if(num_controlled_to_target_info>1)
				{
					mul_v3_fl(average_position,1.0f/(float)num_controlled_to_target_info);
					normalize_qt(average_orientation);
				}

				loc_quat_size_to_mat4(pchan->pose_mat,average_position,average_orientation,pchan_bepuik->rest_pose_size);
				BKE_armature_mat_pose_to_bone(pchan,pchan->pose_mat,pchan->chan_mat);
				BKE_pchan_apply_mat4(pchan,pchan->chan_mat,true);

				pchan->bepuikflag |= BONE_BEPUIK_FEEDBACK;
			}
			else
			{
				BKE_pchan_calc_mat(pchan);
				BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
			}
		}
	}

	//finally, save the new locrotsize data in the bepuik twin information for later using in dynamic mode
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		copy_v3_v3(pchan->bepuik_loc,pchan->loc);
		copy_v3_v3(pchan->bepuik_eul,pchan->eul);
		copy_qt_qt(pchan->bepuik_quat,pchan->quat);
		copy_v3_v3(pchan->bepuik_rotAxis,pchan->rotAxis);
		pchan->bepuik_rotAngle = pchan->rotAngle;
	}

	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
		for(bConstraint * constraint = (bConstraint *)pchan->constraints.first; constraint; constraint = constraint->next)
		{
			if(constraint->type == CONSTRAINT_TYPE_BEPUIK_CONTROL)
			{
				bBEPUikControl * bepuik_control = (bBEPUikControl *)constraint->data;


				float quat[4];
				mat4_to_quat(quat,pchan->pose_mat);

				//In the main constraint creation loop, string start was set to the scale applied local offset.
				//Now, we update the graphical effect of the "pulling string" to attach to the post-solve bone position.
				mul_qt_v3(quat,bepuik_control->string_start);
				add_v3_v3(bepuik_control->string_start,pchan->pose_mat[3]);
			}
		}
	
	delete iksolver;
	
	//The following need to be deleted in this order, or it can cause problems.	
	for(size_t i = 0; i < controls.size(); i++)
		delete controls[i];
	
	for(size_t i = 0; i < joints.size(); i++)
		delete joints[i];
	
	for(size_t i = 0; i < ikbones.size(); i++)
		delete ikbones[i];
}

void bepu_end(Object *ob)
{
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		bepu_restore_locrotsize(pchan);
		
		BEPUikTempSolvingData * bepuik_temp_solving_data = (BEPUikTempSolvingData *)pchan->bepuik;
		BOOST_FOREACH(ControlledToTarget * controlled_to_target, bepuik_temp_solving_data->controlled_to_target_info)
		{
			delete controlled_to_target;
		}

		delete bepuik_temp_solving_data;
		pchan->bepuik = NULL;
	}
}


#ifdef __cplusplus
}
#endif
