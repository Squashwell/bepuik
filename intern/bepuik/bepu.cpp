
#include <string.h>
#include <vector>

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
	//TODO:BEPUIK Which of these is better?
 rpos[0] = 0;
 rpos[1] = offset;
 rpos[2] = 0;
 
 mul_qt_v3(qt,rpos);
}

void bepu_print_flags(bPose * pose)
{
	printf("\n");
	if(pose->bepuikflag & POSE_BEPUIK_DYNAMIC) printf("feedback\n");
	if(pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_DRAGCONTROL) printf("dragcontrol\n");
	if(pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_STATECONTROL) printf("statecontrol\n");
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

static void pchan_bepuik_solved_match_ikbone(Object * ob, bPoseChannel * pchan, IKBone * ikbone)
{
	float bepupos[3];
	float bepuquat[4];
	v3_bepuv3(bepupos,ikbone->Position);
	qt_bepuqt(bepuquat,ikbone->Orientation);
	
	internal_bepuik_position_to_pchan_bepuik_position(pchan->bepuik_solved_position,ikbone->Position,ikbone->Orientation,ikbone->GetLength());
	copy_qt_qt(pchan->bepuik_solved_orientation,bepuquat);
	copy_v3_v3(pchan->bepuik_solved_head,pchan->bepuik_solved_position);
	
	get_offset_along_quat(pchan->bepuik_solved_tail,bepuquat,ikbone->GetLength());
	add_v3_v3(pchan->bepuik_solved_tail,pchan->bepuik_solved_position);
}






//void bepu_test_bend_stick(Object * ob)
//{
// vector <IKBone *> ikbones = vector <IKBone *>();
// vector <IKJoint *> ikjoints = vector <IKJoint *>();

// float y = .5;
// for (bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
//	{
// IKBone * ikbone = new IKBone(Vector3(0,y,0),Quaternion(0,0,0,1),.2,1);
//// ikbone->SetRadius(.2);
// ikbones.push_back(ikbone);
// y+= 1;
 
//	}
 
// y = 1;
// for(size_t i = 0; i < ikbones.size()-1; i++)
// {
// Vector3 anchorPosition = Vector3();
// ikbones[i]->GetTailPosition(anchorPosition);
 
// IKBallSocketJoint * ikballsocketjoint = new IKBallSocketJoint(ikbones[i],ikbones[i+1],ikbones[i],ikbones[i+1],Vector3(0,y,0));
// ikballsocketjoint->SetRigidity(.05f);
 
// IKAngularJoint * ikangularjoint = new IKAngularJoint(ikbones[i],ikbones[i+1]);
// ikangularjoint->SetRigidity(.05f); 
 
// ikjoints.push_back(ikballsocketjoint);
// ikjoints.push_back(ikangularjoint);
 
// y+=1;
// }

// IKBone * first_ikbone = ikbones[0];
// IKBone * last_ikbone = ikbones[ikbones.size()-1];
//// float loc[3] = {10,0,0};
//// float quat[4] = {0,0, 0.7071068286895752, -0.7071068286895752};
 
// DragControl * dragControl = new DragControl();
// dragControl->SetTargetBone(last_ikbone);
// dragControl->GetLinearMotor()->SetOffset(Vector3(0,.5,0));
// dragControl->GetLinearMotor()->TargetPosition = Vector3(10,0,0);
// dragControl->GetLinearMotor()->SetMaximumForce(MAXFLOAT);
// dragControl->GetLinearMotor()->SetRigidity(0.2);
 
//// StateControl * stateControl = new StateControl();
//// stateControl->SetTargetBone(last_ikbone);
//// stateControl->GetLinearMotor()->TargetPosition = Vector3(10,0,0);
//// stateControl->GetAngularMotor()->TargetOrientation = Quaternion(1,0,0,0);
//// stateControl->GetLinearMotor()->SetStiffnessConstant(0.2);
//// stateControl->GetLinearMotor()->SetDamping(0.2);
//// stateControl->GetAngularMotor()->SetStiffnessConstant(0.2);
//// stateControl->GetAngularMotor()->SetDamping(0.2);
 
// first_ikbone->Pinned = true;
// vector <Control *> controls = vector <Control *>();
//// controls.push_back(stateControl);
// controls.push_back(dragControl);

// IKSolver * iksolver = new IKSolver();
// iksolver->FixerIterationCount = 0;
// iksolver->ControlIterationCount = 20;
// iksolver->VelocitySubiterationCount = 3;
// iksolver->AutoscaleControlImpulses = false;
// iksolver->AutoscaleControlMaximumAngularForce = MAXFLOAT;
// iksolver->AutoscaleControlMaximumLinearForce = MAXFLOAT;
 
// for(int i = 0; i < 1000; i++)
// iksolver->Solve(controls);
 


// size_t i = 0;
// for (bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
//	{
 
// bepu_pchan_bepuik_solved_match_ikbone(ob,pchan,ikbones[i]);
// i++;
//// ikbones.push_back(bepu_ikbone_new_from_pchan(pchan));
//	}
 
// delete iksolver;
 
//// delete stateControl;
// delete dragControl;
 
// for(size_t i = 0; i < ikjoints.size(); i++)
// {
// delete ikjoints[i];
// }
 
// for(size_t i = 0; i < ikbones.size(); i++)
// {
// delete ikbones[i];
// }
//}

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

static void ikbone_match_mat_length(IKBone * ikbone, float mat[4][4], float bone_rest_length)
{	
	float vec[3];
	copy_v3_v3(vec, mat[1]);
	mul_v3_fl(vec, bone_rest_length);
	
	float bepuik_length = len_v3(vec);
	ikbone->SetLength(bepuik_length);
	ikbone->SetRadius(BEPUIK_BONE_LENGTH_TO_RADIUS(bepuik_length));
	
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
	
	Quaternion dynamic_stiffness_orientation;
};

#define BEPUIK_COPY_IMPORTANT_CHAN_DATA(destination,source) \
copy_v3_v3(destination->loc,source->loc); \
copy_v3_v3(destination->eul,source->eul); \
copy_qt_qt(destination->quat,source->quat); \
copy_v3_v3(destination->rotAxis,source->rotAxis); \
destination->rotAngle = pchan->rotAngle;



static void bepu_store_locrotsize(bPoseChannel * pchan)
{
	BEPUikTempSolvingData * bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
	BEPUIK_COPY_IMPORTANT_CHAN_DATA(bepuik,pchan);
	copy_v3_v3(bepuik->size,pchan->size);
	
		
}

static void bepu_restore_locrotsize(bPoseChannel * pchan)
{
	BEPUikTempSolvingData * bepuik = (BEPUikTempSolvingData *)pchan->bepuik;
	if(!(pchan->bepuikflag & BONE_BEPUIK_FEEDBACK))
	{
		BEPUIK_COPY_IMPORTANT_CHAN_DATA(pchan,bepuik);
	}
	//always restore the size, otherwise accumulated error makes things go bonkers
	copy_v3_v3(pchan->size,bepuik->size);
	
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


#define LOC(IDENTIFIER) bPoseChannel * pchan_##IDENTIFIER = BKE_pose_channel_find_name(ob->pose,bjoint->IDENTIFIER##_subtarget); \
Vector3 v3_##IDENTIFIER; \
if(pchan_##IDENTIFIER)	\
{ \
float bv3_##IDENTIFIER[3]; \
interp_v3_v3v3(bv3_##IDENTIFIER,pchan_##IDENTIFIER->bepuik_rest_pose_mat[3],pchan_##IDENTIFIER->bepuik_rest_tail,bjoint->IDENTIFIER##_head_tail); \
bepuv3_v3(v3_##IDENTIFIER,bv3_##IDENTIFIER); \
} else { break; }


//Can't use pose_mat for axis, because it causes instability when feedback is in use
#define AXIS(IDENTIFIER) bPoseChannel * pchan_##IDENTIFIER = BKE_pose_channel_find_name(ob->pose,bjoint->IDENTIFIER##_subtarget); \
Vector3 v3_##IDENTIFIER; \
if(pchan_##IDENTIFIER) \
{ \
float bv3_##IDENTIFIER[3]; \
mat_get_axis(bv3_##IDENTIFIER,bjoint->IDENTIFIER,pchan_##IDENTIFIER->bepuik_rest_pose_mat); \
bepuv3_v3(v3_##IDENTIFIER,bv3_##IDENTIFIER); \
} else { break; }

#define CAN_BE_TWEAK_STATECONTROL(ob,pchan) (((ob)->pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_STATECONTROL) && ((pchan)->bone->flag & BONE_TRANSFORM) && ((pchan)->bepuikflag & BONE_BEPUIK))
#define CAN_BE_TWEAK_DRAGCONTROL(ob,pchan) (((ob)->pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_DRAGCONTROL) && ((pchan)->bone->flag & BONE_TRANSFORM) && ((pchan)->bepuikflag & BONE_BEPUIK))

#define PCHAN_BEPUIK_BONE_LENGTH(pchan) (((BEPUikTempSolvingData *)((pchan)->bepuik))->ikbone->GetLength())

static void ikjoints_enable_by_partition(vector <IKJoint *> &joints)
{
	BOOST_FOREACH(IKJoint * ikjoint, joints)
	{
		if((ikjoint->GetConnectionA()->pchan->bepuikflag & BONE_BEPUIK_IN_SOLVING_PARTITION) && (ikjoint->GetConnectionB()->pchan->bepuikflag & BONE_BEPUIK_IN_SOLVING_PARTITION))
			ikjoint->SetEnabled(true);
		else
			ikjoint->SetEnabled(false);
	}	
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



static void flag_child_to_root_for_solve(bPoseChannel * pchan)
{
	if(!(pchan->bepuikflag & BONE_BEPUIK_IN_SOLVING_PARTITION))
	{
		pchan->bepuikflag |= BONE_BEPUIK_IN_SOLVING_PARTITION;
		if(pchan->parent)
		{
			flag_child_to_root_for_solve(pchan->parent);
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
	
	statecontrol->GetAngularMotor()->SetRigidity(orientation_rigidity);
	statecontrol->GetLinearMotor()->SetRigidity(position_rigidity);	
	
	return statecontrol;
}

static void setup_bepuik_target_mats(bConstraint * con, float pose_destination_mat[4][4], float pose_destination_position[3], float pose_destination_orientation[4], float bone_local_offset[3], float controlled_pose_size[3])
{
	bBEPUikTarget * bepuik_target = (bBEPUikTarget *)con->data;
	copy_m4_m4(bepuik_target->mat,pose_destination_mat);
	
	//figure out the size for the bepuik target visualization mat
	float size_mat[4][4];
	size_to_mat4(size_mat,controlled_pose_size);
	mul_m4_m4m4(bepuik_target->mat,bepuik_target->mat,size_mat);
	
	copy_v3_v3(bepuik_target->pulled_start_pose_space,bone_local_offset);
	copy_v3_v3(bepuik_target->pulled_destination_pose_space,bone_local_offset);
	mul_qt_v3(pose_destination_orientation,bepuik_target->pulled_destination_pose_space);
	add_v3_v3(bepuik_target->pulled_destination_pose_space,pose_destination_position);
	
	con->flag |= CONSTRAINT_BEPUIK_DRAWABLE;
}

static void setup_bepuik_target(Object * ob, bConstraint * constraint, IKBone * ikbone, vector <Control *> &controls)
{	
	bBEPUikTarget * bepuik_target = (bBEPUikTarget *)constraint->data;
	if(!bepuik_target->connection_target) return;
	bPoseChannel * pchan_controlled = ikbone->pchan;
		
	float local_offset[3];
	float local_offset_bepuik_internal[3];
	get_scale_applied_bone_local_offsets(local_offset,local_offset_bepuik_internal,pchan_controlled->bone->length,pchan_controlled->bepuik_rest_pose_size,bepuik_target->pulled_point);
	
	float effective_orientation_rigidity = bepuik_target->orientation_rigidity;
	float effective_position_rigidity = constraint->bepuik_rigidity;
	
	if(bepuik_target->connection_target->type == OB_ARMATURE)
	{
		bPoseChannel * pchan_target = BKE_pose_channel_find_name(bepuik_target->connection_target->pose,bepuik_target->connection_subtarget);
		if(!pchan_target) return;
		
		if(bepuik_target->connection_target == ob) //the target pchan is in the same armature
		{			
			float mat[4][4];
	
			copy_m4_m4(mat,pchan_controlled->bone->arm_mat);
			invert_m4(mat);
			
			float arm_controlled_to_arm_target[4][4];
			mul_m4_m4m4(arm_controlled_to_arm_target,mat,pchan_target->bone->arm_mat);
			
			float target_no_scale_bepuik_rest[4][4];
			float target_inverse_no_scale_bepuik_rest[4][4];
			unit_m4(target_no_scale_bepuik_rest);
			mul_m4_m4m4(target_no_scale_bepuik_rest,pchan_controlled->bepuik_rest_pose_mat,arm_controlled_to_arm_target);
			normalize_m4(target_no_scale_bepuik_rest);
			invert_m4_m4(target_inverse_no_scale_bepuik_rest,target_no_scale_bepuik_rest);
			
			float controlled_no_scale_bepuik_rest_mat[4][4];
			normalize_m4_m4(controlled_no_scale_bepuik_rest_mat,pchan_controlled->bepuik_rest_pose_mat);
			
			ControlledToTarget * controlled_to_target = new ControlledToTarget;
			controlled_to_target->pchan_controlled = pchan_controlled;
			
			float controlled_no_scale_inverse_bepuik_rest_mat[4][4];
			invert_m4_m4(controlled_no_scale_inverse_bepuik_rest_mat,controlled_no_scale_bepuik_rest_mat);
			mul_m4_m4m4(controlled_to_target->controlled_to_target_mat,controlled_no_scale_inverse_bepuik_rest_mat,target_no_scale_bepuik_rest);
			
			BEPUikTempSolvingData * pchan_target_bepuik = (BEPUikTempSolvingData *)pchan_target->bepuik;
			pchan_target_bepuik->controlled_to_target_info.push_back(controlled_to_target);
			
			float target_to_control_bepuik_rest[4][4];
			mul_m4_m4m4(target_to_control_bepuik_rest,target_inverse_no_scale_bepuik_rest,controlled_no_scale_bepuik_rest_mat);
			
			float absolute_destination_mat[4][4];
			float normalized_target_pose_mat[4][4];
			normalize_m4_m4(normalized_target_pose_mat,pchan_target->pose_mat);
			mul_m4_m4m4(absolute_destination_mat,normalized_target_pose_mat,target_to_control_bepuik_rest);
			
			float destination_position[3];
			float destination_orientation[4];
			mat4_to_loc_quat(destination_position,destination_orientation,absolute_destination_mat);

			setup_bepuik_target_mats(constraint,absolute_destination_mat,destination_position,destination_orientation,local_offset,pchan_controlled->bepuik_rest_pose_size);
			

			//if an absolute target was previously created for this bone, then we dont need to create any other targets
			if(pchan_controlled->bepuikflag & BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET) return;

			
			if(bepuik_target->bepuikflag & BEPUIK_CONSTRAINT_ABSOLUTE)
			{
				pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET;
				
				copy_v3_v3(pchan_controlled->bepuik_absolute_controlled_position,destination_position);
				copy_qt_qt(pchan_controlled->bepuik_absolute_controlled_orientation,destination_orientation);
				
				effective_orientation_rigidity += 10.0f;
				effective_position_rigidity += 1000.0f;
			}
			
			if(effective_orientation_rigidity >= FLT_EPSILON)
			{
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
			}
			
			if((effective_position_rigidity >= FLT_EPSILON) || (effective_orientation_rigidity >= FLT_EPSILON))
			{
				if(pchan_target)
					pchan_target->bepuikflag |= BONE_BEPUIK_IS_ACTIVE_BEPUIK_TARGET;
				
				StateControl * statecontrol = new_statecontrol(ikbone,local_offset_bepuik_internal,bepuik_target->pulled_destination_pose_space,destination_orientation,effective_position_rigidity,effective_orientation_rigidity);
				
				controls.push_back(statecontrol);
			}

		}
		else //targeting a bone in a different armature
		{
			float destination_mat[4][4];
			float destination_position[3];
			float destination_orientation[4];
	//		BKE_armature_mat_world_to_pose(ob,bepuik_target->connection_target->obmat,destination_mat); this function is exactly opposite of what it says it does???
			

			mul_m4_m4m4(destination_mat, bepuik_target->connection_target->obmat, pchan_target->pose_mat);
			
			float imat[4][4];
			invert_m4_m4(imat, ob->obmat);
			
			mul_m4_m4m4(destination_mat,imat,destination_mat);
			
			
			normalize_m4(destination_mat);
	
			mat4_to_loc_quat(destination_position,destination_orientation,destination_mat);
	
			setup_bepuik_target_mats(constraint,destination_mat,destination_position,destination_orientation,local_offset,pchan_controlled->bepuik_rest_pose_size);
			
			//if an absolute target was previously created for this bone, then we dont need to create any other targets
			if(pchan_controlled->bepuikflag & BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET) return;
			
			if(bepuik_target->bepuikflag & BEPUIK_CONSTRAINT_ABSOLUTE)
			{
				pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET;
				
				copy_v3_v3(pchan_controlled->bepuik_absolute_controlled_position,destination_position);
				copy_qt_qt(pchan_controlled->bepuik_absolute_controlled_orientation,destination_orientation);
				
				effective_orientation_rigidity += 10.0f;
				effective_position_rigidity += 1000.0f;
			}
			
			if(effective_orientation_rigidity >= FLT_EPSILON)
			{
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
			}
			
			if((effective_position_rigidity >= FLT_EPSILON) || (effective_orientation_rigidity >= FLT_EPSILON))
			{
				
				StateControl * statecontrol = new_statecontrol(ikbone,local_offset_bepuik_internal,bepuik_target->pulled_destination_pose_space,destination_orientation,effective_position_rigidity,effective_orientation_rigidity);
				
				controls.push_back(statecontrol);
			}
		}
	}
	else //targeting a different object completely, use objects worldspace to drive the target
	{
		
		float destination_mat[4][4];
		float destination_position[3];
		float destination_orientation[4];
//		BKE_armature_mat_world_to_pose(ob,bepuik_target->connection_target->obmat,destination_mat); this function is exactly opposite of what it says it does???
		
		float imat[4][4];
		invert_m4_m4(imat, ob->obmat);
		mul_m4_m4m4(destination_mat, imat, bepuik_target->connection_target->obmat);
		
		normalize_m4(destination_mat);

		mat4_to_loc_quat(destination_position,destination_orientation,destination_mat);

		setup_bepuik_target_mats(constraint,destination_mat,destination_position,destination_orientation,local_offset,pchan_controlled->bepuik_rest_pose_size);
		
		//if an absolute target was previously created for this bone, then we dont need to create any other targets
		if(pchan_controlled->bepuikflag & BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET) return;
		
		if(bepuik_target->bepuikflag & BEPUIK_CONSTRAINT_ABSOLUTE)
		{
			pchan_controlled->bepuikflag |= BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET;
			
			copy_v3_v3(pchan_controlled->bepuik_absolute_controlled_position,destination_position);
			copy_qt_qt(pchan_controlled->bepuik_absolute_controlled_orientation,destination_orientation);
			
			effective_orientation_rigidity += 10.0f;
			effective_position_rigidity += 1000.0f;
		}
		
		if(effective_orientation_rigidity >= FLT_EPSILON)
		{
			ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
		}
		
		if((effective_position_rigidity >= FLT_EPSILON) || (effective_orientation_rigidity >= FLT_EPSILON))
		{
			
			StateControl * statecontrol = new_statecontrol(ikbone,local_offset_bepuik_internal,bepuik_target->pulled_destination_pose_space,destination_orientation,effective_position_rigidity,effective_orientation_rigidity);
			
			controls.push_back(statecontrol);
		}
		
	}
}

void bepu_solve(Scene * scene, Object * ob,float ctime) 
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
		mul_m4_m4m4(pchan->bepuik_prev_pose_mat,pchan->constinv,pchan->pose_mat);
		
		size_to_mat4(pchan->chan_mat, pchan->size);
		BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
		
		copy_v3_v3(pchan->pose_head, pchan->pose_mat[3]);
		BKE_pose_where_is_bone_tail(pchan);
		
		copy_m4_m4(pchan->bepuik_rest_pose_mat,pchan->pose_mat);
		copy_v3_v3(pchan->bepuik_rest_tail,pchan->pose_tail);
		mat4_to_size(pchan->bepuik_rest_pose_size,pchan->bepuik_rest_pose_mat);
		
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
			ikbone_match_mat_length(ikbone,pchan->bepuik_rest_pose_mat,pchan->bone->length);
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
		pchan->bepuikflag |= BONE_BEPUIK_HAS_PREPOSE;
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
			if(constraint->type != CONSTRAINT_TYPE_BEPUIK_TARGET) continue;
			if(constraint->flag & (CONSTRAINT_DISABLE|CONSTRAINT_OFF)) continue;
			
			bBEPUikTarget * bepuik_target = (bBEPUikTarget *)constraint->data;
			if (!(bepuik_target->bepuikflag & BEPUIK_CONSTRAINT_ABSOLUTE)) continue;
			
			setup_bepuik_target(ob,constraint,ikbone,controls);
		}
		
		//create constraints but exclude bepuik targets if there was an absolute target found earlier
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
				
				ikjoint = new IKAngularJoint(ikbone,ikbone_connection);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_BALL_SOCKET_JOINT:
			{
				CONNECTION_VARS(bBEPUikBallSocketJoint)
				LOC(anchor)
						
				ikjoint = new IKBallSocketJoint(ikbone,ikbone_connection,v3_anchor);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_DISTANCE_JOINT:
			{
				CONNECTION_VARS(bBEPUikDistanceJoint)
				LOC(anchor_a)
				LOC(anchor_b)

				ikjoint = new IKDistanceJoint(ikbone,ikbone_connection,v3_anchor_a,v3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_DISTANCE_LIMIT:
			{
				CONNECTION_VARS(bBEPUikDistanceLimit)
				LOC(anchor_a)
				LOC(anchor_b)
				
				ikjoint = new IKDistanceLimit(ikbone,ikbone_connection,v3_anchor_a,v3_anchor_b,bjoint->min_distance,bjoint->max_distance);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_LINEAR_AXIS_LIMIT:
			{
				CONNECTION_VARS(bBEPUikLinearAxisLimit)
				LOC(line_anchor)
				AXIS(line_direction)
				LOC(anchor_b)
				
				ikjoint = new IKLinearAxisLimit(ikbone,ikbone_connection,v3_line_anchor,v3_line_direction,v3_anchor_b,bjoint->min_distance,bjoint->max_distance);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_POINT_ON_LINE_JOINT:
			{
				CONNECTION_VARS(bBEPUikPointOnLineJoint)
				LOC(line_anchor)
				AXIS(line_direction)
				LOC(anchor_b)
				
				ikjoint = new IKPointOnLineJoint(ikbone,ikbone_connection,v3_line_anchor,v3_line_direction,v3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_POINT_ON_PLANE_JOINT:
			{
				CONNECTION_VARS(bBEPUikPointOnPlaneJoint)
				LOC(plane_anchor)
				LOC(anchor_b)
				AXIS(plane_normal)
				
				ikjoint = new IKPointOnPlaneJoint(ikbone,ikbone_connection,v3_plane_anchor,v3_plane_normal,v3_anchor_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_REVOLUTE_JOINT:
			{
				CONNECTION_VARS(bBEPUikRevoluteJoint)
				AXIS(free_axis)
				
				ikjoint = new IKRevoluteJoint(ikbone,ikbone_connection,v3_free_axis);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_SWING_LIMIT:
			{
				CONNECTION_VARS(bBEPUikSwingLimit)
				AXIS(axis_a)
				AXIS(axis_b)
				
				ikjoint = new IKSwingLimit(ikbone,ikbone_connection,v3_axis_a,v3_axis_b,bjoint->max_swing);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_SWIVEL_HINGE_JOINT:
			{
				CONNECTION_VARS(bBEPUikSwivelHingeJoint)
				AXIS(hinge_axis)
				AXIS(twist_axis)
				
				ikjoint = new IKSwivelHingeJoint(ikbone,ikbone_connection,v3_hinge_axis,v3_twist_axis);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_TWIST_JOINT:
			{
				CONNECTION_VARS(bBEPUikTwistJoint)
				AXIS(axis_a)
				AXIS(axis_b)
				
				ikjoint = new IKTwistJoint(ikbone,
				ikbone_connection,v3_axis_a,v3_axis_b);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_TWIST_LIMIT:
			{
				CONNECTION_VARS(bBEPUikTwistLimit)
				AXIS(axis_a)
				AXIS(axis_b)
				AXIS(measurement_axis_a)
				AXIS(measurement_axis_b)
				
				ikjoint = new IKTwistLimit(ikbone,ikbone_connection,v3_axis_a,v3_measurement_axis_a,v3_axis_b,v3_measurement_axis_b,bjoint->max_twist);
				break;
			}
			case CONSTRAINT_TYPE_BEPUIK_TARGET:
			{
				bBEPUikTarget * bepuik_target = (bBEPUikTarget *)constraint->data;
				if (bepuik_target->bepuikflag & BEPUIK_CONSTRAINT_ABSOLUTE) break;
				
				setup_bepuik_target(ob,constraint,ikbone,controls);
				break;
			}
			}
	
			
			if(ikjoint)
			{
				ikjoint->SetRigidity(constraint->bepuik_rigidity);
				ikjoint->constraint = constraint;
				joints.push_back(ikjoint);
			}
		}
		
		//check for auto ball socket joint
		if(pchan->parent && pchan_get_bepuik_bone(pchan->parent) && pchan->bepuik_ball_socket_rigidity >= FLT_EPSILON)
		{
			IKBone * child_connection = ikbone;
			IKBone * parent_connection = pchan_get_bepuik_bone(pchan->parent);
			
			Vector3 bepuv3 = Vector3();
			bepuv3_v3(bepuv3,pchan->bepuik_rest_pose_mat[3]);
			IKJoint * auto_ballsocket = new IKBallSocketJoint(child_connection,parent_connection,bepuv3);
			auto_ballsocket->SetRigidity(pchan->bepuik_ball_socket_rigidity);
			joints.push_back(auto_ballsocket);
		}
		
		if(pchan->bone->flag & BONE_TRANSFORM) 
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
			}
			else if(ob->pose->bepuikflag & POSE_BEPUIK_SELECTION_AS_STATECONTROL) 
			{
				StateControl * statecontrol = new StateControl();
				statecontrol->SetTargetBone(ikbone);
				statecontrol->GetLinearMotor()->SetRigidity(ob->bepuik_dynamic_position_rigidity);
				statecontrol->GetAngularMotor()->SetRigidity(ob->bepuik_dynamic_orientation_rigidity);
				
				ikbone->SetInertiaTensorScaling(BEPUIK_INTERTIA_TENSOR_SCALING_MIN);
				
				float quat[4];
				
				normalize_qt_qt(quat,pchan->bepuik_transform_orientation);
				bepuqt_qt(statecontrol->GetAngularMotor()->TargetOrientation,quat);
				pchan_bepuik_position_to_internal_bepuik_position(statecontrol->GetLinearMotor()->TargetPosition,pchan->bepuik_transform_position,quat,PCHAN_BEPUIK_BONE_LENGTH(pchan));
				controls.push_back(statecontrol);
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
	iksolver->AutoscaleControlMaximumLinearForce = MAXFLOAT;
	iksolver->AutoscaleControlMaximumAngularForce = MAXFLOAT;
	iksolver->VelocitySubiterationCount = ob->bepuik_velocity_subiterations;
	

	

		
	vector <Control *> controls_to_solve = vector <Control *>();
	
	BOOST_FOREACH(Control * control, controls)
	{
		IKBone * target_ikbone = (IKBone *)control->GetTargetBone();
		if(!target_ikbone->Pinned)
		{
			controls_to_solve.push_back(control);
			flag_child_to_root_for_solve(target_ikbone->pchan);
		}
		
	}
	
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		if(pchan->bepuikflag & BONE_BEPUIK_ALWAYS_SOLVE)
			flag_child_to_root_for_solve(pchan);
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
			if(pchan->parent && !(pchan->bepuikflag & BONE_BEPUIK_IN_SOLVING_PARTITION)) {
				IKBone * child = pchan_get_bepuik_bone(pchan);
				if(!child) continue;
				IKBone * parent = pchan_get_bepuik_bone(pchan->parent);
				if(!parent) continue;
				
				BEPUikTempSolvingData * bepuik = (BEPUikTempSolvingData*)pchan->bepuik;
				
				if(ob->pose->bepuikflag & POSE_BEPUIK_UPDATE_DYNAMIC_STIFFNESS_MAT)
				{										
					mat4_to_quat(qa,pchan->parent->bepuik_prev_pose_mat);
					mat4_to_quat(qb,pchan->bepuik_prev_pose_mat);
					
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
				ikangularjoint->SetRigidity(ob->bepuik_dynamic_peripheral_stiffness);
				joints.push_back(ikangularjoint);
			}
		}
	}
	
	ob->pose->bepuikflag &= ~POSE_BEPUIK_UPDATE_DYNAMIC_STIFFNESS_MAT;
	
	if(ob->bepuikflag & OB_BEPUIK_PARENTED_PERIPHERAL_BONES)
	{
		ikjoints_enable_by_partition(joints);
	}
	
	//Finally, after all constraints have been built based on the ikbones at their "bepuik rest pose," 
	//teleport the bones to the positions from which they will be solved.
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{					
		IKBone * ikbone = pchan_get_bepuik_bone(pchan);
		if(ikbone)
		{
			if(pchan->bepuikflag & BONE_BEPUIK_AFFECTED_BY_ABSOLUTE_TARGET)
			{
				pchan_bepuik_position_to_internal_bepuik_position(ikbone->Position,pchan->bepuik_absolute_controlled_position,pchan->bepuik_absolute_controlled_orientation,ikbone->GetLength());
				bepuqt_qt(ikbone->Orientation,pchan->bepuik_absolute_controlled_orientation);
			}
			else
			{
				if(ob->pose->bepuikflag & POSE_BEPUIK_DYNAMIC)
				{
					ikbone_position_orientation_match_mat(ikbone,pchan->bepuik_prev_pose_mat);
				}
				else
				{
					ikbone_position_orientation_match_mat(ikbone,pchan->pose_mat);
				}
			}
		}
	}
	
	iksolver->Solve(controls_to_solve);
	
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
	{
		IKBone * ikbone = pchan_get_bepuik_bone(pchan);
		
		if(ikbone && ikbone->IsActive)
		{
			pchan_bepuik_solved_match_ikbone(ob,pchan,ikbone);
			loc_quat_size_to_mat4(pchan->pose_mat,pchan->bepuik_solved_position,pchan->bepuik_solved_orientation,pchan->bepuik_rest_pose_size);
			BKE_armature_mat_pose_to_bone(pchan,pchan->pose_mat,pchan->chan_mat);
			BKE_pchan_apply_mat4(pchan,pchan->chan_mat,FALSE);
		
			if(ob->pose->bepuikflag & POSE_BEPUIK_DYNAMIC)
				pchan->bepuikflag |= BONE_BEPUIK_FEEDBACK;
		}
		else
		{
			BKE_pchan_calc_mat(pchan);
			BKE_armature_mat_bone_to_pose(pchan,pchan->chan_mat,pchan->pose_mat);
		}
	}
	
	if(ob->pose->bepuikflag & POSE_BEPUIK_INACTIVE_TARGETS_FOLLOW)
	{
		for(bPoseChannel * pchan_target = (bPoseChannel *)ob->pose->chanbase.first; pchan_target; pchan_target = pchan_target->next)
		{
			if(!(pchan_target->bepuikflag & BONE_BEPUIK_IS_ACTIVE_BEPUIK_TARGET) && !(pchan_target->bone->flag & BONE_TRANSFORM))
			{
				BEPUikTempSolvingData * pchan_target_bepuik = (BEPUikTempSolvingData *)pchan_target->bepuik;
			
				float average_position[3];
				zero_v3(average_position);
				
				float average_orientation[4];
				zero_v4(average_orientation);
				
				BOOST_FOREACH(ControlledToTarget * controlled_to_target, pchan_target_bepuik->controlled_to_target_info)
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
				
				
				size_t size = pchan_target_bepuik->controlled_to_target_info.size();
				
				if(size>1)
				{
					mul_v3_fl(average_position,1.0f/(float)size);
					normalize_qt(average_orientation);
				}
				
				if(size > 0)
				{					
					loc_quat_size_to_mat4(pchan_target->pose_mat,average_position,average_orientation,pchan_target->bepuik_rest_pose_size);
					BKE_armature_mat_pose_to_bone(pchan_target,pchan_target->pose_mat,pchan_target->chan_mat);
					BKE_pchan_apply_mat4(pchan_target,pchan_target->chan_mat,FALSE);
					
					pchan_target->bepuikflag |= BONE_BEPUIK_FEEDBACK;
				}
							
			}
		}
	}
			
	for(bPoseChannel * pchan = (bPoseChannel *)ob->pose->chanbase.first; pchan; pchan = pchan->next)
		for(bConstraint * constraint = (bConstraint *)pchan->constraints.first; constraint; constraint = constraint->next)
		{
			if(constraint->type == CONSTRAINT_TYPE_BEPUIK_TARGET)
			{
				
				bBEPUikTarget * bepuik_target_constraint = (bBEPUikTarget *)constraint->data;
				bPoseChannel * pchan_target = BKE_pose_channel_find_name(ob->pose,bepuik_target_constraint->connection_subtarget);
				
				if(pchan_target)
				{
					float quat[4];
					mat4_to_quat(quat,pchan->pose_mat);
					
					//by now, pchan_pulled_point is the scale applied local offset
					mul_qt_v3(quat,bepuik_target_constraint->pulled_start_pose_space);
					add_v3_v3(bepuik_target_constraint->pulled_start_pose_space,pchan->pose_mat[3]);
					
				}
			}
		}
	
	delete iksolver;
	
	//The following need to be deleted in this order, or it can cause problems.	
	for(size_t i = 0; i < controls.size(); i++)
	{
		delete controls[i];
	}
	
	for(size_t i = 0; i < joints.size(); i++)
		delete joints[i];
	
	for(size_t i = 0; i < ikbones.size(); i++)
	{
		delete ikbones[i];
	}	
}

void bepu_end(Scene *scene, Object *ob, float ctime)
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
