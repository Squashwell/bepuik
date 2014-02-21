#ifndef BEPU_H
#define BEPU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_action_types.h"
#include "DNA_anim_types.h"

void bepu_solve(Scene * scene, Object * ob, float ctime);

void bepu_end(Scene * scene, Object * ob, float ctime);

#define BEPUIK_BONE_LENGTH_TO_RADIUS(length) ((length)/5.0f)
#define BEPUIK_DEFAULT_BONE_RADIUS 0.2f
#define BEPUIK_TARGET_CONSTRAINT_GET_PCHAN_TARGET(bepuik_target_constraint,ob) BEPUIK_CONSTRAINT_HAS_VALID_POSEOB((bepuik_target_constraint),(ob)) ? BKE_pose_channel_find_name((ob)->pose,(bepuik_target_constraint)->connection_subtarget) : NULL
#ifdef __cplusplus
}
#endif

#endif /* BEPU_H */
