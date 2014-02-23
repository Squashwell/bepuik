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
