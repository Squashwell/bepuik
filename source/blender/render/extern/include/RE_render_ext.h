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
 * The Original Code is Copyright (C) 2006 by Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL/BL DUAL LICENSE BLOCK *****
 */
/** \file RE_render_ext.h
 *  \ingroup render
 */


#ifndef __RE_RENDER_EXT_H__
#define __RE_RENDER_EXT_H__

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* this include is for non-render pipeline exports (still old cruft here) */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* called by meshtools */
struct DerivedMesh;
struct ImagePool;
struct MTex;
struct Scene;

/* particle.c, effect.c, editmesh_modes.c and brush.c, returns 1 if rgb, 0 otherwise */
int	externtex(struct MTex *mtex, const float vec[3], float *tin, float *tr, float *tg, float *tb, float *ta, const int thread, struct ImagePool *pool, const bool skip_load_image);

/* particle.c */
void texture_rgb_blend(float in[3], const float tex[3], const float out[3], float fact, float facg, int blendtype);
float texture_value_blend(float tex, float out, float fact, float facg, int blendtype);

/* node_composite.c */
void ibuf_sample(struct ImBuf *ibuf, float fx, float fy, float dx, float dy, float result[4]);
void antialias_tagbuf(int xsize, int ysize, char *rectmove);

/* dynamicpaint.c */
struct Material *RE_init_sample_material(struct Material *orig_mat, struct Scene *scene);
void RE_free_sample_material(struct Material *mat);
void RE_sample_material_color(
        struct Material *mat, float color[3], float *alpha, const float volume_co[3], const float surface_co[3],
        int tri_index, struct DerivedMesh *orcoDm, struct Object *ob);
/* pointdensity.c */

struct PointDensity;

void RE_sample_point_density(struct Scene *scene, struct PointDensity *pd, int resolution, float *values);

void RE_init_texture_rng(void);
void RE_exit_texture_rng(void);
#endif /* __RE_RENDER_EXT_H__ */

