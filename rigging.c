#include "rigging.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>

/************************************************************************/
/* rg_pose_srt                                                          */
/************************************************************************/

static inline void
_rotate_vector(float* src, float rad, float* dst) {
	float s = sin(rad), c = cos(rad);
	dst[0] = src[0] * c - src[1] * s;
	dst[1] = src[0] * s + src[1] * c;
}

void 
rg_pose_srt_identity(struct rg_pose_srt* pos) {
	pos->trans[0] = 0;
	pos->trans[1] = 0;
	pos->rot      = 0;
	pos->scale[0] = 1;
	pos->scale[1] = 1;
}

void
rg_local2world(const struct rg_pose_srt* src, const struct rg_pose_srt* local, struct rg_pose_srt* dst) {
	// scale
	dst->scale[0] = src->scale[0] * local->scale[0];
	dst->scale[1] = src->scale[1] * local->scale[1];
	// rot
	dst->rot      = src->rot + local->rot;
	// trans
	// dst.trans  = src.trans + sm::rotate_vector(local.trans * src.scale, src.rot);
	float tmp[2]  = { local->trans[0] * src->scale[0], local->trans[1] * src->scale[1] };
	_rotate_vector(tmp, src->rot, dst->trans);
	dst->trans[0] += src->trans[0];
	dst->trans[1] += src->trans[1];
}

/************************************************************************/
/* rg_pose_mat                                                          */
/************************************************************************/

void 
rg_pose_mat_identity(struct rg_pose_mat* pose) {
	memset(pose->m, 0, sizeof(float) * 6);
	pose->m[0] = pose->m[3] = 1;
}

void
_build(float* m, float x, float y, float angle, float sx, float sy) {
	float c = cos(angle), s = sin(angle);

	m[0] = c * sx;
	m[1] = s * sx;
	m[2] = - s * sy;
	m[3] = c * sy;
	m[4] = x;
	m[5] = y;
}

void 
rg_pose_mat_build(struct rg_pose_mat* dst, const struct rg_pose_srt* src) {
	_build(dst->m, src->trans[0], src->trans[1], src->rot, src->scale[0], src->scale[1]);
}

static inline void
_mul(float* m, const float* m1, const float* m2) {
	m[0] = m1[0] * m2[0] + m1[1] * m2[2];
	m[1] = m1[0] * m2[1] + m1[1] * m2[3];
	m[2] = m1[2] * m2[0] + m1[3] * m2[2];
	m[3] = m1[2] * m2[1] + m1[3] * m2[3];
	m[4] = m1[4] * m2[0] + m1[5] * m2[2] + m2[4];
	m[5] = m1[4] * m2[1] + m1[5] * m2[3] + m2[5];
}

void 
rg_local2worldmat(const struct rg_pose_mat* src, const struct rg_pose_srt* local, struct rg_pose_mat* dst) {
	float m[6];
	_build(m, local->trans[0], local->trans[1], local->rot, local->scale[0], local->scale[1]);

	_mul(dst->m, m, src->m);
}

/************************************************************************/
/* rg_joint                                                             */
/************************************************************************/

void 
rg_joint_update(struct rg_joint* joint, struct rg_skeleton* sk) {
	if (joint->parent != 0xff) {
		assert(joint->parent < sk->joint_count);
		struct rg_joint* parent = sk->joints[joint->parent];
		rg_local2worldmat(&parent->world_pose, &joint->local_pose, &joint->world_pose);
	}
	for (int i = 0; i < joint->children_count; ++i) {
		assert(joint->children[i] < sk->joint_count);
		struct rg_joint* child = sk->joints[joint->children[i]];
		rg_joint_update(child, sk);
	}
}

/************************************************************************/
/* rg_skeleton_pose                                                     */
/************************************************************************/

static void 
_update_joint(struct rg_skeleton_pose* pose, const struct rg_skeleton* sk, int joint_idx) {
	assert(joint_idx >= 0 && joint_idx < sk->joint_count);
	struct rg_joint* joint = sk->joints[joint_idx];
	if (joint->parent != 0xff) {
		assert(joint->parent < sk->joint_count);
		rg_local2worldmat(&pose->poses[joint->parent].world, &pose->poses[joint_idx].local, &pose->poses[joint_idx].world);
	}
	for (int i = 0; i < joint->children_count; ++i) {
		_update_joint(pose, sk, joint->children[i]);
	}
}

void 
rg_skeleton_pose_update(struct rg_skeleton_pose* pose, const struct rg_skeleton* sk, const struct rg_tl_joint** joints, int time) {
	uint64_t dims_ptr = 0;
	for (int i = 0; i < sk->joint_count; ++i) {
		struct rg_joint* joint = sk->joints[i];
		if (joints[i]) {
			rg_pose_srt_identity(&pose->poses[i].local);
			rg_pose_mat_identity(&pose->poses[i].world);

			struct rg_tl_joint_state state;
			rg_tl_query_joint(joints[i], time, &dims_ptr, &state);

			pose->poses[i].local.trans[0] = joint->local_pose.trans[0] + state.trans[0];
			pose->poses[i].local.trans[1] = joint->local_pose.trans[1] + state.trans[1];
			pose->poses[i].local.rot      = joint->local_pose.rot + state.rot;
			pose->poses[i].local.scale[0] = joint->local_pose.scale[0] * state.scale[0];
			pose->poses[i].local.scale[1] = joint->local_pose.scale[1] * state.scale[1];
		} else {
			pose->poses[i].local = joint->local_pose;
		}
	}

	struct rg_pose_pair* root = &pose->poses[sk->root];
	rg_pose_mat_build(&root->world, &root->local);

	_update_joint(pose, sk, sk->root);
}

/************************************************************************/
/* rg_skeleton_skin                                                     */
/************************************************************************/

static void (*UPDATE_SKIN_FUNC)(void* sym, const struct rg_skeleton_pose*);
static void (*UPDATE_MESH_FUNC)(void* sym, const struct rg_tl_deform_state*, const float*);

void 
rg_skeleton_skin_init(void (*update_skin_func)(void* sym, const struct rg_skeleton_pose*),
					  void (*update_mesh_func)(void* sym, const struct rg_tl_deform_state*, const float*)) {
						  UPDATE_SKIN_FUNC = update_skin_func;
						  UPDATE_MESH_FUNC = update_mesh_func;
}

void 
rg_skeleton_skin_update(struct rg_skeleton_skin* ss, const struct rg_skeleton* sk, const struct rg_skeleton_pose* sk_pose, const struct rg_tl_skin** ts, const struct rg_tl_deform** td, int time) {
	for (int i = 0; i < sk->slot_count; ++i) {
		uint16_t skin = RG_SKIN_UNKNOWN;
		if (ts[i]) {
			skin = rg_tl_query_skin(ts[i], time);
		} else {
			skin = sk->slots[i].skin;
		}
		ss->skins[i] = skin;

		if (skin != RG_SKIN_UNKNOWN && skin != RG_SKIN_NULL) {
			void* sym = sk->skins[skin].ud;
			UPDATE_SKIN_FUNC(sym, sk_pose);

			if (sk->skins[skin].type == SKIN_MESH && td[skin]) {
				struct rg_tl_deform_state deform_state;
				const float* vertices = rg_tl_query_deform(td[skin], time, &deform_state);
				UPDATE_MESH_FUNC(sym, &deform_state, vertices);
			}
		}
	}
}

/************************************************************************/
/* rg_skeleton                                                          */
/************************************************************************/

static void (*RENDER_FUNC)(void* sym, float* mat, const void* ud);

void 
rg_skeleton_init(void (*render_func)(void* sym, float* mat, const void* ud)) {
	RENDER_FUNC = render_func;
}

void 
rg_skeleton_draw(const struct rg_skeleton* sk, const struct rg_skeleton_pose* pose, const struct rg_skeleton_skin* ss, const void* ud) {
	for (int i = 0; i < sk->slot_count; ++i) {
		const struct rg_slot* slot = &sk->slots[i];
		uint16_t skin_idx = RG_SKIN_UNKNOWN;
		if (ss->skins[i]) {
			skin_idx = ss->skins[i];
		} 
		if (skin_idx == RG_SKIN_UNKNOWN) {
			skin_idx = slot->skin;
		}
		if (skin_idx == RG_SKIN_UNKNOWN || skin_idx == RG_SKIN_NULL) {
			continue;
		}

		const struct rg_skin* skin = &sk->skins[skin_idx];
		assert(skin->ud);

		struct rg_pose_mat world;
		if (skin->type != SKIN_JOINT_MESH) {
			rg_local2worldmat(&pose->poses[slot->joint].world, &skin->local, &world);
		} else {
			rg_pose_mat_identity(&world);
		}

		RENDER_FUNC(skin->ud, world.m, ud);
	}
}

/************************************************************************/
/* rg_timeline                                                          */
/************************************************************************/

#define MESH_BUF_SIZE 1024

static float* MESH_BUF = NULL;

void 
rg_timeline_init() {
	int sz = sizeof(float) * MESH_BUF_SIZE;
	MESH_BUF = malloc(sz);
	memset(MESH_BUF, 0, sz);
}

static inline float
_format_angle(float angle) {
	const float PI = 3.1415926f;
	if (angle > PI) {
		angle -= PI * 2;
	}
	if (angle < -PI) {
		angle += PI * 2;
	}
	return angle;
}

static inline float
_float_lerp(uint16_t time_begin, uint16_t time_end, float begin, float end, uint16_t time) {
	return (time - time_begin) * (end - begin) / (time_end - time_begin) + begin;
}

static inline bool
_query_joint(const struct rg_joint_sample* samples, int sample_count, int time, bool is_angle, uint8_t* ptr, float* ret) {
	assert(sample_count > 0);

	if (time < samples[0].time) {
		*ptr = 0;
		return false;
	}
	if (sample_count == 1 || time > samples[sample_count - 1].time) {
		*ptr = 0;
		*ret = samples[sample_count - 1].data;
		return true;
	}

	*ptr = 0;

	int curr = 0, next = 1;
	while (true)
	{
		if (samples[curr].time <= time && time <= samples[next].time) {
			const struct rg_joint_sample* c = &samples[curr];
			const struct rg_joint_sample* n = &samples[next];
			float cd, nd;
			if (is_angle) {
				cd = _format_angle(c->data);
				nd = _format_angle(n->data);
			} else {
				cd = c->data;
				nd = n->data;
			}
			*ret = _float_lerp(c->time, n->time, cd, nd, time);
			return true;
		} else {
			++curr;
			++next;
			if (next >= sample_count) {
				break;
			}
		}
	}

	return false;
}

void 
rg_tl_query_joint(const struct rg_tl_joint* joint, int time, uint64_t* dims_ptr, struct rg_tl_joint_state* state) {
	memset(state, 0, sizeof(*state));
	state->scale[0] = state->scale[1] = 1;

	int ptr_dims = 0, ptr_sample = 0;
	uint64_t old_dims_ptr = *dims_ptr;
	uint64_t new_dims_ptr = 0;
	if (joint->type & DIM_FLAG_TRANS_X) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_TRANS_X], time, false, &dim_ptr, &state->trans[0]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_TRANS_X];
	}
	if (joint->type & DIM_FLAG_TRANS_Y) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_TRANS_Y], time, false, &dim_ptr, &state->trans[1]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_TRANS_Y];
	}
	if (joint->type & DIM_FLAG_ROT) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_ROT], time, true, &dim_ptr, &state->rot);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_ROT];
	}
	if (joint->type & DIM_FLAG_SCALE_X) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_SCALE_X], time, false, &dim_ptr, &state->scale[0]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_SCALE_X];
	}
	if (joint->type & DIM_FLAG_SCALE_Y) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_SCALE_Y], time, false, &dim_ptr, &state->scale[1]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_SCALE_Y];
	}
	if (joint->type & DIM_FLAG_SHEAR_X) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_SHEAR_X], time, false, &dim_ptr, &state->shear[0]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_SHEAR_X];
	}
	if (joint->type & DIM_FLAG_SHEAR_Y) {
		uint8_t dim_ptr = (old_dims_ptr >> (ptr_dims * 8)) & 0xff;
		_query_joint(&joint->samples[ptr_sample], joint->dims_count[DIM_IDX_SHEAR_Y], time, false, &dim_ptr, &state->shear[1]);
		new_dims_ptr = (new_dims_ptr << 8) & dim_ptr;
		++ptr_dims;
		ptr_sample += joint->dims_count[DIM_IDX_SHEAR_Y];
	}
	*dims_ptr = new_dims_ptr;
}

uint16_t 
rg_tl_query_skin(const struct rg_tl_skin* skin, int time) {
	assert(skin && skin->skins && skin->skin_count > 0);

	if (time < skin->skins[0].time) {
		return RG_SKIN_UNKNOWN;
	}
	if (skin->skin_count == 1 || time >= skin->skins[skin->skin_count - 1].time) {
		return skin->skins[skin->skin_count - 1].skin;
	}

	int curr = 0, next = 1;
	while (true)
	{
		if (skin->skins[curr].time <= time && time < skin->skins[next].time) {
			return skin->skins[curr].skin;
		} else {
			++curr;
			++next;
			if (next >= skin->skin_count) {
				break;
			}
		}
	}

	return RG_SKIN_UNKNOWN;
}

static void 
_query_deform(const struct rg_tl_deform* deform, int time, const struct rg_deform_sample** curr, const struct rg_deform_sample** next) {
	*curr = NULL;
	*next = NULL;
	if (time < deform->samples[0].time) {
		return;
	}
	if (deform->count == 1 || time > deform->samples[deform->count - 1].time) {
		*curr = &deform->samples[deform->count - 1];
		return;
	}

	int curr_ptr = 0, next_ptr = 1;
	while (true)
	{
		if (deform->samples[curr_ptr].time <= time && time < deform->samples[next_ptr].time) {
			*curr = &deform->samples[curr_ptr];
			*next = &deform->samples[next_ptr];
			return;
		} else {
			++curr_ptr;
			++next_ptr;
			if (next_ptr >= deform->count) {
				break;
			}
		}
	}
}

const float* 
rg_tl_query_deform(const struct rg_tl_deform* deform, int time, struct rg_tl_deform_state* state) {
	state->offset0 = 0;
	state->count0  = 0;
	state->offset1 = 0;
	state->count1  = 0;

	const struct rg_deform_sample* curr;
	const struct rg_deform_sample* next;
	_query_deform(deform, time, &curr, &next);
	if (!curr && !next) {
		return NULL;
	}

	int count = 0;
	if (curr) {
		count += curr->count;
	}
	if (next) {
		count += next->count;
	}
	assert(count < MESH_BUF_SIZE * 2);

	int buf_ptr = 0;
	if (curr && next) {
		int ptr = 0;
		for (int i = 0; i < curr->count; ++i) {
			MESH_BUF[buf_ptr++] = _float_lerp(curr->time, next->time, curr->data[ptr++], 0, time);
			MESH_BUF[buf_ptr++] = _float_lerp(curr->time, next->time, curr->data[ptr++], 0, time);
		}
		ptr = 0;
		for (int i = 0; i < next->count; ++i) {
			MESH_BUF[buf_ptr++] = _float_lerp(curr->time, next->time, 0, next->data[ptr++], time);
			MESH_BUF[buf_ptr++] = _float_lerp(curr->time, next->time, 0, next->data[ptr++], time);
		}
	} else if (curr) {
		memcpy(MESH_BUF, curr->data, curr->count * 2);
	} else {
		assert(next);
		memcpy(MESH_BUF, next->data, next->count * 2);
	}

	if (curr) {
		state->offset0 = curr->offset;
		state->count0  = curr->count;
	}
	if (next) {
		state->offset1 = next->offset;
		state->count1  = next->count;
	}

	return MESH_BUF;
}