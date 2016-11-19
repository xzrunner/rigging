#ifdef __cplusplus
extern "C"
{
#endif

#ifndef rigging_h
#define rigging_h

#include <stdint.h>

#define PTR_SIZE_DIFF (8 - sizeof(void *))
#define SIZEOF_POINTER 8

#define RG_SKIN_UNKNOWN 0xffff
#define RG_SKIN_NULL    0xfffe

/**
 *  @brief
 *    todo: rg_pose_srt
 */

struct rg_pose_srt {
	float trans[2];
	float rot;
	float scale[2];
};

void rg_pose_srt_identity(struct rg_pose_srt*);

void rg_local2world(const struct rg_pose_srt* src, const struct rg_pose_srt* local, struct rg_pose_srt* dst);

/**
 *  @brief
 *    todo: rg_pose_mat
 */

struct rg_pose_mat {
	float m[6];
};

void rg_pose_mat_identity(struct rg_pose_mat*);

void rg_pose_mat_build(struct rg_pose_mat* dst, const struct rg_pose_srt* src);

void rg_local2worldmat(const struct rg_pose_mat* src, const struct rg_pose_srt* local, struct rg_pose_mat* dst);

/**
 *  @brief
 *    todo: rg_joint
 */

struct rg_joint {
	const char* name;

	struct rg_pose_mat world_pose;
	struct rg_pose_srt local_pose;

	uint8_t  parent;

	uint8_t  children_count;
	uint8_t  children[1];
};

#define SIZEOF_RG_JOINT (sizeof(struct rg_joint) - sizeof(uint8_t) + PTR_SIZE_DIFF)

void rg_joint_update(struct rg_joint*, struct rg_skeleton*);

/**
 *  @brief
 *    todo: rg_skin
 */

enum RG_SKIN {
	SKIN_IMG,
	SKIN_MESH,
	SKIN_JOINT_MESH
};

struct rg_skin {
	int type;
	struct rg_pose_srt local;
	void* ud;
};

#define SIZEOF_RG_SKIN (sizeof(struct rg_skin) + PTR_SIZE_DIFF)

/**
 *  @brief
 *    todo: rg_slot
 */

struct rg_slot {
	uint16_t joint;
	uint16_t skin;
};

#define SIZEOF_RG_SLOT (sizeof(struct rg_slot))

/**
 *  @brief
 *    todo: rg_skeleton_pose
 */

struct rg_pose_pair {
	struct rg_pose_srt local;
	struct rg_pose_mat world;
};

struct rg_skeleton_pose {
	struct rg_pose_pair poses[1];
};

#define SIZEOF_RG_SKELETON_POSE (sizeof(struct rg_skeleton_pose) - sizeof(struct rg_pose_pair))

void rg_skeleton_pose_update(struct rg_skeleton_pose*, const struct rg_skeleton*, const struct rg_tl_joint**, int time);

/**
 *  @brief
 *    todo: rg_skeleton_skin
 */

struct rg_skeleton_skin {
	uint16_t skins[1];
};

#define SIZEOF_RG_SKELETON_SKIN (sizeof(struct rg_skeleton_skin) - sizeof(uint16_t))

void rg_skeleton_skin_init(void (*update_skin_func)(void* sym, const struct rg_skeleton_pose*),
						   void (*update_mesh_func)(void* sym, const struct rg_tl_deform_state*, const float*));

void rg_skeleton_skin_update(struct rg_skeleton_skin*, const struct rg_skeleton*, const struct rg_skeleton_pose*, const struct rg_tl_skin**, const struct rg_tl_deform**, int time);

/**
 *  @brief
 *    todo: rg_skeleton
 */

struct rg_skeleton {
	int joint_count;
	struct rg_joint** joints;

	int root;

	int slot_count;
	struct rg_slot* slots;

	int skin_count;
	struct rg_skin skins[1];
};

#define SIZEOF_RG_SKELETON (sizeof(struct rg_skeleton) - sizeof(struct rg_skin) + PTR_SIZE_DIFF * 2)

void rg_skeleton_init(void (*render_func)(void* sym, float* mat, const void* ud));

void rg_skeleton_draw(const struct rg_skeleton*, const struct rg_skeleton_pose*, const struct rg_skeleton_skin*, const void* ud);

/**
 *  @brief
 *    todo: rg_timeline
 */

enum DIM_IDX {
	DIM_IDX_TRANS_X = 0,
	DIM_IDX_TRANS_Y,
	DIM_IDX_ROT,
	DIM_IDX_SCALE_X,
	DIM_IDX_SCALE_Y,
	DIM_IDX_SHEAR_X,
	DIM_IDX_SHEAR_Y,

	DIM_COUNT
};

enum DIM_FLAG {
	DIM_FLAG_TRANS_X = 1<<0,
	DIM_FLAG_TRANS_Y = 1<<1,
	DIM_FLAG_ROT     = 1<<2,
	DIM_FLAG_SCALE_X = 1<<3,
	DIM_FLAG_SCALE_Y = 1<<4,
	DIM_FLAG_SHEAR_X = 1<<5,
	DIM_FLAG_SHEAR_Y = 1<<6,
};

enum INTERPOLATION {
	LERP_NULL = 0,
	LERP_LINEAR,
};

struct rg_joint_sample {
	uint16_t time;
	uint8_t	 lerp;
	uint8_t  padding;
	float	 data;
};

struct rg_tl_joint_state {
	float    trans[2];
	float    rot;
	float    scale[2];
	float    shear[2];
};

struct rg_tl_joint {
	uint8_t	               type;
	uint8_t                dims_count[DIM_COUNT];
	uint8_t                padding[3];
	struct rg_joint_sample samples[1];
};

#define SIZEOF_RG_TIMELINE_JOINT (sizeof(struct rg_tl_joint) - sizeof(struct rg_joint_sample))

struct rg_skin_sample {
	uint16_t time;
	uint16_t skin;
};

struct rg_tl_skin {
	uint8_t               skin_count;
	struct rg_skin_sample skins[1];
};

#define SIZEOF_RG_TIMELINE_SKIN (sizeof(struct rg_tl_skin) - sizeof(struct rg_skin_sample))

struct rg_tl_deform_state {
	uint16_t offset0, count0;
	uint16_t offset1, count1;
};

struct rg_deform_sample {
	uint16_t time;
	uint16_t offset;
	uint16_t count;
	uint16_t padding;
	float*   data;
};

#define SIZEOF_RG_DEFORM_SAMPLE (sizeof(struct rg_deform_sample) + PTR_SIZE_DIFF)

struct rg_tl_deform {
	int                     count;
	struct rg_deform_sample samples[1];
};

#define SIZEOF_RG_TIMELINE_DEFORM (sizeof(struct rg_tl_deform) - sizeof(struct rg_deform_sample))

struct rg_timeline {
	struct rg_tl_joint**  joints;
	struct rg_tl_skin**   skins;
	struct rg_tl_deform** deforms;
};

void rg_timeline_init();

void rg_tl_query_joint(const struct rg_tl_joint*, int time, uint64_t* dims_ptr, struct rg_tl_joint_state*);

uint16_t rg_tl_query_skin(const struct rg_tl_skin*, int time);

const float* rg_tl_query_deform(const struct rg_tl_deform*, int time, struct rg_tl_deform_state*);

/**
 *  @brief
 *    todo: rg_animation
 */

struct rg_animation {
	struct rg_skeleton* sk;

	struct rg_timeline timeline;

	int max_frame;
};

#define SIZEOF_RG_ANIM (sizeof(struct rg_animation) + PTR_SIZE_DIFF)

#endif // rigging_h

#ifdef __cplusplus
}
#endif