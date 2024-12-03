

#ifndef USER_UNITTRACK_H
#define USER_UNITTRACK_H

#include "RadarSysParams.h"
#include "TrackDbscan.h"
#include "track_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// #pragma once

/* ===============宏定义=================== */

#define FUNC_DECOR

#define TRACK_ALWAYS_CALCUL_A 1
#define TRACK_ALWAYS_CALCUL_Q 1

// #define TRACK_EKF_ACC            0.15 //alpha时间常数的倒数

// double TRACK_EKF_ACC = 0.15;  //alpha时间常数的倒数

#define TRACK_ADAPTIVE_CST 1

#if TRK_CONF_3D
#define MEASURE_ELEM_NUM 4 /* r v a a_elv */
#define STATE_ELEM_NUM 9   /* rx ry rz vx vy vz ax ay az */
#define DIM_NUM 3
#else
#define MEASURE_ELEM_NUM 3 /* r v a */
#define STATE_ELEM_NUM 6   /* rx ry vx vy ax ay */
#define DIM_NUM 2

#endif

// #ifdef TRACK_ADAPTIVE_CST
// #define TRACK_CST_THR_RNG        20.0f  /* Unitless */
// #define TRACK_CST_THR_VEL        10.0f  /* Unitless */
// #define TRACK_CST_THR_ANG        10.0f  /* Unitless */
// #else
// #define TRACK_CST_THR_RNG        10.0 /* Unit in BIN*/
// #define TRACK_CST_THR_VEL        6.0 /* Unit in BIN*/
// #define TRACK_CST_THR_ANG        10.0 /* Unit in BIN*/
// #endif
//
//
// #define RNG_NOISE_STD            4    /* Unit in BIN*/
// #define VEL_NOISE_STD            2    /* Unit in BIN*/
// #define ANG_NOISE_STD            2    /* Unit in BIN*/

#if TRK_CONF_3D
#define ANG_ELV_NOISE_STD 12 /* Unit in BIN*/
#endif

#define TRACK_DIS_THR_VEL 10.0 /* Unit in BIN */

#define TRACK_IDX_NUL -1

//#define TRACK_CST_MAX_2D        (TRACK_CST_THR_RNG * TRACK_CST_THR_RNG \
//                               + TRACK_CST_THR_VEL * TRACK_CST_THR_VEL \
//                               + TRACK_CST_THR_ANG * TRACK_CST_THR_ANG)
//
// #if TRK_CONF_3D
// #define TRACK_CST_MAX (TRACK_CST_MAX_2D + TRACK_CST_THR_ANG_ELV * TRACK_CST_THR_ANG_ELV)
// #else
// #define TRACK_CST_MAX TRACK_CST_MAX_2D
// #endif

/* ===============常用结构体=================== */
/* state */
typedef struct
{
	track_float_t rng_x; /* range along x axes        */
	track_float_t rng_y; /* range along y axes        */
#if TRK_CONF_3D
	track_float_t rng_z; /* range along z axes        */
#endif
	track_float_t vel_x; /* velocity along x axes     */
	track_float_t vel_y; /* velocity along y axes     */
#if TRK_CONF_3D
	track_float_t vel_z; /* velocity along z axes     */
#endif
	track_float_t acc_x; /* acceleration along x axes */
	track_float_t acc_y; /* acceleration along y axes */
#if TRK_CONF_3D
	track_float_t acc_z; /* acceleration along z axes */
#endif
} track_state_t;

/* tracker type */
typedef enum
{
	TRACK_TYP_NUL = 0,
	TRACK_TYP_PRE,
	TRACK_TYP_VAL,
} track_trk_type_t;

typedef enum
{
	OBJECT_SmallCar = 0,
	OBJECT_MediumCar,
	OBJECT_BigCar,
} Object_class_t;

typedef struct
{
	track_float_t SmallCarProb;
	track_float_t MediumCarProb;
	track_float_t BigCarProb;
	track_float_t ChangeState;
	track_float_t StepNumofChangeClassify;
	track_float_t ClusterPointNum;
	track_float_t RCS_ema;
} ClassifyParamsDef;

/* tracker */
typedef struct
{
	uint32_t UUID;
	track_trk_type_t type;
	int8_t hit_time;
	int8_t miss_time;
	int16_t idx_1;
	int16_t idx_2;
	track_float_t cst_1;
	track_float_t cst_2;
	track_float_t sigma_x;
	track_float_t sigma_y;
#if TRK_CONF_3D
	track_float_t sigma_z;
#endif
	track_float_t P[STATE_ELEM_NUM * STATE_ELEM_NUM]; /* could be reduced to 21 for 2D; 45 for 3D */
	track_state_t x;								  // 当前帧的卡尔曼融合的值，直角坐标
	track_measu_t flt_z;							  // 当前帧的观测值，极坐标
	track_measu_t pre_z;							  // 下一帧的预测值，极坐标
	track_float_t meas_var_inv[MEASURE_ELEM_NUM];
	track_float_t Length;
	track_float_t Width;
	track_float_t OrienAngle;
	ClassifyParamsDef ClassifyParams;
	int8_t ObjectClass; /* class: 0 : Medium Car 1: Big Car 2: Small Car */
} track_trk_t;

/* tracker package */
typedef struct
{
	track_trk_t trk[TRACK_NUM_TRK];
	uint32_t output_trk_number;
	bool has_run;
	// uint32_t		f_numb;
	track_float_t f_numb;
	track_float_t frame_int;

	uint16_t track_type_thres_pre;
	uint16_t track_type_thres_val;

	track_cdi_pkg_t *raw_input;
	track_obj_output_t *output_obj;
	track_header_output_t *output_hdr;
} track_trk_pkg_t;

/* ===============函数声明=================== */
/* Please use same name for each algorithm */

/* interfaces to track */
FUNC_DECOR void func_track_init(void *data, void *ctx);
FUNC_DECOR void func_track_pre(void *data, void *ctx);
FUNC_DECOR void func_track_run(void *data, void *ctx);
FUNC_DECOR void func_track_stop(void *data, void *ctx);
FUNC_DECOR void func_track_post(void *data, void *ctx);
FUNC_DECOR void func_set_frame_int(void *data, track_float_t delta);
FUNC_DECOR void func_track_header_update(void *data, void *ctx);
FUNC_DECOR void func_track_obj_info_update(void *data, void *ctx, uint32_t i);
FUNC_DECOR bool func_has_run(void *data);
FUNC_DECOR void func_track_Object_Classification(void *data, void *ctx);
FUNC_DECOR void func_track_ParamsUpdate(void *data, void *ctx);

#if TEST_MODE
extern FILE *TrackingObjectData_fp;
#endif
extern AdjustParams adjust_Params;



//-------------------------添加目标起始帧需要的结构-------------
typedef struct OneFrame_t {
	track_obj_output_t data;
	track_obj_output_t* prev;
	track_obj_output_t* next;
}OneFrame;

typedef struct {
	int frame_count;
	OneFrame* begin;
	OneFrame* end;
}OneObjList;
typedef struct BeginListElem_t{
	uint32_t UID;
	OneObjList* oneObjList;
	bool IsOut;
	bool IsDelete;
	BeginListElem_t *prev;
	BeginListElem_t *next;
}BeginListElem;

typedef struct {
	int ObjCount;
	BeginListElem* begin;
	BeginListElem* end;
}AllBeginList;
#ifdef __cplusplus
}
#endif

#endif