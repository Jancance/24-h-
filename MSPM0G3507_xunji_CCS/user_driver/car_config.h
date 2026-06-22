#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/*
 * H 题目标一、目标二、目标三和目标四的集中调参区。
 * 直线段的核心是：
 *   编码器负责距离和主要纠偏；
 *   MPU6050 只辅助抑制车头偏航。
 */

/*
 * 目标一距离参数，单位 mm。
 * 题目 A->B 是 100cm，所以先填 1000mm。
 * 如果停早了，加大；停过了，减小。
 */
#define CAR_TARGET1_DISTANCE_MM     1000.0f

/*
 * 目标一和目标二的直线段共用这组参数，速度单位近似 mm/s。
 * ENCODER_KP：左右轮累计里程差的主纠偏系数。
 * HEADING_KP：MPU yaw 偏角的辅助纠偏系数，并单独限幅，避免零漂主导车轮。
 * 某个环越修越偏时，把该 KP 改为负数；左右摆动时减小绝对值。
 */
#define CAR_STRAIGHT_BASE_SPEED       300.0f
#define CAR_STRAIGHT_MIN_SPEED        120.0f
#define CAR_STRAIGHT_MAX_SPEED        480.0f
#define CAR_STRAIGHT_ENCODER_KP         2.0f
#define CAR_STRAIGHT_HEADING_KP         4.0f
#define CAR_STRAIGHT_MPU_MAX           40.0f
#define CAR_STRAIGHT_CORRECTION_MAX   120.0f
#define CAR_STRAIGHT_CONTROL_DT_S       0.01f /* 10ms 控制周期 */

/*
 * 超时保护。
 * 目标一要求 15s 内完成，这里也按 15s 设置。
 */
#define CAR_TARGET1_TIMEOUT_MS      15000U

/*
 * 目标二路线：A -> B -> C -> D -> A。
 *
 * A->B 和 C->D 是没有黑线的 100cm 直线：
 *   前一段先用编码器 + MPU6050 走直；
 *   走过最小距离后，灰度看到黑线就认为到达下一个圆弧起点。
 *
 * 为什么不是一看到黑线就切换：
 *   小车出发时就在 A 或 C 的黑色圆弧端点，刚启动可能仍能看到起点黑线。
 *   必须先离开起点一段距离，再允许识别终点黑线。
 */
#define CAR_TARGET2_STRAIGHT_MIN_MM          700.0f
#define CAR_TARGET2_STRAIGHT_MAX_MM         1150.0f

/*
 * B->C 和 D->A 是半径 40cm 的半圆弧，理论弧长约为：
 *   pi * 400mm = 1256.6mm。
 *
 * 先至少循迹 900mm，再允许用“看不到黑线”判断圆弧结束；
 * 1500mm 是安全上限，超过还没结束就停车报错，防止小车一直绕出去。
 */
#define CAR_TARGET2_ARC_MIN_MM               900.0f
#define CAR_TARGET2_ARC_MAX_MM              1500.0f

/* 直线入弧和圆弧出线都连续确认 3 次，避免 GPIO 瞬时误判。 */
#define CAR_TARGET2_LINE_CONFIRM_COUNT           3U

/* 圆弧循迹速度和丢线找线速度，所有车轮都保持前进。 */
#define CAR_TARGET2_FOLLOW_BASE_SPEED          280.0f
#define CAR_TARGET2_FOLLOW_MIN_SPEED           120.0f
#define CAR_TARGET2_FOLLOW_MAX_SPEED           520.0f

/*
 * 8 路灰度圆弧循迹 P 环的比例系数，不是 MPU 方向环。
 * 输入：黑线相对灰度中心的位置误差；输出：左右轮的速度差修正量。
 * 转弯跟不上黑线时可适当增大；沿黑线左右蛇形摆动时应减小。
 */
#define CAR_TARGET2_FOLLOW_KP                     0.75f
#define CAR_TARGET2_SEARCH_SLOW_SPEED          120.0f
#define CAR_TARGET2_SEARCH_FAST_SPEED          360.0f

/* 每个点的蜂鸣 + 板载 RGB 提示时间。 */
#define CAR_POINT_SIGNAL_MS                     80U

/* 目标二整圈题目要求不超过 30s；代码超过该估算时间就主动停车。 */
#define CAR_TARGET2_TIMEOUT_MS              30000U

/*
 * 目标三路线：A -> C -> B -> D -> A。
 * A->C 和 B->D 都是 100cm × 80cm 矩形的对角线，理论长度约 1280.6mm。
 * 先在 1000mm 后降速，1100mm 后才允许识别终点黑线，避免把起点误判成终点。
 */
#define CAR_TARGET3_APPROACH_START_MM        1000.0f
#define CAR_TARGET3_DIAGONAL_MIN_MM          1100.0f
#define CAR_TARGET3_DIAGONAL_MAX_MM          1450.0f
#define CAR_TARGET3_APPROACH_SPEED            160.0f

/*
 * 斜线与圆弧端点的理论切线夹角约 atan(80/100)=38.7 度。
 * C、D 入弧时先至少转 25 度，再允许灰度确认黑线；超过 60 度仍未抓线就停车。
 * B 点没有黑线指引，用 MPU 转到理论角度后进入 B->D 斜直线。
 */
#define CAR_TARGET3_TURN_ANGLE_DEG             38.7f
#define CAR_TARGET3_LINE_TURN_MIN_DEG          25.0f
#define CAR_TARGET3_LINE_TURN_MAX_DEG          60.0f
#define CAR_TARGET3_TURN_INNER_SPEED           100.0f
#define CAR_TARGET3_TURN_OUTER_SPEED           240.0f
#define CAR_TARGET3_TURN_TIMEOUT_MS            2500U

/* 灰度误差绝对值不超过 100，并连续确认 3 次，才认为车头已经对准圆弧。 */
#define CAR_TARGET3_CENTER_ERROR_MAX             100
#define CAR_TARGET3_LINE_CONFIRM_COUNT             3U

/* 目标三要求 40s 内完成；达到 40s 时主动停车，防止失控后继续冲出场地。 */
#define CAR_TARGET3_TIMEOUT_MS              40000U

/*
 * 目标四按目标三路线连续跑 4 圈，中间经过 A 点不停车。
 * 先复用目标三已验证的速度和转向参数；确认四圈稳定后，再逐步提高共用速度参数。
 * 160s 只作为失控保护上限，不是比赛目标时间。
 */
#define CAR_TARGET4_LAP_COUNT                    4U
#define CAR_TARGET4_TIMEOUT_MS              160000U

/* 上电校准 MPU6050 Z 轴陀螺仪零偏时采样次数，采样期间车必须静止。 */
#define CAR_MPU_GYRO_CALIB_SAMPLES    300U

#endif /* CAR_CONFIG_H */
