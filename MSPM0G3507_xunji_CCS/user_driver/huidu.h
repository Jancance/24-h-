#ifndef HUIDU_H
#define HUIDU_H

#include <stdint.h>
#include "board_pins.h"

/*
 * 8 路数字灰度传感器接线，从左到右：
 *   S1 -> PB6
 *   S2 -> PB7
 *   S3 -> PB11
 *   S4 -> PB12
 *   S5 -> PB13
 *   S6 -> PB17
 *   S7 -> PA17
 *   S8 -> PB15
 *
 * HUIDU_ACTIVE_LEVEL 用来设置“黑线有效电平”：
 *   1：传感器压到黑线时输出高电平，代码读到 1 就认为检测到黑线。
 *   0：传感器压到黑线时输出低电平，代码读到 0 就认为检测到黑线。
 *
 * 如果这个值填反了，小车会把白底和黑线判断反，循迹就会乱。
 * 不确定时，用万用表或串口打印看传感器放在黑线上的输出电平。
 */

#define HUIDU_SENSOR_COUNT 8U

/*
 * 这里默认写 1U，只是一个初始值，不代表你的模块一定是这样。
 * 后面做半圆循迹前，必须实际测一次：
 *   传感器放黑线上，如果 OUT 是高电平，就保持 1U；
 *   传感器放黑线上，如果 OUT 是低电平，就改成 0U。
 */
#define HUIDU_ACTIVE_LEVEL 1U

/*
 * 保存 8 路灰度当前读数。
 * huidu_value[0] 对应最左边 S1，huidu_value[7] 对应最右边 S8。
 * 每个值只有 0 或 1。
 */
extern uint8_t huidu_value[HUIDU_SENSOR_COUNT];

/* 读取 8 路灰度 GPIO 电平，并更新 huidu_value[]。 */
void huidu_get_value(void);

/*
 * 重新读取 8 路灰度并返回当前检测到黑线的通道数量。
 * 返回 0：当前完全没有看到黑线。
 * 返回 1~8：有对应数量的传感器看到黑线。
 *
 * 目标二用它判断：
 *   直线段是否到达下一条圆弧；
 *   圆弧段是否已经走到黑线末端。
 */
uint8_t huidu_get_active_count(void);

/*
 * 计算黑线相对小车中心的位置误差。
 * 返回值大概含义：
 *   负数：黑线在左边，需要向左修。
 *   0：黑线在中间。
 *   正数：黑线在右边，需要向右修。
 */
int16_t huidu_get_error(void);

/*
 * 根据灰度误差调整左右轮目标速度。
 * 这个函数不会直接输出 PWM，而是改 motor.c 里的 target_speed_1/2。
 * 真正 PWM 仍然由电机 PID 去调。
 */
void huidu_update_target_speeds(void);

/* 每次进入新的圆弧前，清掉上一段留下的循迹误差。 */
void huidu_reset_follow(void);

#endif /* HUIDU_H */
