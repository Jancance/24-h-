#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "board_pins.h"

#define PI 3.14f
#define MOTOR_ENCODER_LINES 500U
#define MOTOR_BIANMAQI MOTOR_ENCODER_LINES
#define MOTOR_WHEEL_D 48U
#define MOTOR_PWM_MAX 4000U

/*
 * 电机模块说明：
 * 1. TB6612 负责给左右直流电机输出 PWM。
 * 2. 编码器 A 相用 GPIO 中断计数。
 * 3. MOTOR_PID 定时器每 10ms 计算一次速度并做 PI 调速。
 *
 * TB6612 wiring:
 *   PWMA -> PA12 (TIMG0_C0 PWM)
 *   PWMB -> PA13 (TIMG0_C1 PWM)
 *   AIN1 -> PA8
 *   AIN2 -> PA9
 *   BIN1 -> PB18
 *   BIN2 -> PA7
 *   STBY -> pulled up to 3.3 V on hardware, no MCU pin used
 *
 * Encoder wiring:
 *   Encoder1_A -> PA21, GPIO interrupt
 *   Encoder1_B -> PA22, GPIO input
 *   Encoder2_A -> PB19, GPIO interrupt
 *   Encoder2_B -> PB20, GPIO input
 *
 * TB6612 power:
 *   VM -> motor battery, VCC -> 3.3 V, all GND must be common.
 */

typedef enum {
    MOTOR_BRAKE = 0,
    MOTOR_FORWARD = 1,
    MOTOR_REVERSE = 2
} motor_direction_t;

/* 初始化单个电机 PWM 和方向引脚，motor_id 只能填 1 或 2。 */
void motor_init(uint8_t motor_id);

/* 初始化两个电机，并启动 PWM 定时器和 10ms 电机 PID 定时器。 */
void motor_init_all(void);

/* 直接设置 PWM 占空比，范围 0 ~ MOTOR_PWM_MAX，普通调车时较少直接用。 */
void motor_set_duty(uint8_t motor_id, uint32_t duty);

/* 设置电机方向：刹车、前进、后退。 */
void motor_set_direction(uint8_t motor_id, uint8_t direction);

/*
 * 开关灰度循迹。
 * enabled=0：关闭循迹，左右轮按目标速度跑。
 * enabled=1：打开循迹，huidu_update_target_speeds() 会根据黑线位置
 * 修正左右轮目标速度。
 */
void motor_set_line_follow_enabled(uint8_t enabled);

/* 设置左右电机目标速度，单位近似 mm/s。 */
void motor_set_target_speed(float motor1_mm_s, float motor2_mm_s);

/* 两个电机立即停车，并清掉 PID 的历史误差和 PWM 累积量。 */
void motor_stop_all(void);

/* 清零累计里程，跑每一段路线前都要先调用。 */
void motor_reset_distance(void);

/* 获取左轮从上次清零后走过的距离，单位 mm。 */
float motor_get_left_distance_mm(void);

/* 获取右轮从上次清零后走过的距离，单位 mm。 */
float motor_get_right_distance_mm(void);

/* 获取左右轮平均距离，路线控制默认用它判断某一段是否跑够。 */
float motor_get_average_distance_mm(void);

#endif /* MOTOR_H */
