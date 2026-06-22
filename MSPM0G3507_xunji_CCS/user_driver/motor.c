#include "motor.h"
#include "huidu.h"
#include "pid.h"

extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;
extern volatile uint32_t encoder_1_total;
extern volatile uint32_t encoder_2_total;

float speed_1 = 0.0f;
float speed_2 = 0.0f;

float target_speed_1 = 0.0f;
float target_speed_2 = 0.0f;

static pid_controller_t motor1_speed_pid;
static pid_controller_t motor2_speed_pid;
static uint8_t motor_pid_ready = 0U;
static uint8_t line_follow_enabled = 1U;

static float motor_pulses_to_mm(uint32_t pulse_count)
{
    /*
     * 把编码器脉冲数换成车轮走过的距离，单位 mm。
     *
     * 当前代码只统计 A 相上升沿，所以一圈按 MOTOR_BIANMAQI=500 个脉冲算。
     * 公式：
     *   距离 = 脉冲数 / 每圈脉冲数 * 轮子周长
     *   轮子周长 = pi * 轮径
     *
     * 如果以后改成 AB 四倍频，MOTOR_BIANMAQI 要从 500 改成 2000。
     */
    return ((float)pulse_count / (float)MOTOR_BIANMAQI) *
           PI * (float)MOTOR_WHEEL_D;
}

void motor_init(uint8_t motor_id)
{
    if (motor_id == 1U) {
        motor_set_direction(1U, MOTOR_BRAKE);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0U, GPIO_PWMAB_C0_IDX);
    } else if (motor_id == 2U) {
        motor_set_direction(2U, MOTOR_BRAKE);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0U, GPIO_PWMAB_C1_IDX);
    }
}

void motor_init_all(void)
{
    /*
     * 速度环 PID 初始化。
     *
     * 这两个结构体分别对应左右电机：
     *   target  = 目标速度 target_speed_x
     *   feedback = 编码器算出的实际速度 speed_x
     *   output  = PWM 占空比
     *
     * 这里调用的是 pid_calc_incremental()。
     * 由于 kd = 0，当前实际效果是“增量式 PI”：
     *   output = 上一次 PWM + 本次 PWM 修正量
     *
     * 参数沿用之前散变量版本：
     *   kp = 0.5
     *   ki = 0.4
     *   kd = 0.0，先不加 D 项，避免实车还没调稳时抖动变大
     */
    pid_init(&motor1_speed_pid, 0.5f, 0.4f, 0.0f, 0.0f, (float)MOTOR_PWM_MAX);
    pid_init(&motor2_speed_pid, 0.5f, 0.4f, 0.0f, 0.0f, (float)MOTOR_PWM_MAX);
    motor_pid_ready = 1U;

    motor_init(1U);
    motor_init(2U);
    DL_Timer_startCounter(PWMAB_INST);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    if (duty > MOTOR_PWM_MAX) {
        duty = MOTOR_PWM_MAX;
    }

    if (motor_id == 1U) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    } else if (motor_id == 2U) {
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1U) {
        if (direction == MOTOR_BRAKE) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == MOTOR_FORWARD) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == MOTOR_REVERSE) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    } else if (motor_id == 2U) {
        if (direction == MOTOR_BRAKE) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == MOTOR_FORWARD) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == MOTOR_REVERSE) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

void motor_set_line_follow_enabled(uint8_t enabled)
{
    /*
     * enabled = 0：关闭循迹，左右轮按 motor_set_target_speed() 的速度跑。
     * enabled = 1：打开循迹，10ms 定时中断里会调用
     * huidu_update_target_speeds() 自动修正左右轮目标速度。
     */
    line_follow_enabled = enabled ? 1U : 0U;
}

void motor_set_target_speed(float motor1_mm_s, float motor2_mm_s)
{
    /*
     * 设置左右电机目标速度。
     * 真正的 PWM 不是这里直接给，而是在 motor_update_speed_pid() 里
     * 根据目标速度和实际速度之间的误差慢慢调。
     */
    target_speed_1 = motor1_mm_s;
    target_speed_2 = motor2_mm_s;
}

void motor_stop_all(void)
{
    /*
     * 停车时把目标速度、PID 内部状态、PWM 都清掉。
     * 不清 PID 的 output 的话，下次启动会沿用上一次 PWM，可能突然冲一下。
     */
    target_speed_1 = 0.0f;
    target_speed_2 = 0.0f;
    pid_reset(&motor1_speed_pid);
    pid_reset(&motor2_speed_pid);
    motor_set_duty(1U, 0U);
    motor_set_duty(2U, 0U);
    motor_set_direction(1U, MOTOR_BRAKE);
    motor_set_direction(2U, MOTOR_BRAKE);
}

void motor_reset_distance(void)
{
    /*
     * 清零累计里程。
     * 每次开始跑一段新路前都要调用，比如 A->B 开始前、半圆开始前。
     */
    encoder_1_total = 0U;
    encoder_2_total = 0U;
}

float motor_get_left_distance_mm(void)
{
    return motor_pulses_to_mm(encoder_1_total);
}

float motor_get_right_distance_mm(void)
{
    return motor_pulses_to_mm(encoder_2_total);
}

float motor_get_average_distance_mm(void)
{
    /*
     * 取左右轮平均距离作为小车前进距离。
     * 这样比只看单边编码器稳一点，左右轮轻微打滑时误差会小一些。
     */
    return (motor_get_left_distance_mm() + motor_get_right_distance_mm()) * 0.5f;
}

static void motor_update_measured_speed(uint8_t motor_id)
{
    /*
     * 速度计算每 10ms 调一次。
     * counter_x_A 是“这 10ms 内新增的脉冲”，用完后清零。
     * 乘以 100 是因为 10ms * 100 = 1s，换算成 mm/s。
     */
    if (motor_id == 1U) {
        uint32_t pulse_count = counter_1_A;
        counter_1_A = 0U;
        speed_1 = motor_pulses_to_mm(pulse_count) * 100.0f;
    } else if (motor_id == 2U) {
        uint32_t pulse_count = counter_2_A;
        counter_2_A = 0U;
        speed_2 = motor_pulses_to_mm(pulse_count) * 100.0f;
    }
}

static void motor_update_speed_pid(uint8_t motor_id)
{
    /*
     * 增量式 PI 调速。
     * 大白话：目标速度比实际速度高，就慢慢加 PWM；实际速度太高，就减 PWM。
     *
     * 这里虽然调用 pid_calc_incremental()，但 motor_init_all() 里 kd = 0，
     * 所以当前只有 P 和 I 在起作用。后面如果速度波动明显，可以再尝试加一点 kd。
     *
     * kp：主要影响响应速度，太大容易抖。
     * ki：主要消除长期误差，太大容易越调越冲。
     */
    if (motor_pid_ready == 0U) {
        return;
    }

    if (motor_id == 1U) {
        float pwm = pid_calc_incremental(&motor1_speed_pid,
                                         target_speed_1,
                                         speed_1);
        motor_set_duty(1U, (uint32_t)pwm);
    } else if (motor_id == 2U) {
        float pwm = pid_calc_incremental(&motor2_speed_pid,
                                         target_speed_2,
                                         speed_2);
        motor_set_duty(2U, (uint32_t)pwm);
    }
}

void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST)) {
    case DL_TIMER_IIDX_LOAD:
        /*
         * 这个中断 10ms 进来一次，是整车运动控制的心跳。
         * 先根据灰度循迹修正目标速度，再根据编码器速度做电机 PID。
         */
        if (line_follow_enabled != 0U) {
            huidu_update_target_speeds();
        }
        motor_update_measured_speed(1U);
        motor_update_speed_pid(1U);
        motor_update_measured_speed(2U);
        motor_update_speed_pid(2U);
        break;
    default:
        break;
    }
}
