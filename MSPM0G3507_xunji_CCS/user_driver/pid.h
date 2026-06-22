#ifndef PID_H
#define PID_H

#include <stdint.h>

/*
 * 通用 PID 控制器结构体。
 *
 * 为什么要单独做这个结构体：
 *   以前速度环把 kp、ki、last_error、pwm_duty 分散写在 motor.c 里；
 *   灰度循迹又自己写一套 turn_gain。
 *   后续如果 MPU 方向环、循迹环、速度环都要 PID，就会越来越乱。
 *
 * 现在统一成 pid_controller_t：
 *   先把目标值 target 和反馈值 feedback 填进去；
 *   调用 pid_calc_position() 或 pid_calc_incremental()；
 *   函数返回 output，外部拿 output 去控制速度/PWM/转向修正。
 *
 * 注意：
 *   kp/ki/kd 不一定永远是正数，要看“误差正方向”和“输出正方向”怎么定义。
 *   例如灰度循迹里，我们已经让 error 和 correction 的方向配好了，所以 kp 用正数。
 */
typedef struct {
    float kp;          /* 比例系数：按当前误差大小给输出，正负号由控制方向决定 */
    float ki;          /* 积分系数：消除长期小误差 */
    float kd;          /* 微分系数：看误差变化快慢，用来抑制冲过头 */

    float target;      /* 目标值，比如目标速度、目标偏差 0 */
    float feedback;    /* 反馈值，比如实际速度、当前灰度误差 */
    float error;       /* 当前误差 = target - feedback */
    float last_error;  /* 上一次误差 */
    float last_last_error; /* 上上次误差，增量式 PID 的 D 项要用 */
    float integral;    /* 积分累计值，位置式 PID 使用 */

    float output;      /* PID 输出结果 */
    float output_min;  /* 输出下限 */
    float output_max;  /* 输出上限 */
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_min, float output_max);
void pid_reset(pid_controller_t *pid);

/*
 * 位置式 PID：
 *   output = kp*本次误差 + ki*误差累计 + kd*误差变化。
 *   返回值就是“这一次最终要输出多少”。
 *   适合灰度纠偏、MPU 角度纠偏这种直接算修正量的场景。
 */
float pid_calc_position(pid_controller_t *pid, float target, float feedback);

/*
 * 增量式 PID：
 *   先算这一次 output 应该变化多少 delta_output，
 *   再执行 output += delta_output。
 *   适合电机速度环这种 PWM 需要在上一拍基础上慢慢加减的场景。
 *
 * 如果 kd = 0，它就是增量式 PI。
 */
float pid_calc_incremental(pid_controller_t *pid, float target, float feedback);

#endif /* PID_H */
