#include "pid.h"

static float pid_limit(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float output_min, float output_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float pid_calc_position(pid_controller_t *pid, float target, float feedback)
{
    float delta_error;

    if (pid == 0) {
        return 0.0f;
    }

    /*
     * 位置式 PID。
     *
     * 它直接计算“当前这一拍最终应该输出多少”：
     *   output = kp * error + ki * error 累计 + kd * error 变化量
     *
     * 适合“直接算一个修正量”的场景，比如灰度循迹：
     *   target = 0，表示希望黑线在小车正中间；
     *   feedback = 当前灰度偏差；
     *   output = 左右轮速度差修正量。
     *
     * 和增量式的区别：
     *   位置式是直接覆盖 output；
     *   增量式是在上一次 output 基础上加/减一点。
     */
    pid->target = target;
    pid->feedback = feedback;
    pid->error = pid->target - pid->feedback;
    pid->integral += pid->error;
    delta_error = pid->error - pid->last_error;

    pid->output = (pid->kp * pid->error) +
                  (pid->ki * pid->integral) +
                  (pid->kd * delta_error);
    pid->output = pid_limit(pid->output, pid->output_min, pid->output_max);

    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    return pid->output;
}

float pid_calc_incremental(pid_controller_t *pid, float target, float feedback)
{
    float delta_output;

    if (pid == 0) {
        return 0.0f;
    }

    /*
     * 增量式 PID。
     *
     * 适合电机速度环这种“在上次 PWM 基础上加一点/减一点”的场景。
     *
     * 它不是直接算最终 PWM，而是先算本次要改多少：
     *   delta_output =
     *       kp * (本次误差 - 上次误差)
     *     + ki * 本次误差
     *     + kd * (本次误差 - 2 * 上次误差 + 上上次误差)
     *
     * 最后执行：
     *   output = 上一次 output + delta_output
     *
     * 如果 kd = 0，它就退化成增量式 PI。
     * 当前电机速度环的 kd 先设为 0，是为了先把车调稳，后面需要再加 D。
     */
    pid->target = target;
    pid->feedback = feedback;
    pid->error = pid->target - pid->feedback;

    delta_output = (pid->kp * (pid->error - pid->last_error)) +
                   (pid->ki * pid->error) +
                   (pid->kd * (pid->error -
                               (2.0f * pid->last_error) +
                               pid->last_last_error));

    pid->output += delta_output;
    pid->output = pid_limit(pid->output, pid->output_min, pid->output_max);

    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    return pid->output;
}
