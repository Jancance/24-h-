#include "huidu.h"
#include "car_config.h"
#include "motor.h"
#include "pid.h"

uint8_t huidu_value[HUIDU_SENSOR_COUNT] = {0};

/*
 * 上一次识别到黑线时的偏差。
 * 如果某一瞬间所有灰度都没看到黑线，就沿用这个方向继续找线。
 */
static int16_t last_error = 0;
static pid_controller_t line_pid;
static uint8_t line_pid_ready = 0U;

static uint8_t huidu_read_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio)
{
    /*
     * 读取某一路 GPIO 电平。
     * 返回 1：这个引脚当前是高电平。
     * 返回 0：这个引脚当前是低电平。
     */
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio);
    return ((high_bits & gpio) != 0U) ? 1U : 0U;
}

static uint8_t huidu_is_line_detected(uint8_t value)
{
    /*
     * 把“传感器原始电平”转换成“是否检测到黑线”。
     *
     * 有些灰度模块压到黑线输出高电平，有些输出低电平。
     * HUIDU_ACTIVE_LEVEL 就是用来适配这两种模块的。
     */
#if HUIDU_ACTIVE_LEVEL
    return value != 0U;
#else
    return value == 0U;
#endif
}

void huidu_get_value(void)
{
    /*
     * 按从左到右的顺序读取 8 路灰度。
     *
     * 数组对应关系：
     *   huidu_value[0] = S1，最左边
     *   huidu_value[1] = S2
     *   ...
     *   huidu_value[7] = S8，最右边
     */
    huidu_value[0] = huidu_read_gpio_state(BOARD_HUIDU_S1_PORT, BOARD_HUIDU_S1_PIN);
    huidu_value[1] = huidu_read_gpio_state(BOARD_HUIDU_S2_PORT, BOARD_HUIDU_S2_PIN);
    huidu_value[2] = huidu_read_gpio_state(BOARD_HUIDU_S3_PORT, BOARD_HUIDU_S3_PIN);
    huidu_value[3] = huidu_read_gpio_state(BOARD_HUIDU_S4_PORT, BOARD_HUIDU_S4_PIN);
    huidu_value[4] = huidu_read_gpio_state(BOARD_HUIDU_S5_PORT, BOARD_HUIDU_S5_PIN);
    huidu_value[5] = huidu_read_gpio_state(BOARD_HUIDU_S6_PORT, BOARD_HUIDU_S6_PIN);
    huidu_value[6] = huidu_read_gpio_state(BOARD_HUIDU_S7_PORT, BOARD_HUIDU_S7_PIN);
    huidu_value[7] = huidu_read_gpio_state(BOARD_HUIDU_S8_PORT, BOARD_HUIDU_S8_PIN);
}

uint8_t huidu_get_active_count(void)
{
    uint8_t active_count = 0U;

    /*
     * 每次调用都重新读 GPIO，不能直接使用上一次留下的 huidu_value[]，
     * 否则小车已经离开黑线时，路线模块仍可能拿到旧状态。
     */
    huidu_get_value();

    for (uint8_t i = 0U; i < HUIDU_SENSOR_COUNT; i++) {
        if (huidu_is_line_detected(huidu_value[i])) {
            active_count++;
        }
    }

    return active_count;
}

int16_t huidu_get_error(void)
{
    /*
     * 权重表表示每个传感器离小车中心有多远。
     *
     * 左边传感器给负数，右边传感器给正数：
     *   S1 S2 S3 S4 | S5 S6 S7 S8
     *  -350 ... -50 | 50 ... 350
     *
     * 如果黑线压在中间，算出来接近 0。
     * 如果黑线偏左，算出来是负数。
     * 如果黑线偏右，算出来是正数。
     */
    static const int16_t weights[HUIDU_SENSOR_COUNT] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    int32_t weighted_sum = 0;
    uint8_t active_count = 0;

    huidu_get_value();

    /*
     * 只统计检测到黑线的那些通道。
     * weighted_sum 是所有有效通道权重之和；
     * active_count 是看到黑线的传感器数量。
     */
    for (uint8_t i = 0; i < HUIDU_SENSOR_COUNT; i++) {
        if (huidu_is_line_detected(huidu_value[i])) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0U) {
        /*
         * 8 路都没看到黑线。
         * 这时不能直接返回 0，因为 0 会让车以为线在中间。
         * 所以返回 last_error，让车按上一次偏差方向继续找线。
         */
        return last_error;
    }

    /*
     * 多个传感器同时压线时，取平均位置。
     * 例如 S4 和 S5 都压线：(-50 + 50) / 2 = 0，说明线在中间。
     */
    last_error = (int16_t)(weighted_sum / active_count);
    return last_error;
}

extern float target_speed_1;
extern float target_speed_2;

static float huidu_limit_follow_speed(float speed, float min_speed, float max_speed)
{
    /*
     * 限制正常循迹时给出的目标速度范围。
     *
     * 为什么要有 min_speed：
     *   左轮 = base - correction，右轮 = base + correction。
     *   如果 correction 太大，一侧轮子会被压到接近 0，
     *   小车就不是“修方向”了，而是接近原地转，直线段会很难控。
     *
     * 所以正常循迹时给左右轮都留一个最低速度。
     * 真正丢线时，下面会单独用“低速找线”逻辑处理。
     */
    if (speed < min_speed) {
        return min_speed;
    }
    if (speed > max_speed) {
        return max_speed;
    }
    return speed;
}

void huidu_update_target_speeds(void)
{
    /*
     * 灰度循迹调速函数。
     *
     * 这个函数的输入：
     *   huidu_get_error() 算出的黑线偏差。
     *
     * 这个函数的输出：
     *   target_speed_1 / target_speed_2，也就是左右轮目标速度。
     *
     * 注意：
     *   它不直接控制 PWM。
     *   motor.c 的 10ms 定时中断会再根据目标速度做电机 PID。
     */
    const float base_speed = CAR_TARGET2_FOLLOW_BASE_SPEED;
    const float min_follow_speed = CAR_TARGET2_FOLLOW_MIN_SPEED;
    const float max_follow_speed = CAR_TARGET2_FOLLOW_MAX_SPEED;
    const float max_correction = base_speed - min_follow_speed;
    int16_t error = huidu_get_error();
    float correction;
    uint8_t active_count = 0;

    if (line_pid_ready == 0U) {
        /*
         * 灰度循迹 PID 初始化。
         *
         * target = 0，意思是希望黑线在 8 路传感器正中间。
         * feedback = huidu_get_error() 算出的黑线偏差。
         * output = correction，作为左右轮速度差修正量。
         *
         * kp = 0.75：
         *   huidu_get_error() 返回负数，表示黑线在左边。
         *   pid_calc_position() 内部会算 pid_error = target - feedback。
         *
         *   例子：
         *     线在左边，feedback = -100；
         *     pid_error = 0 - (-100) = +100；
         *     correction 为正数；
         *     左轮 = base - correction，右轮 = base + correction；
         *     左轮慢、右轮快，小车向左修。
         *
         * correction 限幅不能太大：
         *   max_correction = base_speed - min_follow_speed。
         *   这样正常循迹时，任何一侧轮子都不会低于 min_follow_speed，
         *   避免因为修正过猛变成接近原地自转。
         */
        pid_init(&line_pid, CAR_TARGET2_FOLLOW_KP, 0.0f, 0.0f,
                 -max_correction, max_correction);
        line_pid_ready = 1U;
    }

    for (uint8_t i = 0; i < HUIDU_SENSOR_COUNT; i++) {
        if (huidu_is_line_detected(huidu_value[i])) {
            active_count++;
        }
    }

    motor_set_direction(1, MOTOR_FORWARD);
    motor_set_direction(2, MOTOR_FORWARD);

    if (active_count == 0U) {
        /*
         * 完全丢线时，按上一次偏差方向找线：
         *   上次线在左边，就让车继续往左找；
         *   上次线在右边，就让车继续往右找。
         */
        if (error < 0) {
            target_speed_1 = CAR_TARGET2_SEARCH_SLOW_SPEED;
            target_speed_2 = CAR_TARGET2_SEARCH_FAST_SPEED;
        } else {
            target_speed_1 = CAR_TARGET2_SEARCH_FAST_SPEED;
            target_speed_2 = CAR_TARGET2_SEARCH_SLOW_SPEED;
        }
        return;
    }

    if (active_count >= 7U) {
        /*
         * 大部分传感器都检测到黑线，通常不是正常细黑线：
         *   可能压到了很宽的黑色区域；
         *   也可能 HUIDU_ACTIVE_LEVEL 写反；
         *   也可能传感器离地高度/阈值有问题。
         *
         * 这里先停车，方便调试时发现异常。
         */
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        return;
    }

    /*
     * 正常循迹：
     *   用 PID 结构体先算出 correction，再输出左右轮目标速度。
     *
     *   error < 0：线偏左，correction > 0，
     *              左轮 = base - correction，右轮 = base + correction。
     *   error > 0：线偏右，correction < 0，
     *              左轮变快，右轮变慢。
     */
    correction = pid_calc_position(&line_pid, 0.0f, (float)error);
    target_speed_1 = huidu_limit_follow_speed(base_speed - correction,
                                              min_follow_speed,
                                              max_follow_speed);
    target_speed_2 = huidu_limit_follow_speed(base_speed + correction,
                                               min_follow_speed,
                                               max_follow_speed);
}

void huidu_reset_follow(void)
{
    last_error = 0;
    if (line_pid_ready != 0U) {
        pid_reset(&line_pid);
    }
}
