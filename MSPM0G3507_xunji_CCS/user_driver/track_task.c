#include "track_task.h"
#include "buzzer.h"
#include "car_config.h"
#include "delay.h"
#include "huidu.h"
#include "motor.h"
#include "mpu6050.h"
#include "uart.h"
#include "ti_msp_dl_config.h"

/*
 * 目标一：A -> B。
 * 目标二：A -> B -> C -> D -> A。
 * 目标三：A -> C -> B -> D -> A。
 *
 * 直线段用编码器判断距离，用 MPU6050 保持车头方向；
 * 圆弧段用 8 路灰度循迹。路线只会设置两个电机前进，不会倒车。
 */

static uint8_t mpu_ready = 0U;

typedef enum {
    TRACK_TURN_LEFT = 0,
    TRACK_TURN_RIGHT = 1
} track_turn_direction_t;

/* 把浮点数限制在 min_value ~ max_value，防止速度修正量过大。 */
static float track_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* 转向时只关心已经转过多少度，不依赖 MPU 安装方向造成的 yaw 正负号。 */
static float track_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 经过 A/B/C/D 点时，向串口发送点名，同时蜂鸣并点亮 RGB。 */
static void track_point_signal(const char *name)
{
    UART_send_string(DEBUG_INST, name);
    UART_send_string(DEBUG_INST, "\r\n");
    buzzer_beep_with_light(CAR_POINT_SIGNAL_MS, 0U, 1U);
}

/*
 * 行驶中经过点使用非阻塞提示：这里只打开声光，不等待 80ms。
 * 后面的 10ms 路线循环会一边转向/循迹，一边更新并关闭提示。
 */
static void track_point_signal_async(const char *name)
{
    UART_send_string(DEBUG_INST, name);
    UART_send_string(DEBUG_INST, "\r\n");
    buzzer_signal_start(CAR_POINT_SIGNAL_MS);
}

/* 任务异常的统一处理：关闭循迹、立即停车、打印原因并连续报警。 */
static void track_fail_signal(const char *reason)
{
    buzzer_signal_stop();
    motor_set_line_follow_enabled(0U);
    motor_stop_all();
    UART_send_string(DEBUG_INST, reason);
    UART_send_string(DEBUG_INST, "\r\n");
    buzzer_beep_with_light(160U, 80U, 2U);
}

/*
 * 直线纠偏：编码器为主，MPU6050 为辅。
 * 左轮里程比右轮大时，让左轮减速、右轮加速，把里程差拉回。
 * MPU 修正量单独限幅；读取失败时关闭 MPU 辅助，编码器仍继续工作。
 */
static void track_update_straight_heading_with_speed(float base_speed)
{
    float encoder_error;
    float encoder_correction;
    float mpu_correction = 0.0f;
    float correction;
    float left_speed;
    float right_speed;

    encoder_error = motor_get_left_distance_mm() -
                    motor_get_right_distance_mm();
    encoder_correction = encoder_error * CAR_STRAIGHT_ENCODER_KP;

    if (mpu_ready != 0U) {
        if (mpu6050_update_yaw(CAR_STRAIGHT_CONTROL_DT_S)) {
            float yaw_error = 0.0f - mpu6050_get_yaw_deg();

            mpu_correction = track_clamp_float(
                yaw_error * CAR_STRAIGHT_HEADING_KP,
                -CAR_STRAIGHT_MPU_MAX,
                CAR_STRAIGHT_MPU_MAX);
        } else {
            mpu_ready = 0U;
            UART_send_string(DEBUG_INST, "mpu lost, encoder only\r\n");
        }
    }

    correction = track_clamp_float(encoder_correction + mpu_correction,
                                   -CAR_STRAIGHT_CORRECTION_MAX,
                                   CAR_STRAIGHT_CORRECTION_MAX);

    left_speed = track_clamp_float(base_speed - correction,
                                   CAR_STRAIGHT_MIN_SPEED,
                                   CAR_STRAIGHT_MAX_SPEED);
    right_speed = track_clamp_float(base_speed + correction,
                                    CAR_STRAIGHT_MIN_SPEED,
                                    CAR_STRAIGHT_MAX_SPEED);
    motor_set_target_speed(left_speed, right_speed);
}

/* 目标一、目标二仍使用原来的直线基础速度。 */
static void track_update_straight_heading(void)
{
    track_update_straight_heading_with_speed(CAR_STRAIGHT_BASE_SPEED);
}

/*
 * 开始一段新直线。
 * 圆弧出线后立即调用时，当前车头切线方向就成为 yaw=0 的直线参考。
 * 只保持车头方向，不保留圆弧时的左右轮差，避免小车继续拐弯。
 */
static void track_begin_straight(void)
{
    motor_set_line_follow_enabled(0U);
    motor_reset_distance();
    if (mpu_ready != 0U) {
        mpu6050_reset_yaw();
    }
    motor_set_direction(1U, MOTOR_FORWARD);
    motor_set_direction(2U, MOTOR_FORWARD);
    motor_set_target_speed(CAR_STRAIGHT_BASE_SPEED,
                           CAR_STRAIGHT_BASE_SPEED);
}

/* 目标一 A->B：按编码器距离直行，用里程差主纠偏、MPU 辅助。 */
static uint8_t track_drive_target1_straight(void)
{
    uint32_t elapsed_ms = 0U;

    while (motor_get_average_distance_mm() < CAR_TARGET1_DISTANCE_MM) {
        if (elapsed_ms >= CAR_TARGET1_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target1 timeout\r\n");
            return 0U;
        }

        track_update_straight_heading();

        delay_ms(10U);
        elapsed_ms += 10U;
    }

    motor_stop_all();
    return 1U;
}

/* 目标一总流程：A 点提示 -> 直行 -> B 点停车提示。 */
static void track_run_target1_ab(void)
{
    UART_send_string(DEBUG_INST, "target1 A->B start\r\n");

    track_point_signal("A start");
    track_begin_straight();
    if (track_drive_target1_straight()) {
        track_point_signal("B stop");
    } else {
        track_fail_signal("target1 failed");
    }
}

/*
 * 目标二的无黑线直线段，A->B 和 C->D 共用。
 * 1. 编码器距离未到 700mm 时，忽略灰度，防止误认起点黑线。
 * 2. 超过 700mm 后，连续 3 次看到黑线，才认为到达 B 或 D。
 * 3. 走到 1150mm 仍没找到黑线则返回 0，防止小车继续冲出场地。
 */
static uint8_t track_drive_target2_straight(uint32_t *elapsed_ms)
{
    uint8_t line_count = 0U;

    while (motor_get_average_distance_mm() < CAR_TARGET2_STRAIGHT_MAX_MM) {
        float distance_mm;

        if (*elapsed_ms >= CAR_TARGET2_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target2 timeout\r\n");
            return 0U;
        }

        track_update_straight_heading();

        distance_mm = motor_get_average_distance_mm();
        if (distance_mm >= CAR_TARGET2_STRAIGHT_MIN_MM) {
            if (huidu_get_active_count() > 0U) {
                line_count++;
                if (line_count >= CAR_TARGET2_LINE_CONFIRM_COUNT) {
                    /* 先保持直行，紧接着由入弧函数打开循迹。 */
                    motor_set_target_speed(CAR_TARGET2_FOLLOW_BASE_SPEED,
                                           CAR_TARGET2_FOLLOW_BASE_SPEED);
                    return 1U;
                }
            } else {
                line_count = 0U;
            }
        }

        delay_ms(10U);
        *elapsed_ms += 10U;
    }

    UART_send_string(DEBUG_INST, "straight line not found\r\n");
    return 0U;
}

/*
 * 进入 B->C 或 D->A 圆弧前的准备。
 * 清空上一段灰度误差和编码器里程，然后打开 10ms 定时中断循迹。
 */
static void track_begin_target2_arc(void)
{
    huidu_reset_follow();
    motor_reset_distance();
    motor_set_direction(1U, MOTOR_FORWARD);
    motor_set_direction(2U, MOTOR_FORWARD);
    motor_set_target_speed(CAR_TARGET2_FOLLOW_BASE_SPEED,
                           CAR_TARGET2_FOLLOW_BASE_SPEED);
    motor_set_line_follow_enabled(1U);
}

/*
 * 目标二的黑线半圆循迹，B->C 和 D->A 共用。
 * 前 900mm 不允许退出圆弧；之后连续 3 次看不到黑线，认为到达圆弧末端。
 * 如果圆弧走过 1500mm 仍未找到末端，返回 0 让上层停车报警。
 */
static uint8_t track_drive_target2_arc(uint32_t *elapsed_ms)
{
    uint8_t no_line_count = 0U;

    while (motor_get_average_distance_mm() < CAR_TARGET2_ARC_MAX_MM) {
        float distance_mm = motor_get_average_distance_mm();

        if (*elapsed_ms >= CAR_TARGET2_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target2 timeout\r\n");
            return 0U;
        }

        /* 前 900mm 即使瞬间丢线也继续找线，避免把圆弧中段当成终点。 */
        if (distance_mm >= CAR_TARGET2_ARC_MIN_MM) {
            if (huidu_get_active_count() == 0U) {
                no_line_count++;
                if (no_line_count >= CAR_TARGET2_LINE_CONFIRM_COUNT) {
                    motor_set_line_follow_enabled(0U);
                    return 1U;
                }
            } else {
                no_line_count = 0U;
            }
        }

        delay_ms(10U);
        *elapsed_ms += 10U;
    }

    UART_send_string(DEBUG_INST, "arc end not found\r\n");
    return 0U;
}

/*
 * 目标二状态流程：
 *   A 提示 -> A-B 直线 -> B 提示 -> B-C 圆弧
 *   -> C 提示 -> C-D 直线 -> D 提示 -> D-A 圆弧 -> A 停车。
 * 任何一段返回失败都会立即结束，不会带着错误状态进入下一段。
 */
static void track_run_target2_abcd(void)
{
    uint32_t elapsed_ms = 0U;

    UART_send_string(DEBUG_INST, "target2 A->B->C->D->A start\r\n");
    /* A 点出发，第一段走无黑线直线。 */
    track_point_signal("A start");
    elapsed_ms += CAR_POINT_SIGNAL_MS;
    track_begin_straight();

    if (!track_drive_target2_straight(&elapsed_ms)) {
        track_fail_signal("A->B failed");
        return;
    }
    /* 灰度连续看到黑线后认为到 B，立即打开圆弧循迹。 */
    track_begin_target2_arc();
    track_point_signal("B pass");
    elapsed_ms += CAR_POINT_SIGNAL_MS;

    if (!track_drive_target2_arc(&elapsed_ms)) {
        track_fail_signal("B->C failed");
        return;
    }
    track_begin_straight();
    /* 圆弧黑线结束即到 C，关闭循迹后开始第二段直线。 */
    track_point_signal("C pass");
    elapsed_ms += CAR_POINT_SIGNAL_MS;

    if (!track_drive_target2_straight(&elapsed_ms)) {
        track_fail_signal("C->D failed");
        return;
    }
    /* 到 D 后再次打开循迹，沿第二个半圆回 A。 */
    track_begin_target2_arc();
    track_point_signal("D pass");
    elapsed_ms += CAR_POINT_SIGNAL_MS;

    if (!track_drive_target2_arc(&elapsed_ms)) {
        track_fail_signal("D->A failed");
        return;
    }

    /* 第二个圆弧出线即回到 A，必须先停车再做终点提示。 */
    motor_set_line_follow_enabled(0U);
    motor_stop_all();
    track_point_signal("A stop");
    UART_send_string(DEBUG_INST, "target2 finished\r\n");
}

/*
 * 目标三专用的 10ms 控制节拍。
 * delay 只占一个很短的控制周期；醒来后同时更新任务总时间和非阻塞声光计时。
 */
static void track_target3_wait_period(uint32_t *elapsed_ms)
{
    delay_ms(10U);
    buzzer_signal_update(10U);
    *elapsed_ms += 10U;
}

/*
 * 目标三的 A->C、B->D 斜直线。
 * 理论长度约 1281mm；接近端点时先降速，再连续确认灰度看到黑线。
 */
static uint8_t track_drive_target3_diagonal(uint32_t *elapsed_ms)
{
    uint8_t line_count = 0U;

    while (motor_get_average_distance_mm() < CAR_TARGET3_DIAGONAL_MAX_MM) {
        float distance_mm = motor_get_average_distance_mm();
        float base_speed = CAR_STRAIGHT_BASE_SPEED;

        if (*elapsed_ms >= CAR_TARGET3_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target3 timeout\r\n");
            return 0U;
        }

        /* 在理论终点前约 28cm 开始降速，避免看到端点后仍高速冲过圆弧。 */
        if (distance_mm >= CAR_TARGET3_APPROACH_START_MM) {
            base_speed = CAR_TARGET3_APPROACH_SPEED;
        }
        track_update_straight_heading_with_speed(base_speed);

        if (distance_mm >= CAR_TARGET3_DIAGONAL_MIN_MM) {
            if (huidu_get_active_count() > 0U) {
                line_count++;
                if (line_count >= CAR_TARGET3_LINE_CONFIRM_COUNT) {
                    motor_set_target_speed(CAR_TARGET3_APPROACH_SPEED,
                                           CAR_TARGET3_APPROACH_SPEED);
                    return 1U;
                }
            } else {
                line_count = 0U;
            }
        }

        track_target3_wait_period(elapsed_ms);
    }

    UART_send_string(DEBUG_INST, "target3 diagonal line not found\r\n");
    return 0U;
}

/*
 * 准备端点转向。左右轮都保持前进，只用内外轮速度差改变车头方向。
 * 左转：左轮慢、右轮快；右转则相反，符合题目“只能前进、不得后退”。
 */
static void track_begin_target3_turn(track_turn_direction_t direction)
{
    motor_set_line_follow_enabled(0U);
    motor_reset_distance();
    mpu6050_reset_yaw();
    motor_set_direction(1U, MOTOR_FORWARD);
    motor_set_direction(2U, MOTOR_FORWARD);

    if (direction == TRACK_TURN_LEFT) {
        motor_set_target_speed(CAR_TARGET3_TURN_INNER_SPEED,
                               CAR_TARGET3_TURN_OUTER_SPEED);
    } else {
        motor_set_target_speed(CAR_TARGET3_TURN_OUTER_SPEED,
                               CAR_TARGET3_TURN_INNER_SPEED);
    }
}

/*
 * C、D 点入弧：先按指定方向低速转，再用 MPU 和灰度共同确认。
 * 必须先转过最小角度，且黑线落在灰度中间连续 3 次，才算真正抓住圆弧。
 */
static uint8_t track_turn_target3_to_line(track_turn_direction_t direction,
                                          uint32_t *elapsed_ms)
{
    uint32_t turn_elapsed_ms = 0U;
    uint8_t center_count = 0U;

    track_begin_target3_turn(direction);

    while (turn_elapsed_ms < CAR_TARGET3_TURN_TIMEOUT_MS) {
        float yaw_abs;

        if (*elapsed_ms >= CAR_TARGET3_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target3 timeout in line turn\r\n");
            return 0U;
        }
        if (!mpu6050_update_yaw(CAR_STRAIGHT_CONTROL_DT_S)) {
            UART_send_string(DEBUG_INST, "mpu lost in line turn\r\n");
            mpu_ready = 0U;
            return 0U;
        }

        yaw_abs = track_abs_float(mpu6050_get_yaw_deg());
        if (yaw_abs >= CAR_TARGET3_LINE_TURN_MIN_DEG) {
            uint8_t active_count = huidu_get_active_count();

            if (active_count > 0U) {
                int16_t line_error = huidu_get_error();
                int16_t error_abs = (line_error < 0) ? -line_error : line_error;

                if (error_abs <= CAR_TARGET3_CENTER_ERROR_MAX) {
                    center_count++;
                    if (center_count >= CAR_TARGET3_LINE_CONFIRM_COUNT) {
                        return 1U;
                    }
                } else {
                    center_count = 0U;
                }
            } else {
                center_count = 0U;
            }
        }

        if (yaw_abs >= CAR_TARGET3_LINE_TURN_MAX_DEG) {
            UART_send_string(DEBUG_INST, "target3 arc line not acquired\r\n");
            return 0U;
        }

        track_target3_wait_period(elapsed_ms);
        turn_elapsed_ms += 10U;
    }

    UART_send_string(DEBUG_INST, "target3 line turn timeout\r\n");
    return 0U;
}

/* B 点从右圆弧离开后没有黑线可参考，因此用 MPU 完成约 38.7 度定角左转。 */
static uint8_t track_turn_target3_by_angle(track_turn_direction_t direction,
                                           uint32_t *elapsed_ms)
{
    uint32_t turn_elapsed_ms = 0U;

    track_begin_target3_turn(direction);

    while (turn_elapsed_ms < CAR_TARGET3_TURN_TIMEOUT_MS) {
        if (*elapsed_ms >= CAR_TARGET3_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target3 timeout in angle turn\r\n");
            return 0U;
        }
        if (!mpu6050_update_yaw(CAR_STRAIGHT_CONTROL_DT_S)) {
            UART_send_string(DEBUG_INST, "mpu lost in angle turn\r\n");
            mpu_ready = 0U;
            return 0U;
        }
        if (track_abs_float(mpu6050_get_yaw_deg()) >=
            CAR_TARGET3_TURN_ANGLE_DEG) {
            return 1U;
        }

        track_target3_wait_period(elapsed_ms);
        turn_elapsed_ms += 10U;
    }

    UART_send_string(DEBUG_INST, "target3 angle turn timeout\r\n");
    return 0U;
}

/* 目标三的两个半圆与目标二尺寸相同，但使用 40s 的目标三总超时。 */
static uint8_t track_drive_target3_arc(uint32_t *elapsed_ms)
{
    uint8_t no_line_count = 0U;

    while (motor_get_average_distance_mm() < CAR_TARGET2_ARC_MAX_MM) {
        float distance_mm = motor_get_average_distance_mm();

        if (*elapsed_ms >= CAR_TARGET3_TIMEOUT_MS) {
            UART_send_string(DEBUG_INST, "target3 timeout in arc\r\n");
            return 0U;
        }

        if (distance_mm >= CAR_TARGET2_ARC_MIN_MM) {
            if (huidu_get_active_count() == 0U) {
                no_line_count++;
                if (no_line_count >= CAR_TARGET3_LINE_CONFIRM_COUNT) {
                    motor_set_line_follow_enabled(0U);
                    return 1U;
                }
            } else {
                no_line_count = 0U;
            }
        }

        track_target3_wait_period(elapsed_ms);
    }

    UART_send_string(DEBUG_INST, "target3 arc end not found\r\n");
    return 0U;
}

/*
 * 目标三完整状态流程：
 *   A->C 斜线；C 左转抓右圆弧；C->B 循迹；B 左转进入 B->D 斜线；
 *   D 右转抓左圆弧；D->A 循迹，最后在 A 停车提示。
 */
static void track_run_target3_acbda(void)
{
    uint32_t elapsed_ms = 0U;

    /* B 点定角转向必须依赖 MPU；初始化失败时直接停车比盲跑更安全。 */
    if (mpu_ready == 0U) {
        track_fail_signal("target3 needs MPU6050");
        return;
    }

    UART_send_string(DEBUG_INST, "target3 A->C->B->D->A start\r\n");
    UART_send_string(DEBUG_INST, "place car facing C\r\n");
    track_point_signal("A start");
    elapsed_ms += CAR_POINT_SIGNAL_MS;
    track_begin_straight();

    if (!track_drive_target3_diagonal(&elapsed_ms)) {
        track_fail_signal("A->C failed");
        return;
    }

    /* 已提前降速；提示期间继续左转并用灰度寻找 C->B 圆弧。 */
    track_point_signal_async("C pass");
    if (!track_turn_target3_to_line(TRACK_TURN_LEFT, &elapsed_ms)) {
        track_fail_signal("C turn failed");
        return;
    }
    track_begin_target2_arc();
    if (!track_drive_target3_arc(&elapsed_ms)) {
        track_fail_signal("C->B failed");
        return;
    }

    /* 到 B 后一边声光提示，一边低速左转约 38.7 度，对准 D。 */
    track_point_signal_async("B pass");
    if (!track_turn_target3_by_angle(TRACK_TURN_LEFT, &elapsed_ms)) {
        track_fail_signal("B turn failed");
        return;
    }
    track_begin_straight();
    if (!track_drive_target3_diagonal(&elapsed_ms)) {
        track_fail_signal("B->D failed");
        return;
    }

    /* D 点需要向右调整车头，灰度居中后沿左侧半圆回到 A。 */
    track_point_signal_async("D pass");
    if (!track_turn_target3_to_line(TRACK_TURN_RIGHT, &elapsed_ms)) {
        track_fail_signal("D turn failed");
        return;
    }
    track_begin_target2_arc();
    if (!track_drive_target3_arc(&elapsed_ms)) {
        track_fail_signal("D->A failed");
        return;
    }

    buzzer_signal_stop();
    motor_set_line_follow_enabled(0U);
    motor_stop_all();
    track_point_signal("A stop");
    UART_send_string(DEBUG_INST, "target3 finished\r\n");
}

void track_task_init(void)
{
    buzzer_init();
    motor_set_line_follow_enabled(0U);
    motor_stop_all();

    /* MPU6050 零偏校准期间小车必须保持静止。 */
    UART_send_string(DEBUG_INST, "mpu init...\r\n");
    if (mpu6050_init() &&
        mpu6050_calibrate_gyro_z(CAR_MPU_GYRO_CALIB_SAMPLES)) {
        mpu_ready = 1U;
        UART_send_string(DEBUG_INST, "mpu ready\r\n");
        buzzer_beep(50U, 50U, 1U);
    } else {
        mpu_ready = 0U;
        UART_send_string(DEBUG_INST, "mpu init fail\r\n");
        buzzer_beep(80U, 80U, 3U);
    }
}

/* 根据菜单选中的 mode 进入目标一、目标二或目标三，非法编号直接停车。 */
void track_task_run(track_mode_t mode)
{
    if (mode == TRACK_MODE_AB) {
        track_run_target1_ab();
    } else if (mode == TRACK_MODE_ABCD) {
        track_run_target2_abcd();
    } else if (mode == TRACK_MODE_ACBD) {
        track_run_target3_acbda();
    } else {
        track_fail_signal("unsupported target");
    }
}
