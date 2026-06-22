#ifndef TRACK_TASK_H
#define TRACK_TASK_H

#include <stdint.h>

/*
 * H 题路线任务编号。
 * main.c 只负责选模式，真正怎么跑在 track_task.c 里。
 */
typedef enum {
    TRACK_MODE_AB = 1,       /* 目标一：A -> B */
    TRACK_MODE_ABCD = 2,     /* 目标二：A -> B -> C -> D -> A */
    TRACK_MODE_ACBD = 3,     /* 目标三：A -> C -> B -> D -> A */
    TRACK_MODE_ACBD_4LAPS = 4 /* 目标四：按目标三路线连续行驶 4 圈 */
} track_mode_t;

/*
 * 初始化路线模块。
 * 主要做三件事：
 *   1. 关蜂鸣器。
 *   2. 关闭循迹并停车，保证上电安全。
 *   3. 初始化 MPU6050 并校准 Z 轴陀螺仪零偏。
 */
void track_task_init(void);

/*
 * 跑选中的比赛目标。
 * 这是阻塞函数，完成整条路线或异常停车后才返回。
 *
 * 参数 mode：
 *   TRACK_MODE_AB：目标一。
 *   TRACK_MODE_ABCD：目标二。
 *   TRACK_MODE_ACBD：目标三。
 *   TRACK_MODE_ACBD_4LAPS：目标四。
 */
void track_task_run(track_mode_t mode);

#endif /* TRACK_TASK_H */
