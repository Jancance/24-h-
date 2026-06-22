#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include "board_pins.h"

/*
 * 预留的两路步进电机引脚，以后可用于云台。
 *
 * 1 轴驱动器：
 *   STEP -> PA28
 *   DIR  -> PA29
 *   EN   -> PA30
 *
 * 2 轴驱动器：
 *   STEP -> PA31
 *   DIR  -> PB22
 *   EN   -> PB23
 *
 * 驱动器供电：
 *   A4988/DRV8825 VMOT -> motor supply, VDD -> 3.3 V, GND common ground.
 *   MS1/MS2/MS3 are recommended to be fixed by jumpers or resistors first.
 *
 * 当前工程只在 SysConfig 中预留这些引脚，还没有 stepper.c。
 * 低速时可用 GPIO 翻转生成 STEP 脉冲；如果以后需要更高、更稳定的
 * 步进频率，应改用定时器输出或定时中断。
 */

typedef enum {
    STEPPER_AXIS_1 = 1,
    STEPPER_AXIS_2 = 2
} stepper_axis_t;

#endif /* STEPPER_H */
