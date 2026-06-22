#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include "board_pins.h"

/*
 * 蜂鸣器接 PA14。
 * 起点、终点和故障报警可以使用阻塞式短响；
 * 行驶中的经过点使用非阻塞声光提示，避免提示期间停止灰度和转向判断。
 *
 * 如果你接的是有源蜂鸣器：
 *   PA14 拉高就响，拉低就停。
 *
 * 如果你接的是无源蜂鸣器：
 *   这里只能听到很轻的“哒”声，需要后续改成 PWM 发声。
 */

/* 初始化蜂鸣器，默认关闭。 */
void buzzer_init(void);

/* 蜂鸣器打开。 */
void buzzer_on(void);

/* 蜂鸣器关闭。 */
void buzzer_off(void);

/*
 * 蜂鸣器短响。
 * on_ms：响多久
 * off_ms：两次响之间停多久
 * repeat：重复几次
 */
void buzzer_beep(uint16_t on_ms, uint16_t off_ms, uint8_t repeat);

/*
 * 蜂鸣的同时点亮板载 RGB 的 PA15/PA16 两个通道。
 * RGB 是共阳极，所以这两路 GPIO 拉低才会亮。
 */
void buzzer_beep_with_light(uint16_t on_ms, uint16_t off_ms, uint8_t repeat);

/*
 * 启动一次非阻塞声光提示。
 * 调用后会立刻打开蜂鸣器和 RGB，但不会 delay，路线控制可以继续运行。
 * duration_ms 为提示时间；传入 0 会保持关闭。
 */
void buzzer_signal_start(uint16_t duration_ms);

/*
 * 更新非阻塞提示计时。
 * 路线控制每经过 10ms 调用一次并传入 10，剩余时间归零后自动关闭声光。
 */
void buzzer_signal_update(uint16_t elapsed_ms);

/* 立即结束非阻塞提示，任务失败或停车时用于保证蜂鸣器和灯关闭。 */
void buzzer_signal_stop(void);

/* 返回 1 表示非阻塞提示仍在进行，返回 0 表示已经结束。 */
uint8_t buzzer_signal_is_active(void);

#endif /* BUZZER_H */
