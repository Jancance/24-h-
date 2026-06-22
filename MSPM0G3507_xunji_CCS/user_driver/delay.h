#ifndef DELAY_H
#define DELAY_H

#include "board_pins.h"

/*
 * 阻塞式延时模块，不使用外部引脚，依赖 SysConfig 生成的 CPUCLK_FREQ。
 * ms 的单位是毫秒；不要在需要快速响应的控制循环里使用很大的延时值。
 */

void delay_ms(uint32_t ms);

#endif /* DELAY_H */
