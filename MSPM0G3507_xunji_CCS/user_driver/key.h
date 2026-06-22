#ifndef KEY_H
#define KEY_H

#include <stdint.h>
#include "board_pins.h"

/*
 * 扩展板按键接线，全部使用内部上拉并按低电平有效：
 *   MODE -> PB25
 *   PLUS -> PB26
 *   MINUS -> PB27
 *   OK -> PB16
 */

/* 以下函数返回 1 表示按下，返回 0 表示松开；函数本身不做消抖。 */
uint8_t get_key_mode_state(void);  /* MODE：切换目标模式 */
uint8_t get_key_plus_state(void);  /* PLUS：预留给参数增加 */
uint8_t get_key_minus_state(void); /* MINUS：预留给参数减小 */
uint8_t get_key_ok_state(void);    /* OK：启动当前目标 */

/* 旧接口兼容：KEY1 等同 MODE，KEY2 等同 OK。新代码优先使用上面的明确名称。 */
uint8_t get_key1_state(void);
uint8_t get_key2_state(void);

#endif /* KEY_H */
