#include "delay.h"

/*
 * 阻塞式毫秒延时：CPU 在这段时间内不执行主循环，但硬件中断仍可响应。
 * 因此蜂鸣提示期间电机的 10ms PID 中断仍会继续工作。
 */
void delay_ms(uint32_t ms)
{
    /* CPUCLK_FREQ 由 SysConfig 生成，先换算出 1ms 对应的 CPU 周期数。 */
    uint32_t cycles = (CPUCLK_FREQ / 1000) * ms;
    delay_cycles(cycles);
}

