#include "key.h"

/*
 * 这四个计数由 GPIO 中断更新，由 motor.c 读取。
 * counter_x_A 只保存最近 10ms 的脉冲，用来算速度；
 * encoder_x_total 保存本路段累计脉冲，用来算行驶距离。
 * 它们暂时放在 key.c 是因为按键和编码器都属于 GPIO 输入，后续若单独建
 * encoder 模块再迁移，当前阶段不拆文件，避免改动已经能工作的中断入口。
 */
volatile uint32_t counter_1_A = 0U;
volatile uint32_t counter_2_A = 0U;
volatile uint32_t encoder_1_total = 0U;
volatile uint32_t encoder_2_total = 0U;

static uint8_t get_active_low_key(GPIO_Regs *port, uint32_t pin)
{
    /*
     * 按键硬件是一端接 GPIO、一端接 GND，GPIO 内部上拉。
     * 没按：读到高电平；按下：读到低电平。
     */
    return ((DL_GPIO_readPins(port, pin) & pin) == 0U) ? 1U : 0U;
}

uint8_t get_key_mode_state(void)
{
    return get_active_low_key(BOARD_KEY_MODE_PORT, BOARD_KEY_MODE_PIN);
}

uint8_t get_key_plus_state(void)
{
    return get_active_low_key(BOARD_KEY_PLUS_PORT, BOARD_KEY_PLUS_PIN);
}

uint8_t get_key_minus_state(void)
{
    return get_active_low_key(BOARD_KEY_MINUS_PORT, BOARD_KEY_MINUS_PIN);
}

uint8_t get_key_ok_state(void)
{
    return get_active_low_key(BOARD_KEY_OK_PORT, BOARD_KEY_OK_PIN);
}

uint8_t get_key1_state(void)
{
    /* 兼容旧代码里的 KEY1 名称，实际对应现在的 MODE 键。 */
    return get_key_mode_state();
}

uint8_t get_key2_state(void)
{
    /* 兼容旧代码里的 KEY2 名称，实际对应现在的 OK 键。 */
    return get_key_ok_state();
}

void GROUP1_IRQHandler(void)
{
    /*
     * 编码器中断。
     *
     * counter_1_A / counter_2_A：
     *   给 motor.c 每 10ms 算速度用，算完会清零。
     *
     * encoder_1_total / encoder_2_total：
     *   给路线控制算累计距离用，只有调用 motor_reset_distance() 才清零。
     */
    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
    case DC_MOTOR_BA_IIDX:
        counter_2_A++;
        encoder_2_total++;
        break;
    default:
        break;
    }

    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
    case DC_MOTOR_AA_IIDX:
        counter_1_A++;
        encoder_1_total++;
        break;
    default:
        break;
    }
}
