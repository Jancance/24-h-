#include "buzzer.h"
#include "delay.h"

/* 非阻塞声光提示的剩余时间，只由主程序中的路线控制循环更新。 */
static uint16_t signal_remaining_ms = 0U;

/* 初始化蜂鸣器和 RGB，保证进入菜单前蜂鸣器关闭、PA15/PA16 灯熄灭。 */
void buzzer_init(void)
{
    /* RGB 是共阳极，PA15/PA16 先拉高保持熄灭。 */
    DL_GPIO_setPins(BOARD_RGB_PORT, BOARD_RGB_CH1_PIN | BOARD_RGB_CH2_PIN);
    buzzer_off();
    signal_remaining_ms = 0U;
}

/* PA14 输出高电平，驱动外接有源蜂鸣器发声。 */
void buzzer_on(void)
{
    DL_GPIO_setPins(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
}

/* PA14 输出低电平，关闭外接有源蜂鸣器。 */
void buzzer_off(void)
{
    DL_GPIO_clearPins(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
}

/* 按指定鸣响时间、间隔和次数输出纯声音提示；该函数会阻塞主循环。 */
void buzzer_beep(uint16_t on_ms, uint16_t off_ms, uint8_t repeat)
{
    for (uint8_t i = 0U; i < repeat; i++) {
        buzzer_on();
        delay_ms(on_ms);
        buzzer_off();
        if (i + 1U < repeat) {
            delay_ms(off_ms);
        }
    }
}

/*
 * 声光同步提示：蜂鸣时将共阳 RGB 的 PA15/PA16 拉低点亮，结束后拉高熄灭。
 * PA14 还与板载 RGB 的第三个颜色复用，所以蜂鸣前后看到颜色变化属于正常现象。
 */
void buzzer_beep_with_light(uint16_t on_ms, uint16_t off_ms, uint8_t repeat)
{
    for (uint8_t i = 0U; i < repeat; i++) {
        DL_GPIO_clearPins(BOARD_RGB_PORT,
                          BOARD_RGB_CH1_PIN | BOARD_RGB_CH2_PIN);
        buzzer_on();
        delay_ms(on_ms);
        buzzer_off();
        DL_GPIO_setPins(BOARD_RGB_PORT,
                        BOARD_RGB_CH1_PIN | BOARD_RGB_CH2_PIN);

        if (i + 1U < repeat) {
            delay_ms(off_ms);
        }
    }
}

void buzzer_signal_start(uint16_t duration_ms)
{
    /* 先清掉上一次提示，避免连续启动时遗留错误的剩余时间。 */
    buzzer_signal_stop();
    if (duration_ms == 0U) {
        return;
    }

    signal_remaining_ms = duration_ms;
    DL_GPIO_clearPins(BOARD_RGB_PORT,
                      BOARD_RGB_CH1_PIN | BOARD_RGB_CH2_PIN);
    buzzer_on();
}

void buzzer_signal_update(uint16_t elapsed_ms)
{
    if (signal_remaining_ms == 0U) {
        return;
    }

    if (elapsed_ms >= signal_remaining_ms) {
        buzzer_signal_stop();
    } else {
        signal_remaining_ms -= elapsed_ms;
    }
}

void buzzer_signal_stop(void)
{
    signal_remaining_ms = 0U;
    buzzer_off();
    DL_GPIO_setPins(BOARD_RGB_PORT,
                    BOARD_RGB_CH1_PIN | BOARD_RGB_CH2_PIN);
}

uint8_t buzzer_signal_is_active(void)
{
    return (signal_remaining_ms > 0U) ? 1U : 0U;
}
