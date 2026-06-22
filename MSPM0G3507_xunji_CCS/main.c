#include "ti_msp_dl_config.h"
#include "buzzer.h"
#include "delay.h"
#include "huidu.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "track_task.h"
#include "uart.h"
#include <stdio.h>

extern float speed_1;
extern float speed_2;

/*
 * 当前选中的比赛任务模式。
 *
 * 默认选目标二；MODE 键按目标一、目标二、目标三、目标四的顺序循环切换。
 */
static track_mode_t current_mode = TRACK_MODE_ABCD;

static void wait_key_release(void)
{
    /*
     * 简单按键消抖。
     * 按下按键后，程序会等你松手，再延时 80ms。
     * 这样一次按键只会触发一次，不会因为手抖连跳多个模式。
     */
    while (get_key_mode_state() || get_key_plus_state() ||
           get_key_minus_state() || get_key_ok_state()) {
        delay_ms(10U);
    }
    delay_ms(80U);
}

static void show_menu(void)
{
    char line1[17];
    char line2[17];

    /*
     * OLED 一行最多显示 16 个英文字符左右，所以这里字符串故意写短。
     * 如果显示不全，优先缩短这里的英文提示，不影响车运行。
     */
    sprintf(line1, "H TARGET %d", (int)current_mode);
    sprintf(line2, "OK start");

    OLED_Clear();
    OLED_ShowString(1, 1, line1, 16);
    OLED_ShowString(2, 1, line2, 16);
    OLED_ShowString(3, 1, "Keep car still", 14);
    OLED_Refresh();
}

static void show_running(void)
{
    char line1[17];
    char line2[17];

    /*
     * 运行时只显示当前模式和左右轮速度。
     * speed_1 / speed_2 来自 motor.c 的 10ms 定时中断测速。
     */
    sprintf(line1, "RUN TARGET%d", (int)current_mode);
    sprintf(line2, "V:%3d,%3d", (int)speed_1, (int)speed_2);

    OLED_Clear();
    OLED_ShowString(1, 1, line1, 16);
    OLED_ShowString(2, 1, line2, 16);
    OLED_ShowString(3, 1, "Do not touch", 12);
    OLED_Refresh();
}

int main(void)
{
    /*
     * SysConfig 生成的总初始化函数。
     * GPIO、PWM、I2C、UART、定时器这些外设都在 empty.syscfg 里配置。
     */
    SYSCFG_DL_init();

    /* 共阳 RGB 上电默认可能短亮，先关掉 PA15/PA16 两个通道。 */
    buzzer_init();

    /* OLED 只负责菜单和状态显示，调车时看模式是否选对。 */
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    /*
     * 编码器 A 相接在 GPIO 中断上：
     *   电机 1 A 相 -> PA21 -> GPIOA 中断
     *   电机 2 A 相 -> PB19 -> GPIOB 中断
     * 不开这两个中断，车就无法计算速度和距离。
     */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /*
     * 舵机 PWM 目前只给一个中位值，占位备用。
     * H 题当前路线方案没有用舵机，后续如果加机械结构再改这里。
     */
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50U, GPIO_SERVO_C1_IDX);

    /* 启动电机 PWM、速度 PID 定时器，并让路线模块进入停车安全状态。 */
    motor_init_all();
    track_task_init();

    UART_send_string(DEBUG_INST, "H task car ready\r\n");
    show_menu();

    while (1) {
        /* MODE：停车菜单中按 1 -> 2 -> 3 -> 4 -> 1 循环切换比赛目标。 */
        if (get_key_mode_state()) {
            wait_key_release();
            if (current_mode == TRACK_MODE_ACBD_4LAPS) {
                current_mode = TRACK_MODE_AB;
            } else {
                current_mode = (track_mode_t)((int)current_mode + 1);
            }
            show_menu();
        }

        /*
         * OK：正式开始跑当前模式。
         * 路线函数内部是阻塞式执行，跑完才会回到菜单。
         * 调试时如果车异常，直接断电或按复位，不要用手拦车。
         */
        if (get_key_ok_state()) {
            wait_key_release();
            show_running();
            track_task_run(current_mode);
            motor_stop_all();
            UART_send_string(DEBUG_INST, "task finished\r\n");
            delay_ms(300U);
            show_menu();
        }

        delay_ms(20U);
    }
}
