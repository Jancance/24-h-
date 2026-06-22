#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "board_pins.h"

/*
 * 串口发送模块。三个接口都是阻塞发送，适合调试信息和短控制帧，
 * 不适合在 10ms 电机 PID 中断里发送大量数据。
 *
 * 调试串口接线：
 *   DEBUG_TX -> PA26, UART3_TX
 *   DEBUG_RX -> PA25, UART3_RX
 *
 * ZDT TTL 串口接线：
 *   ZDT1_TX -> PB0, UART0_TX
 *   ZDT1_RX -> PB1, UART0_RX
 *   ZDT2_TX -> PB4, UART1_TX
 *   ZDT2_RX -> PB5, UART1_RX
 */

void UART_send_string(UART_Regs *uart, const char *str); /* 发送 '\0' 结尾字符串 */
void UART_send_char(UART_Regs *uart, const uint8_t chr); /* 发送单个字节 */
void UART_send_bytes(UART_Regs *uart, const uint8_t *data,
                     uint16_t length);                    /* 发送二进制帧 */

#endif /* UART_H */
