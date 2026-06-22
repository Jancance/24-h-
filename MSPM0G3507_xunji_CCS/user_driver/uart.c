#include "uart.h"

/* 阻塞发送一个字节：只有字节写入 UART 发送通道后才返回。 */
void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

/* 阻塞发送以 '\0' 结尾的字符串，不会把末尾的 '\0' 发出去。 */
void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

/* 阻塞发送指定长度的原始字节，可用于包含 0x00 的电机协议帧。 */
void UART_send_bytes(UART_Regs *uart, const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        UART_send_char(uart, data[i]);
    }
}

/* 下面是未启用的串口回显调试模板，当前工程没有打开对应接收中断。 */
// void PRINT_INST_IRQHandler()
// {
//     switch (DL_UART_getPendingInterrupt(PRINT_INST))
//     {
//     case DL_UART_IIDX_RX:
//         {   
//             uint8_t rec = DL_UART_receiveData(PRINT_INST);
//             UART_send_char(PRINT_INST, rec);
//             break;
//         }
    
//     default:
//         break;
//     }
// }

