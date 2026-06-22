#include "zdt.h"

/*
 * 本文件只负责把函数参数按照 ZDT 协议拼成字节帧，再通过 uart.c 阻塞发送。
 * 它不等待电机回包，也不判断电机是否真的执行成功；需要闭环确认时，应另加
 * UART 接收和回包解析，不能把“发送完成”当成“运动完成”。
 */

/* Modbus 写单寄存器和写多寄存器功能码。 */
#define ZDT_FUNC_WRITE_SINGLE 0x06U
#define ZDT_FUNC_WRITE_MULTI  0x10U

#define ZDT_REG_EMM_ENABLE   0x00F3U
#define ZDT_REG_EMM_VELOCITY 0x00F6U
#define ZDT_REG_EMM_POSITION 0x00FDU
#define ZDT_REG_EMM_FAST_CFG 0x00F1U
#define ZDT_REG_FAST_MOVE    0x00FCU

#define ZDT_REG_X_ENABLE     0x00E0U
#define ZDT_REG_X_VELOCITY   0x00E6U
#define ZDT_REG_X_POSITION   0x00F0U

#define ZDT_REG_STOP         0x00FEU
#define ZDT_REG_SYNC_RUN     0x00FFU

#define ZDT_CODE_EMM_ENABLE   0xF3U
#define ZDT_CODE_EMM_VELOCITY 0xF6U
#define ZDT_CODE_EMM_POSITION 0xFDU
#define ZDT_CODE_EMM_FAST_CFG 0xF1U
#define ZDT_CODE_FAST_MOVE    0xFCU
#define ZDT_CODE_STOP         0xFEU
#define ZDT_CODE_SYNC_RUN     0xFFU
#define ZDT_6B_CHECKSUM       0x6BU

/* 把位置模式放在高字节、同步标志放在低字节，组成 X 系列位置寄存器。 */
static uint16_t zdt_pack_mode_sync(zdt_position_mode_t mode, bool sync)
{
    return (uint16_t)(((uint16_t)mode << 8) | (sync ? 0x01U : 0x00U));
}

/* 给自由协议有效数据尾部补固定 0x6B，并返回实际发送长度。 */
static uint16_t zdt_send_6b_frame(UART_Regs *uart, uint8_t *frame,
                                  uint16_t payload_length)
{
    frame[payload_length] = ZDT_6B_CHECKSUM;
    UART_send_bytes(uart, frame, (uint16_t)(payload_length + 1U));
    return (uint16_t)(payload_length + 1U);
}

/* 标准 Modbus CRC16：初值 0xFFFF，多项式 0xA001。 */
uint16_t zdt_modbus_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Modbus 帧的 CRC 按“低字节在前、高字节在后”追加。 */
static void zdt_append_crc(uint8_t *frame, uint16_t payload_length)
{
    uint16_t crc = zdt_modbus_crc16(frame, payload_length);

    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1U] = (uint8_t)(crc >> 8);
}

/* 组装 0x06 写单寄存器帧：[地址][功能码][寄存器][数值][CRC]。 */
uint16_t zdt_write_single_register(UART_Regs *uart, uint8_t addr,
                                   uint16_t reg, uint16_t value)
{
    uint8_t frame[8];

    frame[0] = addr;
    frame[1] = ZDT_FUNC_WRITE_SINGLE;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)reg;
    frame[4] = (uint8_t)(value >> 8);
    frame[5] = (uint8_t)value;
    zdt_append_crc(frame, 6U);
    UART_send_bytes(uart, frame, sizeof(frame));
    return sizeof(frame);
}

/* 组装 0x10 连续写寄存器帧，64 字节缓冲区最多容纳 27 个寄存器。 */
uint16_t zdt_write_registers(UART_Regs *uart, uint8_t addr, uint16_t start_reg,
                             const uint16_t *regs, uint8_t reg_count)
{
    uint8_t frame[64];
    uint16_t index = 0U;

    if ((reg_count == 0U) || (reg_count > 27U)) {
        return 0U;
    }

    frame[index++] = addr;
    frame[index++] = ZDT_FUNC_WRITE_MULTI;
    frame[index++] = (uint8_t)(start_reg >> 8);
    frame[index++] = (uint8_t)start_reg;
    frame[index++] = 0x00U;
    frame[index++] = reg_count;
    frame[index++] = (uint8_t)(reg_count * 2U);

    for (uint8_t i = 0U; i < reg_count; i++) {
        frame[index++] = (uint8_t)(regs[i] >> 8);
        frame[index++] = (uint8_t)regs[i];
    }

    zdt_append_crc(frame, index);
    index += 2U;
    UART_send_bytes(uart, frame, index);
    return index;
}

/* 自由协议：使能或失能 Emm 系列电机。 */
void zdt_emm_enable(UART_Regs *uart, uint8_t addr, bool enable, bool sync)
{
    uint8_t frame[6];

    frame[0] = addr;
    frame[1] = ZDT_CODE_EMM_ENABLE;
    frame[2] = 0xABU;
    frame[3] = enable ? 0x01U : 0x00U;
    frame[4] = sync ? 0x01U : 0x00U;
    (void)zdt_send_6b_frame(uart, frame, 5U);
}

/* 自由协议：按方向、转速和加速度档位连续运行。 */
void zdt_emm_velocity(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                      uint16_t rpm, uint8_t acceleration, bool sync)
{
    uint8_t frame[8];

    frame[0] = addr;
    frame[1] = ZDT_CODE_EMM_VELOCITY;
    frame[2] = (uint8_t)dir;
    frame[3] = (uint8_t)(rpm >> 8);
    frame[4] = (uint8_t)rpm;
    frame[5] = acceleration;
    frame[6] = sync ? 0x01U : 0x00U;
    (void)zdt_send_6b_frame(uart, frame, 7U);
}

/* 自由协议：按给定脉冲数进行相对或绝对位置运动。 */
void zdt_emm_position(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                      uint16_t rpm, uint8_t acceleration, uint32_t pulses,
                      zdt_position_mode_t mode, bool sync)
{
    uint8_t frame[13];

    frame[0] = addr;
    frame[1] = ZDT_CODE_EMM_POSITION;
    frame[2] = (uint8_t)dir;
    frame[3] = (uint8_t)(rpm >> 8);
    frame[4] = (uint8_t)rpm;
    frame[5] = acceleration;
    frame[6] = (uint8_t)(pulses >> 24);
    frame[7] = (uint8_t)(pulses >> 16);
    frame[8] = (uint8_t)(pulses >> 8);
    frame[9] = (uint8_t)pulses;
    frame[10] = (uint8_t)mode;
    frame[11] = sync ? 0x01U : 0x00U;
    (void)zdt_send_6b_frame(uart, frame, 12U);
}

/* 自由协议快速位置模式的第一步：设置后续位置命令共用的运动参数。 */
void zdt_emm_fast_position_setup(UART_Regs *uart, uint8_t addr, uint16_t rpm,
                                 uint8_t acceleration,
                                 zdt_position_mode_t mode, bool sync)
{
    uint8_t frame[8];

    frame[0] = addr;
    frame[1] = ZDT_CODE_EMM_FAST_CFG;
    frame[2] = (uint8_t)(rpm >> 8);
    frame[3] = (uint8_t)rpm;
    frame[4] = acceleration;
    frame[5] = (uint8_t)mode;
    frame[6] = sync ? 0x01U : 0x00U;
    (void)zdt_send_6b_frame(uart, frame, 7U);
}

/* 自由协议快速位置模式的第二步：发送有符号脉冲目标，正负号代表运动方向。 */
void zdt_emm_fast_position_move(UART_Regs *uart, uint8_t addr, int32_t pulses)
{
    uint8_t frame[7];
    uint32_t raw = (uint32_t)pulses;

    frame[0] = addr;
    frame[1] = ZDT_CODE_FAST_MOVE;
    frame[2] = (uint8_t)(raw >> 24);
    frame[3] = (uint8_t)(raw >> 16);
    frame[4] = (uint8_t)(raw >> 8);
    frame[5] = (uint8_t)raw;
    (void)zdt_send_6b_frame(uart, frame, 6U);
}

/* Modbus X 系列：通过连续写两个寄存器设置使能和同步标志。 */
void zdt_x_enable(UART_Regs *uart, uint8_t addr, bool enable, bool sync)
{
    uint16_t regs[2];

    regs[0] = (uint16_t)(0xAB00U | (enable ? 0x01U : 0x00U));
    regs[1] = sync ? 0x0100U : 0x0000U;
    (void)zdt_write_registers(uart, addr, ZDT_REG_X_ENABLE, regs, 2U);
}

/* Modbus X 系列速度模式：方向、加速度、0.1rpm 转速和同步标志各占寄存器。 */
void zdt_x_velocity(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                    uint16_t acceleration_rpm_s, uint16_t rpm_x10, bool sync)
{
    uint16_t regs[4];

    regs[0] = (uint16_t)((uint16_t)dir << 8);
    regs[1] = acceleration_rpm_s;
    regs[2] = rpm_x10;
    regs[3] = sync ? 0x0100U : 0x0000U;
    (void)zdt_write_registers(uart, addr, ZDT_REG_X_VELOCITY, regs, 4U);
}

/* Modbus X 系列位置模式：32 位角度拆成高低两个 16 位寄存器发送。 */
void zdt_x_position_direct(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                           uint16_t rpm_x10, uint32_t angle_x10,
                           zdt_position_mode_t mode, bool sync)
{
    uint16_t regs[5];

    regs[0] = (uint16_t)((uint16_t)dir << 8);
    regs[1] = rpm_x10;
    regs[2] = (uint16_t)(angle_x10 >> 16);
    regs[3] = (uint16_t)angle_x10;
    regs[4] = zdt_pack_mode_sync(mode, sync);
    (void)zdt_write_registers(uart, addr, ZDT_REG_X_POSITION, regs, 5U);
}

/* 自由协议紧急停止；sync=true 时与其他缓存命令一起触发。 */
void zdt_stop(UART_Regs *uart, uint8_t addr, bool sync)
{
    uint8_t frame[5];

    frame[0] = addr;
    frame[1] = ZDT_CODE_STOP;
    frame[2] = 0x98U;
    frame[3] = sync ? 0x01U : 0x00U;
    (void)zdt_send_6b_frame(uart, frame, 4U);
}

/* 自由协议同步触发命令，常用于让两个地址的电机同时开始运动。 */
void zdt_sync_run(UART_Regs *uart, uint8_t addr)
{
    uint8_t frame[4];

    frame[0] = addr;
    frame[1] = ZDT_CODE_SYNC_RUN;
    frame[2] = 0x66U;
    (void)zdt_send_6b_frame(uart, frame, 3U);
}
