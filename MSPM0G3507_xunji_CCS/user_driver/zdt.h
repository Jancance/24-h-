#ifndef ZDT_H
#define ZDT_H

#include <stdbool.h>
#include <stdint.h>
#include "uart.h"

/*
 * ZDT 闭环步进电机串口驱动。本模块是云台/步进电机预留，目标二的
 * 直流轮毂电机控制不会调用这里的函数。
 *
 * ZDT X42S TTL 接线：
 *   ZDT1_RX <- PB0 / UART0_TX, ZDT1_TX -> PB1 / UART0_RX
 *   ZDT2_RX <- PB4 / UART1_TX, ZDT2_TX -> PB5 / UART1_RX
 *
 * 使用前需要在电机菜单里设置：
 *   Baudrate: 115200
 *   Checksum: 0x6B
 *
 * zdt_emm_* 使用 ZDT 自由协议，帧尾固定为 0x6B；
 * zdt_x_* 使用 Modbus 寄存器协议，帧尾为 CRC16。两套接口不能混着用，
 * 必须与电机菜单当前选择的通信协议一致。
 */

#define ZDT1_ADDR 0x01U
#define ZDT2_ADDR 0x02U

typedef enum {
    ZDT_DIR_CW = 0,  /* 顺时针 */
    ZDT_DIR_CCW = 1  /* 逆时针 */
} zdt_direction_t;

typedef enum {
    ZDT_POS_REL_LAST_TARGET = 0, /* 相对上一次目标位置 */
    ZDT_POS_ABS_ZERO = 1,       /* 相对电机零点的绝对位置 */
    ZDT_POS_REL_CURRENT = 2     /* 相对当前实际位置 */
} zdt_position_mode_t;

/* 计算 Modbus CRC16，返回 16 位校验值；发送时先放低字节，再放高字节。 */
uint16_t zdt_modbus_crc16(const uint8_t *data, uint16_t length);

/* 写一个 16 位寄存器，返回实际发送字节数，固定为 8。 */
uint16_t zdt_write_single_register(UART_Regs *uart, uint8_t addr,
                                   uint16_t reg, uint16_t value);

/* 连续写 1~27 个 16 位寄存器；参数越界返回 0，否则返回实际发送字节数。 */
uint16_t zdt_write_registers(UART_Regs *uart, uint8_t addr, uint16_t start_reg,
                             const uint16_t *regs, uint8_t reg_count);

/*
 * 自由协议接口：uart 选择 ZDT1/ZDT2 串口，addr 是电机地址。
 * sync=false 立即执行；sync=true 只缓存命令，之后调用 zdt_sync_run() 同步启动。
 */
void zdt_emm_enable(UART_Regs *uart, uint8_t addr, bool enable, bool sync);

/* rpm 单位为转/分，acceleration 是 ZDT 协议定义的 0~255 加速度档位。 */
void zdt_emm_velocity(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                      uint16_t rpm, uint8_t acceleration, bool sync);

/* 位置运行：pulses 是目标脉冲数，mode 决定相对/绝对定位基准。 */
void zdt_emm_position(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                      uint16_t rpm, uint8_t acceleration, uint32_t pulses,
                      zdt_position_mode_t mode, bool sync);

/* 快速位置模式分两步：先设置速度/加速度/模式，再发送有符号目标脉冲。 */
void zdt_emm_fast_position_setup(UART_Regs *uart, uint8_t addr, uint16_t rpm,
                                 uint8_t acceleration,
                                 zdt_position_mode_t mode, bool sync);
void zdt_emm_fast_position_move(UART_Regs *uart, uint8_t addr, int32_t pulses);

/* 以下为 Modbus X 系列寄存器接口，sync 含义与自由协议相同。 */
void zdt_x_enable(UART_Regs *uart, uint8_t addr, bool enable, bool sync);

/* acceleration_rpm_s 单位 rpm/s；rpm_x10 以 0.1rpm 为一单位。 */
void zdt_x_velocity(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                    uint16_t acceleration_rpm_s, uint16_t rpm_x10, bool sync);

/* rpm_x10 单位 0.1rpm；angle_x10 单位 0.1 度。 */
void zdt_x_position_direct(UART_Regs *uart, uint8_t addr, zdt_direction_t dir,
                           uint16_t rpm_x10, uint32_t angle_x10,
                           zdt_position_mode_t mode, bool sync);

/* zdt_stop() 停止指定电机；zdt_sync_run() 触发之前缓存的同步命令。 */
void zdt_stop(UART_Regs *uart, uint8_t addr, bool sync);
void zdt_sync_run(UART_Regs *uart, uint8_t addr);

#endif /* ZDT_H */
