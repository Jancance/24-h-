# MSPM0G3507 小车拓展板引脚说明

这份文档记录当前拓展板原理图和代码使用的引脚分配。后续写代码、排查接线、修改 PCB 时优先以这里为准。

## 1. 电源与电平原则

| 电源网 | 用途 | 注意 |
| --- | --- | --- |
| `3V3` | MSPM0、OLED、MPU6050、灰度、CCD、TB6612 逻辑、按键上拉 | MCU 信号基准电平 |
| `5V` | 舵机、串口屏、部分外设模块 | 5V 模块信号进 MCU 前要确认电平 |
| `VM_MOTOR` | TB6612 直流电机动力电源 | 和 3.3V 逻辑电源分开 |
| `V_ZDT` | ZDT 闭环步进电机独立供电 | 按 ZDT 电机手册供电 |
| `GND` | 所有模块共地 | MCU、TB6612、电机电源、ZDT、传感器必须共地 |

MSPM0 是 3.3V 逻辑。所有外部模块输出到 MCU 的信号都必须保证不超过 3.3V，尤其是 `CCD_AO`、超声波 `ECHO`、串口屏 `TX`、编码器 `A/B`、5V 灰度输出。

## 2. OLED

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| OLED SDA | `PA0` | `I2C0 SDA` |
| OLED SCL | `PA1` | `I2C0 SCL` |

I2C 上拉电阻接到 `3V3`，不要上拉到 5V。

## 3. MPU6050

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| MPU6050 SDA | `PB3` | `I2C1 SDA` |
| MPU6050 SCL | `PB2` | `I2C1 SCL` |
| MPU6050 INT | `PB14` | 普通 GPIO 输入/中断备用 |

MPU6050 建议使用 3.3V 供电，I2C 上拉到 `3V3`。

## 4. 调试串口 / 串口屏 / 蓝牙

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| DEBUG TX | `PA26` | `UART3 TX` |
| DEBUG RX | `PA25` | `UART3 RX` |

这个接口用于 USB-TTL、蓝牙串口或串口屏，三者建议只接一种。

接线方向：

```text
MCU PA26_TX -> 模块 RX
MCU PA25_RX <- 模块 TX
GND         -> GND
```

如果串口屏或蓝牙模块是 5V TTL，模块 `TX -> PA25` 需要分压或电平转换。

## 5. ZDT 闭环步进电机 TTL 串口

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| ZDT1 TX | `PB0` | `UART0 TX` |
| ZDT1 RX | `PB1` | `UART0 RX` |
| ZDT2 TX | `PB4` | `UART1 TX` |
| ZDT2 RX | `PB5` | `UART1 RX` |

接线方向：

```text
PB0 / MCU_TX -> ZDT1_RX
PB1 / MCU_RX <- ZDT1_TX
PB4 / MCU_TX -> ZDT2_RX
PB5 / MCU_RX <- ZDT2_TX
GND          -> ZDT_GND
```

当前代码里的 ZDT 驱动按 `Emm 固件 + 0x6B 固定校验 + 115200` 编写。

默认地址：

```text
ZDT1_ADDR = 1
ZDT2_ADDR = 2
```

## 6. TB6612 直流电机驱动

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| PWMA | `PA12` | `TIMG0_C0 PWM` |
| PWMB | `PA13` | `TIMG0_C1 PWM` |
| AIN1 | `PA8` | GPIO 输出 |
| AIN2 | `PA9` | GPIO 输出 |
| BIN1 | `PB18` | GPIO 输出 |
| BIN2 | `PA7` | GPIO 输出 |
| STBY | 不占 MCU | 硬件上拉到 `3V3` |

TB6612 电源：

```text
VCC -> 3V3
VM  -> VM_MOTOR
GND -> GND
STBY -> 3V3
```

电机输出：

```text
电机1 -> AO1 / AO2
电机2 -> BO1 / BO2
```

## 7. 直流电机编码器

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| Encoder1 A | `PA21` | GPIO 中断，上升沿计数 |
| Encoder1 B | `PA22` | GPIO 输入 |
| Encoder2 A | `PB19` | GPIO 中断，上升沿计数 |
| Encoder2 B | `PB20` | GPIO 输入 |

当前代码只统计 A 相上升沿，编码器线数按 `500` 处理。

如果以后改成 AB 四倍频计数，代码里的编码器常数需要从 `500` 改为 `2000`。

如果编码器使用 5V 供电，A/B 输出进 MCU 前要分压或电平转换。

## 8. CCD 模块

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| CCD AO | `PA24` | `ADC0` |
| CCD CLK | `PB8` | GPIO 输出 |
| CCD SI | `PB9` | GPIO 输出 |

`PB10 = CCD_LED/EN` 已取消，不再占用。

`CCD_AO` 是 ADC 输入，电压必须在 `0V ~ 3.3V` 之间。CCD 模块优先使用 3.3V 供电。

## 9. 8 路灰度传感器

| 路数 | 引脚 | 代码名 |
| --- | --- | --- |
| S1 | `PB6` | `huidu_value[0]` |
| S2 | `PB7` | `huidu_value[1]` |
| S3 | `PB11` | `huidu_value[2]` |
| S4 | `PB12` | `huidu_value[3]` |
| S5 | `PB13` | `huidu_value[4]` |
| S6 | `PB17` | `huidu_value[5]` |
| S7 | `PA17` | `huidu_value[6]` |
| S8 | `PB15` | `huidu_value[7]` |

灰度模块建议使用 3.3V 供电。如果模块只能 5V 供电，输出信号要处理成 3.3V 再进 MCU。

## 10. 按键

| 功能 | 引脚 | 建议用途 |
| --- | --- | --- |
| KEY_MODE | `PB25` | 切换菜单/参数 |
| KEY_PLUS | `PB26` | 参数加 |
| KEY_MINUS | `PB27` | 参数减 |
| KEY_OK | `PB16` | 确认/保存 |

建议按键一端接 GPIO，一端接 GND，GPIO 使用上拉。代码按低电平按下处理。

## 11. 蜂鸣器与 RGB

| 功能 | 引脚 | 注意 |
| --- | --- | --- |
| 蜂鸣器 | `PA14` | 建议使用三极管/MOS 管驱动 |
| RGB/LED 通道 1 | `PA15` | 备用 |
| RGB/LED 通道 2 | `PA16` | 备用 |

GPIO 不建议直接驱动大电流蜂鸣器。普通 LED 每路都要串限流电阻。

## 12. 舵机

| 功能 | 引脚 | 代码/外设 |
| --- | --- | --- |
| Servo PWM | `PA27` | `TIMG7_C1 PWM` |

舵机电源建议使用独立 `5V~6V`，不要从 MCU 3.3V 取电。PWM 信号为 3.3V，一般舵机可以识别。

## 13. 超声波模块预留

| 功能 | 推荐引脚 | 注意 |
| --- | --- | --- |
| TRIG | `PB10` | GPIO 输出 |
| ECHO | `PB24` | GPIO 输入 |

如果使用普通 HC-SR04，`ECHO` 通常是 5V 输出，必须分压到 3.3V 再接 `PB24`。

推荐分压：

```text
ECHO -- 10k -- PB24 -- 20k -- GND
```

## 14. 普通步进接口预留

| 功能 | 引脚 |
| --- | --- |
| STEP1 | `PA28` |
| DIR1 | `PA29` |
| EN1 | `PA30` |
| STEP2 | `PA31` |
| DIR2 | `PB22` |
| EN2 | `PB23` |

这组用于 A4988/DRV8825/TMC 等 STEP/DIR 类驱动预留。ZDT TTL 闭环步进不走这组接口。

## 15. 保留或避免使用的引脚

| 引脚 | 原因 |
| --- | --- |
| `PA5`, `PA6` | 40MHz 晶振 |
| `PA19`, `PA20` | SWD 下载调试 |
| `NRST` | 复位 |
| `PA23` | VREF+ |
| `PA2` | ROSC |
| `PA3`, `PA4` | 低频晶振相关，尽量不占 |
| `PA10`, `PA11` | 主板原有串口/保留功能，避免占用 |
| `PA18` | 主板 USER_KEY / BSL 相关 |
| `PB21` | 主板 KEY2，建议备用 |

## 16. PCB 调试建议

建议预留测试点：

```text
3V3
5V
GND
PA26_UART3_TX
PA25_UART3_RX
PWMA
PWMB
ENC1_A
ENC2_A
CCD_AO
I2C0_SDA / I2C0_SCL
I2C1_SDA / I2C1_SCL
NRST
```

PCB 布线优先级：

```text
电源入口和 DC-DC
TB6612 与电机大电流线
ZDT 电源和串口
编码器与传感器
I2C / UART / 按键
铺 GND
DRC 检查
```

动力线尽量粗且短，CCD/编码器/I2C 远离电机输出和开关电源区域。
