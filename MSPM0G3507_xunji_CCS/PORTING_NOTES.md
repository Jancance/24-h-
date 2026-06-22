# MSPM0G3507 8 路循迹改装说明

## CCS 打开方式

在 CCS 中选择 `File -> Import -> Code Composer Studio -> CCS Projects`，
工程目录选择：

`C:\Users\lenovo\Desktop\2024年e题小车\MSPM0G3507_xunji_CCS`

本工程保留 `.project`、`.cproject`、`.ccsproject`、`empty.syscfg` 和
`targetConfigs/MSPM0G3507.ccxml`，可作为 CCS 工程导入。没有拷贝旧 `Debug`
目录，避免旧生成文件覆盖新引脚配置；第一次编译时 CCS/SysConfig 会重新生成。

## 已适配的主要引脚

- OLED: PA0=SDA, PA1=SCL, I2C0
- MPU6050 预留: PB3=SDA, PB2=SCL, I2C1
- Debug UART: PA26=UART3_TX, PA25=UART3_RX
- ZDT TTL: PB0/PB1=UART0, PB4/PB5=UART1
- TB6612: PA12=PWMA, PA13=PWMB, PA8=AIN1, PA9=AIN2, PB18=BIN1, PA7=BIN2, STBY 硬件上拉到 3V3
- 编码器: PA21/PA22 为 1 号，PB19/PB20 为 2 号
- 8 路循迹: PB6, PB7, PB11, PB12, PB13, PB17, PA17, PB15，从左到右 S1-S8
- CCD: PA24=AO, PB8=CLK, PB9=SI
- 舵机: PA27
- 按键: PB25=MODE, PB26=PLUS, PB27=MINUS, PB16=OK，低电平按下
- 蜂鸣器: PA14
- 步进云台预留: PA28=STEP1, PA29=DIR1, PA30=EN1, PA31=STEP2, PB22=DIR2, PB23=EN2

## 循迹逻辑

`user_driver/huidu.c` 使用 8 路加权误差：

`S1..S8 = -350, -250, -150, -50, 50, 150, 250, 350`

中间压线时左右轮目标速度相同；线偏左时左轮降速、右轮升速；线偏右时相反。
如果你的循迹模块是“黑线输出低电平”，把 `user_driver/huidu.h` 中
`HUIDU_ACTIVE_LEVEL` 从 `1U` 改为 `0U`。

## H 题路线代码

- `user_driver/car_config.h`：集中调距离、速度、转向标定量。
- `user_driver/track_task.c`：H 题 1/2/3/4 项路线状态机。
- `user_driver/buzzer.c`：经过 A/B/C/D 点和停车提示。
- `main.c`：OLED 显示模式，MODE/PLUS/MINUS 选模式，OK 启动。

## 已做检查

- SysConfig 引脚和 `EXTENSION_BOARD_PINOUT.md` 对齐。
- 命令行执行 `gmake -k all` 已通过，生成 `Debug/MSPM0G3507_8line_tracking_car.out`。
- Debug makefile 已改为相对路径，避免中文路径导致 SysConfig 找不到 `empty.syscfg`。
