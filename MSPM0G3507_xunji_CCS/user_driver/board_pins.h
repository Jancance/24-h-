#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "ti_msp_dl_config.h"

/*
 * MSPM0G3507 car wiring summary.
 *
 * OLED on board I2C0:
 *   OLED_SDA -> PA0
 *   OLED_SCL -> PA1
 *
 * MPU6050 on external I2C1:
 *   MPU6050_SDA -> PB3
 *   MPU6050_SCL -> PB2
 *   MPU6050_INT -> PB14
 *
 * UART:
 *   DEBUG_TX -> PA26, UART3_TX
 *   DEBUG_RX -> PA25, UART3_RX
 *   ZDT1_TX  -> PB0, UART0_TX
 *   ZDT1_RX  -> PB1, UART0_RX
 *   ZDT2_TX  -> PB4, UART1_TX
 *   ZDT2_RX  -> PB5, UART1_RX
 *
 * TB6612:
 *   PWMA -> PA12, TIMG0_C0 PWM
 *   PWMB -> PA13, TIMG0_C1 PWM
 *   AIN1 -> PA8
 *   AIN2 -> PA9
 *   BIN1 -> PB18
 *   BIN2 -> PA7
 *   STBY -> pulled up to 3.3 V on hardware, no MCU pin used
 *
 * Encoders:
 *   Encoder1_A -> PA21, GPIO interrupt
 *   Encoder1_B -> PA22, GPIO input
 *   Encoder2_A -> PB19, GPIO interrupt
 *   Encoder2_B -> PB20, GPIO input
 *
 * CCD:
 *   CCD_AO     -> PA24, ADC0 channel 3
 *   CCD_CLK    -> PB8
 *   CCD_SI     -> PB9
 *
 * 8-channel line tracker, left to right. These pins are intentionally not
 * continuous, so PB8/PB9/PB10 can stay dedicated to the CCD interface.
 *   S1 -> PB6
 *   S2 -> PB7
 *   S3 -> PB11
 *   S4 -> PB12
 *   S5 -> PB13
 *   S6 -> PB17
 *   S7 -> PA17
 *   S8 -> PB15
 *
 * Tuning keys:
 *   MODE -> PB25
 *   PLUS -> PB26
 *   MINUS -> PB27
 *   OK -> PB16
 *
 * Buzzer:
 *   BUZZER -> PA14
 *
 * Servo:
 *   SERVO_PWM -> PA27, TIMG7_C1 PWM
 *
 * Stepper gimbal reserved, A4988/DRV8825 style:
 *   STEP1 -> PA28
 *   DIR1  -> PA29
 *   EN1   -> PA30
 *   STEP2 -> PA31
 *   DIR2  -> PB22
 *   EN2   -> PB23
 *   MS1/MS2/MS3 should be fixed by jumpers or resistors unless dynamic
 *   microstep control is required.
 *
 * On-board RGB LED (common anode, low level turns a channel on):
 *   PA14 -> one RGB channel, shared with the external buzzer signal
 *   PA15 -> RGB channel 1
 *   PA16 -> RGB channel 2
 *
 * Reserved/not used by this expansion pinout:
 *   USER_KEY -> PA18
 *   KEY2     -> PB21
 */

#define BOARD_KEY_MODE_PORT  GPIOB
#define BOARD_KEY_MODE_PIN   DL_GPIO_PIN_25
#define BOARD_KEY_PLUS_PORT  GPIOB
#define BOARD_KEY_PLUS_PIN   DL_GPIO_PIN_26
#define BOARD_KEY_MINUS_PORT GPIOB
#define BOARD_KEY_MINUS_PIN  DL_GPIO_PIN_27
#define BOARD_KEY_OK_PORT    GPIOB
#define BOARD_KEY_OK_PIN     DL_GPIO_PIN_16

#define BOARD_HUIDU_S1_PORT GPIOB
#define BOARD_HUIDU_S1_PIN  DL_GPIO_PIN_6
#define BOARD_HUIDU_S2_PORT GPIOB
#define BOARD_HUIDU_S2_PIN  DL_GPIO_PIN_7
#define BOARD_HUIDU_S3_PORT GPIOB
#define BOARD_HUIDU_S3_PIN  DL_GPIO_PIN_11
#define BOARD_HUIDU_S4_PORT GPIOB
#define BOARD_HUIDU_S4_PIN  DL_GPIO_PIN_12
#define BOARD_HUIDU_S5_PORT GPIOB
#define BOARD_HUIDU_S5_PIN  DL_GPIO_PIN_13
#define BOARD_HUIDU_S6_PORT GPIOB
#define BOARD_HUIDU_S6_PIN  DL_GPIO_PIN_17
#define BOARD_HUIDU_S7_PORT GPIOA
#define BOARD_HUIDU_S7_PIN  DL_GPIO_PIN_17
#define BOARD_HUIDU_S8_PORT GPIOB
#define BOARD_HUIDU_S8_PIN  DL_GPIO_PIN_15

#define BOARD_CCD_AO_PORT     GPIOA
#define BOARD_CCD_AO_PIN      DL_GPIO_PIN_24
#define BOARD_CCD_CLK_PORT    GPIOB
#define BOARD_CCD_CLK_PIN     DL_GPIO_PIN_8
#define BOARD_CCD_SI_PORT     GPIOB
#define BOARD_CCD_SI_PIN      DL_GPIO_PIN_9

#define BOARD_MPU6050_INT_PORT GPIOB
#define BOARD_MPU6050_INT_PIN  DL_GPIO_PIN_14

#define BOARD_BUZZER_PORT GPIOA
#define BOARD_BUZZER_PIN  DL_GPIO_PIN_14

#define BOARD_RGB_PORT    GPIOA
#define BOARD_RGB_CH1_PIN DL_GPIO_PIN_15
#define BOARD_RGB_CH2_PIN DL_GPIO_PIN_16

#define BOARD_STEPPER1_STEP_PORT GPIOA
#define BOARD_STEPPER1_STEP_PIN  DL_GPIO_PIN_28
#define BOARD_STEPPER1_DIR_PORT  GPIOA
#define BOARD_STEPPER1_DIR_PIN   DL_GPIO_PIN_29
#define BOARD_STEPPER1_EN_PORT   GPIOA
#define BOARD_STEPPER1_EN_PIN    DL_GPIO_PIN_30
#define BOARD_STEPPER2_STEP_PORT GPIOA
#define BOARD_STEPPER2_STEP_PIN  DL_GPIO_PIN_31
#define BOARD_STEPPER2_DIR_PORT  GPIOB
#define BOARD_STEPPER2_DIR_PIN   DL_GPIO_PIN_22
#define BOARD_STEPPER2_EN_PORT   GPIOB
#define BOARD_STEPPER2_EN_PIN    DL_GPIO_PIN_23

#endif /* BOARD_PINS_H */
