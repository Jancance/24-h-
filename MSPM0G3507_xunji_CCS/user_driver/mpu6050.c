#include "mpu6050.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

#define MPU6050_ADDR             0x68U

/*
 * MPU6050 寄存器地址。
 * 这里只列目标一/目标二直线方向环需要的寄存器：
 *   WHO_AM_I：确认 I2C 上是不是真的接了 MPU6050。
 *   PWR_MGMT_1：唤醒芯片。
 *   GYRO_CONFIG：设置陀螺仪量程。
 *   GYRO_ZOUT_H：读取 Z 轴陀螺仪高字节。
 */
#define MPU6050_REG_SMPLRT_DIV   0x19U
#define MPU6050_REG_CONFIG       0x1AU
#define MPU6050_REG_GYRO_CONFIG  0x1BU
#define MPU6050_REG_GYRO_ZOUT_H  0x47U
#define MPU6050_REG_PWR_MGMT_1   0x6BU
#define MPU6050_REG_WHO_AM_I     0x75U

#define MPU6050_I2C_TIMEOUT      50000U

/*
 * 当陀螺仪量程是 +/-250 deg/s 时，
 * 原始值 131 大约等于 1 deg/s。
 */
#define MPU6050_GYRO_SENS_250DPS 131.0f

/*
 * Z 轴角速度死区，单位 deg/s。
 *
 * 作用：
 *   MPU6050 静止时也会有一点点小抖动。
 *   如果滤波后的角速度绝对值小于这个值，就认为车没有转，直接当 0 处理。
 *
 * 调参建议：
 *   值太小：静止时 yaw 还是会慢慢漂。
 *   值太大：小车很轻微的真实偏转会被忽略。
 */
#define MPU6050_GYRO_Z_DEADZONE_DPS 0.30f

/*
 * Z 轴角速度一阶低通滤波系数。
 *
 * 公式：
 *   滤波后 = alpha * 上一次滤波值 + (1 - alpha) * 本次新值
 *
 * alpha 越大，数据越稳，但反应越慢；
 * alpha 越小，反应越快，但抖动越明显。
 */
#define MPU6050_GYRO_Z_LPF_ALPHA    0.80f

/* 静止时 Z 轴陀螺仪的原始零偏，校准后保存到这里。 */
static float gyro_z_offset_raw = 0.0f;

/* 当前 Z 轴角速度，单位 deg/s。 */
static float gyro_z_dps = 0.0f;

/* 由 Z 轴角速度积分得到的 yaw 角，单位 degree。 */
static float yaw_deg = 0.0f;

static uint8_t i2c_wait_idle(void)
{
    /*
     * 等 I2C 总线空闲。
     * 如果一直不空闲，说明总线可能卡住或设备没响应，超时返回失败。
     */
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t i2c_wait_not_busy(void)
{
    /*
     * 等本次 I2C 传输结束。
     * 结束后还要检查 ERROR 位，避免把失败传输当成功。
     */
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (DL_I2C_getControllerStatus(MPU6050_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout-- == 0U) {
            return 0U;
        }
    }

    if (DL_I2C_getControllerStatus(MPU6050_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        return 0U;
    }

    return 1U;
}

static uint8_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    /*
     * 向 MPU6050 写一个寄存器。
     *
     * I2C 写寄存器格式：
     *   先发寄存器地址 reg；
     *   再发要写进去的值 value。
     *
     * 例如唤醒 MPU6050：
     *   reg = PWR_MGMT_1，value = 0x00。
     */
    uint8_t tx_buf[2];

    tx_buf[0] = reg;
    tx_buf[1] = value;

    if (!i2c_wait_idle()) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, tx_buf, 2U);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    return i2c_wait_not_busy();
}

static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /*
     * 从 MPU6050 连续读取多个寄存器。
     *
     * I2C 读寄存器通常分两步：
     *   1. 先写一次寄存器地址，告诉 MPU6050 从哪里开始读。
     *   2. 再发起读传输，把数据读回来。
     *
     * 例如读取 Z 轴陀螺仪：
     *   从 GYRO_ZOUT_H 开始连续读 2 个字节。
     */
    if ((buf == 0) || (len == 0U)) {
        return 0U;
    }

    if (!i2c_wait_idle()) {
        return 0U;
    }

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1U);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    if (!i2c_wait_not_busy()) {
        return 0U;
    }

    if (!i2c_wait_idle()) {
        return 0U;
    }

    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint8_t i = 0U; i < len; i++) {
        uint32_t timeout = MPU6050_I2C_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST)) {
            if (timeout-- == 0U) {
                return 0U;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(MPU6050_INST);
    }

    return i2c_wait_not_busy();
}

static uint8_t mpu_read_gyro_z_raw(int16_t *gyro_z_raw)
{
    /*
     * 读取 Z 轴陀螺仪原始值。
     *
     * MPU6050 的一个轴数据是 16 位：
     *   高 8 位在 GYRO_ZOUT_H；
     *   低 8 位紧跟在后一个寄存器。
     *
     * 所以这里连续读 2 字节，再拼成 int16_t。
     */
    uint8_t buf[2];

    if (gyro_z_raw == 0) {
        return 0U;
    }

    if (!mpu_read_regs(MPU6050_REG_GYRO_ZOUT_H, buf, 2U)) {
        return 0U;
    }

    *gyro_z_raw = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return 1U;
}

uint8_t mpu6050_init(void)
{
    /*
     * 初始化 MPU6050。
     *
     * 返回 1：说明 I2C 通信正常，并且 WHO_AM_I 读到 0x68。
     * 返回 0：说明没读到 MPU6050，优先检查接线、电源、I2C 上拉。
     */
    uint8_t who_am_i = 0U;

    delay_ms(100U);

    if (!mpu_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1U)) {
        return 0U;
    }

    if (who_am_i != MPU6050_ADDR) {
        /*
         * WHO_AM_I 正常应该是 0x68。
         * 如果不是 0x68，可能地址不对、模块没上电、SDA/SCL 接反。
         */
        return 0U;
    }

    /*
     * PWR_MGMT_1 = 0x00：唤醒 MPU6050，使用内部 8MHz 时钟。
     * GYRO_CONFIG = 0x00：陀螺仪量程 +/-250 deg/s，灵敏度 131 LSB/(deg/s)。
     * CONFIG = 0x03：打开低通滤波，降低电机震动带来的噪声。
     * SMPLRT_DIV = 0x07：采样率分频，够目标一 10ms 更新使用。
     */
    if (!mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00U)) return 0U;
    delay_ms(20U);
    if (!mpu_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00U)) return 0U;
    if (!mpu_write_reg(MPU6050_REG_CONFIG, 0x03U)) return 0U;
    if (!mpu_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07U)) return 0U;

    yaw_deg = 0.0f;
    gyro_z_dps = 0.0f;
    gyro_z_offset_raw = 0.0f;
    return 1U;
}

uint8_t mpu6050_calibrate_gyro_z(uint16_t sample_count)
{
    /*
     * 求 Z 轴零偏。
     *
     * 车静止时，连续读 sample_count 次 Z 轴原始值，
     * 求平均后保存到 gyro_z_offset_raw。
     *
     * 后面每次读陀螺仪，都用 raw - gyro_z_offset_raw，
     * 这样能减小静止漂移。
     */
    int32_t sum = 0;
    int16_t raw = 0;

    if (sample_count == 0U) {
        return 0U;
    }

    /*
     * 校准时车必须静止。
     * 这里求 Z 轴陀螺仪原始值平均数，当作零偏。
     */
    for (uint16_t i = 0U; i < sample_count; i++) {
        if (!mpu_read_gyro_z_raw(&raw)) {
            return 0U;
        }
        sum += raw;
        delay_ms(2U);
    }

    gyro_z_offset_raw = (float)sum / (float)sample_count;
    yaw_deg = 0.0f;
    gyro_z_dps = 0.0f;
    return 1U;
}

uint8_t mpu6050_update_yaw(float dt_s)
{
    /*
     * 根据 Z 轴陀螺仪更新 yaw。
     *
     * 原始值 raw 先减掉零偏，再除以 131，得到角速度 deg/s。
     * yaw 积分公式：
     *   yaw = yaw + gyro_z_dps * dt_s
     *
     * 例如：
     *   gyro_z_dps = 10 deg/s，dt_s = 0.01s，
     *   那这 10ms 内 yaw 增加 0.1 度。
     */
    int16_t raw = 0;
    float gyro_z_now_dps;

    if (!mpu_read_gyro_z_raw(&raw)) {
        return 0U;
    }

    /*
     * 第一步：把本次读到的原始值换算成角速度。
     *
     * raw - gyro_z_offset_raw：
     *   减掉上电静止校准得到的零偏。
     *
     * / 131：
     *   因为当前陀螺仪量程是 +/-250 deg/s，
     *   数据手册规定 131 个原始值约等于 1 deg/s。
     */
    gyro_z_now_dps = ((float)raw - gyro_z_offset_raw) /
                     MPU6050_GYRO_SENS_250DPS;

    /*
     * 第二步：一阶低通滤波。
     *
     * gyro_z_dps 是上一次滤波后的角速度；
     * gyro_z_now_dps 是本次刚读到的新角速度。
     *
     * 这样不会完全相信某一次突然跳变的新数据，
     * 能减小电机震动、地面颠簸带来的角速度毛刺。
     */
    gyro_z_dps = (MPU6050_GYRO_Z_LPF_ALPHA * gyro_z_dps) +
                 ((1.0f - MPU6050_GYRO_Z_LPF_ALPHA) * gyro_z_now_dps);

    /*
     * 第三步：死区处理。
     *
     * 如果滤波后的角速度很小，就认为它是传感器噪声，
     * 不把它积分进 yaw，减少静止时 yaw 慢慢漂的问题。
     */
    if ((gyro_z_dps > -MPU6050_GYRO_Z_DEADZONE_DPS) &&
        (gyro_z_dps <  MPU6050_GYRO_Z_DEADZONE_DPS)) {
        gyro_z_dps = 0.0f;
    }

    yaw_deg += gyro_z_dps * dt_s;

    return 1U;
}

void mpu6050_reset_yaw(void)
{
    /*
     * 把当前车头方向当成 0 度。
     * 每段无黑线直线开始前调用，后面只看偏离该段起始方向多少。
     */
    yaw_deg = 0.0f;
    gyro_z_dps = 0.0f;
}

float mpu6050_get_yaw_deg(void)
{
    /* 返回当前累计 yaw 角，单位度。 */
    return yaw_deg;
}

float mpu6050_get_gyro_z_dps(void)
{
    /* 返回当前 Z 轴角速度，单位 deg/s。 */
    return gyro_z_dps;
}
