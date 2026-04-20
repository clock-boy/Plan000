#include "imu.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <cmath>

#define I2C_MASTER_SCL_IO       GPIO_NUM_22
#define I2C_MASTER_SDA_IO       GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ      400000
#define MPU9250_ADDR            0x68
#define AK8963_ADDR             0x0C

#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_INT_PIN_CFG     0x37

#define AK8963_CNTL1            0x0A
#define AK8963_ST1              0x02
#define AK8963_XOUT_L           0x03

#define ACCEL_SCALE             (9.81f / 16384.0f)
#define GYRO_SCALE_RAD          (M_PI / (131.0f * 180.0f))

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t mpu_dev;
static i2c_master_dev_handle_t ak_dev;

static esp_err_t i2c_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };
    esp_err_t ret = i2c_master_transmit(dev, buf, 2, 100);
    if (ret != ESP_OK) {
        printf("I2C write failed: %s\n", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t i2c_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len) {
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
    if (ret != ESP_OK) {
        printf("I2C read failed: %s\n", esp_err_to_name(ret));
    }
    return ret;
}

static void i2c_init() {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    i2c_device_config_t mpu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mpu_config, &mpu_dev));

    i2c_device_config_t ak_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK8963_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &ak_config, &ak_dev));
}

static esp_err_t mpu9250_init() {
    esp_err_t ret;

    ret = i2c_write(mpu_dev, MPU9250_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = i2c_write(mpu_dev, MPU9250_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK) return ret;

    ret = i2c_write(mpu_dev, MPU9250_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;

    ret = i2c_write(mpu_dev, MPU9250_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ret = i2c_write(mpu_dev, MPU9250_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ret = i2c_write(mpu_dev, MPU9250_INT_PIN_CFG, 0x02);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = i2c_write(ak_dev, AK8963_CNTL1, 0x16);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t imu_init() {
    i2c_init();
    return mpu9250_init();
}

esp_err_t imu_read(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz,
                   float &mx, float &my, float &mz) {
    uint8_t buf[14];
    esp_err_t ret = i2c_read(mpu_dev, MPU9250_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    ax = (int16_t)((buf[0] << 8) | buf[1]) * ACCEL_SCALE;
    ay = (int16_t)((buf[2] << 8) | buf[3]) * ACCEL_SCALE;
    az = (int16_t)((buf[4] << 8) | buf[5]) * ACCEL_SCALE;
    gx = (int16_t)((buf[8] << 8) | buf[9]) * GYRO_SCALE_RAD;
    gy = (int16_t)((buf[10] << 8) | buf[11]) * GYRO_SCALE_RAD;
    gz = (int16_t)((buf[12] << 8) | buf[13]) * GYRO_SCALE_RAD;

    uint8_t st1;
    ret = i2c_read(ak_dev, AK8963_ST1, &st1, 1);
    if (ret != ESP_OK) return ret;

    if (st1 & 0x01) {
        uint8_t mag_buf[7];
        ret = i2c_read(ak_dev, AK8963_XOUT_L, mag_buf, 7);
        if (ret != ESP_OK) return ret;

        mx = (int16_t)((mag_buf[1] << 8) | mag_buf[0]) * 0.15f;
        my = (int16_t)((mag_buf[3] << 8) | mag_buf[2]) * 0.15f;
        mz = (int16_t)((mag_buf[5] << 8) | mag_buf[4]) * 0.15f;
    }

    return ESP_OK;
}

static int imu_fail_count = 0;

static void enter_failsafe() {
    printf("FAILSAFE: IMU failed %d consecutive reads, cutting motors\n", imu_fail_count);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool imu_update(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz,
                float &mx, float &my, float &mz) {
    esp_err_t ret = imu_read(ax, ay, az, gx, gy, gz, mx, my, mz);
    if (ret != ESP_OK) {
        imu_fail_count++;
        if (imu_fail_count >= 10) {
            enter_failsafe();
        }
        return false;
    }
    imu_fail_count = 0;
    return true;
}