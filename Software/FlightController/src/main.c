#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           GPIO_NUM_22       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR         0x68        /*!< Address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_WHO_AM_I_REG_VAL    0x71        /*!< Expected value of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
#define MPU6050_RESET_BIT           7
#define MPU6050_REG_ACCEL_XOUT_H 0x3B       /*!< Register addresses of the accelerometer data registers */

#define COMPASS_ADDRESS 0x0D
#define COMPASS_DREADY_REGISTER 0x06
#define VL53L1X_SENSOR_ADDR 0x29
#define VL53L1X_IDENTIFICATION_MODEL_ID 0x0000

/**
 * @brief Read a sequence of bytes from sensor registers
 */
static esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
    if (ret != ESP_OK) {
        return ret;
    } else {
        ESP_LOGI(TAG, "I2C master bus initialized successfully");
    }

    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(*bus_handle, &imu_config, dev_handle);
    if (ret != ESP_OK) {
        i2c_del_master_bus(*bus_handle);
        return ret;
    } else {
        ESP_LOGI(TAG, "I2C master device added successfully");
    }
    return ESP_OK;

}

static esp_err_t imu_init(i2c_master_dev_handle_t imu_handle)
{
    // Write to the power management register to wake up the sensor
    i2c_register_write_byte(imu_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x00);

    // Read the WHO_AM_I register to verify communication
    uint8_t who_am_i;
    i2c_register_read(imu_handle, MPU6050_WHO_AM_I_REG_ADDR, &who_am_i, 1);
    if (who_am_i != MPU6050_WHO_AM_I_REG_VAL) {
        ESP_LOGE(TAG, "MPU6050 WHO_AM_I mismatch: expected 0x%X, got 0x%X", MPU6050_WHO_AM_I_REG_VAL, who_am_i);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "MPU6050 initialized successfully with WHO_AM_I = 0x%X", who_am_i);
    }

    //auto select best clock
    i2c_register_write_byte(imu_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 0x01);

    //configure ranges for gyro and accel
    i2c_register_write_byte(imu_handle, 0x1B, 0x18); // Gyro full scale range to ±2000 dps
    i2c_register_write_byte(imu_handle, 0x1C, 0x10); // Accel full scale range to ±8g

    return ESP_OK;
}

static esp_err_t qmc8553_init(i2c_master_dev_handle_t qmc_handle)
{
    // Write to the control register to configure the QMC8553
    i2c_register_write_byte(qmc_handle, 0x09, 0b10000001); // OSR = 512, RNG = 8G, ODR = 10Hz, Continuous mode

    // Read the WHO_AM_I register to verify communication
    uint8_t who_am_i;
    i2c_register_read(qmc_handle, 0x0D, &who_am_i, 1);
    if (who_am_i != 0xFF) { 
        ESP_LOGE(TAG, "QMC8553 WHO_AM_I mismatch: expected 0xFF, got 0x%X", who_am_i);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "QMC8553 initialized successfully with WHO_AM_I = 0x%X", who_am_i);
    }

    return ESP_OK;
}

// 16 bit register read/write functions for VL53L1X sensor (rangefinder)

static esp_err_t vl53l1x_read(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t addr_buf[2] = { (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF) };
    return i2c_master_transmit_receive(dev_handle, addr_buf, 2, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t vl53l1x_write_byte(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint8_t data)
{
    uint8_t write_buf[3] = { (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF), data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t vl53l1x_write_word(i2c_master_dev_handle_t dev_handle, uint16_t reg_addr, uint16_t data)
{
    uint8_t write_buf[4] = {
        (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF),
        (uint8_t)(data >> 8), (uint8_t)(data & 0xFF)
    };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

void app_main(void)
{
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    imu_init(dev_handle);
    qmc8553_init(dev_handle);

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(i2c_register_read(dev_handle, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by resetting the MPU6050 */
    ESP_ERROR_CHECK(i2c_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}