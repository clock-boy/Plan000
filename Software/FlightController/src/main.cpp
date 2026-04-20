#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.hpp"
#include "ahrs.hpp"


#define IMU_FAIL_THRESHOLD 10


extern "C" void app_main() {
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(ahrs_init(0.1f));

    float ax, ay, az, gx, gy, gz, mx = 0, my = 0, mz = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        if (imu_update(ax, ay, az, gx, gy, gz, mx, my, mz)) {
            ahrs_update(1000.0f, ax, ay, az, gx, gy, gz, mx, my, mz);
        }

        ahrs_euler_t euler = ahrs_get_euler();
        printf("Roll: %.2f  Pitch: %.2f  Yaw: %.2f\n",
               euler.roll, euler.pitch, euler.yaw);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}