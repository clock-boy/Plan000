#pragma once
#include "esp_err.h"

esp_err_t imu_init();
esp_err_t imu_read(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz,
                   float &mx, float &my, float &mz);
bool imu_update(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz,
                float &mx, float &my, float &mz);