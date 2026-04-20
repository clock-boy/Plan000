#pragma once
#include "esp_err.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
} ahrs_euler_t;

esp_err_t ahrs_init(float beta);
esp_err_t ahrs_update(float sample_rate, float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz);
ahrs_euler_t ahrs_get_euler();