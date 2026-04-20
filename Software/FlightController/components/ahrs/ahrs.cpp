#include "ahrs.hpp"
#include "madgwick_filter.hpp"

static espp::MadgwickFilter *filter = nullptr;

esp_err_t ahrs_init(float beta) {
    filter = new espp::MadgwickFilter(beta);
    return filter ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t ahrs_update(float sample_rate, float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz) {
    if (!filter) return ESP_ERR_INVALID_STATE;
    filter->update(1.0f / sample_rate, ax, ay, az, gx, gy, gz, mx, my, mz);
    return ESP_OK;
}

ahrs_euler_t ahrs_get_euler() {
    ahrs_euler_t euler = {};
    if (filter) {
        filter->get_euler(euler.roll, euler.pitch, euler.yaw);
    }
    return euler;
}