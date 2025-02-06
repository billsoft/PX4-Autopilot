#include "IMUFilter.hpp"
#include <cmath>
#include <cstdlib>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

IMUFilter::IMUFilter()
    : _accelX(0.0f, 1.0f, 0.001f, 0.1f),
      _accelY(0.0f, 1.0f, 0.001f, 0.1f),
      _accelZ(9.81f, 1.0f, 0.001f, 0.1f),
      _gyroX2D(0.0f, 0.0f, 1e-3f, 0.1f),
      _gyroY2D(0.0f, 0.0f, 1e-3f, 0.1f),
      _gyroZ2D(0.0f, 0.0f, 0.5f, 1e-4f),
      _gyroAngleX(0.0f),
      _gyroAngleY(0.0f),
      _gyroAngleZ(0.0f)
{
}

void IMUFilter::update_all(float ax_raw, float ay_raw, float az_raw,
                           float gx_raw, float gy_raw, float gz_raw,
                           float dt_s,
                           float &ax_f, float &ay_f, float &az_f,
                           float &gx_f, float &gy_f, float &gz_f)
{
    // 1. 计算加速度与陀螺仪范数
    float acc_norm = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
    float gyro_norm = sqrtf(gx_raw*gx_raw + gy_raw*gy_raw + gz_raw*gz_raw);

    // 2. 判断运动状态，并调整二维卡尔曼滤波器的过程噪声参数
    const char* motion_state = detect_motion_state(acc_norm, gyro_norm);
    adjust_gyro_parameters(motion_state);

    // 3. 对加速度数据进行一维卡尔曼滤波
    ax_f = _accelX.update(ax_raw);
    ay_f = _accelY.update(ay_raw);
    az_f = _accelZ.update(az_raw);

    // 4. 对陀螺仪数据使用二维卡尔曼滤波器更新
    _gyroX2D.update(gx_raw, dt_s, _gyroAngleX, gx_f);
    _gyroY2D.update(gy_raw, dt_s, _gyroAngleY, gy_f);
    _gyroZ2D.update(gz_raw, dt_s, _gyroAngleZ, gz_f);
}

// 根据加速度与陀螺仪数据的范数判断运动状态
const char* IMUFilter::detect_motion_state(float acc_norm, float gyro_norm)
{
    if (gyro_norm < 0.1f && fabsf(acc_norm - 9.81f) < 0.2f) {
        return "static";
    } else if (gyro_norm < 0.5f) {
        return "low_dynamic";
    } else {
        return "high_dynamic";
    }
}

// 根据运动状态调整二维卡尔曼滤波器中用于陀螺仪的 base_q 参数（仅对X和Y轴做调整）
void IMUFilter::adjust_gyro_parameters(const char* motion_state)
{
    float new_q;
    if (strcmp(motion_state, "static") == 0) {
        new_q = 1e-4f;
    } else if (strcmp(motion_state, "low_dynamic") == 0) {
        new_q = 1e-3f;
    } else {
        new_q = 1e-2f;
    }
    _gyroX2D.setBaseQ(new_q);
    _gyroY2D.setBaseQ(new_q);
    // 对于Z轴可以保持较大噪声，通常不调整或根据需要调整
}
