#include "YawDriftCompensator.hpp"
#include <algorithm>
#include <cmath>

YawDriftCompensator::YawDriftCompensator(float beta)
    : _beta(beta), _yaw_bias(0.0f)
{
}

float YawDriftCompensator::update_bias(float gyro_z, float dt, bool is_static)
{
    // 如果处于静止或低动态状态，则更新偏置（采用指数平滑）
    if (is_static) {
        _yaw_bias = _beta * _yaw_bias + (1.0f - _beta) * gyro_z;
    }
    // 返回经过偏置补偿后的yaw轴角速度
    float gz_comp = gyro_z - _yaw_bias;
    return gz_comp;
}

float YawDriftCompensator::get_yaw_bias() const
{
    return _yaw_bias;
}
