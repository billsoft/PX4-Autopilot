#include "YawDriftCompensator.hpp"
#include <cmath>
#include <cstdlib>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

namespace imu_ahrs {

YawDriftCompensator::YawDriftCompensator(float beta)
    : _beta(beta),
      _yaw_bias(0.0f)
{
}

void YawDriftCompensator::update_bias(float gyro_z)
{
    _yaw_bias = _beta * _yaw_bias + (1.0f - _beta) * gyro_z;
}

float YawDriftCompensator::compensate(float gyro_z)
{
    return gyro_z - _yaw_bias;
}

void YawDriftCompensator::reset()
{
    _yaw_bias = 0.0f;
}

} // namespace imu_ahrs
