#include "QuaternionMath.hpp"
#include <cmath>
#include <cstdlib>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

namespace QuaternionMath {

    void multiply(const float q1[4], const float q2[4], float result[4])
    {
        // 计算 Hamilton 积：result = q1 * q2
        float w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
        float x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
        float y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
        float z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
        result[0] = w;
        result[1] = x;
        result[2] = y;
        result[3] = z;

        // 对结果进行归一化
        normalize(result);
    }

    void normalize(float q[4])
    {
        float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm > 1e-12f) {
            q[0] /= norm;
            q[1] /= norm;
            q[2] /= norm;
            q[3] /= norm;
        }
    }

    void toEuler(const float q[4], float *roll, float *pitch, float *yaw)
    {
        float q0q0 = q[0] * q[0];
        float q0q1 = q[0] * q[1];
        float q0q2 = q[0] * q[2];
        float q0q3 = q[0] * q[3];
        float q1q1 = q[1] * q[1];
        float q1q2 = q[1] * q[2];
        float q1q3 = q[1] * q[3];
        float q2q2 = q[2] * q[2];
        float q2q3 = q[2] * q[3];
        float q3q3 = q[3] * q[3];

        *roll = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
        *pitch = asinf(2.0f * (q0q2 - q1q3));
        *yaw = atan2f(2.0f * (q0q3 + q1q2), q0q0 + q1q1 - q2q2 - q3q3);
    }

    void fromEuler(float roll, float pitch, float yaw, float q[4])
    {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;
    }

}
