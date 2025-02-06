#include "EulerMath.hpp"
#include <cmath>
#include <cstdlib>
#include <px4_platform_common/defines.h>
#include <mathlib/math/Functions.hpp>

namespace imu_ahrs {

    void eulerToQuaternion(float roll, float pitch, float yaw, float q[4])
    {
        // 计算各角的半角值
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);

        // 根据公式计算四元数
        q[0] = cr * cp * cy + sr * sp * sy;  // w
        q[1] = sr * cp * cy - cr * sp * sy;  // x
        q[2] = cr * sp * cy + sr * cp * sy;  // y
        q[3] = cr * cp * sy - sr * sp * cy;  // z

        // 归一化四元数
        float mag = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        mag = math::max(mag, 1e-12f);
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    }

    void quaternion_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
    {
        float q0q0 = q[0] * q[0];
        float q1q1 = q[1] * q[1];
        float q2q2 = q[2] * q[2];
        float q3q3 = q[3] * q[3];

        *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q0q0 - q1q1 - q2q2 + q3q3);
        *pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
        *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), q0q0 + q1q1 - q2q2 - q3q3);
    }

    void normalize_angle(float *angle)
    {
        while (*angle > M_PI_F) {
            *angle -= 2.0f * M_PI_F;
        }
        while (*angle < -M_PI_F) {
            *angle += 2.0f * M_PI_F;
        }
    }

    void compute_gravity_vector(float roll, float pitch, float yaw, float *gx, float *gy, float *gz)
    {
        float sr = sinf(roll);
        float cp = cosf(pitch);
        float sp = sinf(pitch);

        *gx = sp;
        *gy = -sr * cp;
        *gz = sr * sp;
    }

    void compute_gravity_vector_no_yaw(float roll, float pitch, float *gx, float *gy, float *gz)
    {
        float cp = cosf(pitch);
        float sp = sinf(pitch);

        *gx = 0.0f;
        *gy = cp;
        *gz = sp;
    }

    void accel_to_euler(float ax, float ay, float az, float *roll, float *pitch)
    {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 1e-6f) {
            *roll = 0.0f;
            *pitch = 0.0f;
            return;
        }

        ax /= norm;
        ay /= norm;
        az /= norm;

        *roll = atan2f(ay, az);
        *pitch = -asinf(ax);
    }

}
