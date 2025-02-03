#include "EulerMath.hpp"
#include <cmath>
#include <algorithm>

namespace EulerMath {

    void eulerToQuaternion(float roll, float pitch, float yaw, float q[4])
    {
        // 计算各角的半角值
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        // 根据公式计算四元数
        q[0] = cr * cp * cy + sr * sp * sy;  // w
        q[1] = sr * cp * cy - cr * sp * sy;  // x
        q[2] = cr * sp * cy + sr * cp * sy;  // y
        q[3] = cr * cp * sy - sr * sp * cy;  // z

        // 归一化四元数
        float mag = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        mag = std::max(mag, 1e-12f);
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    }

    void quaternionToEuler(const float q[4], float &roll, float &pitch, float &yaw)
    {
        // 确保输入四元数归一化
        float w = q[0], x = q[1], y = q[2], z = q[3];
        float mag = std::sqrt(w*w + x*x + y*y + z*z);
        mag = std::max(mag, 1e-12f);
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;

        // 计算横滚角（roll）
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // 计算俯仰角（pitch）
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f)
            pitch = std::copysign(M_PI / 2.0f, sinp);  // 防止超出范围
        else
            pitch = std::asin(sinp);

        // 计算偏航角（yaw）
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

}
