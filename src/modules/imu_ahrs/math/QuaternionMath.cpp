#include "QuaternionMath.hpp"
#include <cmath>
#include <algorithm>

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
        // 计算四元数模长
        float mag = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        // 防止除零
        mag = std::max(mag, 1e-12f);
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    }

}
