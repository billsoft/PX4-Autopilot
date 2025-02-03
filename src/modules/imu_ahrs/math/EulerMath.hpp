#ifndef EULERMATH_HPP
#define EULERMATH_HPP

/**
 * @file EulerMath.hpp
 * @brief 欧拉角运算相关函数头文件
 *
 * 本文件提供欧拉角与四元数之间的互转函数，
 * 采用 Z-Y-X 顺序（偏航、俯仰、横滚）。
 */

namespace EulerMath {

    /**
     * @brief 将欧拉角转换为四元数。
     * @param roll 横滚角（单位：rad）
     * @param pitch 俯仰角（单位：rad）
     * @param yaw 偏航角（单位：rad）
     * @param q 输出四元数，数组形式 [w, x, y, z]
     */
    void eulerToQuaternion(float roll, float pitch, float yaw, float q[4]);

    /**
     * @brief 将四元数转换为欧拉角。
     * @param q 输入四元数，数组形式 [w, x, y, z]
     * @param roll 输出横滚角（单位：rad）
     * @param pitch 输出俯仰角（单位：rad）
     * @param yaw 输出偏航角（单位：rad）
     */
    void quaternionToEuler(const float q[4], float &roll, float &pitch, float &yaw);

}

#endif // EULERMATH_HPP
