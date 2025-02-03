#ifndef QUATERNIONMATH_HPP
#define QUATERNIONMATH_HPP

/**
 * @file QuaternionMath.hpp
 * @brief 四元数运算相关函数头文件
 *
 * 本文件提供四元数乘法和归一化的基本函数，
 * 用于实现姿态融合时的四元数计算。
 */

namespace QuaternionMath {

    /**
     * @brief 对两个四元数进行乘法运算（Hamilton积）。
     * @param q1 第一个四元数，数组形式 [w, x, y, z]
     * @param q2 第二个四元数，数组形式 [w, x, y, z]
     * @param result 将乘法结果存储到 result 数组中（长度为 4）
     */
    void multiply(const float q1[4], const float q2[4], float result[4]);

    /**
     * @brief 对四元数进行归一化处理。
     * @param q 待归一化的四元数，数组形式 [w, x, y, z]，归一化结果直接存回 q
     */
    void normalize(float q[4]);

}

#endif // QUATERNIONMATH_HPP
