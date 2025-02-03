#ifndef KALMAN1D_HPP
#define KALMAN1D_HPP

/**
 * @file Kalman1D.hpp
 * @brief 一维卡尔曼滤波器头文件
 *
 * 用于对单一轴（如加速度计某轴）数据进行滤波处理。
 * 采用简单的卡尔曼滤波算法：
 *   预测：x_pred = x, p_pred = p + q
 *   更新：K = p_pred / (p_pred + r)
 *         x = x_pred + K*(z - x_pred)
 *         p = (1-K)*p_pred
 */

class Kalman1D
{
public:
    /**
     * @brief 构造函数
     * @param init_x 初始状态值
     * @param init_p 初始协方差
     * @param q 过程噪声
     * @param r 测量噪声
     */
    Kalman1D(float init_x = 0.0f, float init_p = 1.0f, float q = 0.001f, float r = 0.1f);

    /**
     * @brief 单步更新
     * @param z 当前测量值
     * @return 滤波后的状态值
     */
    float update(float z);

private:
    float _x; // 状态值
    float _p; // 状态协方差
    float _q; // 过程噪声
    float _r; // 测量噪声
};

#endif // KALMAN1D_HPP
