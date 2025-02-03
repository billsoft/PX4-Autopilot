#include "Kalman1D.hpp"

// 构造函数初始化状态、协方差、过程噪声与测量噪声
Kalman1D::Kalman1D(float init_x, float init_p, float q, float r)
    : _x(init_x), _p(init_p), _q(q), _r(r)
{
}

// 单步卡尔曼滤波更新函数
float Kalman1D::update(float z)
{
    // 预测步骤：状态预测不变，协方差增加过程噪声
    float x_pred = _x;
    float p_pred = _p + _q;

    // 计算卡尔曼增益
    float K = p_pred / (p_pred + _r);

    // 更新状态和协方差
    _x = x_pred + K * (z - x_pred);
    _p = (1.0f - K) * p_pred;

    return _x;
}
