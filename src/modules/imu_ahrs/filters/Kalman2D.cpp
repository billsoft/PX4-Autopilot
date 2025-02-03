#include "Kalman2D.hpp"
#include <cmath>
#include <algorithm>

// 构造函数初始化状态向量和协方差矩阵
Kalman2D::Kalman2D(float angle_init, float rate_init, float base_q, float r)
    : _base_q(base_q), _R(r)
{
    _x[0] = angle_init;
    _x[1] = rate_init;
    // 初始化协方差矩阵，假设初始角度和角速度的不确定性
    float sigma_angle = 0.087f; // 约5度（弧度）
    float sigma_rate = 0.01f;   // 角速度标准差
    float rho = 0.8f;           // 角度与角速度之间的相关系数
    _P[0][0] = sigma_angle * sigma_angle;
    _P[0][1] = rho * sigma_angle * sigma_rate;
    _P[1][0] = _P[0][1];
    _P[1][1] = sigma_rate * sigma_rate;
}

// 二维卡尔曼滤波更新函数实现
void Kalman2D::update(float rate_meas, float dt_s, float &angle, float &rate)
{
    // 状态转移矩阵 F = [[1, dt_s], [0, 1]]
    float F[2][2] = { {1.0f, dt_s}, {0.0f, 1.0f} };

    // 预测步骤：计算 x_pred = F * _x
    float x_pred[2];
    x_pred[0] = _x[0] + _x[1] * dt_s;
    x_pred[1] = _x[1];

    // 过程噪声矩阵 Q = base_q * [ [0.25*dt_s^4, 0.5*dt_s^3], [0.5*dt_s^3, dt_s^2] ]
    float Q[2][2] = {
        {0.25f * dt_s * dt_s * dt_s * dt_s, 0.5f * dt_s * dt_s * dt_s},
        {0.5f * dt_s * dt_s * dt_s, dt_s * dt_s}
    };
    Q[0][0] *= _base_q;
    Q[0][1] *= _base_q;
    Q[1][0] *= _base_q;
    Q[1][1] *= _base_q;

    // 预测协方差矩阵 P_pred = F * _P * F^T + Q
    float FP[2][2]; // 临时变量: F * _P
    FP[0][0] = F[0][0] * _P[0][0] + F[0][1] * _P[1][0];
    FP[0][1] = F[0][0] * _P[0][1] + F[0][1] * _P[1][1];
    FP[1][0] = F[1][0] * _P[0][0] + F[1][1] * _P[1][0];
    FP[1][1] = F[1][0] * _P[0][1] + F[1][1] * _P[1][1];

    float P_pred[2][2];
    // P_pred = FP * F^T + Q，F^T = [[F[0][0], F[1][0]], [F[0][1], F[1][1]]]
    P_pred[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[0][1] + Q[0][0];
    P_pred[0][1] = FP[0][0] * F[1][0] + FP[0][1] * F[1][1] + Q[0][1];
    P_pred[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[0][1] + Q[1][0];
    P_pred[1][1] = FP[1][0] * F[1][0] + FP[1][1] * F[1][1] + Q[1][1];

    // 观测矩阵 H = [0, 1]，观测仅为角速度
    float H[2] = {0.0f, 1.0f};

    // 计算创新协方差 S = H * P_pred * H^T + R
    float S = H[0] * P_pred[0][0] * H[0] +
              2.0f * H[0] * P_pred[0][1] * H[1] +
              H[1] * P_pred[1][1] * H[1] +
              _R;
    S = std::max(S, 1e-12f); // 防止除0

    // 计算卡尔曼增益 K = P_pred * H^T / S（K 为2x1向量）
    float K[2];
    K[0] = (P_pred[0][0] * H[0] + P_pred[0][1] * H[1]) / S;
    K[1] = (P_pred[1][0] * H[0] + P_pred[1][1] * H[1]) / S;

    // 测量残差 y = rate_meas - (H*x_pred) ，H*x_pred = x_pred[1]
    float y = rate_meas - x_pred[1];

    // 更新状态：_x = x_pred + K*y
    _x[0] = x_pred[0] + K[0] * y;
    _x[1] = x_pred[1] + K[1] * y;

    // 更新协方差矩阵：P = (I - K*H)*P_pred
    float I_KH[2][2] = {
        {1.0f - K[0] * H[0], -K[0] * H[1]},
        {-K[1] * H[0], 1.0f - K[1] * H[1]}
    };
    float newP[2][2];
    newP[0][0] = I_KH[0][0] * P_pred[0][0] + I_KH[0][1] * P_pred[1][0];
    newP[0][1] = I_KH[0][0] * P_pred[0][1] + I_KH[0][1] * P_pred[1][1];
    newP[1][0] = I_KH[1][0] * P_pred[0][0] + I_KH[1][1] * P_pred[1][0];
    newP[1][1] = I_KH[1][0] * P_pred[0][1] + I_KH[1][1] * P_pred[1][1];

    _P[0][0] = newP[0][0];
    _P[0][1] = newP[0][1];
    _P[1][0] = newP[1][0];
    _P[1][1] = newP[1][1];

    // 输出更新后的状态
    angle = _x[0];
    rate  = _x[1];
}
