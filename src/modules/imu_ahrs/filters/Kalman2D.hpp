#ifndef KALMAN2D_HPP
#define KALMAN2D_HPP

/**
 * @file Kalman2D.hpp
 * @brief 二维卡尔曼滤波器头文件
 *
 * 用于对陀螺仪数据进行滤波处理。
 * 状态向量为 [angle, rate]，分别表示角度（积分值）与角速度。
 * 预测步骤采用状态转移矩阵 F = [[1, dt], [0, 1]]，过程噪声 Q 随采样周期 dt 动态缩放，
 * 测量仅为角速度。
 */

class Kalman2D
{
public:
    /**
     * @brief 构造函数
     * @param angle_init 初始角度（单位：rad）
     * @param rate_init 初始角速度（单位：rad/s）
     * @param base_q 基准过程噪声
     * @param r 测量噪声，默认值为 1e-3
     */
    Kalman2D(float angle_init = 0.0f, float rate_init = 0.0f, float base_q = 1e-5f, float r = 1e-3f);

    /**
     * @brief 单步更新滤波器
     * @param rate_meas 当前时刻角速度测量（单位：rad/s）
     * @param dt_s 采样周期（秒）
     * @param angle 输出滤波后的角度（单位：rad）
     * @param rate 输出滤波后的角速度（单位：rad/s）
     */
    void update(float rate_meas, float dt_s, float &angle, float &rate);

    /**
     * @brief 获取当前滤波状态角度
     */
    float getAngle() const { return _x[0]; }
    /**
     * @brief 获取当前滤波状态角速度
     */
    float getRate() const { return _x[1]; }

    /**
     * @brief 设置过程噪声基准值
     */
    void setBaseQ(float q) { _base_q = q; }

private:
    float _x[2];    // 状态向量：_x[0] 为角度, _x[1] 为角速度
    float _P[2][2]; // 状态协方差矩阵 (2x2)
    float _R;       // 测量噪声（标量）
    float _base_q;  // 过程噪声基准值
};

#endif // KALMAN2D_HPP
