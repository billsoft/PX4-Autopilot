#ifndef YAWDRIFTCOMPENSATOR_HPP
#define YAWDRIFTCOMPENSATOR_HPP

/**
 * @file YawDriftCompensator.hpp
 * @brief Yaw漂移补偿器头文件
 *
 * 本类实现基于零速率检测（ZUPT）的Yaw偏置估计与补偿功能，
 * 当检测到系统处于静止或低动态时，更新yaw轴的偏置，
 * 并在后续更新中对陀螺仪z轴角速度进行补偿，减少长时间积分产生的漂移。
 */

class YawDriftCompensator
{
public:
    /**
     * @brief 构造函数
     * @param beta 指数平滑系数，越接近1表示偏置更新越缓慢，推荐值例如0.99
     */
    explicit YawDriftCompensator(float beta = 0.99f);

    /**
     * @brief 根据当前角速度、采样周期及是否静止的判断，更新yaw偏置，并返回补偿后的角速度
     * @param gyro_z 当前滤波后yaw轴角速度，单位：rad/s
     * @param dt 采样周期，单位：秒
     * @param is_static 是否处于静止或低动态状态
     * @return 补偿后的yaw轴角速度（rad/s）
     */
    float update_bias(float gyro_z, float dt, bool is_static);

    /// 获取当前估计的yaw偏置
    float get_yaw_bias() const;

private:
    float _beta;      ///< 指数平滑系数
    float _yaw_bias;  ///< 当前估计的yaw偏置
};

#endif // YAWDRIFTCOMPENSATOR_HPP
