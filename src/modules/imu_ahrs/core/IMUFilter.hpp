#ifndef IMUFILTER_HPP
#define IMUFILTER_HPP

/**
 * @file IMUFilter.hpp
 * @brief IMU数据预处理滤波器头文件
 *
 * 本类封装了对IMU六轴数据进行预处理滤波的算法，
 * 内部调用 filters 下的 Kalman1D 对加速度计数据滤波，
 * 调用 Kalman2D 对陀螺仪数据滤波，并可根据运动状态动态调整陀螺仪滤波器参数。
 */

#include <cstddef>
#include "Kalman1D.hpp"
#include "Kalman2D.hpp"

class IMUFilter
{
public:
    /**
     * @brief 构造函数
     */
    IMUFilter();

    /**
     * @brief 对原始IMU数据进行滤波
     * @param ax_raw 原始X轴加速度，单位 m/s^2
     * @param ay_raw 原始Y轴加速度，单位 m/s^2
     * @param az_raw 原始Z轴加速度，单位 m/s^2
     * @param gx_raw 原始X轴角速度，单位 rad/s
     * @param gy_raw 原始Y轴角速度，单位 rad/s
     * @param gz_raw 原始Z轴角速度，单位 rad/s
     * @param dt_s 采样周期，单位秒
     * @param ax_f 输出滤波后的X轴加速度
     * @param ay_f 输出滤波后的Y轴加速度
     * @param az_f 输出滤波后的Z轴加速度
     * @param gx_f 输出滤波后的X轴角速度
     * @param gy_f 输出滤波后的Y轴角速度
     * @param gz_f 输出滤波后的Z轴角速度
     */
    void update_all(float ax_raw, float ay_raw, float az_raw,
                    float gx_raw, float gy_raw, float gz_raw,
                    float dt_s,
                    float &ax_f, float &ay_f, float &az_f,
                    float &gx_f, float &gy_f, float &gz_f);

private:
    // 三轴加速度滤波器，使用一维卡尔曼滤波器
    Kalman1D _accelX;
    Kalman1D _accelY;
    Kalman1D _accelZ;

    // 三轴陀螺仪滤波器，使用二维卡尔曼滤波器（状态：[angle, rate]）
    Kalman2D _gyroX2D;
    Kalman2D _gyroY2D;
    Kalman2D _gyroZ2D;

    // 用于存储滤波后陀螺仪积分得到的角度（主要供调试参考）
    float _gyroAngleX;
    float _gyroAngleY;
    float _gyroAngleZ;

    /**
     * @brief 判断运动状态：静止、低动态或高动态
     * @param acc_norm 加速度范数
     * @param gyro_norm 陀螺仪角速度范数
     * @return 运动状态字符串："static", "low_dynamic", 或 "high_dynamic"
     */
    const char* detect_motion_state(float acc_norm, float gyro_norm);

    /**
     * @brief 根据运动状态调整二维卡尔曼滤波器参数（base_q）
     * @param motion_state 运动状态字符串
     */
    void adjust_gyro_parameters(const char* motion_state);
};

#endif // IMUFILTER_HPP
