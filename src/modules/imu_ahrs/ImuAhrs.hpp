/****************************************************************************
 * ImuAhrs.hpp - IMU 姿态融合模块主头文件
 *
 * 本文件声明了 ImuAhrs 类，该类实现 PX4 模块接口，
 * 包括模块初始化、Run() 主循环及其他管理接口（启动、停止、状态打印）。
 *
 * 模块主要功能：
 *  - 订阅 uORB 上的 sensor_combined 消息，获取 IMU 原始数据（加速度、角速度等）。
 *  - 调用核心滤波器和融合算法（通过 core 层和 math 层的函数）计算姿态，
 *    得到欧拉角与四元数。
 *  - 发布 vehicle_attitude 和自定义的 imu_ahrs_status 消息到 uORB 总线上。
 *  - 订阅 parameter_update 消息以支持参数动态更新。
 *
 * 同时，通过 DEFINE_PARAMETERS 宏定义了模块所需的参数句柄，
 * 方便参数更新与动态配置。
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/imu_ahrs_status.h>
#include <uORB/topics/parameter_update.h>   // 添加参数更新话题

// 引入核心算法相关头文件
#include "core/IMUFilter.hpp"
#include "core/YawDriftCompensator.hpp"

using matrix::Quatf;
using matrix::Vector3f;
using matrix::Eulerf;

class ImuAhrs : public ModuleBase<ImuAhrs>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    ImuAhrs();
    ~ImuAhrs() override;

    /** @brief 初始化模块 */
    bool init();

    /** @brief 模块任务启动接口 */
    static int task_spawn(int argc, char *argv[]);

    /** @brief 模块状态打印接口 */
    static int print_status();

    /** @brief 自定义命令处理接口 */
    static int custom_command(int argc, char *argv[]);

    /** @brief 使用说明打印接口 */
    static int print_usage(const char *reason = nullptr);

protected:
    /** @brief 周期性任务执行函数 */
    void Run() override;

    /** @brief 参数更新处理函数 */
    void parameters_update(bool force = false);

private:
    // 使用 DEFINE_PARAMETERS 宏定义模块参数句柄，
    // 参数名称应与 PX4 参数系统中定义的一致
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_IMU_AHRS>) _param_sys_imu_ahrs,
        (ParamInt<px4::params::IMU_AHRS_FREQ_HZ>) _param_imu_ahrs_freq_hz,
        (ParamFloat<px4::params::IMU_AHRS_W_ACC>) _param_imu_ahrs_w_acc,
        (ParamFloat<px4::params::IMU_AHRS_W_MAG>) _param_imu_ahrs_w_mag
    )

    // uORB 消息订阅对象
    uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB 消息发布对象
    uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
    uORB::Publication<imu_ahrs_status_s> _ahrs_status_pub{ORB_ID(imu_ahrs_status)};

    // 核心算法对象
    core::IMUFilter _imuFilter;
    core::YawDriftCompensator _yawDriftComp;

    // 内部状态变量：当前姿态四元数（数组形式）、采样周期及上次更新时间戳
    float _q[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float _dt{0.0f};
    hrt_abstime _last_update{0};

    // 模块运行状态
    bool _initialized{false};

    // 私有辅助函数：更新 IMU 数据、融合姿态、以及各类数学运算
    void updateIMU();
    void updateAHRS();

    // 四元数与欧拉角计算相关函数
    void _gyro_to_quat_delta(float gx, float gy, float gz, float dt_s, float dq[4]);
    void _quat_multiply(const float q1[4], const float q2[4], float result[4]);
    void _quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);
    void _normalize_quat(float q[4]);
    void _complementary_accel(const float pred_q[4], float ax_f, float ay_f, float az_f, float fused_q[4]);
    void _initialize_by_accel(float ax, float ay, float az);
    void _compute_gravity_ignore_yaw(float roll, float pitch, float *gpx, float *gpy, float *gpz);
};
