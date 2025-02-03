/****************************************************************************
 * ImuAhrs.hpp - IMU 姿态融合模块主头文件
 *
 * 本文件声明了 ImuAhrs 类，该类实现 PX4 模块接口，
 * 包括模块初始化、Run() 主循环及其他管理接口（启动、停止、状态打印）。
 *
 * 模块主要功能：
 *  - 订阅 uORB 上的 sensor_combined 消息，获取 IMU 原始数据（加速度、角速度等）。
 *  - 调用内部滤波器和融合算法（通过 core 层和 math 层的函数）计算姿态，
 *    得到欧拉角与四元数。
 *  - 发布 vehicle_attitude 和自定义的 imu_ahrs_status 消息到 uORB 总线上。
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

using matrix::Quatf;
using matrix::Vector3f;
using matrix::Eulerf;

/**
 * @brief ImuAhrs 模块类
 *
 * 该类继承自 ModuleBase、ModuleParams 与 ScheduledWorkItem，
 * 实现了模块的初始化、任务循环 (Run()) 以及状态打印接口。
 */
class ImuAhrs : public ModuleBase<ImuAhrs>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// 构造与析构函数
	ImuAhrs();
	~ImuAhrs() override;

	// 模块管理接口
	static int task_spawn(int argc, char *argv[]);
	static ImuAhrs *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	// 模块运行接口
	void Run() override;
	bool init();
	int print_status() override;

private:
	// 订阅 IMU 原始数据 (sensor_combined)
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};

	// 发布处理后姿态数据 (vehicle_attitude)
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	// 发布自定义状态数据 (imu_ahrs_status)
	uORB::Publication<imu_ahrs_status_s> _ahrs_status_pub{ORB_ID(imu_ahrs_status)};

	// 上一次更新时间戳（单位：微秒）
	hrt_abstime _last_update{0};
	// 当前采样间隔（秒）
	float _dt{0.0f};

	// 内部处理函数：分别更新 IMU 数据与融合姿态
	void updateIMU();
	void updateAHRS();

	// 辅助数学运算函数：四元数转欧拉角、归一化等
	Vector3f quaternion_to_euler(const float q[4]);
	void normalize_quaternion(float q[4]);
};

