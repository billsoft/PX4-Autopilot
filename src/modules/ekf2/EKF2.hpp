/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file EKF2.hpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */
#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h" // 引入扩展卡尔曼滤波器的头文件

#include "EKF2Selector.hpp" // 引入EKF2选择器的头文件
#include "mathlib/math/filter/AlphaFilter.hpp" // 引入Alpha滤波器的头文件

#include <float.h> // 引入浮点数相关的头文件

#include <containers/LockGuard.hpp> // 引入锁保护的头文件
#include <drivers/drv_hrt.h> // 引入高分辨率定时器驱动的头文件
#include <lib/mathlib/mathlib.h> // 引入数学库的头文件
#include <lib/perf/perf_counter.h> // 引入性能计数器的头文件
#include <lib/systemlib/mavlink_log.h> // 引入MAVLink日志的头文件
#include <px4_platform_common/defines.h> // 引入平台通用定义的头文件
#include <px4_platform_common/module.h> // 引入模块相关的头文件
#include <px4_platform_common/module_params.h> // 引入模块参数的头文件
#include <px4_platform_common/posix.h> // 引入POSIX相关的头文件
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp> // 引入调度工作项的头文件
#include <px4_platform_common/time.h> // 引入时间相关的头文件
#include <uORB/Publication.hpp> // 引入uORB发布的头文件
#include <uORB/PublicationMulti.hpp> // 引入uORB多重发布的头文件
#include <uORB/Subscription.hpp> // 引入uORB订阅的头文件
#include <uORB/SubscriptionCallback.hpp> // 引入uORB订阅回调的头文件
#include <uORB/SubscriptionMultiArray.hpp> // 引入uORB多重数组订阅的头文件
#include <uORB/topics/ekf2_timestamps.h> // 引入EKF2时间戳主题的头文件
#include <uORB/topics/estimator_bias.h> // 引入估计器偏置主题的头文件
#include <uORB/topics/estimator_bias3d.h> // 引入三维估计器偏置主题的头文件
#include <uORB/topics/estimator_event_flags.h> // 引入估计器事件标志主题的头文件
#include <uORB/topics/estimator_innovations.h> // 引入估计器创新主题的头文件
#include <uORB/topics/estimator_sensor_bias.h> // 引入估计器传感器偏置主题的头文件
#include <uORB/topics/estimator_states.h> // 引入估计器状态主题的头文件
#include <uORB/topics/estimator_status.h> // 引入估计器状态主题的头文件
#include <uORB/topics/estimator_status_flags.h> // 引入估计器状态标志主题的头文件
#include <uORB/topics/launch_detection_status.h> // 引入发射检测状态主题的头文件
#include <uORB/topics/parameter_update.h> // 引入参数更新主题的头文件
#include <uORB/topics/sensor_combined.h> // 引入传感器组合主题的头文件
#include <uORB/topics/sensor_selection.h> // 引入传感器选择主题的头文件
#include <uORB/topics/vehicle_attitude.h> // 引入车辆姿态主题的头文件
#include <uORB/topics/vehicle_command.h> // 引入车辆命令主题的头文件
#include <uORB/topics/vehicle_command_ack.h> // 引入车辆命令确认主题的头文件
#include <uORB/topics/vehicle_global_position.h> // 引入车辆全球位置主题的头文件
#include <uORB/topics/vehicle_imu.h> // 引入车辆IMU主题的头文件
#include <uORB/topics/vehicle_land_detected.h> // 引入车辆着陆检测主题的头文件
#include <uORB/topics/vehicle_local_position.h> // 引入车辆本地位置主题的头文件
#include <uORB/topics/vehicle_odometry.h> // 引入车辆里程计主题的头文件
#include <uORB/topics/vehicle_status.h> // 引入车辆状态主题的头文件
#include <uORB/topics/yaw_estimator_status.h> // 引入偏航估计器状态主题的头文件

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h> // 引入气速主题的头文件
# include <uORB/topics/airspeed_validated.h> // 引入验证气速主题的头文件
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h> // 引入着陆目标姿态主题的头文件
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
# include <uORB/topics/vehicle_air_data.h> // 引入车辆气压数据主题的头文件
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
# include <uORB/topics/estimator_gps_status.h> // 引入估计器GPS状态主题的头文件
# include <uORB/topics/sensor_gps.h> // 引入传感器GPS主题的头文件
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
# include <uORB/topics/vehicle_magnetometer.h> // 引入车辆磁力计主题的头文件
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h> // 引入车辆光流主题的头文件
# include <uORB/topics/vehicle_optical_flow_vel.h> // 引入车辆光流速度主题的头文件
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h> // 引入距离传感器主题的头文件
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_WIND)
# include <uORB/topics/wind.h> // 引入风主题的头文件
#endif // CONFIG_EKF2_WIND

extern pthread_mutex_t ekf2_module_mutex; // 声明EKF2模块的互斥锁

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem // EKF2类，继承自ModuleParams和ScheduledWorkItem
{
public:
	EKF2() = delete; // 禁止默认构造函数
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode); // 构造函数，接收多模式、工作队列配置和重放模式参数
	~EKF2() override; // 析构函数

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 任务生成函数

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 自定义命令函数

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印使用信息函数

	int print_status(bool verbose = false); // 打印状态函数，接收详细参数

	bool should_exit() const { return _task_should_exit.load(); } // 检查是否应该退出

	void request_stop() { _task_should_exit.store(true); } // 请求停止

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); } // 锁定模块
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); } // 尝试锁定模块
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); } // 解锁模块

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_init(int imu, int mag); // 多实例初始化函数，接收IMU和磁力计参数
#endif // CONFIG_EKF2_MULTI_INSTANCE

	int instance() const { return _instance; } // 获取实例编号

private:

	static constexpr uint8_t MAX_NUM_IMUS = 4; // 最大IMU数量
	static constexpr uint8_t MAX_NUM_MAGS = 4; // 最大磁力计数量

	void Run() override; // 重写运行函数

	void AdvertiseTopics(); // 发布主题函数
	void VerifyParams(); // 验证参数函数

	void PublishAidSourceStatus(const hrt_abstime &timestamp); // 发布辅助源状态函数，接收时间戳参数
	void PublishAttitude(const hrt_abstime &timestamp); // 发布姿态函数，接收时间戳参数

#if defined(CONFIG_EKF2_BAROMETER)
	void PublishBaroBias(const hrt_abstime &timestamp); // 发布气压偏置函数，接收时间戳参数
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	void PublishRngHgtBias(const hrt_abstime &timestamp); // 发布范围高度偏置函数，接收时间戳参数
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void PublishEvPosBias(const hrt_abstime &timestamp); // 发布外部视觉位置偏置函数，接收时间戳参数
#endif // CONFIG_EKF2_EXTERNAL_VISION
	estimator_bias_s fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
					      uint64_t timestamp, uint32_t device_id = 0); // 填充估计器偏置消息函数，接收状态、时间戳和设备ID参数
	void PublishEventFlags(const hrt_abstime &timestamp); // 发布事件标志函数，接收时间戳参数
	void PublishGlobalPosition(const hrt_abstime &timestamp); // 发布全球位置函数，接收时间戳参数
	void PublishInnovations(const hrt_abstime &timestamp); // 发布创新函数，接收时间戳参数
	void PublishInnovationTestRatios(const hrt_abstime &timestamp); // 发布创新测试比率函数，接收时间戳参数
	void PublishInnovationVariances(const hrt_abstime &timestamp); // 发布创新方差函数，接收时间戳参数
	void PublishLocalPosition(const hrt_abstime &timestamp); // 发布本地位置函数，接收时间戳参数
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample); // 发布里程计函数，接收时间戳和IMU样本参数
	void PublishSensorBias(const hrt_abstime &timestamp); // 发布传感器偏置函数，接收时间戳参数
	void PublishStates(const hrt_abstime &timestamp); // 发布状态函数，接收时间戳参数
	void PublishStatus(const hrt_abstime &timestamp); // 发布状态函数，接收时间戳参数
	void PublishStatusFlags(const hrt_abstime &timestamp); // 发布状态标志函数，接收时间戳参数
#if defined(CONFIG_EKF2_WIND)
	void PublishWindEstimate(const hrt_abstime &timestamp); // 发布风估计函数，接收时间戳参数
#endif // CONFIG_EKF2_WIND
#if defined(CONFIG_EKF2_AIRSPEED)
	// 更新气速样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	// 更新辅助速度样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	// 更新气压样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 更新外部视觉样本函数，接收时间戳结构体ekf2_timestamps作为参数
	// 返回值为布尔类型，表示更新是否成功
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// 将椭球高度转换为海平面高度（AMSL）
	// 参数ellipsoid_alt为椭球高度
	float altEllipsoidToAmsl(float ellipsoid_alt) const;

	// 将海平面高度（AMSL）转换为椭球高度
	// 参数amsl_alt为海平面高度
	float altAmslToEllipsoid(float amsl_alt) const;

	// 发布GPS状态函数，接收时间戳参数
	void PublishGpsStatus(const hrt_abstime &timestamp);

	// 发布GNSS高度偏置函数，接收时间戳参数
	void PublishGnssHgtBias(const hrt_abstime &timestamp);

	// 发布航向估计器状态函数，接收时间戳参数
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);

	// 更新GPS样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 更新光流样本函数，接收时间戳结构体ekf2_timestamps作为参数
	// 返回值为布尔类型，表示更新是否成功
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps);

	// 发布光流速度函数，接收时间戳参数
	void PublishOpticalFlowVel(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 更新磁力计样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 更新范围传感器样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER

	// 更新系统标志样本函数，接收时间戳结构体ekf2_timestamps作为参数
	void UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps);

	// 用于检查、保存和使用学习到的加速度计/陀螺仪/磁力计偏置的结构体
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< 上次EKF操作估计加速度计偏置的时间（微秒）
		hrt_abstime total_time_us{0};   ///< 自上次保存以来的累计校准时间（微秒）
		matrix::Vector3f bias{};        ///< 存储加速度计偏置的向量
		bool cal_available{false};      ///< 当存在未保存的有效校准时为真
	};

	// 更新校准函数，接收时间戳、校准结构体、偏置、偏置方差、偏置限制、偏置有效性和学习有效性作为参数
	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);

	// 更新加速度计校准函数，接收时间戳参数
	void UpdateAccelCalibration(const hrt_abstime &timestamp);

	// 更新陀螺仪校准函数，接收时间戳参数
	void UpdateGyroCalibration(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 更新磁力计校准函数，接收时间戳参数
	void UpdateMagCalibration(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_MAGNETOMETER

	// 发布估计器辅助源主题的帮助函数
	template <typename T>
	void PublishAidSourceStatus(const T &status, hrt_abstime &status_publish_last, uORB::PublicationMulti<T> &pub)
	{
		// 如果状态的时间戳样本大于上次发布的时间戳，则进行发布
		if (status.timestamp_sample > status_publish_last) {
			// 创建一个新的状态对象并进行发布
			T status_out{status};
			status_out.estimator_instance = _instance; // 设置估计器实例
			status_out.timestamp = hrt_absolute_time(); // 设置当前时间戳
			pub.publish(status_out); // 发布状态

			// 记录时间戳样本
			status_publish_last = status.timestamp_sample;
		}
	}

	// 计算平方的静态常量函数
	static constexpr float sq(float x) { return x * x; };

	const bool _replay_mode{false};			///< 当使用日志中的重放数据时为真
	const bool _multi_mode; // 多实例模式标志
	int _instance{0}; // 实例编号

	px4::atomic_bool _task_should_exit{false}; // 任务退出标志

	// 时间滑移监控
	uint64_t _integrated_time_us = 0;	///< 从开始起陀螺仪增量时间的积分（微秒）
	uint64_t _start_time_us = 0;		///< EKF开始时的系统时间（微秒）
	int64_t _last_time_slip_us = 0;		///< 上次时间滑移（微秒）

	perf_counter_t _ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": EKF update")}; // EKF更新性能计数器
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")}; // IMU消息丢失性能计数器

	InFlightCalibration _accel_cal{}; // 加速度计校准结构体
	InFlightCalibration _gyro_cal{}; // 陀螺仪校准结构体

	uint8_t _accel_calibration_count{0}; // 加速度计校准计数
	uint8_t _gyro_calibration_count{0}; // 陀螺仪校准计数

	uint32_t _device_id_accel{0}; // 加速度计设备ID
	uint32_t _device_id_gyro{0}; // 陀螺仪设备ID

	Vector3f _last_accel_bias_published{}; // 上次发布的加速度计偏置
	Vector3f _last_gyro_bias_published{}; // 上次发布的陀螺仪偏置

	hrt_abstime _last_sensor_bias_published{0}; // 上次发布的传感器偏置时间戳

	hrt_abstime _status_fake_hgt_pub_last{0}; // 上次发布虚假高度的时间戳
	hrt_abstime _status_fake_pos_pub_last{0}; // 上次发布虚假位置的时间戳

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uint32_t _device_id_mag {0}; // 磁力计设备ID

	// 用于控制保存磁偏角以便下次启动使用的标志
	bool _mag_decl_saved = false;	///< 当磁偏角已保存时为真

	InFlightCalibration _mag_cal{}; // 磁力计校准结构体
	uint8_t _mag_calibration_count{0}; // 磁力计校准计数
	Vector3f _last_mag_bias_published{}; // 上次发布的磁力计偏置

	hrt_abstime _status_mag_pub_last{0}; // 上次发布磁力计状态的时间戳

	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)}; // 磁力计订阅

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_mag_pub{ORB_ID(estimator_aid_src_mag)}; // 磁力计估计器辅助源发布
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_hgt_pub {ORB_ID(estimator_aid_src_ev_hgt)}; // 外部视觉高度估计器辅助源发布
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_ev_pos_pub{ORB_ID(estimator_aid_src_ev_pos)}; // 外部视觉位置估计器辅助源发布
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_ev_vel_pub{ORB_ID(estimator_aid_src_ev_vel)}; // 外部视觉速度估计器辅助源发布
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_yaw_pub{ORB_ID(estimator_aid_src_ev_yaw)}; // 外部视觉航向估计器辅助源发布
	hrt_abstime _status_ev_hgt_pub_last{0}; // 上次发布外部视觉高度状态的时间戳
	hrt_abstime _status_ev_pos_pub_last{0}; // 上次发布外部视觉位置状态的时间戳
	hrt_abstime _status_ev_vel_pub_last{0}; // 上次发布外部视觉速度状态的时间戳
	hrt_abstime _status_ev_yaw_pub_last{0}; // 上次发布外部视觉航向状态的时间戳

	matrix::Vector3f _last_ev_bias_published{}; // 上次发布的外部视觉偏置

	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)}; // 外部视觉里程计订阅

	uORB::PublicationMulti<estimator_bias3d_s> _estimator_ev_pos_bias_pub{ORB_ID(estimator_ev_pos_bias)}; // 外部视觉位置偏置估计器发布
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	uORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)}; // 着陆目标姿态订阅

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_vel_pub{ORB_ID(estimator_aid_src_aux_vel)}; // 辅助速度估计器辅助源发布
	hrt_abstime _status_aux_vel_pub_last{0}; // 上次发布辅助速度状态的时间戳
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)}; // 车辆光流订阅
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)}; // 光流速度估计器发布

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_optical_flow_pub{ORB_ID(estimator_aid_src_optical_flow)}; // 光流估计器辅助源发布
	hrt_abstime _status_optical_flow_pub_last{0}; // 上次发布光流状态的时间戳
	hrt_abstime _optical_flow_vel_pub_last{0}; // 上次发布光流速度的时间戳
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_BAROMETER)
	uint8_t _baro_calibration_count {0}; ///< 气压计校准计数，记录气压计的校准次数
	uint32_t _device_id_baro{0}; ///< 气压计设备ID，用于唯一标识气压计设备
	hrt_abstime _status_baro_hgt_pub_last{0}; ///< 上次发布气压高度状态的时间戳

	float _last_baro_bias_published{}; ///< 上次发布的气压计偏置值

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)}; ///< 订阅车辆空气数据，用于获取气压计相关信息

	uORB::PublicationMulti<estimator_bias_s> _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)}; ///< 发布气压计偏置估计的多重发布对象
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_pub {ORB_ID(estimator_aid_src_baro_hgt)}; ///< 发布气压计高度估计的辅助源
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_drag_pub {ORB_ID(estimator_aid_src_drag)}; ///< 发布拖曳融合的辅助源
	hrt_abstime _status_drag_pub_last{0}; ///< 上次发布拖曳状态的时间戳
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	uORB::Subscription _airspeed_sub {ORB_ID(airspeed)}; ///< 订阅气速数据，用于获取气速信息
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)}; ///< 订阅经过验证的气速数据

	float _airspeed_scale_factor{1.0f}; ///< 应用于气速测量的缩放因子修正
	hrt_abstime _airspeed_validated_timestamp_last{0}; ///< 上次发布经过验证的气速时间戳

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_airspeed_pub {ORB_ID(estimator_aid_src_airspeed)}; ///< 发布气速估计的辅助源
	hrt_abstime _status_airspeed_pub_last{0}; ///< 上次发布气速状态的时间戳
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_sideslip_pub {ORB_ID(estimator_aid_src_sideslip)}; ///< 发布侧滑估计的辅助源
	hrt_abstime _status_sideslip_pub_last {0}; ///< 上次发布侧滑状态的时间戳
#endif // CONFIG_EKF2_SIDESLIP

	orb_advert_t _mavlink_log_pub{nullptr}; ///< MAVLink日志发布对象，用于记录日志信息

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; ///< 订阅参数更新，设置更新间隔为1秒

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)}; ///< 订阅传感器选择信息
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)}; ///< 订阅车辆状态信息
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; ///< 订阅车辆着陆检测信息
	uORB::Subscription _launch_detection_status_sub{ORB_ID(launch_detection_status)}; ///< 订阅发射检测状态信息

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)}; ///< 订阅车辆命令信息
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)}; ///< 发布车辆命令确认信息

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)}; ///< 传感器组合数据的回调工作项
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)}; ///< 车辆IMU数据的回调工作项

#if defined(CONFIG_EKF2_RANGE_FINDER)
	hrt_abstime _status_rng_hgt_pub_last {0}; ///< 上次发布范围高度状态的时间戳

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_rng_hgt_pub{ORB_ID(estimator_aid_src_rng_hgt)}; ///< 发布范围高度估计的辅助源

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor}; ///< 订阅多个距离传感器的数据
	hrt_abstime _last_range_sensor_update{0}; ///< 上次更新范围传感器的时间戳
	int _distance_sensor_selected{-1}; ///< 选择的距离传感器索引，-1表示未选择，支持多个传感器实例
#endif // CONFIG_EKF2_RANGE_FINDER

	bool _callback_registered{false}; ///< 回调是否已注册的标志

	hrt_abstime _last_event_flags_publish{0}; ///< 上次发布事件标志的时间戳
	hrt_abstime _last_status_flags_publish{0}; ///< 上次发布状态标志的时间戳

	uint64_t _filter_control_status{0}; ///< 滤波器控制状态
	uint32_t _filter_fault_status{0}; ///< 滤波器故障状态
	uint32_t _innov_check_fail_status{0}; ///< 创新检查失败状态

	uint32_t _filter_control_status_changes{0}; ///< 滤波器控制状态变化计数
	uint32_t _filter_fault_status_changes{0}; ///< 滤波器故障状态变化计数
	uint32_t _innov_check_fail_status_changes{0}; ///< 创新检查失败状态变化计数
	uint32_t _filter_information_event_changes{0}; ///< 滤波器信息事件变化计数

	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)}; ///< 发布EKF2时间戳的多重发布对象
	uORB::PublicationMultiData<estimator_event_flags_s>  _estimator_event_flags_pub{ORB_ID(estimator_event_flags)}; ///< 发布估计器事件标志的多重发布对象
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)}; ///< 发布估计器创新测试比率的多重发布对象
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)}; ///< 发布估计器创新方差的多重发布对象
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)}; ///< 发布估计器创新的多重发布对象
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)}; ///< 发布估计器传感器偏置的多重发布对象
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)}; ///< 发布估计器状态的多重发布对象
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)}; ///< 发布估计器状态标志的多重发布对象
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)}; ///< 发布估计器状态的多重发布对象

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_fake_hgt_pub{ORB_ID(estimator_aid_src_fake_hgt)}; ///< 发布虚假高度估计的辅助源
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_fake_pos_pub{ORB_ID(estimator_aid_src_fake_pos)}; ///< 发布虚假位置估计的辅助源

	// publications with topic dependent on multi-mode
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub; ///< 发布车辆姿态信息的多重发布对象
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub; ///< 发布车辆局部位置的多重发布对象
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub; ///< 发布车辆全球位置的多重发布对象
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub; ///< 发布车辆里程计信息的多重发布对象

#if defined(CONFIG_EKF2_WIND)
	uORB::PublicationMulti<wind_s>              _wind_pub; ///< 发布风信息的多重发布对象
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_GNSS)

	uint64_t _last_geoid_height_update_us{0}; ///< 上次更新大地高程的时间戳（微秒）
	static constexpr float kGeoidHeightLpfTimeConstant = 10.f; ///< 大地高程低通滤波时间常数
	AlphaFilter<float> _geoid_height_lpf;  ///< AMSL与椭球体之间的高度偏移的低通滤波器

	hrt_abstime _last_gps_status_published{0}; ///< 上次发布GPS状态的时间戳

	hrt_abstime _status_gnss_hgt_pub_last{0}; ///< 上次发布GNSS高度状态的时间戳
	hrt_abstime _status_gnss_pos_pub_last{0}; ///< 上次发布GNSS位置状态的时间戳
	hrt_abstime _status_gnss_vel_pub_last{0}; ///< 上次发布GNSS速度状态的时间戳

	float _last_gnss_hgt_bias_published{}; ///< 上次发布的GNSS高度偏置值

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)}; ///< 订阅车辆GPS位置数据

	uORB::PublicationMulti<estimator_bias_s> _estimator_gnss_hgt_bias_pub{ORB_ID(estimator_gnss_hgt_bias)}; ///< 发布GNSS高度偏置估计的多重发布对象
	uORB::PublicationMulti<estimator_gps_status_s> _estimator_gps_status_pub{ORB_ID(estimator_gps_status)}; ///< 发布GNSS状态的多重发布对象
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_hgt_pub{ORB_ID(estimator_aid_src_gnss_hgt)}; ///< 发布GNSS高度估计的辅助源
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_gnss_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)}; ///< 发布GNSS位置估计的辅助源
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gnss_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)}; ///< 发布GNSS速度估计的辅助源

	uORB::PublicationMulti<yaw_estimator_status_s> _yaw_est_pub{ORB_ID(yaw_estimator_status)}; ///< 发布航向估计状态的多重发布对象

# if defined(CONFIG_EKF2_GNSS_YAW)
	hrt_abstime _status_gnss_yaw_pub_last {0}; ///< 上次发布GNSS航向状态的时间戳
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_pub {ORB_ID(estimator_aid_src_gnss_yaw)}; ///< 发布GNSS航向估计的辅助源
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	hrt_abstime _status_gravity_pub_last {0}; ///< 上次发布重力状态的时间戳
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gravity_pub{ORB_ID(estimator_aid_src_gravity)}; ///< 发布重力估计的辅助源
#endif // CONFIG_EKF2_GRAVITY_FUSION

	Ekf _ekf; ///< EKF滤波器实例

	parameters *_params;	///< 指向EKF参数结构体的指针（位于EKF类实例中）

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::EKF2_LOG_VERBOSE>) _param_ekf2_log_verbose, ///< EKF2日志详细程度参数
		(ParamExtInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us, ///< EKF2预测时间参数（微秒）
		(ParamExtFloat<px4::params::EKF2_DELAY_MAX>) _param_ekf2_delay_max, ///< EKF2最大延迟参数
		(ParamExtInt<px4::params::EKF2_IMU_CTRL>) _param_ekf2_imu_ctrl, ///< EKF2 IMU控制参数
		(ParamExtFloat<px4::params::EKF2_VEL_LIM>) _param_ekf2_vel_lim, ///< EKF2速度限制参数

#if defined(CONFIG_EKF2_AUXVEL)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< 辅助速度测量相对于IMU的延迟（毫秒）
#endif // CONFIG_EKF2_AUXVEL

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU角速率噪声，用于协方差预测（弧度/秒）
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU加速度噪声，用于协方差预测（米/秒²）

		// 过程噪声
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< IMU陀螺仪偏置预测的过程噪声（弧度/秒²）
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< IMU加速度计偏置预测的过程噪声（米/秒³）

#if defined(CONFIG_EKF2_WIND)
		(ParamExtFloat<px4::params::EKF2_WIND_NSD>) _param_ekf2_wind_nsd, ///< 风噪声标准差参数
#endif // CONFIG_EKF2_WIND

		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>) _param_ekf2_noaid_noise, ///< 无辅助噪声参数

#if defined(CONFIG_EKF2_GNSS)
		(ParamExtInt<px4::params::EKF2_GPS_CTRL>) _param_ekf2_gps_ctrl, ///< GPS控制选择参数
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>) _param_ekf2_gps_delay, ///< GPS延迟参数

		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x, ///< GPS位置X坐标参数
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y, ///< GPS位置Y坐标参数
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z, ///< GPS位置Z坐标参数

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>) _param_ekf2_gps_v_noise, ///< GPS速度噪声参数
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>) _param_ekf2_gps_p_noise, ///< GPS位置噪声参数

		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>) _param_ekf2_gps_p_gate, ///< GPS位置一致性门限参数
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>) _param_ekf2_gps_v_gate, ///< GPS速度一致性门限参数

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>) _param_ekf2_gps_check, ///< GPS检查参数
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>)    _param_ekf2_req_eph, ///< GPS水平精度要求参数
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>)    _param_ekf2_req_epv, ///< GPS垂直精度要求参数
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>)   _param_ekf2_req_sacc, ///< GPS水平精度要求参数
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>)    _param_ekf2_req_nsats, ///< GPS卫星数量要求参数
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)   _param_ekf2_req_pdop, ///< GPS位置精度要求参数
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>) _param_ekf2_req_hdrift, ///< GPS水平漂移要求参数
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift, ///< GPS垂直漂移要求参数
		(ParamFloat<px4::params::EKF2_REQ_GPS_H>)     _param_ekf2_req_gps_h, ///< GPS高度要求参数

		// EKF-GSF实验性航向估计器使用的参数
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>) _param_ekf2_gsf_tas_default, ///< EKF-GSF实验性航向估计器的默认真空速参数
		(ParamFloat<px4::params::EKF2_GPS_YAW_OFF>) _param_ekf2_gps_yaw_off, ///< GPS航向偏移参数
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_BAROMETER)
		(ParamExtInt<px4::params::EKF2_BARO_CTRL>) _param_ekf2_baro_ctrl,///< 气压计控制选择参数，用于选择气压计的使用方式
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>) _param_ekf2_baro_delay, ///< 气压计测量相对于IMU的延迟（毫秒）
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>) _param_ekf2_baro_noise, ///< 气压计测量噪声，用于状态估计的协方差预测（帕斯卡）
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>) _param_ekf2_baro_gate, ///< 气压计融合创新一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>) _param_ekf2_gnd_eff_dz, ///< 地面效应高度补偿（米）
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>) _param_ekf2_gnd_max_hgt, ///< 最大地面高度限制（米）

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
		// 静压位置误差修正，Ps_error = Ps_meas - Ps_truth
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>) _param_ekf2_aspd_max, ///< 最大允许的气速（米/秒），用于气压计补偿
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>) _param_ekf2_pcoef_xp, ///< 气压计补偿的X方向正系数
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>) _param_ekf2_pcoef_xn, ///< 气压计补偿的X方向负系数
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>) _param_ekf2_pcoef_yp, ///< 气压计补偿的Y方向正系数
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>) _param_ekf2_pcoef_yn, ///< 气压计补偿的Y方向负系数
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>) _param_ekf2_pcoef_z, ///< 气压计补偿的Z方向系数
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay, ///< 空速测量相对于IMU的延迟（毫秒）
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate, ///< 真空速创新一致性门限大小（标准差）
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise, ///< 用于空速融合的测量噪声（米/秒）

		// 空速融合的控制参数
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, ///< 设置为零将禁用空速融合，任何正值设置为使用的最小空速（米/秒）
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>) _param_ekf2_beta_gate, ///< 侧滑角创新一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise, ///< 侧滑角测量噪声（弧度）
		(ParamExtInt<px4::params::EKF2_FUSE_BETA>) _param_ekf2_fuse_beta, ///< 侧滑角融合控制选择
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_MAGNETOMETER)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>) _param_ekf2_mag_delay, ///< 磁力计测量相对于IMU的延迟（毫秒）
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>) _param_ekf2_mag_e_noise, ///< 磁力计东向测量噪声（高斯）
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>) _param_ekf2_mag_b_noise, ///< 磁力计北向测量噪声（高斯）
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>) _param_ekf2_head_noise, ///< 磁头测量噪声（弧度）
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>) _param_ekf2_mag_noise, ///< 磁力计测量噪声（高斯）
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl, ///< 磁偏角（弧度）
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>) _param_ekf2_hdg_gate, ///< 磁航向创新一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>) _param_ekf2_mag_gate, ///< 磁力计融合创新一致性门限（标准差）
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>) _param_ekf2_decl_type, ///< 磁偏角类型选择
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>) _param_ekf2_mag_type, ///< 磁力计类型选择
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>) _param_ekf2_mag_acclim, ///< 磁力计加速度限制（米/秒²）
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check, ///< 磁力计检查控制选择
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_STR>) _param_ekf2_mag_chk_str, ///< 磁力计检查强度阈值
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_INC>) _param_ekf2_mag_chk_inc, ///< 磁力计检查倾斜阈值
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>) _param_ekf2_synthetic_mag_z, ///< 合成磁力计Z轴选择
#endif // CONFIG_EKF2_MAGNETOMETER

		(ParamExtInt<px4::params::EKF2_HGT_REF>) _param_ekf2_hgt_ref,    ///< 选择高度数据的主要来源

		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< 从最后一次融合测量到EKF报告水平导航解无效的最大时间（微秒）

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng, ///< 最小范围限制（米）
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise, ///< 地形测量噪声（米）
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>) _param_ekf2_terr_grad, ///< 地形梯度（米/米）
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
		// 距离传感器融合
		(ParamExtInt<px4::params::EKF2_RNG_CTRL>) _param_ekf2_rng_ctrl, ///< 距离传感器控制选择
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>) _param_ekf2_rng_delay, ///< 距离传感器测量相对于IMU的延迟（毫秒）
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>) _param_ekf2_rng_noise, ///< 距离传感器测量噪声（米）
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe, ///< 距离传感器的状态估计方差（米²）
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>) _param_ekf2_rng_gate, ///< 距离传感器融合创新一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch, ///< 距离传感器的俯仰角（弧度）
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>) _param_ekf2_rng_a_vmax, ///< 距离传感器的最大加速度（米/秒²）
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>) _param_ekf2_rng_a_hmax, ///< 距离传感器的最大高度（米）
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>) _param_ekf2_rng_a_igate, ///< 距离传感器的创新一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>) _param_ekf2_rng_qlty_t, ///< 距离传感器的质量阈值（毫秒）
		(ParamExtFloat<px4::params::EKF2_RNG_K_GATE>) _param_ekf2_rng_k_gate, ///< 距离传感器的质量一致性门限（标准差）
		(ParamExtFloat<px4::params::EKF2_RNG_FOG>) _param_ekf2_rng_fog, ///< 距离传感器的雾霾影响（米）
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x, ///< 距离传感器在机体坐标系中的X位置（米）
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y, ///< 距离传感器在机体坐标系中的Y位置（米）
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z, ///< 距离传感器在机体坐标系中的Z位置（米）
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// 视觉估计融合
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay, ///< 离线视觉测量相对于IMU的延迟（毫秒）

		(ParamExtInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,	 ///< 外部视觉（EV）控制选择
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>) _param_ekf2_ev_noise_md, ///< 确定视觉观测噪声的来源
		(ParamExtInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin, ///< 外部视觉的最小质量阈值
		(ParamExtFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise, ///< 外部视觉测量的默认位置观测噪声（米）
		(ParamExtFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise, ///< 外部视觉测量的默认速度观测噪声（米/秒）
		(ParamExtFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise, ///< 外部视觉测量的默认角度观测噪声（弧度）
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate, ///< 外部视觉速度创新一致性门限大小（标准差）
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate, ///< 外部视觉位置创新一致性门限大小（标准差）

		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x, ///< VI传感器焦点在机体坐标系中的X位置（米）
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y, ///< VI传感器焦点在机体坐标系中的Y位置（米）
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z, ///< VI传感器焦点在机体坐标系中的Z位置（米）
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		// 光流融合
		(ParamExtInt<px4::params::EKF2_OF_CTRL>)
		_param_ekf2_of_ctrl, ///< 光流融合选择
		(ParamExtInt<px4::params::EKF2_OF_GYR_SRC>)
		_param_ekf2_of_gyr_src, ///< 光流测量的陀螺仪源选择
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay, ///< 光流测量相对于IMU的延迟（毫秒），此延迟为光流积分间隔的中间值
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min, ///< 光流LOS速率测量的最佳质量观测噪声（弧度/秒）
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max, ///< 光流LOS速率测量的最差质量观测噪声（弧度/秒）
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin, ///< 在空中时，流量传感器的最小可接受质量整数
		(ParamExtInt<px4::params::EKF2_OF_QMIN_GND>)
		_param_ekf2_of_qmin_gnd, ///< 在地面时，流量传感器的最小可接受质量整数
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate, ///< 光流融合创新一致性门限大小（标准差）
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x, ///< 光流传感器焦点在机体坐标系中的X位置（米）
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y, ///< 光流传感器焦点在机体坐标系中的Y位置（米）
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z, ///< 光流传感器焦点在机体坐标系中的Z位置（米）
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_DRAG_FUSION)
		(ParamExtInt<px4::params::EKF2_DRAG_CTRL>) _param_ekf2_drag_ctrl,		///< 拖曳融合选择
		// 多旋翼拖曳特定力融合
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< 拖曳特定力测量的观测噪声方差（米/秒²）²
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< X轴上的弹道系数（千克/米²）
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< Y轴上的弹道系数（千克/米²）
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< 螺旋桨动量拖曳系数（1/秒）
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
		(ParamExtFloat<px4::params::EKF2_GRAV_NOISE>) _param_ekf2_grav_noise, ///< 重力测量噪声的方差（单位：米/秒²），用于描述重力传感器在测量过程中的不确定性，影响状态估计的精度。
#endif // CONFIG_EKF2_GRAVITY_FUSION

		// 传感器在机体坐标系中的位置
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< IMU在机体坐标系中的X轴位置（单位：米），用于确定IMU相对于机体的空间位置，影响姿态估计。
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< IMU在机体坐标系中的Y轴位置（单位：米），同样用于确定IMU的空间位置，确保数据融合的准确性。
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< IMU在机体坐标系中的Z轴位置（单位：米），影响IMU数据的融合与状态估计。

		// IMU开机时的偏置参数
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 开机时陀螺仪偏置的不确定性（单位：弧度/秒），表示陀螺仪在初始状态下的偏置估计误差，影响角速度测量的准确性。
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 开机时加速度计偏置的不确定性（单位：米/秒²），用于描述加速度计在开机时的偏置估计误差，影响加速度测量的精度。
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 使用重力向量进行初始对准后的倾斜误差（单位：弧度），表示在初始对准过程中可能产生的误差，影响姿态估计的准确性。

		// EKF加速度计偏置学习控制
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< 加速度计偏置学习的限制（单位：米/秒²），用于限制加速度计偏置学习过程中的最大允许偏差，确保学习过程的稳定性。
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< 允许IMU偏置学习的最大加速度幅度（单位：米/秒²），超过此值将停止偏置学习，以避免在高加速度情况下的错误学习。
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< 允许IMU偏置学习的最大陀螺仪角速度幅度（单位：米/秒²），用于限制在高角速度情况下的偏置学习，确保学习的准确性。
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< 用于抑制IMU增量速度偏置学习的时间常数（单位：秒），影响偏置学习的响应速度和稳定性。

		(ParamExtFloat<px4::params::EKF2_GYR_B_LIM>) _param_ekf2_gyr_b_lim,	///< 陀螺仪偏置学习的限制（单位：弧度/秒），用于限制陀螺仪偏置学习过程中的最大允许偏差，确保学习过程的稳定性。

		// 输出预测器滤波器的时间常数
		(ParamFloat<px4::params::EKF2_TAU_VEL>) _param_ekf2_tau_vel, ///< 速度预测器的时间常数（单位：秒），用于调整速度估计的响应时间，影响状态估计的动态特性。
		(ParamFloat<px4::params::EKF2_TAU_POS>) _param_ekf2_tau_pos ///< 位置预测器的时间常数（单位：秒），用于调整位置估计的响应时间，确保位置估计的准确性和稳定性。
	)
};
#endif // !EKF2_HPP
