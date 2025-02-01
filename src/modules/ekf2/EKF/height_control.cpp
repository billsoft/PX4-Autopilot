/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
 *
 * 该软件的源代码和二进制形式的再分发和使用，允许进行修改，前提是满足以下条件：
 *
 * 1. 再分发源代码必须保留上述版权声明、条件列表和免责声明。
 * 2. 再分发二进制形式必须在分发的文档或其他材料中重现上述版权声明、条件列表和免责声明。
 * 3. PX4的名称或其贡献者的名称不得用于推广或宣传基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因）引起的，均不对使用本软件的任何方式负责，即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file height_control.cpp
 * 该文件实现了高度控制的相关功能，包括高度融合和传感器健康检查。
 */

#include "ekf.h"  // 引入EKF头文件

// 控制高度融合的函数
void Ekf::controlHeightFusion(const imuSample &imu_delayed)
{
	checkVerticalAccelerationHealth(imu_delayed);  // 检查垂直加速度的健康状态

#if defined(CONFIG_EKF2_BAROMETER)
	updateGroundEffect();  // 更新地面效应

	controlBaroHeightFusion(imu_delayed);  // 控制气压高度融合
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	controlGnssHeightFusion(_gps_sample_delayed);  // 控制GNSS高度融合
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)
	controlRangeHaglFusion(imu_delayed);  // 控制范围传感器高度融合
#endif // CONFIG_EKF2_RANGE_FINDER

	checkHeightSensorRefFallback();  // 检查高度传感器参考的后备方案
}

// 检查高度传感器参考的后备方案
void Ekf::checkHeightSensorRefFallback()
{
	if (_height_sensor_ref != HeightSensor::UNKNOWN) {
		// 如果参考传感器正在运行，返回
		return;
	}

	HeightSensor fallback_list[4];  // 定义后备传感器列表

	// 根据参数设置后备传感器的优先级
	switch (static_cast<HeightSensor>(_params.height_sensor_ref)) {
	default:

	/* FALLTHROUGH */
	case HeightSensor::UNKNOWN:
		fallback_list[0] = HeightSensor::GNSS;  // GNSS传感器
		fallback_list[1] = HeightSensor::BARO;  // 气压传感器
		fallback_list[2] = HeightSensor::EV;    // 外部视觉传感器
		fallback_list[3] = HeightSensor::RANGE; // 范围传感器
		break;

	case HeightSensor::BARO:
		fallback_list[0] = HeightSensor::BARO;  // 气压传感器
		fallback_list[1] = HeightSensor::GNSS;  // GNSS传感器
		fallback_list[2] = HeightSensor::EV;    // 外部视觉传感器
		fallback_list[3] = HeightSensor::RANGE; // 范围传感器
		break;

	case HeightSensor::GNSS:
		fallback_list[0] = HeightSensor::GNSS;  // GNSS传感器
		fallback_list[1] = HeightSensor::BARO;  // 气压传感器
		fallback_list[2] = HeightSensor::EV;    // 外部视觉传感器
		fallback_list[3] = HeightSensor::RANGE; // 范围传感器
		break;

	case HeightSensor::RANGE:
		fallback_list[0] = HeightSensor::RANGE; // 范围传感器
		fallback_list[1] = HeightSensor::EV;    // 外部视觉传感器
		fallback_list[2] = HeightSensor::BARO;  // 气压传感器
		fallback_list[3] = HeightSensor::GNSS;  // GNSS传感器
		break;

	case HeightSensor::EV:
		fallback_list[0] = HeightSensor::EV;    // 外部视觉传感器
		fallback_list[1] = HeightSensor::RANGE; // 范围传感器
		fallback_list[2] = HeightSensor::BARO;  // 气压传感器
		fallback_list[3] = HeightSensor::GNSS;  // GNSS传感器
		break;
	}

	// 遍历后备传感器列表，检查可用性
	for (unsigned i = 0; i < 4; i++) {
		if (((fallback_list[i] == HeightSensor::BARO) && _control_status.flags.baro_hgt)  // 如果气压传感器可用
		    || ((fallback_list[i] == HeightSensor::GNSS) && _control_status.flags.gps_hgt)  // 如果GNSS传感器可用
		    || ((fallback_list[i] == HeightSensor::RANGE) && _control_status.flags.rng_hgt)  // 如果范围传感器可用
		    || ((fallback_list[i] == HeightSensor::EV) && _control_status.flags.ev_hgt)) {  // 如果外部视觉传感器可用
			ECL_INFO("fallback to secondary height reference");  // 输出信息：切换到次要高度参考
			_height_sensor_ref = fallback_list[i];  // 更新高度传感器参考
			break;  // 退出循环
		}
	}
}

// 检查垂直加速度的健康状态
void Ekf::checkVerticalAccelerationHealth(const imuSample &imu_delayed)
{
	// 检查IMU加速度计的振动引起的剪切现象，表现为垂直创新值为正且未过时。
	// 剪切通常会导致平均加速度读数趋向于零，这使得INS认为它正在下落，并产生正的垂直创新值。

	Likelihood inertial_nav_falling_likelihood = estimateInertialNavFallingLikelihood();  // 估计惯性导航下落的可能性

	const uint16_t kClipCountLimit = 1.f / _dt_ekf_avg;  // 定义剪切计数限制

	bool acc_clip_warning[3] {};  // 警告标志数组
	bool acc_clip_critical[3] {};  // 严重标志数组

	// 遍历三个轴
	for (int axis = 0; axis < 3; axis++) {
		if (imu_delayed.delta_vel_clipping[axis] && (_clip_counter[axis] < kClipCountLimit)) {
			_clip_counter[axis]++;  // 增加剪切计数

		} else if (_clip_counter[axis] > 0) {
			_clip_counter[axis]--;  // 减少剪切计数
		}

		// 如果过去1秒内超过50%的IMU样本受到剪切影响，则发出警告
		acc_clip_warning[axis] = _clip_counter[axis] >= kClipCountLimit / 2;
		acc_clip_critical[axis] = _clip_counter[axis] >= kClipCountLimit;
	}

	// 如果所有轴都报告警告，或者任何轴是严重的，则标记为bad_acc_clipping
	const bool all_axis_warning = (acc_clip_warning[0] && acc_clip_warning[1] && acc_clip_warning[2]);
	const bool any_axis_critical = (acc_clip_critical[0] || acc_clip_critical[1] || acc_clip_critical[2]);

	_fault_status.flags.bad_acc_clipping = all_axis_warning || any_axis_critical;  // 更新故障状态

	// 如果Z轴有警告或任何其他轴是严重的
	const bool is_clipping_frequently = acc_clip_warning[2] || _fault_status.flags.bad_acc_clipping;

	// 如果INS下落的可能性很高，则不需要剪切的证据
	const bool bad_vert_accel = (is_clipping_frequently && (inertial_nav_falling_likelihood == Likelihood::MEDIUM))
				    || (inertial_nav_falling_likelihood == Likelihood::HIGH);

	if (bad_vert_accel) {
		_time_bad_vert_accel = imu_delayed.time_us;  // 记录不良垂直加速度的时间

	} else {
		_time_good_vert_accel = imu_delayed.time_us;  // 记录良好垂直加速度的时间
	}

	// 声明不良的垂直加速度测量，并使声明持续至少BADACC_PROBATION秒
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);  // 检查是否在最近的时间内

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;  // 更新不良垂直加速度状态
	}
}

// 估计惯性导航下落的可能性
Likelihood Ekf::estimateInertialNavFallingLikelihood() const
{
	bool likelihood_high = false;  // 高可能性标志
	bool likelihood_medium = false;  // 中等可能性标志

	enum class ReferenceType { PRESSURE, GNSS, GROUND };  // 定义参考类型

	struct {
		ReferenceType ref_type{};  // 参考类型
		float innov{0.f};  // 创新值
		float innov_var{0.f};  // 创新方差
		bool failed_min{false};  // 最小失败标志
		bool failed_lim{false};  // 限制失败标志
	} checks[6] {};  // 定义检查结构体数组

#if defined(CONFIG_EKF2_BAROMETER)

	if (_control_status.flags.baro_hgt) {
		checks[0] = {ReferenceType::PRESSURE, _aid_src_baro_hgt.innovation, _aid_src_baro_hgt.innovation_variance};  // 检查气压传感器
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)

	if (_control_status.flags.gps_hgt) {
		checks[1] = {ReferenceType::GNSS, _aid_src_gnss_hgt.innovation, _aid_src_gnss_hgt.innovation_variance};  // 检查GNSS高度
	}

	if (_control_status.flags.gps) {
		checks[2] = {ReferenceType::GNSS, _aid_src_gnss_vel.innovation[2], _aid_src_gnss_vel.innovation_variance[2]};  // 检查GNSS速度
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_hgt) {
		// 范围是到地面的距离测量，而不是直接的高度观测，符号相反
		checks[3] = {ReferenceType::GROUND, -_aid_src_rng_hgt.innovation, _aid_src_rng_hgt.innovation_variance};  // 检查范围传感器
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_hgt) {
		checks[4] = {ReferenceType::GROUND, _aid_src_ev_hgt.innovation, _aid_src_ev_hgt.innovation_variance};  // 检查外部视觉高度
	}

	if (_control_status.flags.ev_vel) {
		checks[5] = {ReferenceType::GROUND, _aid_src_ev_vel.innovation[2], _aid_src_ev_vel.innovation_variance[2]};  // 检查外部视觉速度
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 根据所有源的创新比率计算检查
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].innov_var < FLT_EPSILON) {
			continue;  // 如果创新方差无效，跳过
		}

		const float innov_ratio = checks[i].innov / sqrtf(checks[i].innov_var);  // 计算创新比率
		checks[i].failed_min = innov_ratio > _params.vert_innov_test_min;  // 检查最小失败
		checks[i].failed_lim = innov_ratio > _params.vert_innov_test_lim;  // 检查限制失败
	}

	// 检查所有源之间的相互关系
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].failed_lim) {
			// 如果一个源失败测试，则有可能惯性导航正在下落
			likelihood_medium = true;
		}

		for (unsigned j = 0; j < 6; j++) {

			if ((checks[i].ref_type != checks[j].ref_type) && checks[i].failed_lim && checks[j].failed_min) {
				// 如果两个源都失败测试，则有很高的可能性惯性导航正在失败
				likelihood_high = true;
			}
		}
	}

	if (likelihood_high) {
		return Likelihood::HIGH;  // 返回高可能性

	} else if (likelihood_medium) {
		return Likelihood::MEDIUM;  // 返回中等可能性
	}

	return Likelihood::LOW;  // 返回低可能性
}
