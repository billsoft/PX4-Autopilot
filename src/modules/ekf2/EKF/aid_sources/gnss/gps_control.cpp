/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 *    used to endorse或promote products derived from this software
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
 * @file gps_control.cpp
 * 控制函数用于 ekf GNSS 融合
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

// 控制 GPS 融合的主函数
void Ekf::controlGpsFusion(const imuSample &imu_delayed)
{
	// 检查 GPS 缓冲区是否存在以及 GNSS 控制参数是否为 0
	if (!_gps_buffer || (_params.gnss_ctrl == 0)) {
		stopGpsFusion(); // 停止 GPS 融合
		return; // 退出函数
	}

	// 如果陀螺仪偏置未被抑制，则设置陀螺仪偏置
	if (!gyro_bias_inhibited()) {
		_yawEstimator.setGyroBias(getGyroBias(), _control_status.flags.vehicle_at_rest);
	}

	// 每次 imu_delayed 更新时运行 EKF-GSF 偏航估计器
	_yawEstimator.predict(imu_delayed.delta_ang, imu_delayed.delta_ang_dt,
			      imu_delayed.delta_vel, imu_delayed.delta_vel_dt,
			      (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest));

	// 检查 GPS 数据是否间歇性
	_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL);

	// 检查在融合时间范围内是否有新传感器数据到达
	_gps_data_ready = _gps_buffer->pop_first_older_than(imu_delayed.time_us, &_gps_sample_delayed);

	if (_gps_data_ready) {
		const gnssSample &gnss_sample = _gps_sample_delayed; // 获取延迟的 GNSS 样本

		// 运行 GNSS 检查并检查是否超时
		if (runGnssChecks(gnss_sample)
		    && isTimedOut(_last_gps_fail_us, max((uint64_t)1e6, (uint64_t)_min_gps_health_time_us / 10))) {
			if (isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
				// 第一次检查通过，锁定状态
				if (!_gps_checks_passed) {
					_information_events.flags.gps_checks_passed = true; // 标记 GPS 检查已通过
				}

				_gps_checks_passed = true; // 更新 GPS 检查状态
			}

		} else {
			// 跳过此样本
			_gps_data_ready = false;

			// 如果 GPS 状态为真且超时，则停止 GPS 融合
			if (_control_status.flags.gps && isTimedOut(_last_gps_pass_us, _params.reset_timeout_max)) {
				stopGpsFusion();
				ECL_WARN("GPS quality poor - stopping use"); // 警告 GPS 质量差，停止使用
			}
		}

		// 更新 GNSS 位置和速度
		updateGnssPos(gnss_sample, _aid_src_gnss_pos);
		updateGnssVel(imu_delayed, gnss_sample, _aid_src_gnss_vel);

	} else if (_control_status.flags.gps) {
		// 如果没有新样本且 GPS 状态为真，检查是否超时
		if (!isNewestSampleRecent(_time_last_gps_buffer_push, _params.reset_timeout_max)) {
			stopGpsFusion();
			ECL_WARN("GPS data stopped"); // 警告 GPS 数据停止
		}
	}

	if (_gps_data_ready) {
#if defined(CONFIG_EKF2_GNSS_YAW)
		const gnssSample &gnss_sample = _gps_sample_delayed; // 获取延迟的 GNSS 样本
		controlGnssYawFusion(gnss_sample); // 控制 GNSS 偏航融合
#endif // CONFIG_EKF2_GNSS_YAW

		controlGnssYawEstimator(_aid_src_gnss_vel); // 控制 GNSS 偏航估计器

		// 检查 GNSS 速度和位置控制是否启用
		const bool gnss_vel_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL));
		const bool gnss_pos_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));

		// 检查持续条件是否通过
		const bool continuing_conditions_passing = (gnss_vel_enabled || gnss_pos_enabled)
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align;
		const bool starting_conditions_passing = continuing_conditions_passing && _gps_checks_passed;
		const bool gpos_init_conditions_passing = gnss_pos_enabled && _gps_checks_passed;

		if (_control_status.flags.gps) {
			if (continuing_conditions_passing) {
				// 如果 GNSS 速度启用，则融合速度
				if (gnss_vel_enabled) {
					fuseVelocity(_aid_src_gnss_vel);
				}

				// 如果 GNSS 位置启用，则融合水平位置
				if (gnss_pos_enabled) {
					fuseHorizontalPosition(_aid_src_gnss_pos);
				}

				// 检查是否需要重置速度和位置
				bool do_vel_pos_reset = shouldResetGpsFusion();

				// 如果在空中且偏航失败且超时，则尝试偏航紧急重置
				if (_control_status.flags.in_air
				    && isYawFailure()
				    && isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
				    && (_time_last_hor_vel_fuse > _time_last_on_ground_us)) {
					do_vel_pos_reset = tryYawEmergencyReset();
				}

				if (do_vel_pos_reset) {
					ECL_WARN("GPS fusion timeout, resetting"); // 警告 GPS 融合超时，重置
				}

				// 如果 GNSS 速度启用，进行速度重置
				if (gnss_vel_enabled) {
					if (do_vel_pos_reset) {
						resetVelocityToGnss(_aid_src_gnss_vel); // 重置速度到 GNSS

					} else if (isHeightResetRequired()) {
						// 如果高度失败，则重置垂直速度
						resetVerticalVelocityTo(_aid_src_gnss_vel.observation[2], _aid_src_gnss_vel.observation_variance[2]);
					}
				}

				// 如果 GNSS 位置启用且需要重置，则重置水平位置
				if (gnss_pos_enabled && do_vel_pos_reset) {
					resetHorizontalPositionToGnss(_aid_src_gnss_pos);
				}

			} else {
				stopGpsFusion(); // 停止 GPS 融合
			}

		} else {
			// 如果 GPS 状态为假且启动条件通过，则开始 GPS 融合
			if (starting_conditions_passing) {
				ECL_INFO("starting GPS fusion"); // 信息：开始 GPS 融合
				_information_events.flags.starting_gps_fusion = true;

				// 如果已经使用其他速度源，则不需要重置速度
				if (!isHorizontalAidingActive()
				    || isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
				    || !_control_status_prev.flags.yaw_align
				   ) {
					// 重置速度
					if (gnss_vel_enabled) {
						resetVelocityToGnss(_aid_src_gnss_vel);
					}
				}

				if (gnss_pos_enabled) {
					resetHorizontalPositionToGnss(_aid_src_gnss_pos); // 重置水平位置到 GNSS
				}

				_control_status.flags.gps = true; // 更新 GPS 状态为真

			} else if (gpos_init_conditions_passing && !_local_origin_lat_lon.isInitialized()) {
				resetHorizontalPositionToGnss(_aid_src_gnss_pos); // 重置水平位置到 GNSS
			}
		}
	}
}

// 更新 GNSS 速度的函数
void Ekf::updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src)
{
	// 根据 IMU 的相对偏移修正速度
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body; // 计算 GPS 和 IMU 位置的偏移

	const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias; // 计算角速度
	const Vector3f vel_offset_body = angular_velocity % pos_offset_body; // 计算速度偏移
	const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body; // 将速度偏移转换到地球坐标系
	const Vector3f velocity = gnss_sample.vel - vel_offset_earth; // 计算修正后的速度

	const float vel_var = sq(math::max(gnss_sample.sacc, _params.gps_vel_noise, 0.01f)); // 计算速度方差
	const Vector3f vel_obs_var(vel_var, vel_var, vel_var * sq(1.5f)); // 速度观测方差

	const float innovation_gate = math::max(_params.gps_vel_innov_gate, 1.f); // 计算创新门限

	// 更新辅助源状态
	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                  // 样本时间戳
			      velocity,                             // 观测值
			      vel_obs_var,                          // 观测方差
			      _state.vel - velocity,                // 创新值
			      getVelocityVariance() + vel_obs_var,  // 创新方差
			      innovation_gate);                     // 创新门限

	// vz 特殊情况：如果垂直加速度数据不良，则不拒绝测量
	// 如果 GNSS 报告的速度精度可接受，则限制创新以防止尖峰
	bool bad_acc_vz_rejected = _fault_status.flags.bad_acc_vertical
				   && (aid_src.test_ratio[2] > 1.f)                                   // vz 被拒绝
				   && (aid_src.test_ratio[0] < 1.f) && (aid_src.test_ratio[1] < 1.f); // vx 和 vy 被接受

	if (bad_acc_vz_rejected
	    && (gnss_sample.sacc < _params.req_sacc)
	   ) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance[2]); // 计算创新限制
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit); // 限制创新值
		aid_src.innovation_rejected = false; // 标记创新未被拒绝
	}
}

// 更新 GNSS 位置的函数
void Ekf::updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src)
{
	// 根据 IMU 的相对偏移修正位置和高度
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body; // 计算 GPS 和 IMU 位置的偏移
	const Vector3f pos_offset_earth = Vector3f(_R_to_earth * pos_offset_body); // 将偏移转换到地球坐标系
	const LatLonAlt measurement(gnss_sample.lat, gnss_sample.lon, gnss_sample.alt); // 创建 GNSS 测量对象
	const LatLonAlt measurement_corrected = measurement + (-pos_offset_earth); // 修正测量值
	const Vector2f innovation = (_gpos - measurement_corrected).xy(); // 计算创新值

	// 放宽观测噪声上限，防止不良 GPS 影响位置估计
	float pos_noise = math::max(gnss_sample.hacc, _params.gps_pos_noise); // 计算位置噪声

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// 如果没有使用其他辅助源，则依赖 GPS 观测来约束姿态误差，必须限制观测噪声值
		if (pos_noise > _params.pos_noaid_noise) {
			pos_noise = _params.pos_noaid_noise; // 限制位置噪声
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f)); // 计算位置方差
	const Vector2f pos_obs_var(pos_var, pos_var); // 位置观测方差
	const matrix::Vector2d observation(measurement_corrected.latitude_deg(), measurement_corrected.longitude_deg()); // 修正后的观测值

	// 更新辅助源状态
	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // 样本时间戳
			      observation,                                            // 观测值
			      pos_obs_var,                                            // 观测方差
			      innovation,                                             // 创新值
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // 创新方差
			      math::max(_params.gps_pos_innov_gate, 1.f));            // 创新门限
}

// 控制 GNSS 偏航估计器的函数
void Ekf::controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel)
{
	// 更新偏航估计器速度（对 GNSS 速度数据进行基本的合理性检查）
	const float vel_var = aid_src_vel.observation_variance[0]; // 获取速度方差
	const Vector2f vel_xy(aid_src_vel.observation); // 获取速度观测值

	if ((vel_var > 0.f)
	    && (vel_var < _params.req_sacc)
	    && vel_xy.isAllFinite()) { // 检查速度方差和观测值的有效性

		_yawEstimator.fuseVelocity(vel_xy, vel_var, _control_status.flags.in_air); // 融合速度到偏航估计器

		// 尝试使用估计值对偏航进行对齐
		if (((_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VEL))
		     || (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)))
		    && !_control_status.flags.yaw_align
		    && _control_status.flags.tilt_align) {
			if (resetYawToEKFGSF()) {
				ECL_INFO("GPS yaw aligned using IMU"); // 信息：使用 IMU 对齐 GPS 偏航
			}
		}
	}
}

// 尝试偏航紧急重置的函数
bool Ekf::tryYawEmergencyReset()
{
	bool success = false;

	/* 如果水平速度创新检查持续失败，而偏航紧急估计器与偏航估计之间的差异很大，则执行快速重置
	 * 这使得从不良的偏航估计中恢复。 如果故障条件在飞行前就存在，则不执行重置，以防止由于 GPS 故障或其他传感器错误而触发。
	 */
	if (resetYawToEKFGSF()) {
		ECL_WARN("GPS emergency yaw reset"); // 警告：GPS 紧急偏航重置

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
			// 停止在主 EKF 中使用磁力计，否则其融合可能会拖动偏航并导致另一次导航失败
			_control_status.flags.mag_fault = true; // 标记磁力计故障
		}

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gnss_yaw) {
			_control_status.flags.gnss_yaw_fault = true; // 标记 GNSS 偏航故障
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			_control_status.flags.ev_yaw_fault = true; // 标记外部视觉偏航故障
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		success = true; // 设置成功标志
	}

	return success; // 返回成功状态
}

// 将速度重置到 GNSS 的函数
void Ekf::resetVelocityToGnss(estimator_aid_source3d_s &aid_src)
{
	_information_events.flags.reset_vel_to_gps = true; // 标记重置速度到 GPS
	resetVelocityTo(Vector3f(aid_src.observation), Vector3f(aid_src.observation_variance)); // 重置速度

	resetAidSourceStatusZeroInnovation(aid_src); // 重置辅助源状态为零创新
}

// 将水平位置重置到 GNSS 的函数
void Ekf::resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src)
{
	_information_events.flags.reset_pos_to_gps = true; // 标记重置位置到 GPS
	resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
		      aid_src.observation_variance[0] +
		      aid_src.observation_variance[1]); // 重置经纬度

	resetAidSourceStatusZeroInnovation(aid_src); // 重置辅助源状态为零创新
}

// 检查是否需要重置 GPS 融合的函数
bool Ekf::shouldResetGpsFusion() const
{
	/* 我们依赖辅助来约束漂移，因此在没有辅助的指定时间后
	 * 需要采取措施
	 */
	bool has_horizontal_aiding_timed_out = isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max);

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (has_horizontal_aiding_timed_out) {
		// 如果光流仍然活跃，则水平辅助没有超时
		if (_control_status.flags.opt_flow && isRecent(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max)) {
			has_horizontal_aiding_timed_out = false; // 更新超时状态
		}
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	const bool is_reset_required = has_horizontal_aiding_timed_out
				       || (isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max)
					   && (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS)));

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_inflight_nav_failure); // 返回是否需要重置
}

// 停止 GPS 融合的函数
void Ekf::stopGpsFusion()
{
	if (_control_status.flags.gps) {
		ECL_INFO("stopping GPS position and velocity fusion"); // 信息：停止 GPS 位置和速度融合

		_last_gps_fail_us = 0; // 重置 GPS 失败时间
		_last_gps_pass_us = 0; // 重置 GPS 通过时间

		_control_status.flags.gps = false; // 更新 GPS 状态为假
	}

	stopGpsHgtFusion(); // 停止 GPS 高度融合
#if defined(CONFIG_EKF2_GNSS_YAW)
	stopGnssYawFusion(); // 停止 GNSS 偏航融合
#endif // CONFIG_EKF2_GNSS_YAW

	_yawEstimator.reset(); // 重置偏航估计器
}

// 检查偏航紧急估计是否可用的函数
bool Ekf::isYawEmergencyEstimateAvailable() const
{
	// 在滤波器开始融合速度数据并且偏航估计已收敛之前，不允许重置使用 EKF-GSF 估计
	if (!_yawEstimator.isActive()) {
		return false; // 偏航估计器未激活
	}

	const float yaw_var = _yawEstimator.getYawVar(); // 获取偏航方差

	return (yaw_var > 0.f)
	       && (yaw_var < sq(_params.EKFGSF_yaw_err_max))
	       && PX4_ISFINITE(yaw_var); // 返回偏航方差的有效性
}

// 检查偏航是否失败的函数
bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false; // 偏航紧急估计不可用
	}

	const float euler_yaw = getEulerYaw(_R_to_earth); // 获取地球坐标系下的欧拉偏航
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw()); // 计算偏航误差

	return fabsf(yaw_error) > math::radians(25.f); // 返回偏航误差是否超过阈值
}

// 将偏航重置到 EKF-GSF 的函数
bool Ekf::resetYawToEKFGSF()
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false; // 偏航紧急估计不可用
	}

	// 如果刚刚进行了偏航重置，则不允许重置
	const bool yaw_alignment_changed = (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);
	const bool quat_reset = (_state_reset_status.reset_count.quat != _state_reset_count_prev.quat);

	if (yaw_alignment_changed || quat_reset) {
		return false; // 返回重置失败
	}

	ECL_INFO("yaw estimator reset heading %.3f -> %.3f rad",
		 (double)getEulerYaw(_R_to_earth), (double)_yawEstimator.getYaw()); // 信息：偏航估计器重置

	resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar()); // 重置四元数状态的偏航

	_control_status.flags.yaw_align = true; // 更新偏航对齐状态
	_information_events.flags.yaw_aligned_to_imu_gps = true; // 标记偏航已对齐到 IMU 和 GPS

	return true; // 返回重置成功
}

// 获取 EKF-GSF 数据的函数
bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight); // 获取偏航估计器的日志数据
}
