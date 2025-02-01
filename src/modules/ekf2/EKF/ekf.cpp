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
 * @file ekf.cpp
 * @brief 这是EKF核心实现文件，包含姿态和位置的EKF估计器核心功能。
 *
 * @author Roman Bast
 * @author Paul Riseborough
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

/**
 * @brief 初始化EKF，如果尚未初始化则执行初始化流程
 * @param timestamp 当前时间戳（微秒）
 * @return 如果成功初始化或已初始化返回true，否则返回false
 */
bool Ekf::init(uint64_t timestamp)
{
	// 如果还没有标记已初始化，则先调用initialise_interface进行接口初始化
	if (!_initialised) {
		_initialised = initialise_interface(timestamp);
		// 完成后进行滤波器内部状态等全面重置
		reset();
	}

	return _initialised;
}

/**
 * @brief 对EKF内部状态、参数、变量进行全面重置
 */
void Ekf::reset()
{
	// 输出相关信息：在运行时可记录“reset”到日志或控制台
	ECL_INFO("reset");

	// 重置核心状态量
	_state.quat_nominal.setIdentity();    // 初始姿态四元数设为单位阵
	_state.vel.setZero();                 // 速度置零
	_state.pos.setZero();                 // 位置置零
	_state.gyro_bias.setZero();           // 陀螺仪偏置置零
	_state.accel_bias.setZero();          // 加速度计偏置置零

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_state.mag_I.setZero();               // 地理磁场置零
	_state.mag_B.setZero();               // 机体磁偏置置零
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_state.wind_vel.setZero();            // 风速置零
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	// 估计地形高度时，假设一个初始的地面离机体距离
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 重置测距传感器处理相关参数
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);
	_range_sensor.setMaxFogDistance(_params.rng_fog);
#endif // CONFIG_EKF2_RANGE_FINDER

	_control_status.value = 0;
	_control_status_prev.value = 0;

	// 标记为飞行状态（in_air）为true
	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

#if defined(CONFIG_EKF2_GNSS)
	// GNSS相关漂移检查滤波器重置
	resetGpsDriftCheckFilters();
	_gps_checks_passed = false;
#endif // CONFIG_EKF2_GNSS

	_local_origin_alt = NAN;

	// 重置输出预测器
	_output_predictor.reset();

	// 重置一些时间戳计时
	_time_last_horizontal_aiding = 0;
	_time_last_v_pos_aiding = 0;
	_time_last_v_vel_aiding = 0;

	_time_last_hor_pos_fuse = 0;
	_time_last_hgt_fuse = 0;
	_time_last_hor_vel_fuse = 0;
	_time_last_ver_vel_fuse = 0;
	_time_last_heading_fuse = 0;
	_time_last_terrain_fuse = 0;

	_last_known_gpos.setZero();

#if defined(CONFIG_EKF2_BAROMETER)
	_baro_counter = 0;
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_mag_counter = 0;
#endif // CONFIG_EKF2_MAGNETOMETER

	_time_bad_vert_accel = 0;
	_time_good_vert_accel = 0;

	// 重置陀螺仪饱和截断计数器
	for (auto &clip_count : _clip_counter) {
		clip_count = 0;
	}

	_zero_velocity_update.reset();

	// 最后更新一下参数
	updateParameters();
}

/**
 * @brief EKF的主要更新函数，控制滤波器的预测和融合过程
 * @return 如果完成一次IMU数据更新并成功执行EKF更新则返回true，否则返回false
 */
bool Ekf::update()
{
	// 如果滤波器内部尚未完成首次初始化，则先进行初始化流程
	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// 只有IMU数据更新才进行下一步的滤波器更新
	if (_imu_updated) {
		_imu_updated = false;

		// 从IMU缓冲区中获取最旧的一帧IMU数据（保证延时对齐）
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		// 计算一个平均滤波更新时间，将dt进行一定比例的低通处理
		float input = 0.5f * (imu_sample_delayed.delta_vel_dt + imu_sample_delayed.delta_ang_dt);
		float filter_update_s = 1e-6f * _params.filter_update_interval_us;
		_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);

		// 根据飞行状态或其他条件决定是否禁止更新IMU偏置
		updateIMUBiasInhibit(imu_sample_delayed);

		// 调用主要的预测阶段：更新协方差
		predictCovariance(imu_sample_delayed);
		// 更新状态向量
		predictState(imu_sample_delayed);

		// 根据传感器数据、当前模式等控制融合逻辑
		controlFusionModes(imu_sample_delayed);

		// 根据最新的状态对输出进行修正，并进行相应的外推
		_output_predictor.correctOutputStates(imu_sample_delayed.time_us,
						      _state.quat_nominal,
						      _state.vel,
						      _gpos,
						      _state.gyro_bias,
						      _state.accel_bias);

		return true;
	}

	return false;
}

/**
 * @brief 内部滤波器初始化，包括初始姿态和协方差
 * @return 如果初始化成功返回true，否则返回false
 */
bool Ekf::initialiseFilter()
{
	// 取出最新的IMU数据，用于初始化姿态等
	const imuSample &imu_init = _imu_buffer.get_newest();

	// 保护：避免数据异常(如dt过小)
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	// 若是第一帧IMU数据，则直接重置滤波器内部的LPF状态
	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	// 尝试初始化机体姿态
	if (!initialiseTilt()) {
		return false;
	}

	// 初始化状态协方差矩阵
	initialiseCovariance();

	// 重置输出预测器的历史状态，使输出与EKF当前初始状态对齐
	_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _gpos);

	return true;
}

/**
 * @brief 初始化倾斜姿态（roll、pitch），主要依靠加速度计数据假设静止来判断重力方向
 * @return 如果倾斜姿态初始化成功返回true，否则返回false
 */
bool Ekf::initialiseTilt()
{
	// 计算当前低通滤波后的加速度和角速度模长
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	// 如果加速度不在合理重力范围内，或者角速度过大，则可能无法认为姿态静止，无法初始化
	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// 根据加速度计方向推断飞行器的上下方向，然后构造初始的四元数
	_state.quat_nominal = Quatf(_accel_lpf.getState(), Vector3f(0.f, 0.f, -1.f));
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

/**
 * @brief 在predictCovariance()调用后，对系统进行状态预测(更新状态向量)
 * @param imu_delayed 从IMU缓冲获取到的延时对准数据
 */
void Ekf::predictState(const imuSample &imu_delayed)
{
	// 如果当前纬度变化较大，需要更新地球自转速率
	if (std::fabs(_gpos.latitude_rad() - _earth_rate_lat_ref_rad) > math::radians(1.0)) {
		_earth_rate_lat_ref_rad = _gpos.latitude_rad();
		_earth_rate_NED = calcEarthRateNED((float)_earth_rate_lat_ref_rad);
	}

	// 应用陀螺仪偏置修正
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// 补偿由于地球自转带来的相对角速度
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	// 将修正后的角增量转化为四元数的增量
	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// 更新姿态四元数：q(k+1) = q(k) * dq
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// 计算机体坐标系下的线加速度增量
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;

	// 转换为地理坐标系下的速度增量
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// 保存上一步的速度，用于做梯形积分
	const Vector3f vel_last = _state.vel;

	// 在地理坐标系中累加当前的速度增量
	_state.vel += corrected_delta_vel_ef;

	// 同时补偿重力、科里奥利力与运载体转动（如经纬度引起的运输速率）
	const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G);
	const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
	const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
	_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

	// 位置使用梯形积分：pos(k+1) = pos(k) + (v(k)+v(k+1))/2 * dt
	_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
	// 将全局位置（其中_gpos.altitude()是Z）更新到state的pos(2)
	_state.pos(2) = -_gpos.altitude();

	// 防止速度出现极端过大
	_state.vel = matrix::constrain(_state.vel, -_params.velocity_limit, _params.velocity_limit);

	// 计算并保存水平方向加速度，用作其他机动检测等
	_accel_horiz_lpf.update(corrected_delta_vel_ef.xy() / imu_delayed.delta_vel_dt, imu_delayed.delta_vel_dt);
}

/**
 * @brief 根据外部观测重置全局位置（如来自GPS或外部定位），可选择是否融合高度
 * @param latitude   观测的纬度(度)
 * @param longitude  观测的经度(度)
 * @param altitude   观测的高度(米)
 * @param eph        水平位置误差(1σ, 米)
 * @param epv        垂直位置误差(1σ, 米)
 * @param timestamp_observation  该观测对应的时间戳(微秒)
 * @return 若观测数据有效且重置成功则返回true，否则返回false
 */
bool Ekf::resetGlobalPosToExternalObservation(const double latitude, const double longitude, const float altitude,
		const float eph,
		const float epv, uint64_t timestamp_observation)
{
	// 先检查经纬度有效性
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	// 如果本地坐标原点尚未初始化，则把该观测设置为原点
	if (!_local_origin_lat_lon.isInitialized()) {
		if (!resetLatLonTo(latitude, longitude, sq(eph))) {
			return false;
		}

		initialiseAltitudeTo(altitude, sq(epv));
		return true;
	}

	Vector3f pos_correction;

	// 根据延时对准的思路，用当前速度对位置做一次外推，以匹配观测时间
	if ((timestamp_observation > 0) && isLocalHorizontalPositionValid()) {

		// 观测时间不能超过最新时间戳
		timestamp_observation = math::min(_time_latest_us, timestamp_observation);

		float dt_us;

		if (_time_delayed_us >= timestamp_observation) {
			dt_us = static_cast<float>(_time_delayed_us - timestamp_observation);

		} else {
			dt_us = -static_cast<float>(timestamp_observation - _time_delayed_us);
		}

		const float dt_s = dt_us * 1e-6f;
		pos_correction = _state.vel * dt_s;
	}

	LatLonAlt gpos(latitude, longitude, altitude);
	bool alt_valid = true;

	// 如果外部的高度不可信，则只融合水平位置
	if (!checkAltitudeValidity(gpos.altitude())) {
		gpos.setAltitude(_gpos.altitude());
		alt_valid = false;
	}

	const LatLonAlt gpos_corrected = gpos + pos_correction;

	{
		// 水平观测的方差
		const float obs_var = math::max(sq(eph), sq(0.01f));

		const Vector2f innov = (_gpos - gpos_corrected).xy();
		const Vector2f innov_var = Vector2f(getStateVariance<State::pos>()) + obs_var;

		// 卡方检验阈值，这里硬编码为5
		const float sq_gate = sq(5.f);
		const float test_ratio = sq(innov(0)) / (sq_gate * innov_var(0))
                         + sq(innov(1)) / (sq_gate * innov_var(1));

		const bool innov_rejected = (test_ratio > 1.f);

		// 如果地面或者位置精度足够高，或者创新过大，则直接重置
		if (!_control_status.flags.in_air || (eph > 0.f && eph < 1.f) || innov_rejected) {
			ECL_INFO("reset position to external observation");
			_information_events.flags.reset_pos_to_ext_obs = true;

			resetHorizontalPositionTo(gpos_corrected.latitude_deg(),
						  gpos_corrected.longitude_deg(),
						  obs_var);
			_last_known_gpos.setLatLon(gpos_corrected);

		} else {
			// 否则以观测为测量进行融合（类似卡尔曼融合）
			ECL_INFO("fuse external observation as position measurement");
			fuseDirectStateMeasurement(innov(0), innov_var(0), obs_var, State::pos.idx + 0);
			fuseDirectStateMeasurement(innov(1), innov_var(1), obs_var, State::pos.idx + 1);

			// 重置计数器等，给外部控制器告知可能存在位置突变
			_state_reset_status.reset_count.posNE++;
			_state_reset_status.posNE_change.zero();

			_time_last_hor_pos_fuse = _time_delayed_us;
			_last_known_gpos.setLatLon(gpos_corrected);
		}
	}

	// 如果高度有效，则根据观测对高度进行初始化或重置
	if (alt_valid) {
		const float obs_var = math::max(sq(epv), sq(0.01f));

		ECL_INFO("reset height to external observation");
		initialiseAltitudeTo(gpos_corrected.altitude(), obs_var);
		_last_known_gpos.setAltitude(gpos_corrected.altitude());
	}

	return true;
}

/**
 * @brief 从参数服务更新EKF滤波器所需的各种参数
 */
void Ekf::updateParameters()
{
	// 限幅处理，防止参数错误导致数值过大
	_params.gyro_noise = math::constrain(_params.gyro_noise, 0.f, 1.f);
	_params.accel_noise = math::constrain(_params.accel_noise, 0.f, 1.f);

	_params.gyro_bias_p_noise = math::constrain(_params.gyro_bias_p_noise, 0.f, 1.f);
	_params.accel_bias_p_noise = math::constrain(_params.accel_bias_p_noise, 0.f, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	_params.mage_p_noise = math::constrain(_params.mage_p_noise, 0.f, 1.f);
	_params.magb_p_noise = math::constrain(_params.magb_p_noise, 0.f, 1.f);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	_params.wind_vel_nsd = math::constrain(_params.wind_vel_nsd, 0.f, 1.f);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.updateParameters();
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
}

/**
 * @brief 打印环形缓冲区信息，主要用于调试
 * @tparam T 缓冲区存储的数据类型
 * @param name 缓冲区的名称
 * @param rb   缓冲区指针
 */
template<typename T>
static void printRingBuffer(const char *name, RingBuffer<T> *rb)
{
	if (rb) {
		printf("%s: %d/%d entries (%d/%d Bytes) (%zu Bytes per entry)\n",
		       name,
		       rb->entries(), rb->get_length(), rb->get_used_size(), rb->get_total_size(),
		       sizeof(T));
	}
}

/**
 * @brief 调试和状态信息输出函数，可在终端打印出EKF内部状态等
 */
void Ekf::print_status()
{
	printf("\nStates: (%.4f seconds ago)\n", (_time_latest_us - _time_delayed_us) * 1e-6);
	printf("Orientation (%d-%d): [%.3f, %.3f, %.3f, %.3f] (Euler [%.1f, %.1f, %.1f] deg) var: [%.1e, %.1e, %.1e]\n",
	       State::quat_nominal.idx, State::quat_nominal.idx + State::quat_nominal.dof - 1,
	       (double)_state.quat_nominal(0), (double)_state.quat_nominal(1), (double)_state.quat_nominal(2),
	       (double)_state.quat_nominal(3),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).phi()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).theta()),
	       (double)math::degrees(matrix::Eulerf(_state.quat_nominal).psi()),
	       (double)getStateVariance<State::quat_nominal>()(0), (double)getStateVariance<State::quat_nominal>()(1),
	       (double)getStateVariance<State::quat_nominal>()(2)
	      );

	printf("Velocity (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::vel.idx, State::vel.idx + State::vel.dof - 1,
	       (double)_state.vel(0), (double)_state.vel(1), (double)_state.vel(2),
	       (double)getStateVariance<State::vel>()(0), (double)getStateVariance<State::vel>()(1),
	       (double)getStateVariance<State::vel>()(2)
	      );

	const Vector3f position = getPosition();
	printf("Position (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::pos.idx, State::pos.idx + State::pos.dof - 1,
	       (double)position(0), (double)position(1), (double) position(2),
	       (double)getStateVariance<State::pos>()(0), (double)getStateVariance<State::pos>()(1),
	       (double)getStateVariance<State::pos>()(2)
	      );

	printf("Gyro Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::gyro_bias.idx, State::gyro_bias.idx + State::gyro_bias.dof - 1,
	       (double)_state.gyro_bias(0), (double)_state.gyro_bias(1), (double)_state.gyro_bias(2),
	       (double)getStateVariance<State::gyro_bias>()(0), (double)getStateVariance<State::gyro_bias>()(1),
	       (double)getStateVariance<State::gyro_bias>()(2)
	      );

	printf("Accel Bias (%d-%d): [%.6f, %.6f, %.6f] var: [%.1e, %.1e, %.1e]\n",
	       State::accel_bias.idx, State::accel_bias.idx + State::accel_bias.dof - 1,
	       (double)_state.accel_bias(0), (double)_state.accel_bias(1), (double)_state.accel_bias(2),
	       (double)getStateVariance<State::accel_bias>()(0), (double)getStateVariance<State::accel_bias>()(1),
	       (double)getStateVariance<State::accel_bias>()(2)
	      );

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printf("Magnetic Field (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_I.idx, State::mag_I.idx + State::mag_I.dof - 1,
	       (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
	       (double)getStateVariance<State::mag_I>()(0), (double)getStateVariance<State::mag_I>()(1),
	       (double)getStateVariance<State::mag_I>()(2)
	      );

	printf("Magnetic Bias (%d-%d): [%.3f, %.3f, %.3f] var: [%.1e, %.1e, %.1e]\n",
	       State::mag_B.idx, State::mag_B.idx + State::mag_B.dof - 1,
	       (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2),
	       (double)getStateVariance<State::mag_B>()(0), (double)getStateVariance<State::mag_B>()(1),
	       (double)getStateVariance<State::mag_B>()(2)
	      );
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	printf("Wind velocity (%d-%d): [%.3f, %.3f] var: [%.1e, %.1e]\n",
	       State::wind_vel.idx, State::wind_vel.idx + State::wind_vel.dof - 1,
	       (double)_state.wind_vel(0), (double)_state.wind_vel(1),
	       (double)getStateVariance<State::wind_vel>()(0), (double)getStateVariance<State::wind_vel>()(1)
	      );
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	printf("Terrain position (%d): %.3f var: %.1e\n",
	       State::terrain.idx,
	       (double)_state.terrain,
	       (double)getStateVariance<State::terrain>()(0)
	      );
#endif // CONFIG_EKF2_TERRAIN

	printf("\nP:\n");
	P.print();

	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);
	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	// 分别打印各种传感器环形缓冲区信息
	printRingBuffer("IMU buffer", &_imu_buffer);
	printRingBuffer("system flag buffer", _system_flag_buffer);

#if defined(CONFIG_EKF2_AIRSPEED)
	printRingBuffer("airspeed buffer", _airspeed_buffer);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
	printRingBuffer("aux vel buffer", _auxvel_buffer);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
	printRingBuffer("baro buffer", _baro_buffer);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	printRingBuffer("drag buffer", _drag_buffer);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	printRingBuffer("ext vision buffer", _ext_vision_buffer);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	printRingBuffer("gps buffer", _gps_buffer);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	printRingBuffer("mag buffer", _mag_buffer);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	printRingBuffer("flow buffer", _flow_buffer);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
	printRingBuffer("range buffer", _range_buffer);
#endif // CONFIG_EKF2_RANGE_FINDER

	_output_predictor.print_status();
}
