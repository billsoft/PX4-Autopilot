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
 * @file ekf.h
 * Class for core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */
#ifndef EKF_EKF_H
#define EKF_EKF_H

#include "estimator_interface.h" // 引入估计器接口

#if defined(CONFIG_EKF2_GNSS)
# include "yaw_estimator/EKFGSF_yaw.h" // 如果启用了GNSS，包含航向估计器
#endif // CONFIG_EKF2_GNSS

#include "bias_estimator/bias_estimator.hpp" // 引入偏差估计器
#include "bias_estimator/height_bias_estimator.hpp" // 引入高度偏差估计器
#include "bias_estimator/position_bias_estimator.hpp" // 引入位置偏差估计器

#include <ekf_derivation/generated/state.h> // 引入状态生成的头文件

#include <uORB/topics/estimator_aid_source1d.h> // 引入一维辅助源主题
#include <uORB/topics/estimator_aid_source2d.h> // 引入二维辅助源主题
#include <uORB/topics/estimator_aid_source3d.h> // 引入三维辅助源主题

#include "aid_sources/ZeroGyroUpdate.hpp" // 引入零陀螺更新的辅助源
#include "aid_sources/ZeroVelocityUpdate.hpp" // 引入零速度更新的辅助源

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION)
# include "aid_sources/aux_global_position/aux_global_position.hpp" // 如果启用了辅助全局位置，包含相关头文件
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

// 定义可能的似然性等级
enum class Likelihood { LOW, MEDIUM, HIGH };
class ExternalVisionVel; // 声明外部视觉速度类

// EKF类，继承自估计器接口
class Ekf final : public EstimatorInterface
{
public:
	typedef matrix::Vector<float, State::size> VectorState; // 定义状态向量类型
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState; // 定义状态方阵类型

	Ekf() // 构造函数
	{
		reset(); // 初始化时重置状态
	};

	virtual ~Ekf() = default; // 默认析构函数

	// 初始化变量为合理值（也是接口类的一部分）
	bool init(uint64_t timestamp) override;

	void print_status(); // 打印状态信息

	// 每次新数据推入滤波器时应调用此函数
	bool update();

	const StateSample &state() const { return _state; } // 获取当前状态样本

#if defined(CONFIG_EKF2_BAROMETER)
	const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; } // 获取气压高度辅助源
	const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); } // 获取气压偏差估计器状态
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_TERRAIN)
	// 检查地形估计是否有效
	bool isTerrainEstimateValid() const { return _terrain_valid; }

	// 获取相对于NED原点的估计地形垂直位置
	float getTerrainVertPos() const { return _state.terrain + getEkfGlobalOriginAltitude(); };
	float getHagl() const { return _state.terrain + _gpos.altitude(); } // 获取地面高度

	// 获取地形方差
	float getTerrainVariance() const { return P(State::terrain.idx, State::terrain.idx); }

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 获取范围高度
	const auto &aid_src_rng_hgt() const { return _aid_src_rng_hgt; }

	float getHaglRateInnov() const { return _rng_consistency_check.getInnov(); } // 获取地面高度变化的创新值
	float getHaglRateInnovVar() const { return _rng_consistency_check.getInnovVar(); } // 获取地面高度变化创新的方差
	float getHaglRateInnovRatio() const { return _rng_consistency_check.getSignedTestRatioLpf(); } // 获取地面高度变化创新比率
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	const auto &aid_src_optical_flow() const { return _aid_src_optical_flow; } // 获取光流辅助源

	const Vector2f &getFlowVelBody() const { return _flow_vel_body; } // 获取机体坐标系下的光流速度
	Vector2f getFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFlowVelBody()(0), getFlowVelBody()(1), 0.f)); } // 获取东北坐标系下的光流速度

	const Vector2f &getFilteredFlowVelBody() const { return _flow_vel_body_lpf.getState(); } // 获取滤波后的机体坐标系下光流速度
	Vector2f getFilteredFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFilteredFlowVelBody()(0), getFilteredFlowVelBody()(1), 0.f)); } // 获取滤波后的东北坐标系下光流速度

	const Vector2f &getFlowCompensated() const { return _flow_rate_compensated; } // 获取补偿后的光流速度
	const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_rate; } // 获取未补偿的光流速度

	const Vector3f getFlowGyro() const { return _flow_sample_delayed.gyro_rate; } // 获取光流样本延迟的陀螺仪速率
	const Vector3f &getFlowGyroBias() const { return _flow_gyro_bias; } // 获取光流陀螺仪偏差
	const Vector3f &getFlowRefBodyRate() const { return _ref_body_rate; } // 获取参考机体速率
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float getHeadingInnov() const; // 获取航向创新值
	float getHeadingInnovVar() const; // 获取航向创新方差
	float getHeadingInnovRatio() const; // 获取航向创新比率

#if defined(CONFIG_EKF2_DRAG_FUSION)
	const auto &aid_src_drag() const { return _aid_src_drag; } // 获取拖曳融合的辅助源
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	const auto &aid_src_gravity() const { return _aid_src_gravity; } // 获取重力融合的辅助源
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_WIND)
	// 获取风速（单位：m/s）
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };
	Vector2f getWindVelocityVariance() const { return getStateVariance<State::wind_vel>(); } // 获取风速方差

	/**
	* @brief 将风状态重置为外部观测值
	*
	* @param wind_speed 风速（单位：m/s）
	* @param wind_direction 风向（从真北开始的方位角，单位：弧度）
	* @param wind_speed_accuracy 风速估计的1σ精度（单位：m/s）
	* @param wind_direction_accuracy 风向估计的1σ精度（单位：弧度）
	*/
	void resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
					    float wind_direction_accuracy);
#endif // CONFIG_EKF2_WIND

	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // 调用getStateCovariance().diag()会使用更多的闪存空间

	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); } // 获取状态协方差

	// 获取完整的协方差矩阵
	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }
	float stateCovariance(unsigned r, unsigned c) const { return P(r, c); } // 获取指定位置的协方差值

	// 获取协方差矩阵的对角元素
	matrix::Vector<float, State::size> covariances_diagonal() const { return P.diag(); } // 获取协方差矩阵的对角线元素

	matrix::Vector3f getRotVarBody() const; // 获取机体坐标系下的旋转方差
	matrix::Vector3f getRotVarNed() const; // 获取NED坐标系下的旋转方差
	float getYawVar() const; // 获取航向方差
	float getTiltVariance() const; // 获取倾斜方差

	Vector3f getVelocityVariance() const { return getStateVariance<State::vel>(); }; // 获取速度方差

	Vector3f getPositionVariance() const { return getStateVariance<State::pos>(); } // 获取位置方差

	// 获取EKF WGS-84原点位置和高度以及系统最后设置的时间
	void getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
	bool checkLatLonValidity(double latitude, double longitude); // 检查经纬度的有效性
	bool checkAltitudeValidity(float altitude); // 检查高度的有效性
	bool setEkfGlobalOrigin(double latitude, double longitude, float altitude, float hpos_var = NAN, float vpos_var = NAN); // 设置EKF全局原点
	bool resetGlobalPositionTo(double latitude, double longitude, float altitude, float hpos_var = NAN,
				   float vpos_var = NAN); // 重置全局位置

	// 获取EKF WGS-84位置的1σ水平和垂直位置不确定性
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// 获取EKF局部位置的1σ水平和垂直位置不确定性
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// 获取1σ水平和垂直速度的不确定性
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const;

	// 返回估计器所需的车辆控制限制，以保持在传感器限制范围内。
	//  vxy_max : 最大地面相对水平速度（米/秒）。当不需要限制时为NaN。
	//  vz_max : 最大地面相对垂直速度（米/秒）。当不需要限制时为NaN。
	//  hagl_min : 地面以上的最小高度（米）。当不需要限制时为NaN。
	//  hagl_max_z : 垂直高度控制的最大地面以上高度（米）。当不需要限制时为NaN。
	//  hagl_max_xy : 水平位置控制的最大地面以上高度（米）。当不需要限制时为NaN。
	void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max_z, float *hagl_max_xy) const;

	void resetGyroBias(); // 重置陀螺仪偏差
	void resetGyroBiasCov(); // 重置陀螺仪偏差协方差

	void resetAccelBias(); // 重置加速度计偏差
	void resetAccelBiasCov(); // 重置加速度计偏差协方差

	// 检查全局水平位置是否有效
	bool isGlobalHorizontalPositionValid() const
	{
		return _local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid(); // 检查本地原点是否初始化并且本地水平位置有效
	}

	// 检查全局垂直位置是否有效
	bool isGlobalVerticalPositionValid() const
	{
		return PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid(); // 检查本地原点高度是否有限并且本地垂直位置有效
	}

	// 检查本地水平位置是否有效
	bool isLocalHorizontalPositionValid() const
	{
		return !_horizontal_deadreckon_time_exceeded; // 检查水平死算时间是否超限
	}

	// 检查本地垂直位置是否有效
	bool isLocalVerticalPositionValid() const
	{
		return !_vertical_position_deadreckon_time_exceeded; // 检查垂直位置死算时间是否超限
	}

	// 检查本地垂直速度是否有效
	bool isLocalVerticalVelocityValid() const
	{
		return !_vertical_velocity_deadreckon_time_exceeded; // 检查垂直速度死算时间是否超限
	}

	// 检查航向最终对齐是否完成
	bool isYawFinalAlignComplete() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)
		const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg); // 检查是否使用磁力计
		const bool is_mag_alignment_in_flight_complete = is_using_mag
				&& _control_status.flags.mag_aligned_in_flight
				&& ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6); // 检查磁力计对齐是否完成
		return _control_status.flags.yaw_align
		       && (is_mag_alignment_in_flight_complete || !is_using_mag); // 返回航向对齐状态
#else
		return _control_status.flags.yaw_align; // 返回航向对齐状态
#endif
	}

	// 融合单一直接状态测量（例如NED速度、NED位置、地磁场等）
	void fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index);

	bool measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation); // 进行测量更新

	// 获取陀螺仪偏差
	const Vector3f &getGyroBias() const { return _state.gyro_bias; } // 获取陀螺仪偏差（单位：rad/s）
	Vector3f getGyroBiasVariance() const { return getStateVariance<State::gyro_bias>(); } // 获取陀螺仪偏差方差（单位：rad/s）
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; } // 获取陀螺仪偏差限制
	float getGyroNoise() const { return _params.gyro_noise; } // 获取陀螺仪噪声

	// 获取加速度计偏差
	const Vector3f &getAccelBias() const { return _state.accel_bias; } // 获取加速度计偏差（单位：m/s²）
	Vector3f getAccelBiasVariance() const { return getStateVariance<State::accel_bias>(); } // 获取加速度计偏差方差（单位：m/s²）
	float getAccelBiasLimit() const { return _params.acc_bias_lim; } // 获取加速度计偏差限制

#if defined(CONFIG_EKF2_MAGNETOMETER)
	const Vector3f &getMagEarthField() const { return _state.mag_I; } // 获取地球磁场

	const Vector3f &getMagBias() const { return _state.mag_B; } // 获取磁偏差
	Vector3f getMagBiasVariance() const { return getStateVariance<State::mag_B>(); } // 获取磁偏差方差（单位：Gauss）
	float getMagBiasLimit() const { return 0.5f; } // 获取磁偏差限制（0.5 Gauss）
#endif // CONFIG_EKF2_MAGNETOMETER

	// 检查加速度计偏差是否被抑制
	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }
	// 检查陀螺仪偏差是否被抑制
	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

	const auto &state_reset_status() const { return _state_reset_status; } // 获取状态重置状态

	// 返回上次重置中本地垂直位置变化的量和重置事件的数量
	uint8_t get_posD_reset_count() const { return _state_reset_status.reset_count.posD; }
	void get_posD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.posD_change; // 获取位置变化量
		*counter = _state_reset_status.reset_count.posD; // 获取重置计数
	}

	uint8_t get_hagl_reset_count() const { return _state_reset_status.reset_count.hagl; } // 获取地面高度重置计数
	void get_hagl_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.hagl_change; // 获取地面高度变化量
		*counter = _state_reset_status.reset_count.hagl; // 获取重置计数
	}

	// 返回上次重置中本地垂直速度变化的量和重置事件的数量
	uint8_t get_velD_reset_count() const { return _state_reset_status.reset_count.velD; }
	void get_velD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.velD_change; // 获取速度变化量
		*counter = _state_reset_status.reset_count.velD; // 获取重置计数
	}

	// 返回上次重置中本地水平位置变化的量和重置事件的数量
	uint8_t get_posNE_reset_count() const { return _state_reset_status.reset_count.posNE; }
	void get_posNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.posNE_change.copyTo(delta); // 获取位置变化量
		*counter = _state_reset_status.reset_count.posNE; // 获取重置计数
	}

	// 返回上次重置中本地水平速度变化的量和重置事件的数量
	uint8_t get_velNE_reset_count() const { return _state_reset_status.reset_count.velNE; }
	void get_velNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.velNE_change.copyTo(delta); // 获取速度变化量
		*counter = _state_reset_status.reset_count.velNE; // 获取重置计数
	}

	// 返回上次重置中四元数变化的量和重置事件的数量
	uint8_t get_quat_reset_count() const { return _state_reset_status.reset_count.quat; }
	void get_quat_reset(float delta_quat[4], uint8_t *counter) const
	{
		_state_reset_status.quat_change.copyTo(delta_quat); // 获取四元数变化量
		*counter = _state_reset_status.reset_count.quat; // 获取重置计数
	}

	float getHeadingInnovationTestRatio() const; // 获取航向创新测试比率

	float getHorizontalVelocityInnovationTestRatio() const; // 获取水平速度创新测试比率
	float getVerticalVelocityInnovationTestRatio() const; // 获取垂直速度创新测试比率

	float getHorizontalPositionInnovationTestRatio() const; // 获取水平位置创新测试比率
	float getVerticalPositionInnovationTestRatio() const; // 获取垂直位置创新测试比率

	float getAirspeedInnovationTestRatio() const; // 获取空速创新测试比率
	float getSyntheticSideslipInnovationTestRatio() const; // 获取合成侧滑创新测试比率

	float getHeightAboveGroundInnovationTestRatio() const; // 获取地面高度创新测试比率

	// 返回一个位掩码整数，描述哪些状态估计是有效的
	uint16_t get_ekf_soln_status() const;

	HeightSensor getHeightSensorRef() const { return _height_sensor_ref; } // 获取高度传感器参考

#if defined(CONFIG_EKF2_AIRSPEED)
	const auto &aid_src_airspeed() const { return _aid_src_airspeed; } // 获取空速辅助源
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	const auto &aid_src_sideslip() const { return _aid_src_sideslip; }
#endif // CONFIG_EKF2_SIDESLIP

	const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
	const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
	const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
	const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
	const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }

	const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }
	const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// set minimum continuous period without GPS fail required to mark a healthy GPS status
	void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

	const gps_check_fail_status_u &gps_check_fail_status() const { return _gps_check_fail_status; }
	const decltype(gps_check_fail_status_u::flags) &gps_check_fail_status_flags() const { return _gps_check_fail_status.flags; }

	bool gps_checks_passed() const { return _gps_checks_passed; };

	const BiasEstimator::status &getGpsHgtBiasEstimatorStatus() const { return _gps_hgt_b_est.getStatus(); }

	const auto &aid_src_gnss_hgt() const { return _aid_src_gnss_hgt; }
	const auto &aid_src_gnss_pos() const { return _aid_src_gnss_pos; }
	const auto &aid_src_gnss_vel() const { return _aid_src_gnss_vel; }

# if defined(CONFIG_EKF2_GNSS_YAW)
	const auto &aid_src_gnss_yaw() const { return _aid_src_gnss_yaw; }
# endif // CONFIG_EKF2_GNSS_YAW

	// Returns true if the output of the yaw emergency estimator can be used for a reset
	bool isYawEmergencyEstimateAvailable() const;

	// get solution data from the EKF-GSF emergency yaw estimator
	// returns false when data is not available
	bool getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			   float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// set the magnetic field data returned by the geo library using position
	bool updateWorldMagneticModel(const double latitude_deg, const double longitude_deg);

	const auto &aid_src_mag() const { return _aid_src_mag; }
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AUXVEL)
	const auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }
#endif // CONFIG_EKF2_AUXVEL

	bool resetGlobalPosToExternalObservation(double latitude, double longitude, float altitude, float eph, float epv,
			uint64_t timestamp_observation);

	void updateParameters();

	friend class AuxGlobalPosition;

private:

	friend class ExternalVisionVel;
	friend class EvVelBodyFrameFrd;
	friend class EvVelLocalFrameNed;
	friend class EvVelLocalFrameFrd;

	// set the internal states and status to their default value
	void reset();

	bool initialiseTilt();

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	void updateDeadReckoningStatus();
	void updateHorizontalDeadReckoningstatus();
	void updateVerticalDeadReckoningStatus();

	static constexpr float kGyroBiasVarianceMin{1e-9f};
	static constexpr float kAccelBiasVarianceMin{1e-9f};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	static constexpr float kMagVarianceMin = 1e-6f;
#endif // CONFIG_EKF2_MAGNETOMETER


	struct StateResetCounts {
		uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
		uint8_t hagl{0};	///< number of height above ground level reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
		float hagl_change;	///< Height above ground level change due to last reset (m)

		StateResetCounts reset_count{};
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{};

	StateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	LatLonAlt _gpos{0.0, 0.0, 0.f};

	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
	uint64_t _time_last_v_pos_aiding{0};
	uint64_t _time_last_v_vel_aiding{0};

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_heading_fuse{0};
	uint64_t _time_last_terrain_fuse{0};

	LatLonAlt _last_known_gpos{};

	Vector3f _earth_rate_NED{}; ///< earth rotation vector (NED) in rad/s
	double _earth_rate_lat_ref_rad{0.0}; ///< latitude at which the earth rate was evaluated (radians)

	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	static constexpr float _kAccelHorizLpfTimeConstant = 1.f;
	AlphaFilter<Vector2f> _accel_horiz_lpf{_kAccelHorizLpfTimeConstant}; ///< Low pass filtered horizontal earth frame acceleration (m/sec**2)

#if defined(CONFIG_EKF2_WIND)
	static constexpr float _kHeightRateLpfTimeConstant = 10.f;
	AlphaFilter<float> _height_rate_lpf{_kHeightRateLpfTimeConstant};
#endif // CONFIG_EKF2_WIND
	// 状态协方差矩阵
	SquareMatrixState P{};	///< 状态协方差矩阵

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 多旋翼拖曳融合的辅助源
	estimator_aid_source2d_s _aid_src_drag {};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_TERRAIN)
	// 地形高度状态估计
	float _last_on_ground_posD{0.0f};	///< 当在空中状态为假时的最后垂直位置（米）

	bool _terrain_valid{false}; ///< 地形有效性标志
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 距离传感器辅助源
	estimator_aid_source1d_s _aid_src_rng_hgt {};
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流辅助源
	estimator_aid_source2d_s _aid_src_optical_flow {};

	// 光流处理
	Vector3f _flow_gyro_bias{};	///< 光流传感器陀螺仪输出的偏差误差（弧度/秒）
	Vector3f _ref_body_rate{}; ///< 参考机体角速度

	Vector2f _flow_vel_body{};                      ///< 从修正的光流测量得到的速度（机体坐标系）（米/秒）
	AlphaFilter<Vector2f> _flow_vel_body_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant}; ///< 从修正的光流测量得到的过滤速度（机体坐标系）（米/秒）
	uint32_t _flow_counter{0};                      ///< 初始化时读取的光流样本数量

	Vector2f _flow_rate_compensated{}; ///< 去除机体旋转后的图像在X和Y机体轴上的测量角速度（弧度/秒），右手旋转为正
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
	// 空速辅助源
	estimator_aid_source1d_s _aid_src_airspeed {};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 侧滑辅助源
	estimator_aid_source1d_s _aid_src_sideslip {};
#endif // CONFIG_EKF2_SIDESLIP

	// 假位置和高度的辅助源
	estimator_aid_source2d_s _aid_src_fake_pos{};
	estimator_aid_source1d_s _aid_src_fake_hgt{};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 外部视觉辅助源
	estimator_aid_source1d_s _aid_src_ev_hgt {};
	estimator_aid_source2d_s _aid_src_ev_pos{};
	estimator_aid_source3d_s _aid_src_ev_vel{};
	estimator_aid_source1d_s _aid_src_ev_yaw{};

	uint8_t _nb_ev_pos_reset_available{0}; ///< 可用的外部视觉位置重置数量
	uint8_t _nb_ev_vel_reset_available{0}; ///< 可用的外部视觉速度重置数量
	uint8_t _nb_ev_yaw_reset_available{0}; ///< 可用的外部视觉航向重置数量
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// GPS数据准备状态
	bool _gps_data_ready {false};	///< 当新的GPS数据落后于融合时间范围并可用于融合时为真

	// 用于GPS质量检查的变量
	Vector3f _gps_pos_deriv_filt{};	///< GPS NED位置导数（米/秒）
	Vector2f _gps_velNE_filt{};	///< 过滤后的GPS北向和东向速度（米/秒）

	float _gps_vel_d_filt{0.0f};		///< GNSS过滤后的下行速度（米/秒）
	uint64_t _last_gps_fail_us{0};		///< GPS检查失败的最后系统时间（微秒）
	uint64_t _last_gps_pass_us{0};		///< GPS检查通过的最后系统时间（微秒）
	uint32_t _min_gps_health_time_us{10000000}; ///< GPS在此时间后被标记为健康
	bool _gps_checks_passed{false};		///< 当所有活动GPS检查通过时为真

	gps_check_fail_status_u _gps_check_fail_status{}; ///< GPS检查失败状态
	// 高度传感器状态
	bool _gps_intermittent{true};           ///< 如果数据进入缓冲区是间歇性的则为真

	// GPS高度偏差估计器
	HeightBiasEstimator _gps_hgt_b_est{HeightSensor::GNSS, _height_sensor_ref};

	// GNSS辅助源
	estimator_aid_source1d_s _aid_src_gnss_hgt{};
	estimator_aid_source2d_s _aid_src_gnss_pos{};
	estimator_aid_source3d_s _aid_src_gnss_vel{};

# if defined(CONFIG_EKF2_GNSS_YAW)
	// GNSS航向辅助源
	estimator_aid_source1d_s _aid_src_gnss_yaw {};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力融合辅助源
	estimator_aid_source3d_s _aid_src_gravity {};
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度源
	estimator_aid_source2d_s _aid_src_aux_vel {};
#endif // CONFIG_EKF2_AUXVEL

	// 用于初始滤波器对齐的变量
	bool _is_first_imu_sample{true}; ///< 是否为第一次IMU样本
	static constexpr float _kSensorLpfTimeConstant = 0.09f; ///< 传感器低通滤波时间常数
	AlphaFilter<Vector3f> _accel_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< 用于对齐倾斜的过滤加速度计测量（米/秒²）
	AlphaFilter<Vector3f> _gyro_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< 用于对齐过度运动检查的过滤陀螺仪测量（弧度/秒）

#if defined(CONFIG_EKF2_BAROMETER)
	// 气压计辅助源
	estimator_aid_source1d_s _aid_src_baro_hgt {};

	// 用于执行飞行重置和在高度源之间切换的变量
	AlphaFilter<float> _baro_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< 过滤的气压高度测量（米）
	uint32_t _baro_counter{0};		///< 初始化时读取的气压计样本数量

	// 气压计高度偏差估计器
	HeightBiasEstimator _baro_b_est{HeightSensor::BARO, _height_sensor_ref};

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 用于磁力计融合模式选择的变量
	AlphaFilter<float> _mag_heading_innov_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant}; ///< 磁力计航向创新低通滤波器
	uint32_t _min_mag_health_time_us{1'000'000}; ///< 磁力计在此时间后被标记为健康

	// 磁力计辅助源
	estimator_aid_source3d_s _aid_src_mag{};

	AlphaFilter<Vector3f> _mag_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< 用于瞬时重置的过滤磁力计测量（高斯）
	uint32_t _mag_counter{0};		///< 初始化时读取的磁力计样本数量

	// 用于控制起飞后功能激活的变量
	uint64_t _flt_mag_align_start_time{0};	///< 飞行中磁场对齐开始的时间（微秒）
	uint64_t _time_last_mag_check_failing{0}; ///< 上次磁力计检查失败的时间
#endif // CONFIG_EKF2_MAGNETOMETER

	// 用于抑制加速度偏差学习的变量
	bool _accel_bias_inhibit[3] {};		///< 当指定轴的加速度偏差学习被抑制时为真
	bool _gyro_bias_inhibit[3] {};		///< 当指定轴的陀螺仪偏差学习被抑制时为真
	float _accel_magnitude_filt{0.0f};	///< 应用衰减包络滤波器后的加速度幅度（米/秒²）
	float _ang_rate_magnitude_filt{0.0f};		///< 应用衰减包络滤波器后的角速度幅度（弧度/秒）

	// IMU故障状态
	uint64_t _time_bad_vert_accel{0};	///< 检测到不良垂直加速度的最后时间（微秒）
	uint64_t _time_good_vert_accel{0};	///< 检测到良好垂直加速度的最后时间（微秒）
	uint16_t _clip_counter[3];		///< 每个轴的计数器，当发生剪切时递增，未发生时递减

	// 初始化延迟EKF和实时互补滤波器的滤波器状态
	bool initialiseFilter(void);

	// 初始化EKF协方差矩阵
	void initialiseCovariance();

	// 预测EKF状态
	void predictState(const imuSample &imu_delayed);

	// 预测EKF协方差
	void predictCovariance(const imuSample &imu_delayed);

	// 重置状态协方差
	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.0f); // 将协方差矩阵的指定部分的方差设置为0
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov; // 将新的协方差矩阵切片赋值
	}

	// 设置纬度和经度的原点
	bool setLatLonOrigin(double latitude, double longitude, float hpos_var = NAN);
	// 设置高度原点
	bool setAltOrigin(float altitude, float vpos_var = NAN);

	// 重置纬度和经度
	bool resetLatLonTo(double latitude, double longitude, float hpos_var = NAN);
	// 初始化高度
	bool initialiseAltitudeTo(float altitude, float vpos_var = NAN);

	// 使用创新、观测方差和雅可比向量更新四元数状态和协方差
	bool fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW);
	// 计算航向创新方差和雅可比矩阵
	void computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const;

	// 更新IMU偏差抑制状态
	void updateIMUBiasInhibit(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// EKF顺序融合磁力计测量
	bool fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, estimator_aid_source3d_s &aid_src,
		     bool update_all_states = false, bool update_tilt = false);

	// 融合磁力计的偏角测量
	//  R: 偏角观测方差（弧度²）
	bool fuseDeclination(const float decl_measurement_rad, const float R, bool update_all_states = false);

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	// 控制空气数据观测的融合
	void controlAirDataFusion(const imuSample &imu_delayed);

	// 更新空速
	void updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const;
	// 融合空速
	void fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src);

	// 停止空速融合
	void stopAirspeedFusion();

	// 使用当前空速测量、相对于地面的导航速度、航向角和零侧滑假设重置风状态
	void resetWindUsingAirspeed(const airspeedSample &airspeed_sample);
	void resetVelUsingAirspeed(const airspeedSample &airspeed_sample);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 控制合成侧滑观测的融合
	void controlBetaFusion(const imuSample &imu_delayed);

	// 融合合成零侧滑测量
	void updateSideslip(estimator_aid_source1d_s &_aid_src_sideslip) const;
	bool fuseSideslip(estimator_aid_source1d_s &_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 控制多旋翼拖曳特定力观测的融合
	void controlDragFusion(const imuSample &imu_delayed);

	// 融合机体框架的拖曳特定力以进行多旋翼风估计
	void fuseDrag(const dragSample &drag_sample);
#endif // CONFIG_EKF2_DRAG_FUSION
	// 重置速度到指定值
	void resetVelocityTo(const Vector3f &vel, const Vector3f &new_vel_var);

	// 重置水平速度到新的水平速度和对应的方差
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var);
	// 重载函数，允许使用单一的方差值来重置水平速度
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var) { resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

	// 将水平速度重置为零
	void resetHorizontalVelocityToZero();

	// 重置垂直速度到新的垂直速度和对应的方差
	void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);

	// 重置水平位置到最后已知的位置
	void resetHorizontalPositionToLastKnown();

	// 重置水平位置到新的经纬度和对应的方差
	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				       const Vector2f &new_horz_pos_var);
	// 重载函数，允许使用单一的方差值来重置水平位置
	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude, const float pos_var = NAN) { resetHorizontalPositionTo(new_latitude, new_longitude, Vector2f(pos_var, pos_var)); }
	// 重置水平位置到新的位置和对应的方差
	void resetHorizontalPositionTo(const Vector2f &new_pos, const Vector2f &new_horz_pos_var);

	// 获取当前的局部水平位置
	Vector2f getLocalHorizontalPosition() const;

	// 计算新的经纬度与当前水平位置之间的增量
	Vector2f computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const;
	// 更新水平位置重置状态
	void updateHorizontalPositionResetStatus(const Vector2f &delta);

	// 重置风速到指定值和对应的方差
	void resetWindTo(const Vector2f &wind, const Vector2f &wind_var);

	// 检查是否需要重置高度
	bool isHeightResetRequired() const;

	// 重置高度到新的高度值和对应的方差
	void resetAltitudeTo(float new_altitude, float new_vert_pos_var = NAN);
	// 更新垂直位置重置状态
	void updateVerticalPositionResetStatus(const float delta_z);

	// 将垂直速度重置为零
	void resetVerticalVelocityToZero();

	// 更新垂直位置的辅助源状态
	void updateVerticalPositionAidStatus(estimator_aid_source1d_s &aid_src, const uint64_t &time_us,
					     const float observation, const float observation_variance, const float innovation_gate = 1.f) const;

	// 融合水平和垂直位置
	bool fuseHorizontalPosition(estimator_aid_source2d_s &pos_aid_src);
	bool fuseVerticalPosition(estimator_aid_source1d_s &hgt_aid_src);

	// 融合2D和3D速度
	bool fuseHorizontalVelocity(estimator_aid_source2d_s &vel_aid_src);
	bool fuseVelocity(estimator_aid_source3d_s &vel_aid_src);

#if defined(CONFIG_EKF2_TERRAIN)
	// 初始化地形数据
	void initTerrain();
	// 获取当前地形的垂直位置，如果地形估计有效则返回估计值，否则返回最后已知的地面高度
	float getTerrainVPos() const { return isTerrainEstimateValid() ? _state.terrain : _last_on_ground_posD; }
	// 控制地形虚假融合
	void controlTerrainFakeFusion();

	// 更新地形有效性
	void updateTerrainValidity();
	// 更新地形重置状态
	void updateTerrainResetStatus(const float delta_z);

# if defined(CONFIG_EKF2_RANGE_FINDER)
	// 使用来自测距仪的地面高度测量更新地形的垂直位置估计
	bool fuseHaglRng(estimator_aid_source1d_s &aid_src, bool update_height, bool update_terrain);
	// 更新测距仪的高度估计
	void updateRangeHagl(estimator_aid_source1d_s &aid_src);
	// 将地形重置为测距仪的值
	void resetTerrainToRng(estimator_aid_source1d_s &aid_src);
	// 获取测距仪的方差
	float getRngVar() const;
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 重置地形到光流测量
	void resetTerrainToFlow();
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 控制测距仪高度的融合
	void controlRangeHaglFusion(const imuSample &imu_delayed);
	// 检查条件测距辅助是否适合
	bool isConditionalRangeAidSuitable();
	// 停止测距仪高度融合
	void stopRngHgtFusion();
	// 停止测距仪地形融合
	void stopRngTerrFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 控制光流观测的融合
	void controlOpticalFlowFusion(const imuSample &imu_delayed);
	// 重置光流融合
	void resetFlowFusion(const flowSample &flow_sample);
	// 停止光流融合
	void stopFlowFusion();

	// 更新地面运动以进行光流检查
	void updateOnGroundMotionForOpticalFlowChecks();
	// 重置地面运动以进行光流检查
	void resetOnGroundMotionForOpticalFlowChecks();

	// 计算光流传感器的测量方差 (rad/sec)^2
	float calcOptFlowMeasVar(const flowSample &flow_sample) const;

	// 计算光流的机体角速度补偿
	void calcOptFlowBodyRateComp(const flowSample &flow_sample);

	// 预测光流范围
	float predictFlowRange() const;
	// 预测光流
	Vector2f predictFlow(const Vector3f &flow_gyro) const;

	// 融合光流视线速率测量
	bool fuseOptFlow(VectorState &H, bool update_terrain);

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 返回用于对齐和融合处理的磁偏角（弧度）
	float getMagDeclination();
#endif // CONFIG_EKF2_MAGNETOMETER

	void clearInhibitedStateKalmanGains(VectorState &K) const;

	// 限制协方差矩阵的对角线
	void constrainStateVariances();

	// 限制特定状态的方差范围
	void constrainStateVar(const IdxDof &state, float min, float max);
	// 限制特定状态的方差范围并限制比率
	void constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio = 1.e6f);

	// 通用函数，执行给定卡尔曼增益K和标量创新值的融合步骤
	void fuse(const VectorState &K, float innovation);

	// 根据给定的纬度计算地球自转向量
	Vector3f calcEarthRateNED(float lat_rad) const;

	// 控制滤波器融合模式
	void controlFusionModes(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 控制外部视觉观测的融合
	void controlExternalVisionFusion(const imuSample &imu_sample);
	void updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset);
	void controlEvHeightFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				   const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				   estimator_aid_source1d_s &aid_src);
	void controlEvPosFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				estimator_aid_source2d_s &aid_src);
	void controlEvVelFusion(ExternalVisionVel &ev, const bool common_starting_conditions_passing, const bool ev_reset,
				const bool quality_sufficient, estimator_aid_source3d_s &aid_src);
	void controlEvYawFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				estimator_aid_source1d_s &aid_src);
	void fuseLocalFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				    const Vector3f &measurement_var, const float &innovation_gate);
	void fuseBodyFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				   const Vector3f &measurement_var, const float &innovation_gate);

	void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src);
	void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient,
			       bool reset, estimator_aid_source2d_s &aid_src);

	void stopEvPosFusion();
	void stopEvHgtFusion();
	void stopEvVelFusion();
	void stopEvYawFusion();
	bool fuseEvVelocity(estimator_aid_source3d_s &aid_src, const extVisionSample &ev_sample);
	void fuseBodyVelocity(estimator_aid_source1d_s &aid_src, float &innov_var, VectorState &H)
	{
		VectorState Kfusion = P * H / innov_var;
		measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
		aid_src.fused = true;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// 控制GPS观测的融合
	void controlGpsFusion(const imuSample &imu_delayed);
	void stopGpsFusion();
	void updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src);
	void updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src);
	void controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel);
	bool tryYawEmergencyReset();
	void resetVelocityToGnss(estimator_aid_source3d_s &aid_src);
	void resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src);
	bool shouldResetGpsFusion() const;

	/*
	 * 返回GPS解的质量是否足够。
	 * 检查通过EKF2_GPS_CHECK位掩码参数激活
	 * 检查通过EKF2_REQ_*参数调整
	*/
	bool runGnssChecks(const gnssSample &gps);

	void controlGnssHeightFusion(const gnssSample &gps_sample);
	void stopGpsHgtFusion();

	void resetGpsDriftCheckFilters();

# if defined(CONFIG_EKF2_GNSS_YAW)
	void controlGnssYawFusion(const gnssSample &gps_sample);
	void stopGnssYawFusion();

	// 融合从双天线GPS单元获得的航向角
	void fuseGnssYaw(float antenna_yaw_offset);

	// 使用从双天线GPS单元获得的航向角重置四元数状态
	// 如果重置成功则返回true
	bool resetYawToGnss(float gnss_yaw, float gnss_yaw_offset);

	void updateGnssYaw(const gnssSample &gps_sample);

# endif // CONFIG_EKF2_GNSS_YAW

	// 控制EKF-GSF航向估计器的使用
	bool isYawFailure() const;

	// 将主导航EKF航向重置为EKF-GSF航向估计器的值
	// 如果重置成功则返回true
	bool resetYawToEKFGSF();

	// 航向估计器实例
	EKFGSF_yaw _yawEstimator{};

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 控制磁力计观测的融合
	void controlMagFusion(const imuSample &imu_sample);

	// 检查是否需要重置高度与航向的关系
	bool checkHaglYawResetReq() const;

	// 重置磁头方向
	void resetMagHeading(const Vector3f &mag);
	// 重置磁力计状态
	void resetMagStates(const Vector3f &mag, bool reset_heading = true);
	// 检查是否需要重置高度与航向的关系
	bool haglYawResetReq();

	// 检查磁头方向的一致性
	void checkMagHeadingConsistency(const magSample &mag_sample);

	// 检查磁场是否符合预期
	bool checkMagField(const Vector3f &mag);
	// 检查测量值是否与预期值匹配
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);

	// 停止磁力计融合
	void stopMagFusion();

	// 计算给定3D磁力计传感器测量的合成Z分量值
	float calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted);

#endif // CONFIG_EKF2_MAGNETOMETER

	// 控制虚假位置观测的融合以限制漂移
	void controlFakePosFusion();

	// 控制虚假高度融合
	void controlFakeHgtFusion();
	// 重置虚假高度融合
	void resetFakeHgtFusion();
	// 重置高度到最后已知值
	void resetHeightToLastKnown();
	// 停止虚假高度融合
	void stopFakeHgtFusion();

	// 控制零创新航向更新
	void controlZeroInnovationHeadingUpdate();

#if defined(CONFIG_EKF2_AUXVEL)
	// 控制辅助速度观测的融合
	void controlAuxVelFusion(const imuSample &imu_sample);
	// 停止辅助速度融合
	void stopAuxVelFusion();
#endif // CONFIG_EKF2_AUXVEL

	// 检查垂直加速度的健康状态
	void checkVerticalAccelerationHealth(const imuSample &imu_delayed);
	// 估计惯性导航坠落的可能性
	Likelihood estimateInertialNavFallingLikelihood() const;

	// 控制组合高度融合模式（用于在气压高度和测距高度之间切换）
	void controlHeightFusion(const imuSample &imu_delayed);
	// 检查高度传感器参考的后备方案
	void checkHeightSensorRefFallback();

#if defined(CONFIG_EKF2_BAROMETER)
	// 控制气压高度融合
	void controlBaroHeightFusion(const imuSample &imu_sample);
	// 停止气压高度融合
	void stopBaroHgtFusion();

	// 更新地面效应
	void updateGroundEffect();

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	// 根据动态压力补偿气压高度
	float compensateBaroForDynamicPressure(const imuSample &imu_sample, float baro_alt_uncompensated) const;
# endif // CONFIG_EKF2_BARO_COMPENSATION

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力融合：启用/禁用重力融合
	void controlGravityFusion(const imuSample &imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

	// 重置四元数协方差
	void resetQuatCov(const float yaw_noise = NAN);
	// 重置四元数协方差
	void resetQuatCov(const Vector3f &rot_var_ned);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 重置磁场地球协方差
	void resetMagEarthCov();
	// 重置磁偏差协方差
	void resetMagBiasCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// 重置风协方差
	void resetWindCov();
	// 将风重置为零
	void resetWindToZero();
#endif // CONFIG_EKF2_WIND

	// 重置陀螺仪Z轴偏差协方差
	void resetGyroBiasZCov();

	// 检查传感器时间戳是否超时
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < _time_delayed_us);
	}

	// 检查传感器时间戳是否在接受范围内
	bool isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_delayed_us);
	}

	// 检查最新样本的时间戳是否在接受范围内
	bool isNewestSampleRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_latest_us);
	}

	// 重置虚假位置融合
	void resetFakePosFusion();
	// 运行虚假位置状态机
	bool runFakePosStateMachine(bool enable_condition_passing, bool status_flag, estimator_aid_source2d_s &aid_src);

	// 重置四元数状态和协方差到新的航向值，保持滚转和俯仰不变
	// yaw : 欧拉航向角（弧度）
	// yaw_variance : 航向误差方差（弧度²）
	void resetQuatStateYaw(float yaw, float yaw_variance);

	// 高度传感器引用，默认为未知
	HeightSensor _height_sensor_ref{HeightSensor::UNKNOWN};
	// 位置传感器引用，默认为GNSS
	PositionSensor _position_sensor_ref{PositionSensor::GNSS};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 外部视觉高度偏差估计器
	HeightBiasEstimator _ev_hgt_b_est {HeightSensor::EV, _height_sensor_ref};
	// 外部视觉位置偏差估计器
	PositionBiasEstimator _ev_pos_b_est{PositionSensor::EV, _position_sensor_ref};
	// 四元数误差低通滤波时间常数
	static constexpr float _kQuatErrorLpfTimeConstant = 10.f;
	// 外部视觉四元数误差滤波器
	AlphaFilter<Quatf> _ev_q_error_filt{_dt_ekf_avg, _kQuatErrorLpfTimeConstant};
	// 外部视觉四元数误差初始化标志
	bool _ev_q_error_initialized{false};
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 状态重置为辅助源，保持观测并适当更新所有其他字段（零创新等）
	void resetAidSourceStatusZeroInnovation(estimator_aid_source1d_s &status) const;

	// 用于填充和过滤估计器辅助源结构以进行日志记录的助手
	void updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				   const float &observation, const float &observation_variance,
				   const float &innovation, const float &innovation_variance,
				   float innovation_gate = 1.f) const;

	// 状态重置为辅助源，保持观测并适当更新所有其他字段（零创新等）
	template <typename T>
	void resetAidSourceStatusZeroInnovation(T &status) const
	{
		status.time_last_fuse = _time_delayed_us;

		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {
			status.innovation[i] = 0.f; // 初始化创新值为零
			status.innovation_filtered[i] = 0.f; // 初始化过滤后的创新值为零
			status.innovation_variance[i] = status.observation_variance[i]; // 设置创新方差为观测方差

			status.test_ratio[i] = 0.f; // 初始化测试比率为零
			status.test_ratio_filtered[i] = 0.f; // 初始化过滤后的测试比率为零
		}

		status.innovation_rejected = false; // 设置创新拒绝标志为假
		status.fused = true; // 设置融合标志为真
	}
	// 用于填充和过滤估计器辅助源结构以进行日志记录的助手函数
	template <typename T, typename S, typename D>
	void updateAidSourceStatus(T &status, const uint64_t &timestamp_sample,
				   const D &observation, const S &observation_variance,
				   const S &innovation, const S &innovation_variance,
				   float innovation_gate = 1.f) const
	{
		bool innovation_rejected = false; // 初始化创新拒绝标志为假

		// 计算当前时间戳与上次时间戳之间的时间差（秒），并限制在0.001到1秒之间
		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f; // 低通滤波器的时间常数
		// 计算滤波器的平滑因子alpha，限制在0到1之间
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		// 遍历状态观测数组
		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {

			// 计算测试比率，表示创新与创新方差的比值
			const float test_ratio = sq(innovation(i)) / (sq(innovation_gate) * innovation_variance(i));

			// 检查时间戳是否有效
			if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

				// 处理过滤后的测试比率
				if (PX4_ISFINITE(status.test_ratio_filtered[i])) {
					// 更新过滤后的测试比率
					status.test_ratio_filtered[i] += alpha * (matrix::sign(innovation(i)) * test_ratio - status.test_ratio_filtered[i]);

				} else {
					// 否则，初始化过滤后的测试比率
					status.test_ratio_filtered[i] = test_ratio;
				}

				// 处理过滤后的创新值
				if (PX4_ISFINITE(status.innovation_filtered[i])) {
					// 更新过滤后的创新值
					status.innovation_filtered[i] += alpha * (innovation(i) - status.innovation_filtered[i]);

				} else {
					// 否则，初始化过滤后的创新值
					status.innovation_filtered[i] = innovation(i);
				}

				// 限制过滤值的极端情况
				static constexpr float kNormalizedInnovationLimit = 2.f; // 归一化创新限制
				static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit); // 测试比率限制

				if (test_ratio > kTestRatioLimit) {
					// 限制过滤后的测试比率在指定范围内
					status.test_ratio_filtered[i] = math::constrain(status.test_ratio_filtered[i], -kTestRatioLimit, kTestRatioLimit);

					// 计算创新限制
					const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance(i));
					// 限制过滤后的创新值在指定范围内
					status.innovation_filtered[i] = math::constrain(status.innovation_filtered[i], -innov_limit, innov_limit);
				}

			} else {
				// 如果时间戳无效，重置过滤后的测试比率和创新值
				status.test_ratio_filtered[i] = test_ratio;
				status.innovation_filtered[i] = innovation(i);
			}

			// 更新当前测试比率
			status.test_ratio[i] = test_ratio;

			// 更新观测值和观测方差
			status.observation[i] = static_cast<double>(observation(i));
			status.observation_variance[i] = observation_variance(i);

			// 更新创新值和创新方差
			status.innovation[i] = innovation(i);
			status.innovation_variance[i] = innovation_variance(i);

			// 检查测试比率和创新值的有效性
			if ((test_ratio > 1.f)
			    || !PX4_ISFINITE(test_ratio)
			    || !PX4_ISFINITE(status.innovation[i])
			    || !PX4_ISFINITE(status.innovation_variance[i])
			   ) {
				innovation_rejected = true; // 如果有任何创新被拒绝，设置创新拒绝标志为真
			}
		}

		// 更新状态的时间戳
		status.timestamp_sample = timestamp_sample;

		// 如果任何创新被拒绝，则整体创新被拒绝
		status.innovation_rejected = innovation_rejected;

		// 重置融合标志
		status.fused = false;
	}

	ZeroGyroUpdate _zero_gyro_update{}; // 零陀螺仪更新对象
	ZeroVelocityUpdate _zero_velocity_update{}; // 零速度更新对象

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	AuxGlobalPosition _aux_global_position {}; // 辅助全局位置对象
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
};

#endif // !EKF_EKF_H
