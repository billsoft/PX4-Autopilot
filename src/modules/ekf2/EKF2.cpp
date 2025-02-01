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

#include <px4_platform_common/events.h>
#include "EKF2.hpp"

using namespace time_literals;
using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr float kDefaultExternalPosAccuracy = 50.0f; // [m]
static constexpr float kMaxDelaySecondsExternalPosMeasurement = 15.0f; // [s]

pthread_mutex_t ekf2_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<EKF2 *> _objects[EKF2_MAX_INSTANCES] {};
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
static px4::atomic<EKF2Selector *> _ekf2_selector {nullptr};
#endif // CONFIG_EKF2_MULTI_INSTANCE

// EKF2类的构造函数
EKF2::EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode):
	ModuleParams(nullptr), // 初始化模块参数
	ScheduledWorkItem(MODULE_NAME, config), // 调度工作项，传入模块名称和配置
	_replay_mode(replay_mode && !multi_mode), // 设置重放模式
	_multi_mode(multi_mode), // 设置多模式
	_instance(multi_mode ? -1 : 0), // 根据模式设置实例编号
	_attitude_pub(multi_mode ? ORB_ID(estimator_attitude) : ORB_ID(vehicle_attitude)), // 初始化姿态发布器
	_local_position_pub(multi_mode ? ORB_ID(estimator_local_position) : ORB_ID(vehicle_local_position)), // 初始化本地位置发布器
	_global_position_pub(multi_mode ? ORB_ID(estimator_global_position) : ORB_ID(vehicle_global_position)), // 初始化全球位置发布器
	_odometry_pub(multi_mode ? ORB_ID(estimator_odometry) : ORB_ID(vehicle_odometry)), // 初始化里程计发布器
#if defined(CONFIG_EKF2_WIND)
	_wind_pub(multi_mode ? ORB_ID(estimator_wind) : ORB_ID(wind)), // 初始化风速发布器
#endif // CONFIG_EKF2_WIND
	_params(_ekf.getParamHandle()), // 获取参数句柄
	_param_ekf2_predict_us(_params->filter_update_interval_us), // 预测更新间隔，单位为微秒
	_param_ekf2_delay_max(_params->delay_max_ms), // 最大延迟，单位为毫秒
	_param_ekf2_imu_ctrl(_params->imu_ctrl), // IMU控制参数
	_param_ekf2_vel_lim(_params->velocity_limit), // 速度限制参数
#if defined(CONFIG_EKF2_AUXVEL)
	_param_ekf2_avel_delay(_params->auxvel_delay_ms), // 辅助速度延迟，单位为毫秒
#endif // CONFIG_EKF2_AUXVEL
	_param_ekf2_gyr_noise(_params->gyro_noise), // 陀螺仪噪声参数
	_param_ekf2_acc_noise(_params->accel_noise), // 加速度计噪声参数
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise), // 陀螺仪偏置噪声参数
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise), // 加速度计偏置噪声参数
#if defined(CONFIG_EKF2_WIND)
	_param_ekf2_wind_nsd(_params->wind_vel_nsd), // 风速噪声参数
#endif // CONFIG_EKF2_WIND
	_param_ekf2_noaid_noise(_params->pos_noaid_noise), // 无辅助噪声参数
#if defined(CONFIG_EKF2_GNSS)
	_param_ekf2_gps_ctrl(_params->gnss_ctrl), // GNSS控制参数
	_param_ekf2_gps_delay(_params->gps_delay_ms), // GPS延迟，单位为毫秒
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)), // GPS位置X坐标
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)), // GPS位置Y坐标
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)), // GPS位置Z坐标
	_param_ekf2_gps_v_noise(_params->gps_vel_noise), // GPS速度噪声参数
	_param_ekf2_gps_p_noise(_params->gps_pos_noise), // GPS位置噪声参数
	_param_ekf2_gps_p_gate(_params->gps_pos_innov_gate), // GPS位置创新门限
	_param_ekf2_gps_v_gate(_params->gps_vel_innov_gate), // GPS速度创新门限
	_param_ekf2_gps_check(_params->gps_check_mask), // GPS检查掩码
	_param_ekf2_req_eph(_params->req_hacc), // GPS水平精度请求
	_param_ekf2_req_epv(_params->req_vacc), // GPS垂直精度请求
	_param_ekf2_req_sacc(_params->req_sacc), // GPS速度精度请求
	_param_ekf2_req_nsats(_params->req_nsats), // GPS卫星数量请求
	_param_ekf2_req_pdop(_params->req_pdop), // GPS位置精度请求
	_param_ekf2_req_hdrift(_params->req_hdrift), // GPS水平漂移请求
	_param_ekf2_req_vdrift(_params->req_vdrift), // GPS垂直漂移请求
	_param_ekf2_gsf_tas_default(_params->EKFGSF_tas_default), // 默认的GSF真空速度
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_BAROMETER)
	_param_ekf2_baro_ctrl(_params->baro_ctrl), // 气压计控制参数
	_param_ekf2_baro_delay(_params->baro_delay_ms), // 气压计延迟，单位为毫秒
	_param_ekf2_baro_noise(_params->baro_noise), // 气压计噪声参数
	_param_ekf2_baro_gate(_params->baro_innov_gate), // 气压计创新门限
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone), // 地面效应死区
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt), // 地面效应最大高度
# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	_param_ekf2_aspd_max(_params->max_correction_airspeed), // 最大修正空速
	_param_ekf2_pcoef_xp(_params->static_pressure_coef_xp), // 静压系数XP
	_param_ekf2_pcoef_xn(_params->static_pressure_coef_xn), // 静压系数XN
	_param_ekf2_pcoef_yp(_params->static_pressure_coef_yp), // 静压系数YP
	_param_ekf2_pcoef_yn(_params->static_pressure_coef_yn), // 静压系数YN
	_param_ekf2_pcoef_z(_params->static_pressure_coef_z), // 静压系数Z
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_AIRSPEED)
	_param_ekf2_asp_delay(_params->airspeed_delay_ms), // 空速延迟，单位为毫秒
	_param_ekf2_tas_gate(_params->tas_innov_gate), // 真空速度创新门限
	_param_ekf2_eas_noise(_params->eas_noise), // 真实空速噪声参数
	_param_ekf2_arsp_thr(_params->arsp_thr), // 空速阈值
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	_param_ekf2_beta_gate(_params->beta_innov_gate), // 侧滑角创新门限
	_param_ekf2_beta_noise(_params->beta_noise), // 侧滑角噪声参数
	_param_ekf2_fuse_beta(_params->beta_fusion_enabled), // 侧滑角融合使能
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_MAGNETOMETER)
	_param_ekf2_mag_delay(_params->mag_delay_ms), // 磁力计延迟，单位为毫秒
	_param_ekf2_mag_e_noise(_params->mage_p_noise), // 磁力计电噪声参数
	_param_ekf2_mag_b_noise(_params->magb_p_noise), // 磁力计偏置噪声参数
	_param_ekf2_head_noise(_params->mag_heading_noise), // 磁力计航向噪声参数
	_param_ekf2_mag_noise(_params->mag_noise), // 磁力计噪声参数
	_param_ekf2_mag_decl(_params->mag_declination_deg), // 磁偏角度
	_param_ekf2_hdg_gate(_params->heading_innov_gate), // 航向创新门限
	_param_ekf2_mag_gate(_params->mag_innov_gate), // 磁力计创新门限
	_param_ekf2_decl_type(_params->mag_declination_source), // 磁偏角来源类型
	_param_ekf2_mag_type(_params->mag_fusion_type), // 磁力计融合类型
	_param_ekf2_mag_acclim(_params->mag_acc_gate), // 磁力计加速度门限
	_param_ekf2_mag_check(_params->mag_check), // 磁力计检查使能
	_param_ekf2_mag_chk_str(_params->mag_check_strength_tolerance_gs), // 磁力计检查强度容忍度
	_param_ekf2_mag_chk_inc(_params->mag_check_inclination_tolerance_deg), // 磁力计检查倾斜容忍度
	_param_ekf2_synthetic_mag_z(_params->synthesize_mag_z), // 合成磁场Z轴
#endif // CONFIG_EKF2_MAGNETOMETER
	_param_ekf2_hgt_ref(_params->height_sensor_ref), // 高度传感器参考
	_param_ekf2_noaid_tout(_params->valid_timeout_max), // 无辅助超时最大值
#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_min_rng(_params->rng_gnd_clearance), // 最小地面清晰度
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
	_param_ekf2_terr_noise(_params->terrain_p_noise), // 地形噪声参数
	_param_ekf2_terr_grad(_params->terrain_gradient), // 地形梯度
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
	_param_ekf2_rng_ctrl(_params->rng_ctrl), // 距离传感器控制参数
	_param_ekf2_rng_delay(_params->range_delay_ms), // 距离传感器延迟，单位为毫秒
	_param_ekf2_rng_noise(_params->range_noise), // 距离传感器噪声参数
	_param_ekf2_rng_sfe(_params->range_noise_scaler), // 距离传感器噪声缩放因子
	_param_ekf2_rng_gate(_params->range_innov_gate), // 距离传感器创新门限
	_param_ekf2_rng_pitch(_params->rng_sens_pitch), // 距离传感器俯仰角
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid), // 距离辅助的最大速度
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid), // 距离辅助的最大高度
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate), // 距离辅助创新门限
	_param_ekf2_rng_qlty_t(_params->range_valid_quality_s), // 距离有效质量时间
	_param_ekf2_rng_k_gate(_params->range_kin_consistency_gate), // 距离运动一致性门限
	_param_ekf2_rng_fog(_params->rng_fog), // 距离传感器雾霾参数
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)), // 距离传感器X坐标
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)), // 距离传感器Y坐标
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)), // 距离传感器Z坐标
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_param_ekf2_ev_delay(_params->ev_delay_ms), // 外部视觉延迟，单位为毫秒
	_param_ekf2_ev_ctrl(_params->ev_ctrl), // 外部视觉控制参数
	_param_ekf2_ev_qmin(_params->ev_quality_minimum), // 外部视觉最小质量
	_param_ekf2_evp_noise(_params->ev_pos_noise), // 外部视觉位置噪声参数
	_param_ekf2_evv_noise(_params->ev_vel_noise), // 外部视觉速度噪声参数
	_param_ekf2_eva_noise(_params->ev_att_noise), // 外部视觉姿态噪声参数
	_param_ekf2_evv_gate(_params->ev_vel_innov_gate), // 外部视觉速度创新门限
	_param_ekf2_evp_gate(_params->ev_pos_innov_gate), // 外部视觉位置创新门限
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)), // 外部视觉位置X坐标
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)), // 外部视觉位置Y坐标
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)), // 外部视觉位置Z坐标
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_param_ekf2_of_ctrl(_params->flow_ctrl), // 光流控制参数
	_param_ekf2_of_gyr_src(_params->flow_gyro_src), // 光流陀螺仪源
	_param_ekf2_of_delay(_params->flow_delay_ms), // 光流延迟，单位为毫秒
	_param_ekf2_of_n_min(_params->flow_noise), // 光流最小噪声
	_param_ekf2_of_n_max(_params->flow_noise_qual_min), // 光流最小质量噪声
	_param_ekf2_of_qmin(_params->flow_qual_min), // 光流最小质量
	_param_ekf2_of_qmin_gnd(_params->flow_qual_min_gnd), // 地面光流最小质量
	_param_ekf2_of_gate(_params->flow_innov_gate), // 光流创新门限
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)), // 光流位置X坐标
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)), // 光流位置Y坐标
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)), // 光流位置Z坐标
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_DRAG_FUSION)
	_param_ekf2_drag_ctrl(_params->drag_ctrl), // 拖曳融合控制参数
	_param_ekf2_drag_noise(_params->drag_noise), // 拖曳噪声参数
	_param_ekf2_bcoef_x(_params->bcoef_x), // 拖曳系数X
	_param_ekf2_bcoef_y(_params->bcoef_y), // 拖曳系数Y
	_param_ekf2_mcoef(_params->mcoef), // 拖曳质量系数
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	_param_ekf2_grav_noise(_params->gravity_noise), // 重力融合噪声参数
#endif // CONFIG_EKF2_GRAVITY_FUSION
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)), // IMU位置X坐标
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)), // IMU位置Y坐标
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)), // IMU位置Z坐标
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias), // 陀螺仪偏置初始化开关
	_param_ekf2_abias_init(_params->switch_on_accel_bias), // 加速度计偏置初始化开关
	_param_ekf2_angerr_init(_params->initial_tilt_err), // 初始倾斜误差
	_param_ekf2_abl_lim(_params->acc_bias_lim), // 加速度偏置限制
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim), // 加速度偏置学习加速度限制
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim), // 加速度偏置学习陀螺仪限制
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc), // 加速度偏置学习时间常数
	_param_ekf2_gyr_b_lim(_params->gyro_bias_lim) // 陀螺仪偏置限制
{
	AdvertiseTopics(); // 广播主题
}

// EKF2类的析构函数
EKF2::~EKF2()
{
	perf_free(_ekf_update_perf); // 释放EKF更新性能计数器
	perf_free(_msg_missed_imu_perf); // 释放IMU消息丢失性能计数器
}

// 广播主题的函数
void EKF2::AdvertiseTopics()
{
	// 立即广播预期的最小主题集以进行日志记录
	_attitude_pub.advertise(); // 广播姿态主题
	_local_position_pub.advertise(); // 广播本地位置主题
	_estimator_event_flags_pub.advertise(); // 广播估计器事件标志主题
	_estimator_sensor_bias_pub.advertise(); // 广播估计器传感器偏置主题
	_estimator_status_pub.advertise(); // 广播估计器状态主题
	_estimator_status_flags_pub.advertise(); // 广播估计器状态标志主题

	if (_multi_mode) {
		// 仅在多模式下强制广播这些，以确保一致的uORB实例编号
		_global_position_pub.advertise(); // 广播全球位置主题
		_odometry_pub.advertise(); // 广播里程计主题

#if defined(CONFIG_EKF2_WIND)
		_wind_pub.advertise(); // 广播风速主题
#endif // CONFIG_EKF2_WIND
	}

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_ctrl.get()) {
		_estimator_gps_status_pub.advertise(); // 广播GPS状态主题
		_yaw_est_pub.advertise(); // 广播航向估计主题
	}

#endif // CONFIG_EKF2_GNSS

	// 详细日志记录
	if (_param_ekf2_log_verbose.get()) {
		_estimator_innovation_test_ratios_pub.advertise(); // 广播创新测试比率主题
		_estimator_innovation_variances_pub.advertise(); // 广播创新方差主题
		_estimator_innovations_pub.advertise(); // 广播创新主题
		_estimator_states_pub.advertise(); // 广播状态主题

#if defined(CONFIG_EKF2_AIRSPEED)

		if (_param_ekf2_arsp_thr.get() > 0.f) {
			_estimator_aid_src_airspeed_pub.advertise(); // 广播空速辅助源主题
		}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_BAROMETER)

		if (_param_ekf2_baro_ctrl.get()) {
			_estimator_aid_src_baro_hgt_pub.advertise(); // 广播气压计高度辅助源主题
			_estimator_baro_bias_pub.advertise(); // 广播气压计偏置主题
		}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION) // 检查是否启用拖曳融合功能

		if (_param_ekf2_drag_ctrl.get()) { // 获取拖曳控制参数
			_estimator_aid_src_drag_pub.advertise(); // 广播拖曳辅助源主题
		}

#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION) // 检查是否启用外部视觉功能

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VPOS)) { // 检查是否启用外部视觉高度控制
			_estimator_aid_src_ev_hgt_pub.advertise(); // 广播外部视觉高度辅助源主题
			_estimator_ev_pos_bias_pub.advertise(); // 广播外部视觉位置偏置主题
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::HPOS)) { // 检查是否启用外部视觉位置控制
			_estimator_aid_src_ev_pos_pub.advertise(); // 广播外部视觉位置辅助源主题
			_estimator_ev_pos_bias_pub.advertise(); // 广播外部视觉位置偏置主题
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::VEL)) { // 检查是否启用外部视觉速度控制
			_estimator_aid_src_ev_vel_pub.advertise(); // 广播外部视觉速度辅助源主题
		}

		if (_param_ekf2_ev_ctrl.get() & static_cast<int32_t>(EvCtrl::YAW)) { // 检查是否启用外部视觉航向控制
			_estimator_aid_src_ev_yaw_pub.advertise(); // 广播外部视觉航向辅助源主题
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS) // 检查是否启用GNSS功能

		if (_param_ekf2_gps_ctrl.get()) { // 获取GNSS控制参数
			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VPOS)) { // 检查是否启用GNSS高度控制
				_estimator_aid_src_gnss_hgt_pub.advertise(); // 广播GNSS高度辅助源主题
				_estimator_gnss_hgt_bias_pub.advertise(); // 广播GNSS高度偏置主题
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::HPOS)) { // 检查是否启用GNSS位置控制
				_estimator_aid_src_gnss_pos_pub.advertise(); // 广播GNSS位置辅助源主题
			}

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::VEL)) { // 检查是否启用GNSS速度控制
				_estimator_aid_src_gnss_vel_pub.advertise(); // 广播GNSS速度辅助源主题
			}

# if defined(CONFIG_EKF2_GNSS_YAW) // 检查是否启用GNSS航向控制

			if (_param_ekf2_gps_ctrl.get() & static_cast<int32_t>(GnssCtrl::YAW)) { // 检查是否启用GNSS航向控制
				_estimator_aid_src_gnss_yaw_pub.advertise(); // 广播GNSS航向辅助源主题
			}

# endif // CONFIG_EKF2_GNSS_YAW
		}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION) // 检查是否启用重力融合功能

		if (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GravityVector)) { // 检查是否启用重力向量控制
			_estimator_aid_src_gravity_pub.advertise(); // 广播重力辅助源主题
		}

#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_MAGNETOMETER) // 检查是否启用磁力计功能

		if (_param_ekf2_mag_type.get() != MagFuseType::NONE) { // 检查磁力计类型是否有效
			_estimator_aid_src_mag_pub.advertise(); // 广播磁力计辅助源主题
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW) // 检查是否启用光流功能

		if (_param_ekf2_of_ctrl.get()) { // 获取光流控制参数
			_estimator_optical_flow_vel_pub.advertise(); // 广播光流速度辅助源主题
			_estimator_aid_src_optical_flow_pub.advertise(); // 广播光流辅助源主题
		}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER) // 检查是否启用测距仪功能

		// RNG 广播
		if (_param_ekf2_rng_ctrl.get()) { // 获取测距仪控制参数
			_estimator_aid_src_rng_hgt_pub.advertise(); // 广播测距仪高度辅助源主题
		}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_SIDESLIP) // 检查是否启用侧滑功能

		if (_param_ekf2_fuse_beta.get()) { // 获取侧滑融合参数
			_estimator_aid_src_sideslip_pub.advertise(); // 广播侧滑辅助源主题
		}

#endif // CONFIG_EKF2_SIDESLIP

	} // end verbose logging
}
#if defined(CONFIG_EKF2_MULTI_INSTANCE) // 检查是否启用多实例功能
bool EKF2::multi_init(int imu, int mag) // 初始化多实例，参数为IMU和磁力计的实例ID
{
	bool changed_instance = _vehicle_imu_sub.ChangeInstance(imu); // 尝试更改IMU实例

#if defined(CONFIG_EKF2_MAGNETOMETER) // 检查是否启用磁力计功能

	if (!_magnetometer_sub.ChangeInstance(mag)) { // 尝试更改磁力计实例
		changed_instance = false; // 如果更改失败，标记实例未更改
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	const int status_instance = _estimator_states_pub.get_instance(); // 获取当前状态实例

	// 检查状态实例是否有效，并且IMU和其他发布主题的实例是否一致
	if ((status_instance >= 0) && changed_instance
	    && (_attitude_pub.get_instance() == status_instance) // 姿态发布主题实例
	    && (_local_position_pub.get_instance() == status_instance) // 本地位置发布主题实例
	    && (_global_position_pub.get_instance() == status_instance)) { // 全球位置发布主题实例

		_instance = status_instance; // 更新当前实例为状态实例

		ScheduleNow(); // 立即调度更新
		return true; // 返回成功
	}

	PX4_ERR("publication instance problem: %d att: %d lpos: %d gpos: %d", status_instance,
		_attitude_pub.get_instance(), _local_position_pub.get_instance(), _global_position_pub.get_instance()); // 输出错误信息

	return false; // 返回失败
}
#endif // CONFIG_EKF2_MULTI_INSTANCE

int EKF2::print_status(bool verbose) // 打印状态信息，参数verbose控制详细程度
{
	PX4_INFO_RAW("ekf2:%d EKF dt: %.4fs, attitude: %d, local position: %d, global position: %d\n",
		     _instance, (double)_ekf.get_dt_ekf_avg(), _ekf.attitude_valid(), // 打印实例ID、EKF平均时间间隔、姿态有效性
		     _ekf.isLocalHorizontalPositionValid(), _ekf.isGlobalHorizontalPositionValid()); // 打印本地和全球位置有效性

	perf_print_counter(_ekf_update_perf); // 打印EKF更新性能计数
	perf_print_counter(_msg_missed_imu_perf); // 打印丢失IMU消息的性能计数

	if (verbose) { // 如果需要详细信息
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
		_ekf.print_status(); // 打印EKF详细状态
#endif // CONFIG_EKF2_VERBOSE_STATUS
	}

	return 0; // 返回成功
}

void EKF2::Run() // 运行EKF2主循环
{
	if (should_exit()) { // 检查是否需要退出
		_sensor_combined_sub.unregisterCallback(); // 注销传感器组合回调
		_vehicle_imu_sub.unregisterCallback(); // 注销车辆IMU回调

		return; // 退出
	}

	// 检查参数更新
	if (_parameter_update_sub.updated() || !_callback_registered) { // 如果参数更新或回调未注册
		// 清除更新
		parameter_update_s pupdate; // 创建参数更新结构
		_parameter_update_sub.copy(&pupdate); // 复制参数更新

		// 从存储中更新参数
		updateParams(); // 更新参数

		VerifyParams(); // 验证参数

		// 立即强制发布主题以进行日志记录（EKF2_LOG_VERBOSE，按辅助源控制）
		AdvertiseTopics(); // 广播主题

#if defined(CONFIG_EKF2_GNSS) // 检查是否启用GNSS
		_ekf.set_min_required_gps_health_time(_param_ekf2_req_gps_h.get() * 1_s); // 设置所需的GPS健康时间
#endif // CONFIG_EKF2_GNSS

		const matrix::Vector3f imu_pos_body(_param_ekf2_imu_pos_x.get(), // 获取IMU在机体坐标系中的位置
						    _param_ekf2_imu_pos_y.get(),
						    _param_ekf2_imu_pos_z.get());
		_ekf.output_predictor().set_imu_offset(imu_pos_body); // 设置IMU偏移
		_ekf.output_predictor().set_pos_correction_tc(_param_ekf2_tau_pos.get()); // 设置位置修正时间常数
		_ekf.output_predictor().set_vel_correction_tc(_param_ekf2_tau_vel.get()); // 设置速度修正时间常数

#if defined(CONFIG_EKF2_AIRSPEED) // 检查是否启用气速功能
		// 气速缩放因子修正仅通过参数可用，由气速模块使用
		param_t param_aspd_scale = param_find("ASPD_SCALE_1"); // 查找气速缩放参数

		if (param_aspd_scale != PARAM_INVALID) { // 如果参数有效
			param_get(param_aspd_scale, &_airspeed_scale_factor); // 获取气速缩放因子
		}

#endif // CONFIG_EKF2_AIRSPEED

		_ekf.updateParameters(); // 更新EKF参数
	}

	if (!_callback_registered) { // 如果回调未注册
#if defined(CONFIG_EKF2_MULTI_INSTANCE) // 检查是否启用多实例功能

		if (_multi_mode) { // 如果处于多模式
			_callback_registered = _vehicle_imu_sub.registerCallback(); // 注册IMU回调

		} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
		{
			_callback_registered = _sensor_combined_sub.registerCallback(); // 注册传感器组合回调
		}

		if (!_callback_registered) { // 如果回调注册失败
			ScheduleDelayed(10_ms); // 延迟10毫秒后重新调度
			return; // 退出
		}
	}

	if (_vehicle_command_sub.updated()) { // 检查车辆命令是否更新
		vehicle_command_s vehicle_command; // 创建车辆命令结构

		if (_vehicle_command_sub.update(&vehicle_command)) { // 更新车辆命令

			vehicle_command_ack_s command_ack{}; // 创建命令确认结构
			command_ack.command = vehicle_command.command; // 设置命令
			command_ack.target_system = vehicle_command.source_system; // 设置目标系统
			command_ack.target_component = vehicle_command.source_component; // 设置目标组件

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) { // 检查命令是否为设置GPS全球原点
				double latitude = vehicle_command.param5; // 获取纬度
				double longitude = vehicle_command.param6; // 获取经度
				float altitude = vehicle_command.param7; // 获取高度

				if (_ekf.setEkfGlobalOrigin(latitude, longitude, altitude)) { // 设置EKF全球原点
					// 验证EKF原点状态
					uint64_t origin_time {}; // 创建原点时间变量
					_ekf.getEkfGlobalOrigin(origin_time, latitude, longitude, altitude); // 获取EKF全球原点信息
					PX4_INFO("%d - New NED origin (LLA): %3.10f, %3.10f, %4.3f\n", // 输出新的NED原点信息
						 _instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED; // 设置命令确认结果为接受

				} else {
					PX4_ERR("%d - Failed to set new NED origin (LLA): %3.10f, %3.10f, %4.3f\n", // 输出设置失败信息
						_instance, latitude, longitude, static_cast<double>(altitude));

					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED; // 设置命令确认结果为失败
				}

				command_ack.timestamp = hrt_absolute_time(); // 设置时间戳
				_vehicle_command_ack_pub.publish(command_ack); // 发布命令确认

			} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE) { // 检查命令是否为外部位置估计

				if ((_ekf.control_status_flags().wind_dead_reckoning || _ekf.control_status_flags().inertial_dead_reckoning // 检查控制状态标志
				     || (!_ekf.control_status_flags().in_air && !_ekf.control_status_flags().gps)) && PX4_ISFINITE(vehicle_command.param2) // 检查参数有效性
				    && PX4_ISFINITE(vehicle_command.param5) && PX4_ISFINITE(vehicle_command.param6)
				   ) {

					const float measurement_delay_seconds = math::constrain(vehicle_command.param2, 0.0f, // 获取测量延迟
										kMaxDelaySecondsExternalPosMeasurement);
					const uint64_t timestamp_observation = vehicle_command.timestamp - measurement_delay_seconds * 1_s; // 计算观察时间戳

					float accuracy = kDefaultExternalPosAccuracy; // 设置默认外部位置精度

					if (PX4_ISFINITE(vehicle_command.param3) && vehicle_command.param3 > FLT_EPSILON) { // 检查精度参数有效性
						accuracy = vehicle_command.param3; // 更新精度
					}

					if (_ekf.resetGlobalPosToExternalObservation(vehicle_command.param5, vehicle_command.param6, vehicle_command.param7, // 重置全球位置为外部观察
							accuracy, accuracy, timestamp_observation)
					   ) {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED; // 设置命令确认结果为接受

					} else {
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED; // 设置命令确认结果为失败
					}

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED; // 设置命令确认结果为暂时拒绝
				}

				command_ack.timestamp = hrt_absolute_time(); // 设置时间戳
				_vehicle_command_ack_pub.publish(command_ack); // 发布命令确认
			}

			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE) { // 检查命令是否为外部风估计
#if defined(CONFIG_EKF2_WIND) // 检查是否启用风功能
				// 风向以方位角给出，表示风从哪里吹来
				// PX4后端期望风向是风吹向的方向
				const float wind_direction_rad = wrap_pi(math::radians(vehicle_command.param3) + M_PI_F); // 计算风向（弧度）
				const float wind_direction_accuracy_rad = math::radians(vehicle_command.param4); // 计算风向精度（弧度）
				_ekf.resetWindToExternalObservation(vehicle_command.param1, wind_direction_rad, vehicle_command.param2, // 重置风为外部观察
								    wind_direction_accuracy_rad);
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED; // 设置命令确认结果为接受
#else
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED; // 设置命令确认结果为不支持
#endif // CONFIG_EKF2_WIND
				command_ack.timestamp = hrt_absolute_time(); // 设置时间戳
				_vehicle_command_ack_pub.publish(command_ack); // 发布命令确认
			}
		}
	}

	bool imu_updated = false; // 标记IMU是否更新
	imuSample imu_sample_new {}; // 创建新的IMU样本

	hrt_abstime imu_dt = 0; // 用于跟踪时间滑移

#if defined(CONFIG_EKF2_MULTI_INSTANCE) // 检查是否启用多实例功能

	if (_multi_mode) { // 如果处于多模式
		const unsigned last_generation = _vehicle_imu_sub.get_last_generation(); // 获取最后一代IMU数据
		vehicle_imu_s imu; // 创建IMU数据结构
		imu_updated = _vehicle_imu_sub.update(&imu); // 更新IMU数据

		if (imu_updated && (_vehicle_imu_sub.get_last_generation() != last_generation + 1)) { // 检查IMU更新是否丢失
			perf_count(_msg_missed_imu_perf); // 记录丢失IMU消息的性能计数
		}

		if (imu_updated) { // 如果IMU数据更新
			imu_sample_new.time_us = imu.timestamp_sample; // 设置IMU样本时间戳
			imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1.e-6f; // 设置角速度增量时间
			imu_sample_new.delta_ang = Vector3f{imu.delta_angle}; // 设置角速度增量
			imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f; // 设置线速度增量时间
			imu_sample_new.delta_vel = Vector3f{imu.delta_velocity}; // 设置线速度增量

			if (imu.delta_velocity_clipping > 0) { // 检查速度剪切标志
				imu_sample_new.delta_vel_clipping[0] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_X; // 设置X轴剪切
				imu_sample_new.delta_vel_clipping[1] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Y; // 设置Y轴剪切
				imu_sample_new.delta_vel_clipping[2] = imu.delta_velocity_clipping & vehicle_imu_s::CLIPPING_Z; // 设置Z轴剪切
			}

			imu_dt = imu.delta_angle_dt; // 更新IMU时间增量

			if ((_device_id_accel == 0) || (_device_id_gyro == 0)) { // 检查加速度计和陀螺仪设备ID
				_device_id_accel = imu.accel_device_id; // 更新加速度计设备ID
				_device_id_gyro = imu.gyro_device_id; // 更新陀螺仪设备ID
				_accel_calibration_count = imu.accel_calibration_count; // 更新加速度计校准计数
				_gyro_calibration_count = imu.gyro_calibration_count; // 更新陀螺仪校准计数

			} else {
				if ((imu.accel_calibration_count != _accel_calibration_count) // 检查加速度计校准计数是否变化
				    || (imu.accel_device_id != _device_id_accel)) { // 检查加速度计设备ID是否变化

					PX4_DEBUG("%d - resetting accelerometer bias", _instance); // 输出重置加速度计偏置信息
					_device_id_accel = imu.accel_device_id; // 更新加速度计设备ID

					_ekf.resetAccelBias(); // 重置加速度计偏置
					_accel_calibration_count = imu.accel_calibration_count; // 更新加速度计校准计数

					// 重置偏置学习
					_accel_cal = {}; // 清空加速度计偏置学习数据
				}

				if ((imu.gyro_calibration_count != _gyro_calibration_count) // 检查陀螺仪校准计数是否变化
				    || (imu.gyro_device_id != _device_id_gyro)) { // 检查陀螺仪设备ID是否变化

					PX4_DEBUG("%d - resetting rate gyro bias", _instance); // 输出重置陀螺仪偏置信息
					_device_id_gyro = imu.gyro_device_id; // 更新陀螺仪设备ID

					_ekf.resetGyroBias(); // 重置陀螺仪偏置
					_gyro_calibration_count = imu.gyro_calibration_count; // 更新陀螺仪校准计数

					// 重置偏置学习
					_gyro_cal = {}; // 清空陀螺仪偏置学习数据
				}
			}
		}

	} else
#endif // CONFIG_EKF2_MULTI_INSTANCE
	{
		// 获取传感器组合的最后一代数据
		const unsigned last_generation = _sensor_combined_sub.get_last_generation();
		sensor_combined_s sensor_combined; // 声明传感器组合数据结构
		imu_updated = _sensor_combined_sub.update(&sensor_combined); // 更新传感器组合数据

		// 检查IMU数据是否更新，并且是否有数据丢失
		if (imu_updated && (_sensor_combined_sub.get_last_generation() != last_generation + 1)) {
			perf_count(_msg_missed_imu_perf); // 记录丢失的IMU数据性能计数
		}

		// 如果IMU数据更新
		if (imu_updated) {
			imu_sample_new.time_us = sensor_combined.timestamp; // 设置IMU样本的时间戳
			imu_sample_new.delta_ang_dt = sensor_combined.gyro_integral_dt * 1.e-6f; // 设置角速度增量时间（转换为秒）
			imu_sample_new.delta_ang = Vector3f{sensor_combined.gyro_rad} * imu_sample_new.delta_ang_dt; // 计算角速度增量
			imu_sample_new.delta_vel_dt = sensor_combined.accelerometer_integral_dt * 1.e-6f; // 设置线速度增量时间（转换为秒）
			imu_sample_new.delta_vel = Vector3f{sensor_combined.accelerometer_m_s2} * imu_sample_new.delta_vel_dt; // 计算线速度增量

			// 检查加速度计是否有剪切标志
			if (sensor_combined.accelerometer_clipping > 0) {
				// 设置X轴剪切
				imu_sample_new.delta_vel_clipping[0] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_X;
				// 设置Y轴剪切
				imu_sample_new.delta_vel_clipping[1] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Y;
				// 设置Z轴剪切
				imu_sample_new.delta_vel_clipping[2] = sensor_combined.accelerometer_clipping & sensor_combined_s::CLIPPING_Z;
			}

			imu_dt = sensor_combined.gyro_integral_dt; // 更新IMU时间增量

			// 检查加速度计校准计数是否变化
			if (sensor_combined.accel_calibration_count != _accel_calibration_count) {
				PX4_DEBUG("%d - resetting accelerometer bias", _instance); // 输出重置加速度计偏置信息

				_ekf.resetAccelBias(); // 重置加速度计偏置
				_accel_calibration_count = sensor_combined.accel_calibration_count; // 更新加速度计校准计数

				// 重置偏置学习
				_accel_cal = {}; // 清空加速度计偏置学习数据
			}

			// 检查陀螺仪校准计数是否变化
			if (sensor_combined.gyro_calibration_count != _gyro_calibration_count) {
				PX4_DEBUG("%d - resetting rate gyro bias", _instance); // 输出重置陀螺仪偏置信息

				_ekf.resetGyroBias(); // 重置陀螺仪偏置
				_gyro_calibration_count = sensor_combined.gyro_calibration_count; // 更新陀螺仪校准计数

				// 重置偏置学习
				_gyro_cal = {}; // 清空陀螺仪偏置学习数据
			}
		}

		// 检查传感器选择是否更新，或者设备ID是否为0
		if (_sensor_selection_sub.updated() || (_device_id_accel == 0 || _device_id_gyro == 0)) {
			sensor_selection_s sensor_selection; // 声明传感器选择数据结构

			// 复制传感器选择数据
			if (_sensor_selection_sub.copy(&sensor_selection)) {
				// 检查加速度计设备ID是否变化
				if (_device_id_accel != sensor_selection.accel_device_id) {
					_device_id_accel = sensor_selection.accel_device_id; // 更新加速度计设备ID

					_ekf.resetAccelBias(); // 重置加速度计偏置

					// 重置偏置学习
					_accel_cal = {}; // 清空加速度计偏置学习数据
				}

				// 检查陀螺仪设备ID是否变化
				if (_device_id_gyro != sensor_selection.gyro_device_id) {
					_device_id_gyro = sensor_selection.gyro_device_id; // 更新陀螺仪设备ID

					_ekf.resetGyroBias(); // 重置陀螺仪偏置

					// 重置偏置学习
					_gyro_cal = {}; // 清空陀螺仪偏置学习数据
				}
			}
		}
	}

	// 如果IMU数据更新
	if (imu_updated) {
		const hrt_abstime now = imu_sample_new.time_us; // 获取当前时间戳

		// 将IMU数据推送到估计器
		_ekf.setIMUData(imu_sample_new);
		PublishAttitude(now); // 立即发布姿态（使用输出预测器的四元数）

		// 整合时间以监控时间漂移
		if (_start_time_us > 0) {
			_integrated_time_us += imu_dt; // 累加整合时间
			_last_time_slip_us = (imu_sample_new.time_us - _start_time_us) - _integrated_time_us; // 计算时间漂移

		} else {
			_start_time_us = imu_sample_new.time_us; // 初始化开始时间
			_last_time_slip_us = 0; // 重置时间漂移
		}

		// ekf2_timestamps（使用0.1毫秒相对时间戳）
		ekf2_timestamps_s ekf2_timestamps {
			.timestamp = now, // 设置时间戳
			.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的空速时间戳
			.airspeed_validated_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的验证空速时间戳
			.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的距离传感器时间戳
			.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的光流时间戳
			.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的车辆气象数据时间戳
			.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的车辆磁力计时间戳
			.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID, // 无效的视觉里程计时间戳
		};

#if defined(CONFIG_EKF2_AIRSPEED)
		UpdateAirspeedSample(ekf2_timestamps); // 更新空速样本
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
		UpdateAuxVelSample(ekf2_timestamps); // 更新辅助速度样本
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
		UpdateBaroSample(ekf2_timestamps); // 更新气压计样本
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		UpdateExtVisionSample(ekf2_timestamps); // 更新外部视觉样本
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		UpdateFlowSample(ekf2_timestamps); // 更新光流样本
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_GNSS)
		UpdateGpsSample(ekf2_timestamps); // 更新GNSS样本
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
		UpdateMagSample(ekf2_timestamps); // 更新磁力计样本
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
		UpdateRangeSample(ekf2_timestamps); // 更新距离传感器样本
#endif // CONFIG_EKF2_RANGE_FINDER
		UpdateSystemFlagsSample(ekf2_timestamps); // 更新系统标志样本

		// 运行EKF更新并输出
		const hrt_abstime ekf_update_start = hrt_absolute_time(); // 记录EKF更新开始时间

		if (_ekf.update()) { // 执行EKF更新
			perf_set_elapsed(_ekf_update_perf, hrt_elapsed_time(&ekf_update_start)); // 设置EKF更新性能计数

			PublishLocalPosition(now); // 发布本地位置
			PublishOdometry(now, imu_sample_new); // 发布里程计数据
			PublishGlobalPosition(now); // 发布全球位置
			PublishSensorBias(now); // 发布传感器偏置

#if defined(CONFIG_EKF2_WIND)
			PublishWindEstimate(now); // 发布风速估计
#endif // CONFIG_EKF2_WIND

			// 发布状态/日志消息
			PublishEventFlags(now); // 发布事件标志
			PublishStatus(now); // 发布状态
			PublishStatusFlags(now); // 发布状态标志

			// 如果启用了详细日志记录
			if (_param_ekf2_log_verbose.get()) {
				PublishAidSourceStatus(now); // 发布辅助源状态
				PublishInnovations(now); // 发布创新数据
				PublishInnovationTestRatios(now); // 发布创新测试比率
				PublishInnovationVariances(now); // 发布创新方差
				PublishStates(now); // 发布状态数据

#if defined(CONFIG_EKF2_BAROMETER)
				PublishBaroBias(now); // 发布气压计偏置
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
				PublishEvPosBias(now); // 发布外部视觉位置偏置
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
				PublishGnssHgtBias(now); // 发布GNSS高度偏置
#endif // CONFIG_EKF2_GNSS

			}

#if defined(CONFIG_EKF2_GNSS)
			PublishGpsStatus(now); // 发布GNSS状态
			PublishYawEstimatorStatus(now); // 发布航向估计器状态
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
			PublishOpticalFlowVel(now); // 发布光流速度
#endif // CONFIG_EKF2_OPTICAL_FLOW

			UpdateAccelCalibration(now); // 更新加速度计校准
			UpdateGyroCalibration(now); // 更新陀螺仪校准
#if defined(CONFIG_EKF2_MAGNETOMETER)
			UpdateMagCalibration(now); // 更新磁力计校准
#endif // CONFIG_EKF2_MAGNETOMETER
		}

		// 发布ekf2时间戳
		_ekf2_timestamps_pub.publish(ekf2_timestamps);
	}

	// re-schedule as backup timeout
	ScheduleDelayed(100_ms);
}

void EKF2::VerifyParams() // 验证参数的有效性
{
#if defined(CONFIG_EKF2_MAGNETOMETER)

	// 检查EKF2_MAG_TYPE参数是否为过时选项
	if ((_param_ekf2_mag_type.get() != MagFuseType::AUTO) // 检查是否为自动模式
	    && (_param_ekf2_mag_type.get() != MagFuseType::HEADING) // 检查是否为航向模式
	    && (_param_ekf2_mag_type.get() != MagFuseType::NONE) // 检查是否为无模式
	    && (_param_ekf2_mag_type.get() != MagFuseType::INIT) // 检查是否为初始化模式
	   ) {

		mavlink_log_critical(&_mavlink_log_pub, "EKF2_MAG_TYPE invalid, resetting to default"); // 记录错误日志，表示EKF2_MAG_TYPE无效，将重置为默认值
		/* 事件
		 * @描述 <param>EKF2_MAG_TYPE</param> 被设置为 {1:.0}。
		 */
		events::send<float>(events::ID("ekf2_mag_type_invalid"), events::Log::Warning,
				    "EKF2_MAG_TYPE invalid, resetting to default", _param_ekf2_mag_type.get());

		_param_ekf2_mag_type.set(0); // 将EKF2_MAG_TYPE重置为默认值
		_param_ekf2_mag_type.commit(); // 提交参数更改
	}

#endif // CONFIG_EKF2_MAGNETOMETER

	float delay_max = _param_ekf2_delay_max.get(); // 获取最大延迟参数

#if defined(CONFIG_EKF2_AUXVEL)

	if (_param_ekf2_avel_delay.get() > delay_max) { // 检查辅助速度延迟是否超过最大延迟
		delay_max = _param_ekf2_avel_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)

	if (_param_ekf2_baro_delay.get() > delay_max) { // 检查气压计延迟是否超过最大延迟
		delay_max = _param_ekf2_baro_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)

	if (_param_ekf2_asp_delay.get() > delay_max) { // 检查空速延迟是否超过最大延迟
		delay_max = _param_ekf2_asp_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_param_ekf2_mag_delay.get() > delay_max) { // 检查磁力计延迟是否超过最大延迟
		delay_max = _param_ekf2_mag_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_param_ekf2_rng_delay.get() > delay_max) { // 检查测距仪延迟是否超过最大延迟
		delay_max = _param_ekf2_rng_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_GNSS)

	if (_param_ekf2_gps_delay.get() > delay_max) { // 检查GNSS延迟是否超过最大延迟
		delay_max = _param_ekf2_gps_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_param_ekf2_of_delay.get() > delay_max) { // 检查光流延迟是否超过最大延迟
		delay_max = _param_ekf2_of_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_param_ekf2_ev_delay.get() > delay_max) { // 检查外部视觉延迟是否超过最大延迟
		delay_max = _param_ekf2_ev_delay.get(); // 更新最大延迟
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	if (delay_max > _param_ekf2_delay_max.get()) { // 如果更新后的最大延迟超过了原最大延迟
		/* 事件
		 * @描述 EKF2_DELAY_MAX({1}ms) 相较于最大传感器延迟 ({2}) 太小
		 */
		events::send<float, float>(events::ID("nf_delay_max_too_small"), events::Log::Warning,
					   "EKF2_DELAY_MAX increased to {2}ms, please reboot", _param_ekf2_delay_max.get(),
					   delay_max);
		_param_ekf2_delay_max.commit_no_notification(delay_max); // 提交新的最大延迟值，不发送通知
	}
}

void EKF2::PublishAidSourceStatus(const hrt_abstime &timestamp) // 发布辅助源状态
{
#if defined(CONFIG_EKF2_AIRSPEED)
	// 发布空速辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_airspeed(), _status_airspeed_pub_last, _estimator_aid_src_airspeed_pub);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	// 发布侧滑辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_sideslip(), _status_sideslip_pub_last, _estimator_aid_src_sideslip_pub);
#endif // CONFIG_EKF2_SIDESLIP
#if defined(CONFIG_EKF2_BAROMETER)
	// 发布气压计高度辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_baro_hgt(), _status_baro_hgt_pub_last, _estimator_aid_src_baro_hgt_pub);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 发布拖曳辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_drag(), _status_drag_pub_last, _estimator_aid_src_drag_pub);
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 发布测距仪高度辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_rng_hgt(), _status_rng_hgt_pub_last, _estimator_aid_src_rng_hgt_pub);
#endif // CONFIG_EKF2_RANGE_FINDER

	// 发布虚假位置辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_fake_pos(), _status_fake_pos_pub_last, _estimator_aid_src_fake_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_fake_hgt(), _status_fake_hgt_pub_last, _estimator_aid_src_fake_hgt_pub);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 发布外部视觉（EV）高度/位置/速度/航向辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_ev_hgt(), _status_ev_hgt_pub_last, _estimator_aid_src_ev_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_pos(), _status_ev_pos_pub_last, _estimator_aid_src_ev_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_vel(), _status_ev_vel_pub_last, _estimator_aid_src_ev_vel_pub);
	PublishAidSourceStatus(_ekf.aid_src_ev_yaw(), _status_ev_yaw_pub_last, _estimator_aid_src_ev_yaw_pub);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// 发布GNSS高度/位置/速度/航向辅助源状态
	PublishAidSourceStatus(_ekf.aid_src_gnss_hgt(), _status_gnss_hgt_pub_last, _estimator_aid_src_gnss_hgt_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_pos(), _status_gnss_pos_pub_last, _estimator_aid_src_gnss_pos_pub);
	PublishAidSourceStatus(_ekf.aid_src_gnss_vel(), _status_gnss_vel_pub_last, _estimator_aid_src_gnss_vel_pub);
# if defined(CONFIG_EKF2_GNSS_YAW)
	PublishAidSourceStatus(_ekf.aid_src_gnss_yaw(), _status_gnss_yaw_pub_last, _estimator_aid_src_gnss_yaw_pub); // 发布GNSS航向辅助源状态
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 发布磁力计三维数据
	// 调用PublishAidSourceStatus函数，发布磁力计的辅助源状态
	// 参数说明：
	// _ekf.aid_src_mag()：获取磁力计的辅助源数据
	// _status_mag_pub_last：上次发布的磁力计状态
	// _estimator_aid_src_mag_pub：磁力计辅助源的发布器
	PublishAidSourceStatus(_ekf.aid_src_mag(), _status_mag_pub_last, _estimator_aid_src_mag_pub);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 发布重力数据
	// 调用PublishAidSourceStatus函数，发布重力的辅助源状态
	// 参数说明：
	// _ekf.aid_src_gravity()：获取重力的辅助源数据
	// _status_gravity_pub_last：上次发布的重力状态
	// _estimator_aid_src_gravity_pub：重力辅助源的发布器
	PublishAidSourceStatus(_ekf.aid_src_gravity(), _status_gravity_pub_last, _estimator_aid_src_gravity_pub);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	// 发布辅助速度数据
	// 调用PublishAidSourceStatus函数，发布辅助速度的辅助源状态
	// 参数说明：
	// _ekf.aid_src_aux_vel()：获取辅助速度的辅助源数据
	// _status_aux_vel_pub_last：上次发布的辅助速度状态
	// _estimator_aid_src_aux_vel_pub：辅助速度辅助源的发布器
	PublishAidSourceStatus(_ekf.aid_src_aux_vel(), _status_aux_vel_pub_last, _estimator_aid_src_aux_vel_pub);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 发布光流数据
	// 调用PublishAidSourceStatus函数，发布光流的辅助源状态
	// 参数说明：
	// _ekf.aid_src_optical_flow()：获取光流的辅助源数据
	// _status_optical_flow_pub_last：上次发布的光流状态
	// _estimator_aid_src_optical_flow_pub：光流辅助源的发布器
	PublishAidSourceStatus(_ekf.aid_src_optical_flow(), _status_optical_flow_pub_last, _estimator_aid_src_optical_flow_pub);
#endif // CONFIG_EKF2_OPTICAL_FLOW
}
void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
	if (_ekf.attitude_valid()) { // 检查当前的姿态是否有效
		// 生成车辆姿态四元数数据
		vehicle_attitude_s att; // 创建一个车辆姿态结构体
		att.timestamp_sample = timestamp; // 设置样本时间戳
		_ekf.getQuaternion().copyTo(att.q); // 获取四元数并复制到姿态结构体中

		// 获取四元数重置信息
		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter); // 获取四元数重置信息并存储
		att.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳，重放模式下使用传入的时间戳
		_attitude_pub.publish(att); // 发布姿态数据

	}  else if (_replay_mode) { // 如果处于重放模式
		// 在重放模式下，我们需要告诉重放模块不等待更新
		// 通过发布一个时间戳为零的姿态来实现
		vehicle_attitude_s att{}; // 创建一个空的姿态结构体
		_attitude_pub.publish(att); // 发布空的姿态数据
	}
}

#if defined(CONFIG_EKF2_BAROMETER)
void EKF2::PublishBaroBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_baro_hgt().timestamp_sample != 0) { // 检查气压计高度辅助源的时间戳是否有效
		const BiasEstimator::status &status = _ekf.getBaroBiasEstimatorStatus(); // 获取气压计偏置估计器的状态

		if (fabsf(status.bias - _last_baro_bias_published) > 0.001f) { // 检查当前偏置与上次发布的偏置是否有显著变化
			_estimator_baro_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.aid_src_baro_hgt().timestamp_sample, timestamp,
							 _device_id_baro)); // 发布气压计偏置消息

			_last_baro_bias_published = status.bias; // 更新上次发布的气压计偏置
		}
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGnssHgtBias(const hrt_abstime &timestamp)
{
	if (_ekf.get_gps_sample_delayed().time_us != 0) { // 检查延迟的GPS样本时间戳是否有效
		const BiasEstimator::status &status = _ekf.getGpsHgtBiasEstimatorStatus(); // 获取GPS高度偏置估计器的状态

		if (fabsf(status.bias - _last_gnss_hgt_bias_published) > 0.001f) { // 检查当前偏置与上次发布的偏置是否有显著变化
			_estimator_gnss_hgt_bias_pub.publish(fillEstimatorBiasMsg(status, _ekf.get_gps_sample_delayed().time_us, timestamp)); // 发布GNSS高度偏置消息

			_last_gnss_hgt_bias_published = status.bias; // 更新上次发布的GNSS高度偏置
		}
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EKF2::PublishEvPosBias(const hrt_abstime &timestamp)
{
	if (_ekf.aid_src_ev_hgt().timestamp_sample) { // 检查外部视觉高度辅助源的时间戳是否有效

		estimator_bias3d_s bias{}; // 创建一个三维偏置估计结构体

		// 获取高度偏置估计状态
		BiasEstimator::status bias_est_status[3]; // 创建一个偏置估计状态数组
		bias_est_status[0] = _ekf.getEvPosBiasEstimatorStatus(0); // 获取外部视觉位置偏置估计状态
		bias_est_status[1] = _ekf.getEvPosBiasEstimatorStatus(1); // 获取外部视觉位置偏置估计状态
		bias_est_status[2] = _ekf.getEvHgtBiasEstimatorStatus(); // 获取外部视觉高度偏置估计状态

		for (int i = 0; i < 3; i++) { // 遍历偏置估计状态
			bias.bias[i] = bias_est_status[i].bias; // 存储偏置
			bias.bias_var[i] = bias_est_status[i].bias_var; // 存储偏置方差

			bias.innov[i] = bias_est_status[i].innov; // 存储创新值
			bias.innov_var[i] = bias_est_status[i].innov_var; // 存储创新方差
			bias.innov_test_ratio[i] = bias_est_status[i].innov_test_ratio; // 存储创新测试比率
		}

		const Vector3f bias_vec{bias.bias}; // 创建偏置向量

		if ((bias_vec - _last_ev_bias_published).longerThan(0.01f)) { // 检查当前偏置与上次发布的偏置是否有显著变化
			bias.timestamp_sample = _ekf.aid_src_ev_hgt().timestamp_sample; // 设置样本时间戳
			bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳，重放模式下使用传入的时间戳
			_estimator_ev_pos_bias_pub.publish(bias); // 发布外部视觉位置偏置消息

			_last_ev_bias_published = Vector3f(bias.bias); // 更新上次发布的外部视觉偏置
		}
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

estimator_bias_s EKF2::fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
		uint64_t timestamp, uint32_t device_id)
{
	estimator_bias_s bias{}; // 创建一个偏置估计消息结构体
	bias.timestamp_sample = timestamp_sample_us; // 设置样本时间戳
	bias.device_id = device_id; // 设置设备ID
	bias.bias = status.bias; // 设置偏置
	bias.bias_var = status.bias_var; // 设置偏置方差
	bias.innov = status.innov; // 设置创新值
	bias.innov_var = status.innov_var; // 设置创新方差
	bias.innov_test_ratio = status.innov_test_ratio; // 设置创新测试比率
	bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳，重放模式下使用传入的时间戳

	return bias; // 返回填充好的偏置估计消息
}

void EKF2::PublishEventFlags(const hrt_abstime &timestamp)
{
	// 信息事件
	uint32_t information_events = _ekf.information_event_status().value; // 获取信息事件状态值
	bool information_event_updated = false; // 初始化信息事件更新标志

	if (information_events != 0) { // 如果有信息事件
		information_event_updated = true; // 标记信息事件已更新
		_filter_information_event_changes++; // 增加信息事件变化计数
	}

	if (information_event_updated) { // 如果信息事件已更新
		estimator_event_flags_s event_flags{}; // 创建事件标志结构体
		event_flags.timestamp_sample = _ekf.time_delayed_us(); // 设置样本时间戳

		// 设置事件标志的各个字段
		event_flags.information_event_changes           = _filter_information_event_changes; // 信息事件变化计数
		event_flags.gps_checks_passed                   = _ekf.information_event_flags().gps_checks_passed; // GPS检查通过标志
		event_flags.reset_vel_to_gps                    = _ekf.information_event_flags().reset_vel_to_gps; // 重置速度到GPS标志
		event_flags.reset_vel_to_flow                   = _ekf.information_event_flags().reset_vel_to_flow; // 重置速度到光流标志
		event_flags.reset_vel_to_vision                 = _ekf.information_event_flags().reset_vel_to_vision; // 重置速度到视觉标志
		event_flags.reset_vel_to_zero                   = _ekf.information_event_flags().reset_vel_to_zero; // 重置速度到零标志
		event_flags.reset_pos_to_last_known             = _ekf.information_event_flags().reset_pos_to_last_known; // 重置位置到最后已知位置标志
		event_flags.reset_pos_to_gps                    = _ekf.information_event_flags().reset_pos_to_gps; // 重置位置到GPS标志
		event_flags.reset_pos_to_vision                 = _ekf.information_event_flags().reset_pos_to_vision; // 重置位置到视觉标志
		event_flags.starting_gps_fusion                 = _ekf.information_event_flags().starting_gps_fusion; // 开始GPS融合标志
		event_flags.starting_vision_pos_fusion          = _ekf.information_event_flags().starting_vision_pos_fusion; // 开始视觉位置融合标志
		event_flags.starting_vision_vel_fusion          = _ekf.information_event_flags().starting_vision_vel_fusion; // 开始视觉速度融合标志
		event_flags.starting_vision_yaw_fusion          = _ekf.information_event_flags().starting_vision_yaw_fusion; // 开始视觉航向融合标志
		event_flags.yaw_aligned_to_imu_gps              = _ekf.information_event_flags().yaw_aligned_to_imu_gps; // 航向与IMU和GPS对齐标志
		event_flags.reset_hgt_to_baro                   = _ekf.information_event_flags().reset_hgt_to_baro; // 重置高度到气压计标志
		event_flags.reset_hgt_to_gps                    = _ekf.information_event_flags().reset_hgt_to_gps; // 重置高度到GPS标志
		event_flags.reset_hgt_to_rng                    = _ekf.information_event_flags().reset_hgt_to_rng; // 重置高度到范围传感器标志
		event_flags.reset_hgt_to_ev                     = _ekf.information_event_flags().reset_hgt_to_ev; // 重置高度到外部视觉标志

		event_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳，重放模式下使用传入的时间戳
		_estimator_event_flags_pub.update(event_flags); // 更新事件标志发布器

		_last_event_flags_publish = event_flags.timestamp; // 更新上次发布的事件标志时间戳

		_ekf.clear_information_events(); // 清除信息事件

	} else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) { // 如果上次发布的时间戳有效且当前时间超过上次发布时间1秒
		// 继续定期发布
		_estimator_event_flags_pub.get().timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳
		_estimator_event_flags_pub.update(); // 更新事件标志发布器
		_last_event_flags_publish = _estimator_event_flags_pub.get().timestamp; // 更新上次发布的事件标志时间戳
	}
}

void EKF2::PublishGlobalPosition(const hrt_abstime &timestamp)
{
	// 检查全局原点是否有效且航向对齐状态是否有效
	if (_ekf.global_origin_valid() && _ekf.control_status().flags.yaw_align) {
		// 生成并发布全局位置数据
		vehicle_global_position_s global_pos{}; // 创建全局位置数据结构
		global_pos.timestamp_sample = timestamp; // 设置样本时间戳

		// 获取GPS / WGS84坐标系下的位置
		const LatLonAlt lla = _ekf.getLatLonAlt(); // 获取纬度、经度和高度
		global_pos.lat = lla.latitude_deg(); // 设置纬度
		global_pos.lon = lla.longitude_deg(); // 设置经度
		global_pos.lat_lon_valid = _ekf.isGlobalHorizontalPositionValid(); // 检查纬度和经度是否有效

		global_pos.alt = lla.altitude(); // 设置高度
		global_pos.alt_valid = _ekf.isGlobalVerticalPositionValid(); // 检查高度是否有效

#if defined(CONFIG_EKF2_GNSS)
		// 将海拔高度转换为椭球高度
		global_pos.alt_ellipsoid = altAmslToEllipsoid(global_pos.alt); // 计算椭球高度
#endif

		// 全局高度与本地下方位置的符号相反
		float delta_z = 0.f; // 高度变化量初始化
		uint8_t z_reset_counter = 0; // 高度重置计数器初始化
		_ekf.get_posD_reset(&delta_z, &z_reset_counter); // 获取高度重置信息
		global_pos.delta_alt = -delta_z; // 设置高度变化量为负值
		global_pos.alt_reset_counter = z_reset_counter; // 设置高度重置计数器

		float delta_xy[2] {}; // 初始化XY方向的变化量
		uint8_t xy_reset_counter = 0; // XY重置计数器初始化
		_ekf.get_posNE_reset(delta_xy, &xy_reset_counter); // 获取XY方向的重置信息
		global_pos.lat_lon_reset_counter = xy_reset_counter; // 设置纬度和经度重置计数器

		_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv); // 获取全局位置的精度信息

#if defined(CONFIG_EKF2_TERRAIN)
		// 获取地形高度（米），WGS84坐标系
		global_pos.terrain_alt = _ekf.getEkfGlobalOriginAltitude() - _ekf.getTerrainVertPos(); // 计算地形高度
		global_pos.terrain_alt_valid = _ekf.isTerrainEstimateValid(); // 检查地形估计是否有效

		float delta_hagl = 0.f; // 初始化地面高度变化量
		_ekf.get_hagl_reset(&delta_hagl, &global_pos.terrain_reset_counter); // 获取地面高度重置信息
		global_pos.delta_terrain = -delta_z; // 设置地形高度变化量为负值
#endif // CONFIG_EKF2_TERRAIN

		// 检查惯性和风的死区推算状态
		global_pos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
					    || _ekf.control_status_flags().wind_dead_reckoning; // 设置死区推算标志

		// 设置时间戳，重放模式下使用传入的时间戳
		global_pos.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		_global_position_pub.publish(global_pos); // 发布全局位置数据
	}
}

#if defined(CONFIG_EKF2_GNSS)
void EKF2::PublishGpsStatus(const hrt_abstime &timestamp)
{
	const hrt_abstime timestamp_sample = _ekf.get_gps_sample_delayed().time_us; // 获取延迟的GPS样本时间戳

	// 如果当前样本时间戳与上次发布的时间戳相同，则返回
	if (timestamp_sample == _last_gps_status_published) {
		return; // 无需更新
	}

	estimator_gps_status_s estimator_gps_status{}; // 创建GPS状态估计器结构体
	estimator_gps_status.timestamp_sample = timestamp_sample; // 设置样本时间戳

	// 获取GPS位置漂移速率
	estimator_gps_status.position_drift_rate_horizontal_m_s = _ekf.gps_horizontal_position_drift_rate_m_s(); // 水平位置漂移速率
	estimator_gps_status.position_drift_rate_vertical_m_s   = _ekf.gps_vertical_position_drift_rate_m_s(); // 垂直位置漂移速率
	estimator_gps_status.filtered_horizontal_speed_m_s      = _ekf.gps_filtered_horizontal_velocity_m_s(); // 过滤后的水平速度

	estimator_gps_status.checks_passed = _ekf.gps_checks_passed(); // 获取GPS检查通过的状态

	// 获取GPS检查失败的状态
	estimator_gps_status.check_fail_gps_fix          = _ekf.gps_check_fail_status_flags().fix; // GPS定位失败标志
	estimator_gps_status.check_fail_min_sat_count    = _ekf.gps_check_fail_status_flags().nsats; // 最小卫星数量失败标志
	estimator_gps_status.check_fail_max_pdop         = _ekf.gps_check_fail_status_flags().pdop; // 最大PDOP失败标志
	estimator_gps_status.check_fail_max_horz_err     = _ekf.gps_check_fail_status_flags().hacc; // 最大水平误差失败标志
	estimator_gps_status.check_fail_max_vert_err     = _ekf.gps_check_fail_status_flags().vacc; // 最大垂直误差失败标志
	estimator_gps_status.check_fail_max_spd_err      = _ekf.gps_check_fail_status_flags().sacc; // 最大速度误差失败标志
	estimator_gps_status.check_fail_max_horz_drift   = _ekf.gps_check_fail_status_flags().hdrift; // 最大水平漂移失败标志
	estimator_gps_status.check_fail_max_vert_drift   = _ekf.gps_check_fail_status_flags().vdrift; // 最大垂直漂移失败标志
	estimator_gps_status.check_fail_max_horz_spd_err = _ekf.gps_check_fail_status_flags().hspeed; // 最大水平速度误差失败标志
	estimator_gps_status.check_fail_max_vert_spd_err = _ekf.gps_check_fail_status_flags().vspeed; // 最大垂直速度误差失败标志
	estimator_gps_status.check_fail_spoofed_gps      = _ekf.gps_check_fail_status_flags().spoofed; // GPS欺骗检测标志

	// 设置时间戳，重放模式下使用传入的时间戳
	estimator_gps_status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	_estimator_gps_status_pub.publish(estimator_gps_status); // 发布GPS状态

	_last_gps_status_published = timestamp_sample; // 更新上次发布的GPS状态时间戳
}
#endif // CONFIG_EKF2_GNSS

void EKF2::PublishInnovations(const hrt_abstime &timestamp)
{
	// 发布估计器创新数据
	estimator_innovations_s innovations{}; // 创建创新数据结构体
	innovations.timestamp_sample = _ekf.time_delayed_us(); // 设置样本时间戳

#if defined(CONFIG_EKF2_GNSS)
	// GPS创新数据
	innovations.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation[0]; // 获取GPS水平速度创新
	innovations.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation[1]; // 获取GPS水平速度创新
	innovations.gps_vvel    = _ekf.aid_src_gnss_vel().innovation[2]; // 获取GPS垂直速度创新
	innovations.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation[0]; // 获取GPS水平位置创新
	innovations.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation[1]; // 获取GPS水平位置创新
	innovations.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation; // 获取GPS垂直位置创新
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 外部视觉创新数据
	innovations.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation[0]; // 获取外部视觉水平速度创新
	innovations.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation[1]; // 获取外部视觉水平速度创新
	innovations.ev_vvel    = _ekf.aid_src_ev_vel().innovation[2]; // 获取外部视觉垂直速度创新
	innovations.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation[0]; // 获取外部视觉水平位置创新
	innovations.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation[1]; // 获取外部视觉水平位置创新
	innovations.ev_vpos    = _ekf.aid_src_ev_hgt().innovation; // 获取外部视觉垂直位置创新
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 高度传感器创新数据
#if defined(CONFIG_EKF2_RANGE_FINDER)
	innovations.rng_vpos = _ekf.aid_src_rng_hgt().innovation; // 获取测距仪垂直位置创新
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	innovations.baro_vpos = _ekf.aid_src_baro_hgt().innovation; // 获取气压计垂直位置创新
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度创新数据
	innovations.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation[0]; // 获取辅助水平速度创新
	innovations.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation[1]; // 获取辅助水平速度创新
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流创新数据
	innovations.flow[0] = _ekf.aid_src_optical_flow().innovation[0]; // 获取光流水平创新
	innovations.flow[1] = _ekf.aid_src_optical_flow().innovation[1]; // 获取光流水平创新
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// 获取航向创新
	innovations.heading = _ekf.getHeadingInnov(); // 获取航向创新

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁场创新数据
	innovations.mag_field[0] = _ekf.aid_src_mag().innovation[0]; // 获取磁场X方向创新
	innovations.mag_field[1] = _ekf.aid_src_mag().innovation[1]; // 获取磁场Y方向创新
	innovations.mag_field[2] = _ekf.aid_src_mag().innovation[2]; // 获取磁场Z方向创新
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力创新数据
	// 从重力辅助源获取创新数据，分别对应X、Y、Z方向的重力创新
	innovations.gravity[0] = _ekf.aid_src_gravity().innovation[0]; // 获取重力在X方向的创新
	innovations.gravity[1] = _ekf.aid_src_gravity().innovation[1]; // 获取重力在Y方向的创新
	innovations.gravity[2] = _ekf.aid_src_gravity().innovation[2]; // 获取重力在Z方向的创新
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 拖曳创新数据
	// 从拖曳辅助源获取创新数据，分别对应X、Y方向的拖曳创新
	innovations.drag[0] = _ekf.aid_src_drag().innovation[0]; // 获取拖曳在X方向的创新
	innovations.drag[1] = _ekf.aid_src_drag().innovation[1]; // 获取拖曳在Y方向的创新
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// 空速创新数据
	// 从空速辅助源获取创新数据，表示当前的空速创新
	innovations.airspeed = _ekf.aid_src_airspeed().innovation; // 获取空速的创新
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 侧滑角创新数据
	// 从侧滑辅助源获取创新数据，表示当前的侧滑角创新
	innovations.beta = _ekf.aid_src_sideslip().innovation; // 获取侧滑角的创新
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// 地面高度创新数据
	// 从测距仪辅助源获取创新数据，表示当前的地面高度创新
	innovations.hagl = _ekf.aid_src_rng_hgt().innovation; // 获取地面高度的创新
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 地面高度变化率创新数据
	// 获取当前的地面高度变化率创新
	innovations.hagl_rate = _ekf.getHaglRateInnov(); // 获取地面高度变化率的创新
#endif // CONFIG_EKF2_RANGE_FINDER

	// 设置创新数据的时间戳
	// 如果处于重放模式，则使用传入的时间戳，否则使用当前绝对时间
	innovations.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	// 发布创新数据
	_estimator_innovations_pub.publish(innovations); // 发布创新数据到估计器创新发布器
}

// 发布创新测试比率数据
void EKF2::PublishInnovationTestRatios(const hrt_abstime &timestamp)
{
	// 创建一个估计器创新测试比率数据结构
	estimator_innovations_s test_ratios{};
	// 设置样本时间戳
	test_ratios.timestamp_sample = _ekf.time_delayed_us(); // 获取延迟的时间戳

#if defined(CONFIG_EKF2_GNSS)
	// GPS创新测试比率
	// 从GNSS辅助源获取测试比率数据，分别对应水平速度、垂直速度和位置
	test_ratios.gps_hvel[0] = _ekf.aid_src_gnss_vel().test_ratio[0]; // 获取GPS水平速度X方向的测试比率
	test_ratios.gps_hvel[1] = _ekf.aid_src_gnss_vel().test_ratio[1]; // 获取GPS水平速度Y方向的测试比率
	test_ratios.gps_vvel    = _ekf.aid_src_gnss_vel().test_ratio[2]; // 获取GPS垂直速度的测试比率
	test_ratios.gps_hpos[0] = _ekf.aid_src_gnss_pos().test_ratio[0]; // 获取GPS水平位置X方向的测试比率
	test_ratios.gps_hpos[1] = _ekf.aid_src_gnss_pos().test_ratio[1]; // 获取GPS水平位置Y方向的测试比率
	test_ratios.gps_vpos    = _ekf.aid_src_gnss_hgt().test_ratio; // 获取GPS垂直位置的测试比率
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 外部视觉创新测试比率
	// 从外部视觉辅助源获取测试比率数据，分别对应水平速度、垂直速度和位置
	test_ratios.ev_hvel[0] = _ekf.aid_src_ev_vel().test_ratio[0]; // 获取外部视觉水平速度X方向的测试比率
	test_ratios.ev_hvel[1] = _ekf.aid_src_ev_vel().test_ratio[1]; // 获取外部视觉水平速度Y方向的测试比率
	test_ratios.ev_vvel    = _ekf.aid_src_ev_vel().test_ratio[2]; // 获取外部视觉垂直速度的测试比率
	test_ratios.ev_hpos[0] = _ekf.aid_src_ev_pos().test_ratio[0]; // 获取外部视觉水平位置X方向的测试比率
	test_ratios.ev_hpos[1] = _ekf.aid_src_ev_pos().test_ratio[1]; // 获取外部视觉水平位置Y方向的测试比率
	test_ratios.ev_vpos    = _ekf.aid_src_ev_hgt().test_ratio; // 获取外部视觉垂直位置的测试比率
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 高度传感器创新测试比率
#if defined(CONFIG_EKF2_RANGE_FINDER)
	test_ratios.rng_vpos = _ekf.aid_src_rng_hgt().test_ratio; // 获取测距仪垂直位置的测试比率
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	test_ratios.baro_vpos = _ekf.aid_src_baro_hgt().test_ratio; // 获取气压计垂直位置的测试比率
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度创新测试比率
	test_ratios.aux_hvel[0] = _ekf.aid_src_aux_vel().test_ratio[0]; // 获取辅助速度X方向的测试比率
	test_ratios.aux_hvel[1] = _ekf.aid_src_aux_vel().test_ratio[1]; // 获取辅助速度Y方向的测试比率
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流创新测试比率
	test_ratios.flow[0] = _ekf.aid_src_optical_flow().test_ratio[0]; // 获取光流X方向的测试比率
	test_ratios.flow[1] = _ekf.aid_src_optical_flow().test_ratio[1]; // 获取光流Y方向的测试比率
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// 航向创新测试比率
	test_ratios.heading = _ekf.getHeadingInnovRatio(); // 获取航向的创新测试比率

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁场创新测试比率
	test_ratios.mag_field[0] = _ekf.aid_src_mag().test_ratio[0]; // 获取磁场X方向的测试比率
	test_ratios.mag_field[1] = _ekf.aid_src_mag().test_ratio[1]; // 获取磁场Y方向的测试比率
	test_ratios.mag_field[2] = _ekf.aid_src_mag().test_ratio[2]; // 获取磁场Z方向的测试比率
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力创新测试比率
	test_ratios.gravity[0] = _ekf.aid_src_gravity().test_ratio[0]; // 获取重力X方向的测试比率
	test_ratios.gravity[1] = _ekf.aid_src_gravity().test_ratio[1]; // 获取重力Y方向的测试比率
	test_ratios.gravity[2] = _ekf.aid_src_gravity().test_ratio[2]; // 获取重力Z方向的测试比率
#endif // CONFIG_EKF2_GRAVITY_FUSION
#if defined(CONFIG_EKF2_DRAG_FUSION)
	// drag - 获取拖曳源的测试比率
	test_ratios.drag[0] = _ekf.aid_src_drag().test_ratio[0]; // 获取拖曳X方向的测试比率
	test_ratios.drag[1] = _ekf.aid_src_drag().test_ratio[1]; // 获取拖曳Y方向的测试比率
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed - 获取气速的测试比率
	test_ratios.airspeed = _ekf.aid_src_airspeed().test_ratio; // 获取气速的测试比率
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// beta - 获取侧滑角的测试比率
	test_ratios.beta = _ekf.aid_src_sideslip().test_ratio; // 获取侧滑角的测试比率
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl - 获取相对地面高度的测试比率
	test_ratios.hagl = _ekf.aid_src_rng_hgt().test_ratio; // 获取相对地面高度的测试比率
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// hagl_rate - 获取相对地面高度变化率的创新测试比率
	test_ratios.hagl_rate = _ekf.getHaglRateInnovRatio(); // 获取相对地面高度变化率的创新测试比率
#endif // CONFIG_EKF2_RANGE_FINDER

	// 设置时间戳，判断是否在重放模式下
	test_ratios.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置测试比率的时间戳
	_estimator_innovation_test_ratios_pub.publish(test_ratios); // 发布测试比率数据
}

void EKF2::PublishInnovationVariances(const hrt_abstime &timestamp)
{
	// 发布估计器创新方差数据
	estimator_innovations_s variances{}; // 创建一个估计器创新方差结构体
	variances.timestamp_sample = _ekf.time_delayed_us(); // 设置时间戳为延迟的时间

#if defined(CONFIG_EKF2_GNSS)
	// GPS相关的创新方差
	variances.gps_hvel[0] = _ekf.aid_src_gnss_vel().innovation_variance[0]; // 获取GPS水平速度X方向的创新方差
	variances.gps_hvel[1] = _ekf.aid_src_gnss_vel().innovation_variance[1]; // 获取GPS水平速度Y方向的创新方差
	variances.gps_vvel    = _ekf.aid_src_gnss_vel().innovation_variance[2]; // 获取GPS垂直速度的创新方差
	variances.gps_hpos[0] = _ekf.aid_src_gnss_pos().innovation_variance[0]; // 获取GPS水平位置X方向的创新方差
	variances.gps_hpos[1] = _ekf.aid_src_gnss_pos().innovation_variance[1]; // 获取GPS水平位置Y方向的创新方差
	variances.gps_vpos    = _ekf.aid_src_gnss_hgt().innovation_variance;    // 获取GPS高度的创新方差
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 外部视觉相关的创新方差
	variances.ev_hvel[0] = _ekf.aid_src_ev_vel().innovation_variance[0]; // 获取外部视觉水平速度X方向的创新方差
	variances.ev_hvel[1] = _ekf.aid_src_ev_vel().innovation_variance[1]; // 获取外部视觉水平速度Y方向的创新方差
	variances.ev_vvel    = _ekf.aid_src_ev_vel().innovation_variance[2]; // 获取外部视觉垂直速度的创新方差
	variances.ev_hpos[0] = _ekf.aid_src_ev_pos().innovation_variance[0]; // 获取外部视觉水平位置X方向的创新方差
	variances.ev_hpos[1] = _ekf.aid_src_ev_pos().innovation_variance[1]; // 获取外部视觉水平位置Y方向的创新方差
	variances.ev_vpos    = _ekf.aid_src_ev_hgt().innovation_variance;    // 获取外部视觉高度的创新方差
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 高度传感器相关的创新方差
#if defined(CONFIG_EKF2_RANGE_FINDER)
	variances.rng_vpos = _ekf.aid_src_rng_hgt().innovation_variance; // 获取测距仪高度的创新方差
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_BAROMETER)
	variances.baro_vpos = _ekf.aid_src_baro_hgt().innovation_variance; // 获取气压计高度的创新方差
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度相关的创新方差
	variances.aux_hvel[0] = _ekf.aid_src_aux_vel().innovation_variance[0]; // 获取辅助速度X方向的创新方差
	variances.aux_hvel[1] = _ekf.aid_src_aux_vel().innovation_variance[1]; // 获取辅助速度Y方向的创新方差
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流相关的创新方差
	variances.flow[0] = _ekf.aid_src_optical_flow().innovation_variance[0]; // 获取光流X方向的创新方差
	variances.flow[1] = _ekf.aid_src_optical_flow().innovation_variance[1]; // 获取光流Y方向的创新方差
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// 航向的创新方差
	variances.heading = _ekf.getHeadingInnovVar(); // 获取航向的创新方差

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁场相关的创新方差
	variances.mag_field[0] = _ekf.aid_src_mag().innovation_variance[0]; // 获取磁场X方向的创新方差
	variances.mag_field[1] = _ekf.aid_src_mag().innovation_variance[1]; // 获取磁场Y方向的创新方差
	variances.mag_field[2] = _ekf.aid_src_mag().innovation_variance[2]; // 获取磁场Z方向的创新方差
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力相关的创新方差
	variances.gravity[0] = _ekf.aid_src_gravity().innovation_variance[0]; // 获取重力X方向的创新方差
	variances.gravity[1] = _ekf.aid_src_gravity().innovation_variance[1]; // 获取重力Y方向的创新方差
	variances.gravity[2] = _ekf.aid_src_gravity().innovation_variance[2]; // 获取重力Z方向的创新方差
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 拖曳相关的创新方差
	variances.drag[0] = _ekf.aid_src_drag().innovation_variance[0]; // 获取拖曳X方向的创新方差
	variances.drag[1] = _ekf.aid_src_drag().innovation_variance[1]; // 获取拖曳Y方向的创新方差
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// 气速相关的创新方差
	variances.airspeed = _ekf.aid_src_airspeed().innovation_variance; // 获取气速的创新方差
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 侧滑角相关的创新方差
	variances.beta = _ekf.aid_src_sideslip().innovation_variance; // 获取侧滑角的创新方差
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN) && defined(CONFIG_EKF2_RANGE_FINDER)
	// 相对地面高度的创新方差
	variances.hagl = _ekf.aid_src_rng_hgt().innovation_variance; // 获取相对地面高度的创新方差
#endif // CONFIG_EKF2_TERRAIN && CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 相对地面高度变化率的创新方差
	variances.hagl_rate = _ekf.getHaglRateInnovVar(); // 获取相对地面高度变化率的创新方差
#endif // CONFIG_EKF2_RANGE_FINDER
	// 设置创新方差的时间戳
	// 如果处于重放模式，则使用传入的时间戳，否则使用当前绝对时间
	variances.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	// 发布创新方差数据到估计器创新方差发布器
	_estimator_innovation_variances_pub.publish(variances);
}

void EKF2::PublishLocalPosition(const hrt_abstime &timestamp)
{
	// 创建一个车辆本地位置数据结构
	vehicle_local_position_s lpos{};
	// 生成车辆本地位置数据
	lpos.timestamp_sample = timestamp; // 设置样本时间戳

	// 在本地NED坐标系中，身体原点的位置
	const Vector3f position{_ekf.getPosition()}; // 获取当前的位置
	lpos.x = position(0); // 设置X坐标
	lpos.y = position(1); // 设置Y坐标
	lpos.z = position(2); // 设置Z坐标

	// 在本地NED坐标系中，身体原点的速度（单位：米/秒）
	const Vector3f velocity{_ekf.getVelocity()}; // 获取当前的速度
	lpos.vx = velocity(0); // 设置X方向速度
	lpos.vy = velocity(1); // 设置Y方向速度
	lpos.vz = velocity(2); // 设置Z方向速度

	// 垂直位置的时间导数（单位：米/秒）
	lpos.z_deriv = _ekf.getVerticalPositionDerivative(); // 获取垂直位置的导数

	// 在本地坐标系中，身体原点的加速度
	const Vector3f vel_deriv{_ekf.getVelocityDerivative()}; // 获取速度的导数
	_ekf.resetVelocityDerivativeAccumulation(); // 重置速度导数累积
	lpos.ax = vel_deriv(0); // 设置X方向加速度
	lpos.ay = vel_deriv(1); // 设置Y方向加速度
	lpos.az = vel_deriv(2); // 设置Z方向加速度

	// 检查XY方向的位置有效性
	lpos.xy_valid = _ekf.isLocalHorizontalPositionValid(); // 检查XY位置是否有效
	lpos.v_xy_valid = _ekf.isLocalHorizontalPositionValid(); // 检查XY速度是否有效

	// TODO: 一些模块（例如：mc_pos_control）未能正确处理v_z_valid与z_valid不相等的情况
	lpos.z_valid = _ekf.isLocalVerticalPositionValid() || _ekf.isLocalVerticalVelocityValid(); // 检查Z位置是否有效
	lpos.v_z_valid = _ekf.isLocalVerticalVelocityValid() || _ekf.isLocalVerticalPositionValid(); // 检查Z速度是否有效

	// 在GPS/WGS84坐标系中，本地NED原点的位置
	if (_ekf.global_origin_valid()) { // 检查全局原点是否有效
		lpos.ref_timestamp = _ekf.global_origin().getProjectionReferenceTimestamp(); // 获取参考时间戳
		lpos.ref_lat = _ekf.global_origin().getProjectionReferenceLat(); // 获取参考点的纬度（单位：度）
		lpos.ref_lon = _ekf.global_origin().getProjectionReferenceLon(); // 获取参考点的经度（单位：度）
		lpos.ref_alt = _ekf.getEkfGlobalOriginAltitude(); // 获取参考点的海拔高度（单位：米）
		lpos.xy_global = true; // 设置XY坐标为全局有效
		lpos.z_global = true; // 设置Z坐标为全局有效

	} else {
		lpos.ref_timestamp = 0; // 如果无效，设置时间戳为0
		lpos.ref_lat = static_cast<double>(NAN); // 设置纬度为NaN
		lpos.ref_lon = static_cast<double>(NAN); // 设置经度为NaN
		lpos.ref_alt = NAN; // 设置海拔高度为NaN
		lpos.xy_global = false; // 设置XY坐标为无效
		lpos.z_global = false; // 设置Z坐标为无效
	}

	Quatf delta_q_reset; // 存储四元数重置信息
	_ekf.get_quat_reset(&delta_q_reset(0), &lpos.heading_reset_counter); // 获取四元数重置信息

	lpos.heading = Eulerf(_ekf.getQuaternion()).psi(); // 获取航向角
	lpos.unaided_heading = _ekf.getUnaidedYaw(); // 获取未经辅助的航向
	lpos.heading_var = _ekf.getYawVar(); // 获取航向方差
	lpos.delta_heading = Eulerf(delta_q_reset).psi(); // 获取航向变化量
	lpos.heading_good_for_control = _ekf.isYawFinalAlignComplete(); // 检查航向是否已完成最终对齐
	lpos.tilt_var = _ekf.getTiltVariance(); // 获取倾斜方差

#if defined(CONFIG_EKF2_TERRAIN)
	// 到底面（地面）的距离（单位：米），必须为正值
	lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid(); // 检查地形估计是否有效
	lpos.dist_bottom = math::max(_ekf.getHagl(), 0.f); // 获取到地面的距离，确保为非负值
	lpos.dist_bottom_var = _ekf.getTerrainVariance(); // 获取地形方差
	_ekf.get_hagl_reset(&lpos.delta_dist_bottom, &lpos.dist_bottom_reset_counter); // 获取地面高度重置信息

	lpos.dist_bottom_sensor_bitfield = vehicle_local_position_s::DIST_BOTTOM_SENSOR_NONE; // 初始化传感器位字段

	if (_ekf.control_status_flags().rng_terrain) { // 检查是否使用测距仪地形
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_RANGE; // 设置测距仪传感器位
	}

	if (_ekf.control_status_flags().opt_flow_terrain) { // 检查是否使用光流地形
		lpos.dist_bottom_sensor_bitfield |= vehicle_local_position_s::DIST_BOTTOM_SENSOR_FLOW; // 设置光流传感器位
	}

#endif // CONFIG_EKF2_TERRAIN

	// 获取本地位置的精度信息
	_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv); // 获取水平和垂直精度
	_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv); // 获取速度精度

	// 获取位置和速度的状态重置信息
	_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter); // 获取Z方向位置重置信息
	_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter); // 获取Z方向速度重置信息
	_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter); // 获取XY方向位置重置信息
	_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter); // 获取XY方向速度重置信息

	// 检查惯性和风的死区推算状态
	lpos.dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning
			      || _ekf.control_status_flags().wind_dead_reckoning; // 设置死区推算标志

	// 获取控制限制信息
	_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max_z, &lpos.hagl_max_xy); // 获取控制限制

	// 将NaN转换为无穷大
	if (!PX4_ISFINITE(lpos.vxy_max)) {
		lpos.vxy_max = INFINITY; // 如果无穷大，设置为INFINITY
	}

	if (!PX4_ISFINITE(lpos.vz_max)) {
		lpos.vz_max = INFINITY; // 如果无穷大，设置为INFINITY
	}

	if (!PX4_ISFINITE(lpos.hagl_min)) {
		lpos.hagl_min = INFINITY; // 如果无穷大，设置为INFINITY
	}

	if (!PX4_ISFINITE(lpos.hagl_max_z)) {
		lpos.hagl_max_z = INFINITY; // 如果无穷大，设置为INFINITY
	}

	if (!PX4_ISFINITE(lpos.hagl_max_xy)) {
		lpos.hagl_max_xy = INFINITY; // 如果无穷大，设置为INFINITY
	}

	// 发布车辆本地位置数据
	lpos.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳
	_local_position_pub.publish(lpos); // 发布本地位置数据
}

void EKF2::PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample)
{
	// 生成车辆里程计数据
	vehicle_odometry_s odom; // 创建里程计数据结构
	odom.timestamp_sample = imu_sample.time_us; // 设置样本时间戳

	// 位置
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED; // 设置位置坐标系为NED
	_ekf.getPosition().copyTo(odom.position); // 获取当前位置并复制到里程计数据

	// 方向四元数
	_ekf.getQuaternion().copyTo(odom.q); // 获取当前方向四元数并复制到里程计数据

	// 速度
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED; // 设置速度坐标系为NED
	_ekf.getVelocity().copyTo(odom.velocity); // 获取当前速度并复制到里程计数据

	// 角速度
	const Vector3f rates{imu_sample.delta_ang / imu_sample.delta_ang_dt}; // 计算角速度
	const Vector3f angular_velocity = rates - _ekf.getGyroBias(); // 减去陀螺仪偏置
	angular_velocity.copyTo(odom.angular_velocity); // 将角速度复制到里程计数据

	// 速度协方差
	_ekf.getVelocityVariance().copyTo(odom.velocity_variance); // 获取速度的协方差并复制到里程计数据

	// 位置协方差
	_ekf.getPositionVariance().copyTo(odom.position_variance); // 获取位置的协方差并复制到里程计数据

	// 方向协方差
	_ekf.getRotVarBody().copyTo(odom.orientation_variance); // 获取方向的协方差并复制到里程计数据

	// 重置计数器
	odom.reset_counter = _ekf.get_quat_reset_count() // 获取四元数重置计数
			     + _ekf.get_velNE_reset_count() // 获取XY方向速度重置计数
			     + _ekf.get_velD_reset_count() // 获取Z方向速度重置计数
			     + _ekf.get_posNE_reset_count() // 获取XY方向位置重置计数
			     + _ekf.get_posD_reset_count(); // 获取Z方向位置重置计数

	odom.quality = 0; // 设置质量标志为0

	// 发布车辆里程计数据
	odom.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置时间戳
	_odometry_pub.publish(odom); // 发布里程计数据
}

void EKF2::PublishSensorBias(const hrt_abstime &timestamp)
{
	// 估计器传感器偏置
	const Vector3f gyro_bias{_ekf.getGyroBias()}; // 获取陀螺仪偏置
	const Vector3f accel_bias{_ekf.getAccelBias()}; // 获取加速度计偏置

#if defined(CONFIG_EKF2_MAGNETOMETER)
	const Vector3f mag_bias {_ekf.getMagBias()}; // 获取磁力计偏置
#endif // CONFIG_EKF2_MAGNETOMETER
	// 以约1 Hz的频率发布传感器偏置数据，若有变化则更快发布
	if ((gyro_bias - _last_gyro_bias_published).longerThan(0.001f) // 检查陀螺仪偏置是否有显著变化
	    || (accel_bias - _last_accel_bias_published).longerThan(0.001f) // 检查加速度计偏置是否有显著变化
#if defined(CONFIG_EKF2_MAGNETOMETER)
	    || (mag_bias - _last_mag_bias_published).longerThan(0.001f) // 检查磁力计偏置是否有显著变化
#endif // CONFIG_EKF2_MAGNETOMETER
	    || (timestamp >= _last_sensor_bias_published + 1_s)) { // 检查时间戳是否超过上次发布的时间戳加1秒

		estimator_sensor_bias_s bias{}; // 创建一个估计器传感器偏置数据结构
		bias.timestamp_sample = _ekf.time_delayed_us(); // 获取延迟的时间戳

		// 如果设备ID不为0且IMU控制参数中包含陀螺仪偏置控制，则获取陀螺仪偏置
		if ((_device_id_gyro != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias))) {
			const Vector3f bias_var{_ekf.getGyroBiasVariance()}; // 获取陀螺仪偏置的方差

			bias.gyro_device_id = _device_id_gyro; // 设置陀螺仪设备ID
			gyro_bias.copyTo(bias.gyro_bias); // 将陀螺仪偏置复制到偏置数据结构
			bias.gyro_bias_limit = _ekf.getGyroBiasLimit(); // 获取陀螺仪偏置限制
			bias_var.copyTo(bias.gyro_bias_variance); // 将陀螺仪偏置方差复制到偏置数据结构
			bias.gyro_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f); // 检查陀螺仪偏置是否有效
			bias.gyro_bias_stable = _gyro_cal.cal_available; // 检查陀螺仪偏置是否稳定
			_last_gyro_bias_published = gyro_bias; // 更新上次发布的陀螺仪偏置
		}

		// 如果设备ID不为0且IMU控制参数中包含加速度计偏置控制，则获取加速度计偏置
		if ((_device_id_accel != 0) && (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias))) {
			const Vector3f bias_var{_ekf.getAccelBiasVariance()}; // 获取加速度计偏置的方差

			bias.accel_device_id = _device_id_accel; // 设置加速度计设备ID
			accel_bias.copyTo(bias.accel_bias); // 将加速度计偏置复制到偏置数据结构
			bias.accel_bias_limit = _ekf.getAccelBiasLimit(); // 获取加速度计偏置限制
			bias_var.copyTo(bias.accel_bias_variance); // 将加速度计偏置方差复制到偏置数据结构
			bias.accel_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f); // 检查加速度计偏置是否有效
			bias.accel_bias_stable = _accel_cal.cal_available; // 检查加速度计偏置是否稳定
			_last_accel_bias_published = accel_bias; // 更新上次发布的加速度计偏置
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)

		// 如果设备ID不为0，则获取磁力计偏置
		if (_device_id_mag != 0) {
			const Vector3f bias_var{_ekf.getMagBiasVariance()}; // 获取磁力计偏置的方差

			bias.mag_device_id = _device_id_mag; // 设置磁力计设备ID
			mag_bias.copyTo(bias.mag_bias); // 将磁力计偏置复制到偏置数据结构
			bias.mag_bias_limit = _ekf.getMagBiasLimit(); // 获取磁力计偏置限制
			bias_var.copyTo(bias.mag_bias_variance); // 将磁力计偏置方差复制到偏置数据结构
			bias.mag_bias_valid = bias_var.longerThan(0.f) && !bias_var.longerThan(0.1f); // 检查磁力计偏置是否有效
			bias.mag_bias_stable = _mag_cal.cal_available; // 检查磁力计偏置是否稳定
			_last_mag_bias_published = mag_bias; // 更新上次发布的磁力计偏置
		}

#endif // CONFIG_EKF2_MAGNETOMETER
		// 设置时间戳，判断是否处于重放模式，如果是则使用传入的时间戳，否则使用当前绝对时间
		bias.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
		// 发布传感器偏置数据到相应的发布器
		_estimator_sensor_bias_pub.publish(bias);

		// 更新上次发布的传感器偏置时间戳
		_last_sensor_bias_published = bias.timestamp;
	}
}

void EKF2::PublishStates(const hrt_abstime &timestamp)
{
	// 发布估计器状态
	estimator_states_s states; // 创建一个状态结构体
	// 获取延迟的时间戳
	states.timestamp_sample = _ekf.time_delayed_us();
	// 获取当前状态向量
	const auto state_vector = _ekf.state().vector();
	// 将状态向量复制到状态结构体中
	state_vector.copyTo(states.states);
	// 设置状态数量
	states.n_states = state_vector.size();
	// 将协方差对角线复制到状态结构体中
	_ekf.covariances_diagonal().copyTo(states.covariances);
	// 设置时间戳，判断是否处于重放模式
	states.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	// 发布状态数据
	_estimator_states_pub.publish(states);
}

void EKF2::PublishStatus(const hrt_abstime &timestamp)
{
	estimator_status_s status{}; // 创建状态结构体并初始化
	// 获取延迟的时间戳
	status.timestamp_sample = _ekf.time_delayed_us();

	// 获取输出跟踪误差并复制到状态结构体中
	_ekf.getOutputTrackingError().copyTo(status.output_tracking_error);

#if defined(CONFIG_EKF2_GNSS)
	// 仅报告启用的GPS检查失败（参数索引向左移1位，因为它们不包括GPS Fix位，后者始终被检查）
	status.gps_check_fail_flags = _ekf.gps_check_fail_status().value & (((uint16_t)_params->gps_check_mask << 1) | 1);
#endif // CONFIG_EKF2_GNSS

	// 获取控制模式标志
	status.control_mode_flags = _ekf.control_status().value;
	// 获取滤波器故障标志
	status.filter_fault_flags = _ekf.fault_status().value;

	// 获取水平和垂直速度创新测试比率
	float vel_xy_test_ratio = _ekf.getHorizontalVelocityInnovationTestRatio();
	float vel_z_test_ratio = _ekf.getVerticalVelocityInnovationTestRatio();

	// 检查速度测试比率的有效性并设置状态
	if (PX4_ISFINITE(vel_xy_test_ratio) && PX4_ISFINITE(vel_z_test_ratio)) {
		// 如果两个比率都有效，取最大值
		status.vel_test_ratio = math::max(vel_xy_test_ratio, vel_z_test_ratio);
	} else if (PX4_ISFINITE(vel_xy_test_ratio)) {
		// 如果只有水平速度比率有效
		status.vel_test_ratio = vel_xy_test_ratio;
	} else if (PX4_ISFINITE(vel_z_test_ratio)) {
		// 如果只有垂直速度比率有效
		status.vel_test_ratio = vel_z_test_ratio;
	} else {
		// 如果都无效，设置为NAN
		status.vel_test_ratio = NAN;
	}

	// 获取其他测试比率
	status.hdg_test_ratio = _ekf.getHeadingInnovationTestRatio();
	status.pos_test_ratio = _ekf.getHorizontalPositionInnovationTestRatio();
	status.hgt_test_ratio = _ekf.getVerticalPositionInnovationTestRatio();
	status.tas_test_ratio = _ekf.getAirspeedInnovationTestRatio();
	status.hagl_test_ratio = _ekf.getHeightAboveGroundInnovationTestRatio();
	status.beta_test_ratio = _ekf.getSyntheticSideslipInnovationTestRatio();

	// 获取位置精度
	_ekf.get_ekf_lpos_accuracy(&status.pos_horiz_accuracy, &status.pos_vert_accuracy);
	// 获取解状态标志
	status.solution_status_flags = _ekf.get_ekf_soln_status();

	// 重置计数器
	status.reset_count_vel_ne = _ekf.state_reset_status().reset_count.velNE;
	status.reset_count_vel_d = _ekf.state_reset_status().reset_count.velD;
	status.reset_count_pos_ne = _ekf.state_reset_status().reset_count.posNE;
	status.reset_count_pod_d = _ekf.state_reset_status().reset_count.posD;
	status.reset_count_quat = _ekf.state_reset_status().reset_count.quat;

	// 计算时间滑移
	status.time_slip = _last_time_slip_us * 1e-6f;

	// 设置最小测试比率
	static constexpr float kMinTestRatioPreflight = 0.5f;
	// 检查预飞行失败的创新比率
	status.pre_flt_fail_innov_heading   = (kMinTestRatioPreflight < status.hdg_test_ratio);
	status.pre_flt_fail_innov_height    = (kMinTestRatioPreflight < status.hgt_test_ratio);
	status.pre_flt_fail_innov_pos_horiz = (kMinTestRatioPreflight < status.pos_test_ratio);
	status.pre_flt_fail_innov_vel_horiz = (kMinTestRatioPreflight < vel_xy_test_ratio);
	status.pre_flt_fail_innov_vel_vert  = (kMinTestRatioPreflight < vel_z_test_ratio);

	// 检查磁场干扰状态
	status.pre_flt_fail_mag_field_disturbed = _ekf.control_status_flags().mag_field_disturbed;

	// 设置设备ID
	status.accel_device_id = _device_id_accel;
#if defined(CONFIG_EKF2_BAROMETER)
	status.baro_device_id = _device_id_baro;
#endif // CONFIG_EKF2_BAROMETER
	status.gyro_device_id = _device_id_gyro;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	status.mag_device_id = _device_id_mag;

	// 获取磁力计检查结果
	_ekf.get_mag_checks(status.mag_inclination_deg, status.mag_inclination_ref_deg, status.mag_strength_gs,
			    status.mag_strength_ref_gs);
#endif // CONFIG_EKF2_MAGNETOMETER

	// 设置时间戳，判断是否处于重放模式
	status.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
	// 发布状态数据
	_estimator_status_pub.publish(status);
}

void EKF2::PublishStatusFlags(const hrt_abstime &timestamp)
{
	// 以约1Hz的频率发布（或在滤波器控制状态或故障状态变化时立即发布）
	bool update = (timestamp >= _last_status_flags_publish + 1_s);

	// 检查滤波器控制状态
	if (_ekf.control_status().value != _filter_control_status) {
		update = true; // 如果状态变化，标记为更新
		_filter_control_status = _ekf.control_status().value;
		_filter_control_status_changes++; // 增加控制状态变化计数
	}

	// 检查滤波器故障状态
	if (_ekf.fault_status().value != _filter_fault_status) {
		update = true; // 如果状态变化，标记为更新
		_filter_fault_status = _ekf.fault_status().value;
		_filter_fault_status_changes++; // 增加故障状态变化计数
	}

	// 检查创新检查失败状态
	if (_ekf.innov_check_fail_status().value != _innov_check_fail_status) {
		update = true; // 如果状态变化，标记为更新
		_innov_check_fail_status = _ekf.innov_check_fail_status().value;
		_innov_check_fail_status_changes++; // 增加创新检查失败状态变化计数
	}

	if (update) {
		estimator_status_flags_s status_flags{}; // 创建状态标志结构体
		// 获取延迟的时间戳
		status_flags.timestamp_sample = _ekf.time_delayed_us();
		// 更新控制状态变化计数
		status_flags.control_status_changes   = _filter_control_status_changes; // 控制状态变化的计数

		// 获取倾斜对齐状态
		status_flags.cs_tilt_align            = _ekf.control_status_flags().tilt_align; // 倾斜对齐状态标志

		// 获取航向对齐状态
		status_flags.cs_yaw_align             = _ekf.control_status_flags().yaw_align; // 航向对齐状态标志

		// 获取GPS状态
		status_flags.cs_gps                   = _ekf.control_status_flags().gps; // GPS状态标志

		// 获取光流状态
		status_flags.cs_opt_flow              = _ekf.control_status_flags().opt_flow; // 光流状态标志

		// 获取磁力计航向状态
		status_flags.cs_mag_hdg               = _ekf.control_status_flags().mag_hdg; // 磁力计航向状态标志

		// 获取三维磁力计状态
		status_flags.cs_mag_3d                = _ekf.control_status_flags().mag_3D; // 三维磁力计状态标志

		// 获取磁偏角状态
		status_flags.cs_mag_dec               = _ekf.control_status_flags().mag_dec; // 磁偏角状态标志

		// 获取在空中状态
		status_flags.cs_in_air                = _ekf.control_status_flags().in_air; // 在空中状态标志

		// 获取风状态
		status_flags.cs_wind                  = _ekf.control_status_flags().wind; // 风状态标志

		// 获取气压高度状态
		status_flags.cs_baro_hgt              = _ekf.control_status_flags().baro_hgt; // 气压高度状态标志

		// 获取测距仪高度状态
		status_flags.cs_rng_hgt               = _ekf.control_status_flags().rng_hgt; // 测距仪高度状态标志

		// 获取GPS高度状态
		status_flags.cs_gps_hgt               = _ekf.control_status_flags().gps_hgt; // GPS高度状态标志

		// 获取外部视觉位置状态
		status_flags.cs_ev_pos                = _ekf.control_status_flags().ev_pos; // 外部视觉位置状态标志

		// 获取外部视觉航向状态
		status_flags.cs_ev_yaw                = _ekf.control_status_flags().ev_yaw; // 外部视觉航向状态标志

		// 获取外部视觉高度状态
		status_flags.cs_ev_hgt                = _ekf.control_status_flags().ev_hgt; // 外部视觉高度状态标志

		// 获取辅助速度融合状态
		status_flags.cs_fuse_beta             = _ekf.control_status_flags().fuse_beta; // 辅助速度融合状态标志

		// 获取磁场干扰状态
		status_flags.cs_mag_field_disturbed   = _ekf.control_status_flags().mag_field_disturbed; // 磁场干扰状态标志

		// 获取固定翼状态
		status_flags.cs_fixed_wing            = _ekf.control_status_flags().fixed_wing; // 固定翼状态标志

		// 获取磁力计故障状态
		status_flags.cs_mag_fault             = _ekf.control_status_flags().mag_fault; // 磁力计故障状态标志

		// 获取气速融合状态
		status_flags.cs_fuse_aspd             = _ekf.control_status_flags().fuse_aspd; // 气速融合状态标志

		// 获取地面效应状态
		status_flags.cs_gnd_effect            = _ekf.control_status_flags().gnd_effect; // 地面效应状态标志

		// 获取测距仪卡住状态
		status_flags.cs_rng_stuck             = _ekf.control_status_flags().rng_stuck; // 测距仪卡住状态标志

		// 获取GNSS航向状态
		status_flags.cs_gnss_yaw               = _ekf.control_status_flags().gnss_yaw; // GNSS航向状态标志

		// 获取飞行中的磁力计对齐状态
		status_flags.cs_mag_aligned_in_flight = _ekf.control_status_flags().mag_aligned_in_flight; // 飞行中磁力计对齐状态标志

		// 获取外部视觉速度状态
		status_flags.cs_ev_vel                = _ekf.control_status_flags().ev_vel; // 外部视觉速度状态标志

		// 获取合成磁场Z状态
		status_flags.cs_synthetic_mag_z       = _ekf.control_status_flags().synthetic_mag_z; // 合成磁场Z状态标志

		// 获取车辆静止状态
		status_flags.cs_vehicle_at_rest       = _ekf.control_status_flags().vehicle_at_rest; // 车辆静止状态标志

		// 获取GNSS航向故障状态
		status_flags.cs_gnss_yaw_fault         = _ekf.control_status_flags().gnss_yaw_fault; // GNSS航向故障状态标志

		// 获取测距仪故障状态
		status_flags.cs_rng_fault             = _ekf.control_status_flags().rng_fault; // 测距仪故障状态标志

		// 获取惯性导航死算状态
		status_flags.cs_inertial_dead_reckoning = _ekf.control_status_flags().inertial_dead_reckoning; // 惯性导航死算状态标志

		// 获取风死算状态
		status_flags.cs_wind_dead_reckoning     = _ekf.control_status_flags().wind_dead_reckoning; // 风死算状态标志

		// 获取测距仪运动学一致性状态
		status_flags.cs_rng_kin_consistent      = _ekf.control_status_flags().rng_kin_consistent; // 测距仪运动学一致性状态标志

		// 获取虚假位置状态
		status_flags.cs_fake_pos                = _ekf.control_status_flags().fake_pos; // 虚假位置状态标志

		// 获取虚假高度状态
		status_flags.cs_fake_hgt                = _ekf.control_status_flags().fake_hgt; // 虚假高度状态标志

		// 获取重力向量状态
		status_flags.cs_gravity_vector          = _ekf.control_status_flags().gravity_vector; // 重力向量状态标志

		// 获取磁力计状态
		status_flags.cs_mag                     = _ekf.control_status_flags().mag; // 磁力计状态标志

		// 获取外部视觉航向故障状态
		status_flags.cs_ev_yaw_fault            = _ekf.control_status_flags().ev_yaw_fault; // 外部视觉航向故障状态标志

		// 获取磁力计航向一致性状态
		status_flags.cs_mag_heading_consistent  = _ekf.control_status_flags().mag_heading_consistent; // 磁力计航向一致性状态标志

		// 获取辅助GPS状态
		status_flags.cs_aux_gpos                = _ekf.control_status_flags().aux_gpos; // 辅助GPS状态标志

		// 获取测距仪地形状态
		status_flags.cs_rng_terrain    = _ekf.control_status_flags().rng_terrain; // 测距仪地形状态标志

		// 获取光流地形状态
		status_flags.cs_opt_flow_terrain    = _ekf.control_status_flags().opt_flow_terrain; // 光流地形状态标志

		// 获取有效虚假位置状态
		status_flags.cs_valid_fake_pos      = _ekf.control_status_flags().valid_fake_pos; // 有效虚假位置状态标志

		// 获取恒定位置状态
		status_flags.cs_constant_pos        = _ekf.control_status_flags().constant_pos; // 恒定位置状态标志

		// 获取气压计故障状态
		status_flags.cs_baro_fault	    = _ekf.control_status_flags().baro_fault; // 气压计故障状态标志

		// 更新故障状态变化计数
		status_flags.fault_status_changes     = _filter_fault_status_changes; // 故障状态变化的计数

		// 获取磁力计X方向故障状态
		status_flags.fs_bad_mag_x             = _ekf.fault_status_flags().bad_mag_x; // 磁力计X方向故障状态标志

		// 获取磁力计Y方向故障状态
		status_flags.fs_bad_mag_y             = _ekf.fault_status_flags().bad_mag_y; // 磁力计Y方向故障状态标志

		// 获取磁力计Z方向故障状态
		status_flags.fs_bad_mag_z             = _ekf.fault_status_flags().bad_mag_z; // 磁力计Z方向故障状态标志

		// 获取航向故障状态
		status_flags.fs_bad_hdg               = _ekf.fault_status_flags().bad_hdg; // 航向故障状态标志

		// 获取磁偏角故障状态
		status_flags.fs_bad_mag_decl          = _ekf.fault_status_flags().bad_mag_decl; // 磁偏角故障状态标志

		// 获取气速故障状态
		status_flags.fs_bad_airspeed          = _ekf.fault_status_flags().bad_airspeed; // 气速故障状态标志

		// 获取侧滑故障状态
		status_flags.fs_bad_sideslip          = _ekf.fault_status_flags().bad_sideslip; // 侧滑故障状态标志

		// 获取光流X方向故障状态
		status_flags.fs_bad_optflow_x         = _ekf.fault_status_flags().bad_optflow_X; // 光流X方向故障状态标志

		// 获取光流Y方向故障状态
		status_flags.fs_bad_optflow_y         = _ekf.fault_status_flags().bad_optflow_Y; // 光流Y方向故障状态标志

		// 获取垂直加速度故障状态
		status_flags.fs_bad_acc_vertical      = _ekf.fault_status_flags().bad_acc_vertical; // 垂直加速度故障状态标志

		// 获取加速度剪切故障状态
		status_flags.fs_bad_acc_clipping      = _ekf.fault_status_flags().bad_acc_clipping; // 加速度剪切故障状态标志

		// 更新创新故障状态变化计数
		status_flags.innovation_fault_status_changes = _innov_check_fail_status_changes; // 创新故障状态变化的计数

		// 获取水平速度拒绝状态
		status_flags.reject_hor_vel                  = _ekf.innov_check_fail_status_flags().reject_hor_vel; // 水平速度拒绝状态标志

		// 获取垂直速度拒绝状态
		status_flags.reject_ver_vel                  = _ekf.innov_check_fail_status_flags().reject_ver_vel; // 垂直速度拒绝状态标志

		// 获取水平位置拒绝状态
		status_flags.reject_hor_pos                  = _ekf.innov_check_fail_status_flags().reject_hor_pos; // 水平位置拒绝状态标志

		// 获取垂直位置拒绝状态
		status_flags.reject_ver_pos                  = _ekf.innov_check_fail_status_flags().reject_ver_pos; // 垂直位置拒绝状态标志

		// 获取航向拒绝状态
		status_flags.reject_yaw                      = _ekf.innov_check_fail_status_flags().reject_yaw; // 航向拒绝状态标志

		// 获取气速拒绝状态
		status_flags.reject_airspeed                 = _ekf.innov_check_fail_status_flags().reject_airspeed; // 气速拒绝状态标志

		// 获取侧滑拒绝状态
		status_flags.reject_sideslip                 = _ekf.innov_check_fail_status_flags().reject_sideslip; // 侧滑拒绝状态标志

		// 获取相对地面高度拒绝状态
		status_flags.reject_hagl                     = _ekf.innov_check_fail_status_flags().reject_hagl; // 相对地面高度拒绝状态标志

		// 获取光流X方向拒绝状态
		status_flags.reject_optflow_x                = _ekf.innov_check_fail_status_flags().reject_optflow_X; // 光流X方向拒绝状态标志

		// 获取光流Y方向拒绝状态
		status_flags.reject_optflow_y                = _ekf.innov_check_fail_status_flags().reject_optflow_Y; // 光流Y方向拒绝状态标志

		// 设置时间戳，判断是否处于重放模式
		status_flags.timestamp = _replay_mode ? timestamp : hrt_absolute_time(); // 设置状态标志的时间戳

		// 发布状态标志数据
		_estimator_status_flags_pub.publish(status_flags); // 发布状态标志数据到估计器状态标志发布器

		// 更新最后发布状态标志的时间
		_last_status_flags_publish = status_flags.timestamp; // 更新最后发布状态标志的时间
	}
}

#if defined(CONFIG_EKF2_GNSS)
// 发布航向估计器状态的函数，参数为时间戳
void EKF2::PublishYawEstimatorStatus(const hrt_abstime &timestamp)
{
	// 静态断言，确保yaw_estimator_status_s结构体中的yaw数组大小与N_MODELS_EKFGSF相同
	static_assert(sizeof(yaw_estimator_status_s::yaw) / sizeof(float) == N_MODELS_EKFGSF,
		      "yaw_estimator_status_s::yaw wrong size");

	// 创建一个yaw估计器状态数据结构
	yaw_estimator_status_s yaw_est_test_data;

	// 获取EKF的航向估计数据
	if (_ekf.getDataEKFGSF(&yaw_est_test_data.yaw_composite, &yaw_est_test_data.yaw_variance,
			       yaw_est_test_data.yaw,
			       yaw_est_test_data.innov_vn, yaw_est_test_data.innov_ve,
			       yaw_est_test_data.weight)) {

		// 检查航向复合估计是否有效
		yaw_est_test_data.yaw_composite_valid = _ekf.isYawEmergencyEstimateAvailable();
		// 获取延迟的时间戳
		yaw_est_test_data.timestamp_sample = _ekf.time_delayed_us();
		// 设置时间戳，判断是否处于重放模式
		yaw_est_test_data.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		// 发布航向估计器状态数据
		_yaw_est_pub.publish(yaw_est_test_data);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_WIND)
// 发布风速估计的函数，参数为时间戳
void EKF2::PublishWindEstimate(const hrt_abstime &timestamp)
{
	// 检查EKF的风速状态是否有效
	if (_ekf.get_wind_status()) {
		// 仅在EKF声明风速估计有效时发布风速估计
		wind_s wind{};
		// 获取延迟的时间戳
		wind.timestamp_sample = _ekf.time_delayed_us();

		// 获取风速和风速方差
		const Vector2f wind_vel = _ekf.getWindVelocity();
		const Vector2f wind_vel_var = _ekf.getWindVelocityVariance();

#if defined(CONFIG_EKF2_AIRSPEED)
		// 获取气速创新和方差
		wind.tas_innov = _ekf.aid_src_airspeed().innovation;
		wind.tas_innov_var = _ekf.aid_src_airspeed().innovation_variance;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
		// 获取侧滑创新和方差
		wind.beta_innov = _ekf.aid_src_sideslip().innovation;
		wind.beta_innov = _ekf.aid_src_sideslip().innovation_variance;
#endif // CONFIG_EKF2_SIDESLIP

		// 设置风速和方差
		wind.windspeed_north = wind_vel(0); // 北向风速
		wind.windspeed_east = wind_vel(1);  // 东向风速
		wind.variance_north = wind_vel_var(0); // 北向风速方差
		wind.variance_east = wind_vel_var(1);  // 东向风速方差
		// 设置时间戳，判断是否处于重放模式
		wind.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		// 发布风速估计数据
		_wind_pub.publish(wind);
	}
}
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
// 发布光流速度的函数，参数为时间戳
void EKF2::PublishOpticalFlowVel(const hrt_abstime &timestamp)
{
	// 获取光流的时间戳
	const hrt_abstime timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

	// 检查时间戳是否有效且大于上次发布的时间戳
	if ((timestamp_sample != 0) && (timestamp_sample > _optical_flow_vel_pub_last)) {

		// 创建光流速度数据结构
		vehicle_optical_flow_vel_s flow_vel{};
		// 获取光流的时间戳
		flow_vel.timestamp_sample = _ekf.aid_src_optical_flow().timestamp_sample;

		// 获取光流速度和过滤后的光流速度
		_ekf.getFlowVelBody().copyTo(flow_vel.vel_body); // 体坐标系下的光流速度
		_ekf.getFlowVelNE().copyTo(flow_vel.vel_ne); // NED坐标系下的光流速度

		// 获取过滤后的光流速度
		_ekf.getFilteredFlowVelBody().copyTo(flow_vel.vel_body_filtered); // 体坐标系下的过滤光流速度
		_ekf.getFilteredFlowVelNE().copyTo(flow_vel.vel_ne_filtered); // NED坐标系下的过滤光流速度

		// 获取未补偿和补偿后的光流速率
		_ekf.getFlowUncompensated().copyTo(flow_vel.flow_rate_uncompensated); // 未补偿光流速率
		_ekf.getFlowCompensated().copyTo(flow_vel.flow_rate_compensated); // 补偿光流速率

		// 获取光流陀螺仪速率和偏置
		_ekf.getFlowGyro().copyTo(flow_vel.gyro_rate); // 光流陀螺仪速率
		_ekf.getFlowGyroBias().copyTo(flow_vel.gyro_bias); // 光流陀螺仪偏置
		_ekf.getFlowRefBodyRate().copyTo(flow_vel.ref_gyro); // 参考体坐标系下的陀螺仪速率

		// 设置时间戳，判断是否处于重放模式
		flow_vel.timestamp = _replay_mode ? timestamp : hrt_absolute_time();

		// 发布光流速度数据
		_estimator_optical_flow_vel_pub.publish(flow_vel);

		// 更新上次发布的光流速度时间戳
		_optical_flow_vel_pub_last = timestamp_sample;
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
// 更新气速样本的函数，参数为EKF时间戳结构体
void EKF2::UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF气速样本
	// 优先使用ORB_ID(airspeed_validated)，如果不可用则回退到原始气速ORB_ID(airspeed)
	if (_airspeed_validated_sub.updated()) {
		airspeed_validated_s airspeed_validated;

		// 更新气速验证数据
		if (_airspeed_validated_sub.update(&airspeed_validated)) {

			// 检查气速是否有效且选择的气速索引大于0
			if (PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
			    && (airspeed_validated.selected_airspeed_index > 0)
			   ) {
				float cas2tas = 1.f; // 定义气速转换因子

				// 如果校准气速有效且大于FLT_EPSILON，则计算气速转换因子
				if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
				    && (airspeed_validated.calibrated_airspeed_m_s > FLT_EPSILON)) {
					cas2tas = airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s;
				}

				// 创建气速样本数据结构
				airspeedSample airspeed_sample {
					.time_us = airspeed_validated.timestamp, // 时间戳
					.true_airspeed = airspeed_validated.true_airspeed_m_s, // 真气速
					.eas2tas = cas2tas, // 气速转换因子
				};
				// 设置气速数据到EKF
				_ekf.setAirspeedData(airspeed_sample);
			}

			// 更新最后验证的气速时间戳
			_airspeed_validated_timestamp_last = airspeed_validated.timestamp;

			// 计算气速验证时间戳与EKF时间戳的相对关系
			ekf2_timestamps.airspeed_validated_timestamp_rel = (int16_t)((int64_t)airspeed_validated.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

	} else if (((ekf2_timestamps.timestamp - _airspeed_validated_timestamp_last) > 3_s) && _airspeed_sub.updated()) {
		// 如果ORB_ID(airspeed_validated)不可用，则使用ORB_ID(airspeed)
		airspeed_s airspeed;

		// 更新气速数据
		if (_airspeed_sub.update(&airspeed)) {
			// 通过ORB_ID(airspeed)接收到的气速测量尚未经过校正
			// 需要应用ASPD_SCALE校正因子
			const float true_airspeed_m_s = airspeed.true_airspeed_m_s * _airspeed_scale_factor;

			// 检查气速是否有效
			if (PX4_ISFINITE(airspeed.true_airspeed_m_s)
			    && PX4_ISFINITE(airspeed.indicated_airspeed_m_s)
			    && (airspeed.indicated_airspeed_m_s > 0.f)
			   ) {
				// 创建气速样本数据结构
				airspeedSample airspeed_sample {
					.time_us = airspeed.timestamp_sample, // 时间戳
					.true_airspeed = true_airspeed_m_s, // 真气速
					.eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s, // 气速转换因子
				};
				// 设置气速数据到EKF
				_ekf.setAirspeedData(airspeed_sample);
			}

			// 计算气速时间戳与EKF时间戳的相对关系
			ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
// 更新辅助速度样本的函数，参数为ekf2时间戳
void EKF2::UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF 辅助速度样本
	//  - 使用着陆目标姿态估计作为另一种速度数据源
	landing_target_pose_s landing_target_pose;

	// 更新着陆目标姿态数据
	if (_landing_target_pose_sub.update(&landing_target_pose)) {
		// 只有在着陆目标具有固定位置和有效速度估计时才能使用
		if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
			// 车辆相对于目标的速度与目标相对于车辆的速度符号相反
			auxVelSample auxvel_sample{
				.time_us = landing_target_pose.timestamp, // 时间戳
				.vel = Vector2f{-landing_target_pose.vx_rel, -landing_target_pose.vy_rel}, // 相对速度，取反
				.velVar = Vector2f{landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel}, // 速度方差
			};
			// 将辅助速度数据设置到EKF中
			_ekf.setAuxVelData(auxvel_sample);
		}
	}
}
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
// 更新气压计样本的函数，参数为ekf2时间戳
void EKF2::UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF 气压计样本
	vehicle_air_data_s airdata;

	// 更新气动数据
	if (_airdata_sub.update(&airdata)) {

		bool reset = false; // 重置标志

		// 检查气压计是否发生变化
		if (airdata.baro_device_id != _device_id_baro) {
			if (_device_id_baro != 0) {
				// 输出气压传感器ID变化的调试信息
				PX4_DEBUG("%d - baro sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_baro, airdata.baro_device_id);
			}

			reset = true; // 设置重置标志

		} else if (airdata.calibration_count != _baro_calibration_count) {
			// 如果现有的校准发生变化，重置保存的气压偏差
			PX4_DEBUG("%d - baro %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_baro);
			reset = true; // 设置重置标志
		}

		if (reset) {
			_device_id_baro = airdata.baro_device_id; // 更新气压计设备ID
			_baro_calibration_count = airdata.calibration_count; // 更新气压计校准计数
		}

		// 设置气体密度
		_ekf.set_air_density(airdata.rho);

		// 设置气压计数据到EKF
		_ekf.setBaroData(baroSample{airdata.timestamp_sample, airdata.baro_alt_meter, reset});

		// 计算气动数据时间戳与EKF时间戳的相对关系
		ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
// 更新外部视觉样本的函数，参数为ekf2时间戳
bool EKF2::UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF 外部视觉样本
	bool new_ev_odom = false; // 新的外部视觉里程计标志

	vehicle_odometry_s ev_odom;

	// 更新外部视觉里程计数据
	if (_ev_odom_sub.update(&ev_odom)) {

		extVisionSample ev_data{}; // 创建外部视觉数据结构
		ev_data.pos.setNaN(); // 初始化位置为NaN
		ev_data.vel.setNaN(); // 初始化速度为NaN
		ev_data.quat.setNaN(); // 初始化四元数为NaN

		// 检查有效的速度数据
		const Vector3f ev_odom_vel(ev_odom.velocity); // 外部视觉里程计速度
		const Vector3f ev_odom_vel_var(ev_odom.velocity_variance); // 外部视觉里程计速度方差

		if (ev_odom_vel.isAllFinite()) { // 检查速度是否有效
			bool velocity_frame_valid = false; // 速度框架有效标志

			// 根据速度框架类型设置速度框架
			switch (ev_odom.velocity_frame) {
			case vehicle_odometry_s::VELOCITY_FRAME_NED:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_NED; // 设置为NED框架
				velocity_frame_valid = true; // 标记为有效
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_FRD:
				ev_data.vel_frame = VelocityFrame::LOCAL_FRAME_FRD; // 设置为FRD框架
				velocity_frame_valid = true; // 标记为有效
				break;

			case vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD:
				ev_data.vel_frame = VelocityFrame::BODY_FRAME_FRD; // 设置为BODY FRD框架
				velocity_frame_valid = true; // 标记为有效
				break;
			}

			if (velocity_frame_valid) { // 如果速度框架有效
				ev_data.vel = ev_odom_vel; // 设置速度数据

				const float evv_noise_var = sq(_param_ekf2_evv_noise.get()); // 获取速度噪声方差

				// 从外部视觉数据或参数中获取速度测量误差
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_vel_var.isAllFinite()) {
					// 选择较大的噪声方差
					ev_data.velocity_var(0) = fmaxf(evv_noise_var, ev_odom_vel_var(0));
					ev_data.velocity_var(1) = fmaxf(evv_noise_var, ev_odom_vel_var(1));
					ev_data.velocity_var(2) = fmaxf(evv_noise_var, ev_odom_vel_var(2));
				} else {
					ev_data.velocity_var.setAll(evv_noise_var); // 设置为默认噪声方差
				}

				new_ev_odom = true; // 标记为新的外部视觉里程计数据
			}
		}

		// 检查有效的位置数据
		const Vector3f ev_odom_pos(ev_odom.position); // 外部视觉里程计位置
		const Vector3f ev_odom_pos_var(ev_odom.position_variance); // 外部视觉里程计位置方差

		if (ev_odom_pos.isAllFinite()) { // 检查位置是否有效
			bool position_frame_valid = false; // 位置框架有效标志

			// 根据位置框架类型设置位置框架
			switch (ev_odom.pose_frame) {
			case vehicle_odometry_s::POSE_FRAME_NED:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_NED; // 设置为NED框架
				position_frame_valid = true; // 标记为有效
				break;

			case vehicle_odometry_s::POSE_FRAME_FRD:
				ev_data.pos_frame = PositionFrame::LOCAL_FRAME_FRD; // 设置为FRD框架
				position_frame_valid = true; // 标记为有效
				break;
			}

			if (position_frame_valid) { // 如果位置框架有效
				ev_data.pos = ev_odom_pos; // 设置位置数据

				const float evp_noise_var = sq(_param_ekf2_evp_noise.get()); // 获取位置噪声方差

				// 从外部视觉数据或参数中获取位置测量误差
				if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_pos_var.isAllFinite()) {
					// 选择较大的噪声方差
					ev_data.position_var(0) = fmaxf(evp_noise_var, ev_odom_pos_var(0));
					ev_data.position_var(1) = fmaxf(evp_noise_var, ev_odom_pos_var(1));
					ev_data.position_var(2) = fmaxf(evp_noise_var, ev_odom_pos_var(2));
				} else {
					ev_data.position_var.setAll(evp_noise_var); // 设置为默认噪声方差
				}

				new_ev_odom = true; // 标记为新的外部视觉里程计数据
			}
		}

		// 检查有效的方向数据
		const Quatf ev_odom_q(ev_odom.q); // 外部视觉里程计四元数
		const Vector3f ev_odom_q_var(ev_odom.orientation_variance); // 外部视觉里程计方向方差
		const bool non_zero = (fabsf(ev_odom_q(0)) > 0.f) || (fabsf(ev_odom_q(1)) > 0.f)
				      || (fabsf(ev_odom_q(2)) > 0.f) || (fabsf(ev_odom_q(3)) > 0.f); // 检查四元数是否非零
		const float eps = 1e-5f; // 容差值
		const bool no_element_larger_than_one = (fabsf(ev_odom_q(0)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(1)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(2)) <= 1.f + eps)
							&& (fabsf(ev_odom_q(3)) <= 1.f + eps); // 检查四元数元素是否小于1
		const bool norm_in_tolerance = fabsf(1.f - ev_odom_q.norm()) <= eps; // 检查四元数范数是否在容差内

		const bool orientation_valid = ev_odom_q.isAllFinite() && non_zero && no_element_larger_than_one && norm_in_tolerance; // 检查方向数据是否有效

		if (orientation_valid) { // 如果方向数据有效
			ev_data.quat = ev_odom_q; // 设置四元数
			ev_data.quat.normalize(); // 归一化四元数

			// 从外部视觉数据或参数中获取方向测量误差
			const float eva_noise_var = sq(_param_ekf2_eva_noise.get());

			if ((_param_ekf2_ev_noise_md.get() == 0) && ev_odom_q_var.isAllFinite()) {
				// 选择较大的噪声方差
				ev_data.orientation_var(0) = fmaxf(eva_noise_var, ev_odom_q_var(0));
				ev_data.orientation_var(1) = fmaxf(eva_noise_var, ev_odom_q_var(1));
				ev_data.orientation_var(2) = fmaxf(eva_noise_var, ev_odom_q_var(2));
			} else {
				ev_data.orientation_var.setAll(eva_noise_var); // 设置为默认噪声方差
			}

			new_ev_odom = true; // 标记为新的外部视觉里程计数据
		}

		// 使用外部计算机的时间戳，当使用MAVROS时，时钟是同步的
		ev_data.time_us = ev_odom.timestamp_sample; // 设置时间戳
		ev_data.reset_counter = ev_odom.reset_counter; // 设置重置计数器
		ev_data.quality = ev_odom.quality; // 设置质量指标

		if (new_ev_odom)  {
			// 将外部视觉数据设置到EKF中
			_ekf.setExtVisionData(ev_data);
		}

		// 计算外部视觉里程计时间戳与EKF时间戳的相对关系
		ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_ev_odom; // 返回是否有新的外部视觉里程计数据
}
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
bool EKF2::UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF光流样本更新函数
	bool new_optical_flow = false; // 标记是否有新的光流数据
	vehicle_optical_flow_s optical_flow; // 定义光流数据结构

	// 更新光流数据
	if (_vehicle_optical_flow_sub.update(&optical_flow)) {

		const float dt = 1e-6f * (float)optical_flow.integration_timespan_us; // 将光流的积分时间转换为秒
		Vector2f flow_rate; // 定义光流速率
		Vector3f gyro_rate; // 定义陀螺仪速率

		if (dt > FLT_EPSILON) { // 如果时间间隔有效
			// 注意：EKF使用与光流传感器相反的符号约定。EKF假设正的视线速率
			// 是由图像绕传感器轴的右手旋转产生的。
			flow_rate = Vector2f(-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]) / dt; // 计算光流速率
			gyro_rate = Vector3f(-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]) / dt; // 计算陀螺仪速率

		} else if (optical_flow.quality == 0) { // 如果光流质量为0
			// 处理SITL和PX4Flow的特殊情况，当质量为0时dt被强制为零
			flow_rate.zero(); // 将光流速率置为零
			gyro_rate.zero(); // 将陀螺仪速率置为零
		}

		// 创建光流样本数据结构
		flowSample flow {
			.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2, // 将时间戳校正为积分区间的中点
			.flow_rate = flow_rate, // 设置光流速率
			.gyro_rate = gyro_rate, // 设置陀螺仪速率
			.quality = optical_flow.quality // 设置光流质量
		};

		// 检查光流数据是否有效
		if (Vector2f(optical_flow.pixel_flow).isAllFinite() && optical_flow.integration_timespan_us < 1e6) {

			// 保存光流传感器报告的传感器限制
			_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						     optical_flow.max_ground_distance);

			_ekf.setOpticalFlowData(flow); // 将光流数据设置到EKF中

			new_optical_flow = true; // 标记为有新的光流数据
		}

#if defined(CONFIG_EKF2_RANGE_FINDER)

		// 如果距离传感器不可用，则使用光流距离作为范围样本
		if (PX4_ISFINITE(optical_flow.distance_m) && (ekf2_timestamps.timestamp > _last_range_sensor_update + 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f; // 计算光流质量百分比

			// 创建范围样本数据结构
			estimator::sensor::rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample, // 设置时间戳
				.rng = optical_flow.distance_m, // 设置距离
				.quality = quality, // 设置质量
			};
			_ekf.setRangeData(range_sample); // 将范围数据设置到EKF中

			// 设置传感器限制
			_ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

#endif // CONFIG_EKF2_RANGE_FINDER

		// 计算光流时间戳与EKF时间戳的相对关系
		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	return new_optical_flow; // 返回是否有新的光流数据
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
// 更新GPS样本的函数，参数为ekf2时间戳
void EKF2::UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF GPS消息结构体，用于存储车辆的GPS位置信息
	sensor_gps_s vehicle_gps_position;

	// 更新车辆GPS位置数据
	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

		// 定义一个三维向量，用于存储NED坐标系下的速度
		Vector3f vel_ned;

		// 检查GPS速度数据是否有效
		if (vehicle_gps_position.vel_ned_valid) {
			// 如果有效，则将GPS速度数据转换为Vector3f类型
			vel_ned = Vector3f(vehicle_gps_position.vel_n_m_s, // 北向速度（米/秒）
					   vehicle_gps_position.vel_e_m_s, // 东向速度（米/秒）
					   vehicle_gps_position.vel_d_m_s); // 垂直速度（米/秒）

		} else {
			// 如果速度数据无效，则返回，TODO: 需要更改并设置为NAN
			return;
		}

		// 检查GPS航向偏移参数是否大于0
		if (fabsf(_param_ekf2_gps_yaw_off.get()) > 0.f) {
			// 如果航向偏移未定义且航向有效，则应用偏移
			if (!PX4_ISFINITE(vehicle_gps_position.heading_offset) && PX4_ISFINITE(vehicle_gps_position.heading)) {
				// 计算航向偏移并应用
				float yaw_offset = matrix::wrap_pi(math::radians(_param_ekf2_gps_yaw_off.get())); // 将偏移角度转换为弧度并限制在[-π, π]范围内
				vehicle_gps_position.heading_offset = yaw_offset; // 设置航向偏移
				vehicle_gps_position.heading = matrix::wrap_pi(vehicle_gps_position.heading - yaw_offset); // 更新航向
			}
		}

		// 获取海平面高度（AMSL）和椭球高度
		const float altitude_amsl = static_cast<float>(vehicle_gps_position.altitude_msl_m); // 海平面高度（米）
		const float altitude_ellipsoid = static_cast<float>(vehicle_gps_position.altitude_ellipsoid_m); // 椭球高度（米）

		// 创建GNSS样本数据结构，存储GPS数据
		gnssSample gnss_sample{
			.time_us = vehicle_gps_position.timestamp, // 时间戳（微秒）
			.lat = vehicle_gps_position.latitude_deg, // 纬度（度）
			.lon = vehicle_gps_position.longitude_deg, // 经度（度）
			.alt = altitude_amsl, // 海平面高度（米）
			.vel = vel_ned, // NED坐标系下的速度
			.hacc = vehicle_gps_position.eph, // 水平精度（米）
			.vacc = vehicle_gps_position.epv, // 垂直精度（米）
			.sacc = vehicle_gps_position.s_variance_m_s, // 速度方差（米/秒）
			.fix_type = vehicle_gps_position.fix_type, // GPS定位类型
			.nsats = vehicle_gps_position.satellites_used, // 使用的卫星数量
			.pdop = sqrtf(vehicle_gps_position.hdop * vehicle_gps_position.hdop // 计算位置精度因子（PDOP）
				      + vehicle_gps_position.vdop * vehicle_gps_position.vdop),
			.yaw = vehicle_gps_position.heading, // 航向（度），TODO: 需要移动到不同的消息中
			.yaw_acc = vehicle_gps_position.heading_accuracy, // 航向精度（度）
			.yaw_offset = vehicle_gps_position.heading_offset, // 航向偏移（度）
			.spoofed = vehicle_gps_position.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE, // 检查是否存在欺骗状态
		};

		// 将GNSS样本数据设置到EKF中
		_ekf.setGpsData(gnss_sample);

		// 计算地球椭球高度与海平面高度的差值
		const float geoid_height = altitude_ellipsoid - altitude_amsl;

		// 检查上次更新的地球椭球高度时间戳是否为0
		if (_last_geoid_height_update_us == 0) {
			_geoid_height_lpf.reset(geoid_height); // 重置低通滤波器状态
			_last_geoid_height_update_us = gnss_sample.time_us; // 更新最后的时间戳

		} else if (gnss_sample.time_us > _last_geoid_height_update_us) {
			// 如果当前时间戳大于最后更新时间戳，则更新低通滤波器
			const float dt = 1e-6f * (gnss_sample.time_us - _last_geoid_height_update_us); // 计算时间间隔（秒）
			_geoid_height_lpf.setParameters(dt, kGeoidHeightLpfTimeConstant); // 设置低通滤波器参数
			_geoid_height_lpf.update(geoid_height); // 更新低通滤波器状态
			_last_geoid_height_update_us = gnss_sample.time_us; // 更新最后的时间戳
		}

	}
}
// 将椭球高度转换为海平面高度（AMSL）
// 输入参数：
// ellipsoid_alt - 椭球高度（米）
// 返回值：
// 返回海平面高度（AMSL），通过从椭球高度中减去地球表面高度的低通滤波器状态
float EKF2::altEllipsoidToAmsl(float ellipsoid_alt) const
{
	return ellipsoid_alt - _geoid_height_lpf.getState(); // 从椭球高度中减去地球表面高度
}

// 将海平面高度（AMSL）转换为椭球高度
// 输入参数：
// amsl_alt - 海平面高度（米）
// 返回值：
// 返回椭球高度，通过在海平面高度上加上地球表面高度的低通滤波器状态
float EKF2::altAmslToEllipsoid(float amsl_alt) const
{
	return amsl_alt + _geoid_height_lpf.getState(); // 在海平面高度上加上地球表面高度
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
// 更新磁力计样本的函数，参数为ekf2时间戳
void EKF2::UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps)
{
	vehicle_magnetometer_s magnetometer; // 声明磁力计数据结构

	// 更新磁力计数据
	if (_magnetometer_sub.update(&magnetometer)) {

		bool reset = false; // 重置标志

		// 检查磁力计是否发生变化
		if (magnetometer.device_id != _device_id_mag) {
			if (_device_id_mag != 0) {
				// 输出磁力计传感器ID变化的调试信息
				PX4_DEBUG("%d - mag sensor ID changed %" PRIu32 " -> %" PRIu32, _instance, _device_id_mag, magnetometer.device_id);
			}

			reset = true; // 设置重置标志

		} else if (magnetometer.calibration_count != _mag_calibration_count) {
			// 如果现有的校准发生变化，重置保存的磁偏差
			PX4_DEBUG("%d - mag %" PRIu32 " calibration updated, resetting bias", _instance, _device_id_mag);
			reset = true; // 设置重置标志
		}

		if (reset) {
			_device_id_mag = magnetometer.device_id; // 更新磁力计设备ID
			_mag_calibration_count = magnetometer.calibration_count; // 更新校准计数

			// 重置磁力计偏差学习
			_mag_cal = {}; // 清空磁力计校准数据
		}

		// 设置磁力计数据到EKF中
		_ekf.setMagData(magSample{magnetometer.timestamp_sample, Vector3f{magnetometer.magnetometer_ga}, reset});

		// 计算磁力计时间戳与EKF时间戳的相对关系
		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
// 更新距离传感器样本的函数，参数为ekf2时间戳
void EKF2::UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps)
{
	distance_sensor_s distance_sensor; // 声明距离传感器数据结构

	// 检查是否选择了距离传感器
	if (_distance_sensor_selected < 0) {

		// 仅考虑在过去0.1秒内更新的距离传感器
		const hrt_abstime timestamp_stale = math::max(ekf2_timestamps.timestamp, 100_ms) - 100_ms;

		// 检查距离传感器是否已被广告
		if (_distance_sensor_subs.advertised()) {
			for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

				// 更新距离传感器数据
				if (_distance_sensor_subs[i].update(&distance_sensor)) {
					// 仅使用具有正确方向的第一个实例
					if ((distance_sensor.timestamp != 0) && (distance_sensor.timestamp > timestamp_stale)
					    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

						int ndist = orb_group_count(ORB_ID(distance_sensor)); // 计算距离传感器的数量

						if (ndist > 1) {
							PX4_INFO("%d - selected distance_sensor:%d (%d advertised)", _instance, i, ndist);
						}

						_distance_sensor_selected = i; // 选择当前的距离传感器
						_last_range_sensor_update = distance_sensor.timestamp; // 更新最后的距离传感器更新时间戳
						break; // 退出循环
					}
				}
			}
		}
	}

	// 如果选择了距离传感器并且更新了数据
	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// EKF距离样本
		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
			estimator::sensor::rangeSample range_sample {
				.time_us = distance_sensor.timestamp, // 时间戳（微秒）
				.rng = distance_sensor.current_distance, // 当前距离（米）
				.quality = distance_sensor.signal_quality, // 信号质量
			};
			_ekf.setRangeData(range_sample); // 设置距离数据到EKF中

			// 保存距离传感器报告的传感器限制
			_ekf.set_rangefinder_limits(distance_sensor.min_distance, distance_sensor.max_distance);

			_last_range_sensor_update = ekf2_timestamps.timestamp; // 更新最后的距离传感器更新时间戳
		}

		// 计算距离传感器时间戳与EKF时间戳的相对关系
		ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)distance_sensor.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}

	// 如果最后的距离传感器更新时间超过1秒，则强制重新选择
	if (_last_range_sensor_update < ekf2_timestamps.timestamp - 1_s) {
		_distance_sensor_selected = -1; // 重置选择
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

void EKF2::UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps)
{
	// EKF system flags
	if (_status_sub.updated() || _vehicle_land_detected_sub.updated()) {

		systemFlagUpdate flags{};
		flags.time_us = ekf2_timestamps.timestamp;

		// vehicle_status
		vehicle_status_s vehicle_status;

		if (_status_sub.copy(&vehicle_status)
		    && (ekf2_timestamps.timestamp < vehicle_status.timestamp + 3_s)) {

			// initially set in_air from arming_state (will be overridden if land detector is available)
			flags.in_air = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			flags.is_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);

#if defined(CONFIG_EKF2_SIDESLIP)

			if (vehicle_status.is_vtol_tailsitter && _params->beta_fusion_enabled) {
				PX4_WARN("Disable EKF beta fusion as unsupported for tailsitter");
				_param_ekf2_fuse_beta.set(0);
				_param_ekf2_fuse_beta.commit_no_notification();
			}

#endif // CONFIG_EKF2_SIDESLIP
		}
		// vehicle_land_detected 结构体用于存储车辆着陆状态信息
		vehicle_land_detected_s vehicle_land_detected;

		// 从车辆着陆检测订阅中复制数据到 vehicle_land_detected 结构体
		// 检查时间戳是否在最近3秒内有效
		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
		    && (ekf2_timestamps.timestamp < vehicle_land_detected.timestamp + 3_s)) {

			// 更新系统标志，表示车辆是否静止
			flags.at_rest = vehicle_land_detected.at_rest; // at_rest 表示车辆是否静止

			// 更新系统标志，表示车辆是否在空中
			flags.in_air = !vehicle_land_detected.landed; // in_air 表示车辆是否在空中，若已着陆则为 false

			// 更新系统标志，表示是否存在地面效应
			flags.gnd_effect = vehicle_land_detected.in_ground_effect; // gnd_effect 表示是否处于地面效应中
		}

		// launch_detection_status 结构体用于存储发射检测状态信息
		launch_detection_status_s launch_detection_status;

		// 从发射检测状态订阅中复制数据到 launch_detection_status 结构体
		// 检查时间戳是否在最近3秒内有效
		if (_launch_detection_status_sub.copy(&launch_detection_status)
		    && (ekf2_timestamps.timestamp < launch_detection_status.timestamp + 3_s)) {

			// 更新系统标志，表示是否处于等待发射状态
			flags.constant_pos = (launch_detection_status.launch_detection_state ==
					      launch_detection_status_s::STATE_WAITING_FOR_LAUNCH); // constant_pos 表示是否处于等待发射状态
		}

		// 将更新后的系统标志数据设置到 EKF 中
		_ekf.setSystemFlagData(flags); // 将系统标志数据传递给 EKF
	}
}

void EKF2::UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			     const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid)
{
	// 在起飞时重置现有的校准数据
	if (!_ekf.control_status_prev_flags().in_air && _ekf.control_status_flags().in_air) {
		cal = {}; // 重置校准数据
	}

	// 检查加速度计偏置值学习的条件是否满足
	// EKF 必须在正确的模式下运行，并且没有滤波器故障
	static constexpr float max_var_allowed = 1e-3f; // 允许的最大方差
	static constexpr float max_var_ratio = 1e2f; // 最大方差比率

	// 验证偏置是否有效，且方差满足条件
	const bool valid = bias_valid
			   && (bias_variance.max() < max_var_allowed) // 检查方差的最大值是否小于允许的最大方差
			   && (bias_variance.max() < max_var_ratio * bias_variance.min()); // 检查方差的最大值是否小于最小值的最大方差比率

	if (valid && learning_valid) {
		// 当所有检查一致通过且偏置变化不超过限制的10%时，认为偏置估计稳定
		const float bias_change_limit = 0.1f * bias_limit; // 偏置变化限制

		// 检查当前偏置与校准偏置的差异是否在允许范围内
		if (!(cal.bias - bias).longerThan(bias_change_limit)) {
			if (cal.last_us != 0) {
				cal.total_time_us += timestamp - cal.last_us; // 累加校准时间
			}

			// 如果累计校准时间超过10秒，则标记为可用
			if (cal.total_time_us > 10_s) {
				cal.cal_available = true; // 设置校准可用标志
			}

		} else {
			// 如果偏置变化超出限制，重置累计时间并更新偏置
			cal.total_time_us = 0; // 重置累计时间
			cal.bias = bias; // 更新当前偏置
			cal.cal_available = false; // 设置校准不可用标志
		}

		// 更新最后一次校准的时间戳
		cal.last_us = timestamp;

	} else {
		// 学习偏置的条件不满足，重置时间戳
		// 但保留累计的校准时间
		cal.last_us = 0; // 重置最后一次校准时间戳

		// 如果偏置无效且累计时间不为零，重置校准数据
		if (!valid && (cal.total_time_us != 0)) {
			// 如果发生滤波器故障，假设之前的学习无效，不计入总学习时间
			cal = {}; // 重置校准数据
		}
	}
}

void EKF2::UpdateAccelCalibration(const hrt_abstime &timestamp)
{
	// EKF 必须在正确的模式下运行，并且没有滤波器故障
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::AccelBias)) // 检查是否启用加速度计偏置控制
				&& _ekf.control_status_flags().tilt_align // 检查是否完成倾斜对齐
				&& (_ekf.fault_status().value == 0) // 检查是否没有滤波器故障
				&& !_ekf.fault_status_flags().bad_acc_clipping // 检查是否没有加速度剪切故障
				&& !_ekf.fault_status_flags().bad_acc_vertical; // 检查是否没有垂直加速度故障

	// 学习条件有效性
	const bool learning_valid = bias_valid && !_ekf.accel_bias_inhibited(); // 检查是否可以学习加速度计偏置

	// 更新加速度计校准
	UpdateCalibration(timestamp, _accel_cal, _ekf.getAccelBias(), _ekf.getAccelBiasVariance(), _ekf.getAccelBiasLimit(),
			  bias_valid, learning_valid); // 调用更新校准函数
}

void EKF2::UpdateGyroCalibration(const hrt_abstime &timestamp)
{
	// EKF在正确的模式下运行，并且没有滤波器故障
	const bool bias_valid = (_param_ekf2_imu_ctrl.get() & static_cast<int32_t>(ImuCtrl::GyroBias)) // 检查是否启用陀螺仪偏置控制
				&& _ekf.control_status_flags().tilt_align // 检查是否完成倾斜对齐
				&& (_ekf.fault_status().value == 0); // 检查是否没有滤波器故障

	const bool learning_valid = bias_valid && !_ekf.gyro_bias_inhibited(); // 检查是否可以学习陀螺仪偏置

	// 调用更新校准函数，传入时间戳、陀螺仪校准数据、陀螺仪偏置、偏置方差、偏置限制、偏置有效性和学习有效性
	UpdateCalibration(timestamp, _gyro_cal, _ekf.getGyroBias(), _ekf.getGyroBiasVariance(), _ekf.getGyroBiasLimit(),
			  bias_valid, learning_valid);
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EKF2::UpdateMagCalibration(const hrt_abstime &timestamp)
{
	const Vector3f mag_bias = _ekf.getMagBias(); // 获取当前磁力计偏置
	const Vector3f mag_bias_var = _ekf.getMagBiasVariance(); // 获取当前磁力计偏置方差

	// 检查偏置有效性：没有滤波器故障、完成航向对齐、偏置方差大于0且不超过0.02
	const bool bias_valid = (_ekf.fault_status().value == 0)
				&& _ekf.control_status_flags().yaw_align // 检查是否完成航向对齐
				&& mag_bias_var.longerThan(0.f) && !mag_bias_var.longerThan(0.02f); // 检查偏置方差

	const bool learning_valid = bias_valid && _ekf.control_status_flags().mag; // 检查是否可以学习磁力计偏置

	// 调用更新校准函数，传入时间戳、磁力计校准数据、磁力计偏置、偏置方差、偏置限制、偏置有效性和学习有效性
	UpdateCalibration(timestamp, _mag_cal, mag_bias, mag_bias_var, _ekf.getMagBiasLimit(), bias_valid, learning_valid);

	// 更新存储的磁偏角值
	if (!_mag_decl_saved) { // 如果磁偏角尚未保存
		float declination_deg; // 磁偏角度数

		if (_ekf.get_mag_decl_deg(declination_deg)) { // 获取磁偏角
			_param_ekf2_mag_decl.update(); // 更新磁偏角参数

			// 检查磁偏角是否有效且与当前参数差异大于0.1
			if (PX4_ISFINITE(declination_deg) && (fabsf(declination_deg - _param_ekf2_mag_decl.get()) > 0.1f)) {
				_param_ekf2_mag_decl.set(declination_deg); // 设置新的磁偏角
				_param_ekf2_mag_decl.commit_no_notification(); // 提交更改但不通知
			}

			_mag_decl_saved = true; // 标记磁偏角已保存
		}
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

int EKF2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 返回未知命令的使用说明
}

int EKF2::task_spawn(int argc, char *argv[])
{
	bool success = false; // 任务成功标志
	bool replay_mode = false; // 回放模式标志

	if (argc > 1 && !strcmp(argv[1], "-r")) { // 检查是否启用回放模式
		PX4_INFO("replay mode enabled"); // 输出回放模式启用信息
		replay_mode = true; // 设置回放模式标志
	}

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	bool multi_mode = false; // 多实例模式标志
	int32_t imu_instances = 0; // IMU实例计数
	int32_t mag_instances = 0; // 磁力计实例计数

	int32_t sens_imu_mode = 1; // IMU模式参数
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode); // 获取IMU模式参数

	if (sens_imu_mode == 0) { // 如果IMU模式为0
		// EKF选择器要求SENS_IMU_MODE = 0
		multi_mode = true; // 启用多实例模式

		// 获取支持的IMU数量（1 - MAX_NUM_IMUS）
		param_get(param_find("EKF2_MULTI_IMU"), &imu_instances); // 获取IMU实例数量

		if (imu_instances < 1 || imu_instances > MAX_NUM_IMUS) { // 检查IMU实例数量是否在有效范围内
			const int32_t imu_instances_limited = math::constrain(imu_instances, static_cast<int32_t>(1),
							      static_cast<int32_t>(MAX_NUM_IMUS)); // 限制IMU实例数量
			PX4_WARN("EKF2_MULTI_IMU limited %" PRId32 " -> %" PRId32, imu_instances, imu_instances_limited); // 输出警告信息
			param_set_no_notification(param_find("EKF2_MULTI_IMU"), &imu_instances_limited); // 设置限制后的IMU实例数量
			imu_instances = imu_instances_limited; // 更新IMU实例数量
		}

#if defined(CONFIG_EKF2_MAGNETOMETER)
		int32_t sens_mag_mode = 1; // 磁力计模式参数
		const param_t param_sens_mag_mode = param_find("SENS_MAG_MODE"); // 获取磁力计模式参数
		param_get(param_sens_mag_mode, &sens_mag_mode); // 获取磁力计模式参数值

		if (sens_mag_mode == 0) { // 如果磁力计模式为0
			const param_t param_ekf2_mult_mag = param_find("EKF2_MULTI_MAG"); // 获取多磁力计参数
			param_get(param_ekf2_mult_mag, &mag_instances); // 获取磁力计实例数量

			// 支持的磁力计数量（1 - MAX_NUM_MAGS）
			if (mag_instances > MAX_NUM_MAGS) { // 检查磁力计实例数量是否在有效范围内
				const int32_t mag_instances_limited = math::constrain(mag_instances, static_cast<int32_t>(1),
								      static_cast<int32_t>(MAX_NUM_MAGS)); // 限制磁力计实例数量
				PX4_WARN("EKF2_MULTI_MAG limited %" PRId32 " -> %" PRId32, mag_instances, mag_instances_limited); // 输出警告信息
				param_set_no_notification(param_ekf2_mult_mag, &mag_instances_limited); // 设置限制后的磁力计实例数量
				mag_instances = mag_instances_limited; // 更新磁力计实例数量

			} else if (mag_instances <= 1) { // 如果磁力计实例数量小于等于1
				// 在传感器集线器级别正确禁用多磁力计
				PX4_WARN("EKF2_MULTI_MAG disabled, resetting SENS_MAG_MODE"); // 输出禁用多磁力计的警告信息

				// 在传感器级别重新启用
				sens_mag_mode = 1; // 设置磁力计模式为1
				param_set(param_sens_mag_mode, &sens_mag_mode); // 更新磁力计模式参数

				mag_instances = 1; // 设置磁力计实例数量为1
			}

		} else {
			mag_instances = 1; // 如果磁力计模式不为0，设置磁力计实例数量为1
		}

#endif // CONFIG_EKF2_MAGNETOMETER
	}
	// 如果处于多实例模式且不在重放模式下
	if (multi_mode && !replay_mode) {
		// 如果EKF2选择器尚未运行，则启动EKF2选择器
		if (_ekf2_selector.load() == nullptr) {
			EKF2Selector *inst = new EKF2Selector(); // 创建EKF2选择器实例

			if (inst) {
				_ekf2_selector.store(inst); // 存储选择器实例

			} else {
				PX4_ERR("创建EKF2选择器失败"); // 输出错误信息
				return PX4_ERROR; // 返回错误
			}
		}

		const hrt_abstime time_started = hrt_absolute_time(); // 记录开始时间
		const int multi_instances = math::min(imu_instances * mag_instances, static_cast<int32_t>(EKF2_MAX_INSTANCES)); // 计算可用的多实例数量
		int multi_instances_allocated = 0; // 已分配的实例数量

		// 分配EKF2实例，直到找到所有实例或进入arming状态
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)}; // 订阅车辆状态数据

		bool ekf2_instance_created[MAX_NUM_IMUS][MAX_NUM_MAGS] {}; // 记录IMU和磁力计的实例创建状态

		// 循环直到分配的实例数量达到上限或进入arming状态
		while ((multi_instances_allocated < multi_instances)
		       && (vehicle_status_sub.get().arming_state != vehicle_status_s::ARMING_STATE_ARMED) // 检查是否处于arming状态
		       && ((hrt_elapsed_time(&time_started) < 30_s) // 检查是否在30秒内
			   || (vehicle_status_sub.get().hil_state == vehicle_status_s::HIL_STATE_ON))) { // 或者HIL状态为ON

			vehicle_status_sub.update(); // 更新车辆状态数据

			for (uint8_t mag = 0; mag < mag_instances; mag++) { // 遍历每个磁力计实例
				uORB::SubscriptionData<vehicle_magnetometer_s> vehicle_mag_sub{ORB_ID(vehicle_magnetometer), mag}; // 订阅当前磁力计数据

				for (uint8_t imu = 0; imu < imu_instances; imu++) { // 遍历每个IMU实例

					uORB::SubscriptionData<vehicle_imu_s> vehicle_imu_sub{ORB_ID(vehicle_imu), imu}; // 订阅当前IMU数据
					vehicle_mag_sub.update(); // 更新磁力计数据

					// 磁力计和IMU数据必须有效，初始时可以忽略第一个磁力计
					if ((vehicle_mag_sub.advertised() || mag == 0) && (vehicle_imu_sub.advertised())) {

						if (!ekf2_instance_created[imu][mag]) { // 如果当前IMU和磁力计组合尚未创建实例
							EKF2 *ekf2_inst = new EKF2(true, px4::ins_instance_to_wq(imu), false); // 创建EKF2实例

							if (ekf2_inst && ekf2_inst->multi_init(imu, mag)) { // 如果实例创建成功并初始化成功
								int actual_instance = ekf2_inst->instance(); // 获取实际实例编号以匹配uORB实例编号

								if ((actual_instance >= 0) && (_objects[actual_instance].load() == nullptr)) { // 检查实例编号有效且未被占用
									_objects[actual_instance].store(ekf2_inst); // 存储EKF2实例
									success = true; // 标记成功
									multi_instances_allocated++; // 增加已分配实例数量
									ekf2_instance_created[imu][mag] = true; // 标记当前IMU和磁力计组合已创建实例

									PX4_DEBUG("启动实例 %d, IMU:%" PRIu8 " (%" PRIu32 "), MAG:%" PRIu8 " (%" PRIu32 ")", actual_instance,
										  imu, vehicle_imu_sub.get().accel_device_id, // 输出IMU设备ID
										  mag, vehicle_mag_sub.get().device_id); // 输出磁力计设备ID

									_ekf2_selector.load()->ScheduleNow(); // 调度选择器立即执行

								} else {
									PX4_ERR("实例编号问题，实例: %d", actual_instance); // 输出实例编号错误信息
									delete ekf2_inst; // 删除创建失败的实例
									break; // 退出循环
								}

							} else {
								PX4_ERR("分配和初始化失败，IMU: %" PRIu8 " 磁力计:%" PRIu8, imu, mag); // 输出分配和初始化失败信息
								px4_usleep(100000); // 暂停100毫秒
								break; // 退出循环
							}
						}

					} else {
						px4_usleep(1000); // 给传感器额外时间启动，暂停1毫秒
						break; // 退出循环
					}
				}
			}

			if (multi_instances_allocated < multi_instances) { // 如果已分配的实例数量小于目标数量
				px4_usleep(10000); // 暂停10毫秒
			}
		}

	} else

#endif // CONFIG_EKF2_MULTI_INSTANCE

	{
	// 启动常规EKF2实例
	// 创建一个新的EKF2实例，参数为false表示不是多实例，使用INS0配置，replay_mode表示是否为重放模式
	        EKF2 *ekf2_inst = new EKF2(false, px4::wq_configurations::INS0, replay_mode);

		if (ekf2_inst) {
			_objects[0].store(ekf2_inst);
			ekf2_inst->ScheduleNow();
			success = true;
		}
	}

	return success ? PX4_OK : PX4_ERROR;
}

int EKF2::print_usage(const char *reason)
{
	// 如果提供了原因，则输出警告信息
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	// 打印模块描述信息
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
使用扩展卡尔曼滤波器的姿态和位置估计器。适用于多旋翼和固定翼飞行器。

文档可以在 [ECL/EKF 概述与调优](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html) 页面找到。

ekf2可以在重放模式下启动（`-r`）：在此模式下，它不访问系统时间，而仅使用传感器主题中的时间戳。

)DESCR_STR");

	// 打印模块名称和类型
	PRINT_MODULE_USAGE_NAME("ekf2", "估计器");
	// 打印启动命令
	PRINT_MODULE_USAGE_COMMAND("start");
	// 打印重放模式参数
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "启用重放模式", true);
	// 打印停止命令
	PRINT_MODULE_USAGE_COMMAND("stop");
	// 打印状态命令描述
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "打印状态信息");
#if defined(CONFIG_EKF2_VERBOSE_STATUS)
	// 打印详细状态参数
	PRINT_MODULE_USAGE_ARG("-v", "详细模式（打印所有状态和完整协方差矩阵）", true);
#endif // CONFIG_EKF2_VERBOSE_STATUS
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	// 打印选择实例命令描述
	PRINT_MODULE_USAGE_COMMAND_DESCR("select_instance", "请求切换到新的估计器实例");
	// 打印指定实例参数
	PRINT_MODULE_USAGE_ARG("<instance>", "指定所需的估计器实例", false);
#endif // CONFIG_EKF2_MULTI_INSTANCE
	return 0;
}

extern "C" __EXPORT int ekf2_main(int argc, char *argv[])
{
	// 检查命令行参数数量，如果参数数量小于等于1或第一个参数为"-h"，则打印使用说明
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return EKF2::print_usage();
	}

	// 检查启动命令
	if (strcmp(argv[1], "start") == 0) {
		int ret = 0; // 返回值初始化
		EKF2::lock_module(); // 锁定模块以防止并发访问

		// 启动任务并获取返回值
		ret = EKF2::task_spawn(argc - 1, argv + 1);

		// 检查任务启动是否成功
		if (ret < 0) {
			PX4_ERR("启动失败 (%i)", ret); // 输出错误信息
		}

		EKF2::unlock_module(); // 解锁模块
		return ret; // 返回启动结果

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	} else if (strcmp(argv[1], "select_instance") == 0) {

		// 尝试锁定模块
		if (EKF2::trylock_module()) {
			// 检查多实例选择器是否加载
			if (_ekf2_selector.load()) {
				// 检查参数数量是否大于2
				if (argc > 2) {
					int instance = atoi(argv[2]); // 将参数转换为整数
					_ekf2_selector.load()->RequestInstance(instance); // 请求切换到指定实例
				} else {
					EKF2::unlock_module(); // 解锁模块
					return EKF2::print_usage("需要实例参数"); // 返回使用说明
				}

			} else {
				PX4_ERR("多EKF未激活，无法选择实例"); // 输出错误信息
			}

			EKF2::unlock_module(); // 解锁模块

		} else {
			PX4_WARN("模块被锁定，请稍后重试"); // 输出警告信息
		}

		return 0; // 返回0表示成功
#endif // CONFIG_EKF2_MULTI_INSTANCE
	} else if (strcmp(argv[1], "status") == 0) {
		// 尝试锁定模块
		if (EKF2::trylock_module()) {
#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			// 检查多实例选择器是否加载
			if (_ekf2_selector.load()) {
				_ekf2_selector.load()->PrintStatus(); // 打印状态信息
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			bool verbose_status = false; // 初始化详细状态标志

#if defined(CONFIG_EKF2_VERBOSE_STATUS)
			// 检查参数是否为详细模式
			if (argc > 2 && (strcmp(argv[2], "-v") == 0)) {
				verbose_status = true; // 设置为详细模式
			}
#endif // CONFIG_EKF2_VERBOSE_STATUS

			// 遍历所有EKF2实例并打印状态
			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n"); // 输出换行
					_objects[i].load()->print_status(verbose_status); // 打印实例状态
				}
			}

			EKF2::unlock_module(); // 解锁模块

		} else {
			PX4_WARN("模块被锁定，请稍后重试"); // 输出警告信息
		}

		return 0; // 返回0表示成功

	} else if (strcmp(argv[1], "stop") == 0) {
		EKF2::lock_module(); // 锁定模块以防止并发访问

		// 检查参数数量
		if (argc > 2) {
			int instance = atoi(argv[2]); // 将参数转换为整数

			// 检查实例编号是否有效
			if (instance >= 0 && instance < EKF2_MAX_INSTANCES) {
				PX4_INFO("停止实例 %d", instance); // 输出停止实例信息
				EKF2 *inst = _objects[instance].load(); // 加载指定实例

				if (inst) {
					inst->request_stop(); // 请求停止实例
					px4_usleep(20000); // 暂停20毫秒
					delete inst; // 删除实例
					_objects[instance].store(nullptr); // 清空对象存储
				}
			} else {
				PX4_ERR("无效实例 %d", instance); // 输出无效实例错误信息
			}

		} else {
			// 否则停止所有实例
			bool was_running = false; // 初始化运行状态标志

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
			// 检查多实例选择器是否加载
			if (_ekf2_selector.load()) {
				PX4_INFO("停止ekf2选择器"); // 输出停止选择器信息
				_ekf2_selector.load()->Stop(); // 停止选择器
				delete _ekf2_selector.load(); // 删除选择器
				_ekf2_selector.store(nullptr); // 清空选择器存储
				was_running = true; // 标记为已运行
			}
#endif // CONFIG_EKF2_MULTI_INSTANCE

			// 遍历所有EKF2实例并停止
			for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
				EKF2 *inst = _objects[i].load(); // 加载实例

				if (inst) {
					PX4_INFO("停止ekf2实例 %d", i); // 输出停止实例信息
					was_running = true; // 标记为已运行
					inst->request_stop(); // 请求停止实例
					px4_usleep(20000); // 暂停20毫秒
					delete inst; // 删除实例
					_objects[i].store(nullptr); // 清空对象存储
				}
			}

			// 检查是否有实例在运行
			if (!was_running) {
				PX4_WARN("未运行"); // 输出未运行警告信息
			}
		}

		EKF2::unlock_module(); // 解锁模块
		return PX4_OK; // 返回成功
	}

	EKF2::lock_module(); // Lock here, as the method could access _object.
	int ret = EKF2::custom_command(argc - 1, argv + 1);
	EKF2::unlock_module();

	return ret;
}
