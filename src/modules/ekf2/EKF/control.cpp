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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

/*
 * 融合控制总线（按状态与传感器质量门控各观测源）
 * 职责：
 * - 监控倾斜对齐并在满足不确定度阈值后置位；
 * - 按参数与状态，调用各观测源融合控制（磁/GPS/光流/气压/侧滑/重力/外视觉/辅助速度等）；
 * - 执行零创新航向更新与零速度更新，必要时施加假位置/高度约束；
 * - 更新死算状态（当不再融合直接约束速度漂移的观测时）。
 */
void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;
	_state_reset_count_prev = _state_reset_status.reset_count;

	if (_system_flag_buffer) {
		systemFlagUpdate system_flags_delayed;

		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {

			set_vehicle_at_rest(system_flags_delayed.at_rest);
			set_in_air_status(system_flags_delayed.in_air);

			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);
			set_in_transition_to_fw(system_flags_delayed.in_transition_to_fw);

			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}

			set_constant_pos(system_flags_delayed.constant_pos);
		}
	}

	// 倾斜对齐监控：方差小于 ~3° 时置位 tilt_align
	if (!_control_status.flags.tilt_align) {
		// 倾斜对齐阶段：监控姿态方差
		// - 当倾斜方差下降到 ~3° 不确定度时认为对齐完成；
		// - 同时报告当前高度源与 IMU/观测缓冲长度，便于调试时序。
		if (getTiltVariance() < sq(math::radians(3.f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = "unknown";

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";
			}

			ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
				 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);

			ECL_DEBUG("tilt aligned, roll: %.3f, pitch %.3f, yaw: %.3f",
				  (double)matrix::Eulerf(_state.quat_nominal).phi(),
				  (double)matrix::Eulerf(_state.quat_nominal).theta(),
				  (double)matrix::Eulerf(_state.quat_nominal).psi()
				 );
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁融合控制（航向/磁场状态）：含磁偏置与地磁一致性检查
	controlMagFusion(imu_delayed);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流融合控制（水平速度辅助，短程/室内场景）：考虑陀螺补偿与距离/质量门限
	controlOpticalFlowFusion(imu_delayed);
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
	// GPS 融合控制（位置/速度/航向，含质量/延时门控）：
	// - 创新检验通过后才融合；
	// - PPS/延时补偿与观测缓冲对齐。
	controlGpsFusion(imu_delayed);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.update(*this, imu_delayed);
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#if defined(CONFIG_EKF2_AIRSPEED)
	// 气压数据融合（高度源与偏置估计）：含地效补偿与偏置发布
	controlAirDataFusion(imu_delayed);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 侧滑融合（阻力模型，提升侧风与航迹稳定）：基于机体气动模型估计侧滑误差
	controlBetaFusion(imu_delayed);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 阻力融合（多旋翼特有，使用空气密度与机体速度）：联动 air_density 与机体速度估计
	controlDragFusion(imu_delayed);
#endif // CONFIG_EKF2_DRAG_FUSION

	// 高度融合（在不同高度源之间切换与一致性检查）：自动选择 Baro/GPS/Range/EV
	controlHeightFusion(imu_delayed);

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力融合（基于加速度统计提升姿态稳定）：在外部航向弱时强化姿态约束
	controlGravityFusion(imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// Additional data odometry data from an external estimator can be fused.
	// 外部视觉融合（位置/姿态，室内/拥挤环境）：处理外源时戳与噪声模型
	controlExternalVisionFusion(imu_delayed);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// Additional horizontal velocity data from an auxiliary sensor can be fused
	// 辅助速度融合（额外水平速度来源）：如轮速计/外部速度传感器
	controlAuxVelFusion(imu_delayed);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_TERRAIN)
	// 地形状态维护（必要时使用仿真约束）
	controlTerrainFakeFusion();
	updateTerrainValidity();
#endif // CONFIG_EKF2_TERRAIN

	// 零创新航向更新：航向观测缺失/不可靠时避免姿态发散
	controlZeroInnovationHeadingUpdate();

	// 零速度更新：静止/近静止场景下约束速度漂移与降低状态方差
	_zero_velocity_update.update(*this, imu_delayed);

	if (_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias)) {
		_zero_gyro_update.update(*this, imu_delayed);
	}

	// Fake position measurement for constraining drift when no other velocity or position measurements
	// 失助航约束：在无位置/速度直接约束时，施加假位置/高度防止发散
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	// 死算状态检测：不再融合直接约束速度的观测源时置位
	updateDeadReckoningStatus();
}
