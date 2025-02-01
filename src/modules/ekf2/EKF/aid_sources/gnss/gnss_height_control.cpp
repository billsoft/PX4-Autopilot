/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file gnss_height_control.cpp
 * 控制EKF GNSS高度融合的相关函数
 */

#include "ekf.h"

void Ekf::controlGnssHeightFusion(const gnssSample &gps_sample)
{
	static constexpr const char *HGT_SRC_NAME = "GNSS"; // GNSS高度数据来源标识

	auto &aid_src = _aid_src_gnss_hgt; // 获取GNSS高度辅助源
	HeightBiasEstimator &bias_est = _gps_hgt_b_est; // 获取GPS高度偏差估计器

	bias_est.predict(_dt_ekf_avg); // 预测偏差状态

	if (_gps_data_ready) { // 检查GPS数据是否准备好

		// 放宽观测噪声的上限，以防止不良GPS数据影响位置估计
		float noise = math::max(gps_sample.vacc, 1.5f * _params.gps_pos_noise); // 使用1.5作为典型的vacc/hacc比率

		if (!isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.gps_hgt)) {
			// 如果没有使用其他垂直位置辅助源，则依赖GPS观测来约束姿态误差，必须限制观测噪声值
			if (noise > _params.pos_noaid_noise) {
				noise = _params.pos_noaid_noise; // 限制噪声值
			}
		}

		const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body; // 计算GPS位置与IMU位置的偏移
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body; // 将偏移转换到地球坐标系
		const float gnss_alt = gps_sample.alt + pos_offset_earth(2); // 计算GNSS高度

		const float measurement = gnss_alt; // 测量值为GNSS高度
		const float measurement_var = sq(noise); // 测量噪声方差

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var); // 检查测量值和方差是否有效

		// 更新GNSS位置，垂直位置GNSS测量值与地球Z轴方向相反
		updateVerticalPositionAidStatus(aid_src,
						gps_sample.time_us, // GPS时间戳
						-(measurement - bias_est.getBias()), // 计算创新值
						measurement_var + bias_est.getBiasVar(), // 计算总方差
						math::max(_params.gps_pos_innov_gate, 1.f)); // 更新创新门限

		// 在更新主滤波器之前更新偏差估计器，但在使用其当前状态计算垂直位置创新之后
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var)); // 设置最大状态噪声
			bias_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd); // 设置过程噪声谱密度
			bias_est.fuseBias(measurement - _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2)); // 融合偏差
		}

		// 确定是否应使用高度辅助
		const bool common_conditions_passing = measurement_valid
						       && _local_origin_lat_lon.isInitialized() // 检查本地原点是否已初始化
						       && _gps_checks_passed; // 检查GPS状态是否通过

		const bool continuing_conditions_passing = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::VPOS)) // 检查GNSS控制参数
				&& common_conditions_passing; // 检查通用条件是否通过

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL); // 检查最新样本是否在时间窗口内

		const bool altitude_initialisation_conditions_passing = common_conditions_passing
				&& !PX4_ISFINITE(_local_origin_alt) // 检查本地原点高度是否有效
				&& _params.height_sensor_ref == static_cast<int32_t>(HeightSensor::GNSS) // 检查高度传感器引用是否为GNSS
				&& isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL); // 检查最新样本是否在时间窗口内

		if (_control_status.flags.gps_hgt) { // 检查GPS高度标志
			if (continuing_conditions_passing) { // 检查持续条件是否通过

				fuseVerticalPosition(aid_src); // 融合垂直位置

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max); // 检查融合是否超时

				if (isHeightResetRequired() && (_height_sensor_ref == HeightSensor::GNSS)) {
					// 所有高度源都失败
					ECL_WARN("%s 高度融合重置需要，所有高度源失败", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_gps = true; // 标记需要重置高度到GPS
					resetAltitudeTo(measurement, measurement_var); // 重置高度
					bias_est.setBias(-_gpos.altitude() + measurement); // 设置偏差
					resetAidSourceStatusZeroInnovation(aid_src); // 重置辅助源状态为零创新

					aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间

				} else if (is_fusion_failing) {
					// 其他高度源仍在工作
					ECL_WARN("停止 %s 高度融合，融合失败", HGT_SRC_NAME);
					stopGpsHgtFusion(); // 停止GPS高度融合
				}

			} else {
				ECL_WARN("停止 %s 高度融合，持续条件失败", HGT_SRC_NAME);
				stopGpsHgtFusion(); // 停止GPS高度融合
			}

		} else {
			if (starting_conditions_passing) { // 检查启动条件是否通过
				if (_params.height_sensor_ref == static_cast<int32_t>(HeightSensor::GNSS)) {
					ECL_INFO("启动 %s 高度融合，重置高度", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::GNSS; // 设置高度传感器引用为GNSS

					_information_events.flags.reset_hgt_to_gps = true; // 标记需要重置高度到GPS

					initialiseAltitudeTo(measurement, measurement_var); // 初始化高度
					bias_est.reset(); // 重置偏差估计器
					resetAidSourceStatusZeroInnovation(aid_src); // 重置辅助源状态为零创新

				} else {
					ECL_INFO("启动 %s 高度融合", HGT_SRC_NAME);
					bias_est.setBias(-_gpos.altitude() + measurement); // 设置偏差
				}

				aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间
				bias_est.setFusionActive(); // 设置融合为活动状态
				_control_status.flags.gps_hgt = true; // 设置GPS高度标志为真

			} if (altitude_initialisation_conditions_passing) {

				// 不启动GNSS高度辅助，但使用测量值
				// 初始化其他高度传感器的高度和偏差
				_information_events.flags.reset_hgt_to_gps = true; // 标记需要重置高度到GPS

				initialiseAltitudeTo(measurement, measurement_var); // 初始化高度
				bias_est.reset(); // 重置偏差估计器
			}
		}

	} else if (_control_status.flags.gps_hgt
		   && !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
		// 不再有数据。停止直到数据回来。
		ECL_WARN("停止 %s 高度融合，没有数据", HGT_SRC_NAME);
		stopGpsHgtFusion(); // 停止GPS高度融合
	}
}

void Ekf::stopGpsHgtFusion()
{
	if (_control_status.flags.gps_hgt) { // 检查GPS高度标志

		if (_height_sensor_ref == HeightSensor::GNSS) {
			_height_sensor_ref = HeightSensor::UNKNOWN; // 将高度传感器引用设置为未知
		}

		_gps_hgt_b_est.setFusionInactive(); // 设置GPS高度偏差估计器为非活动状态

		_control_status.flags.gps_hgt = false; // 设置GPS高度标志为假
	}
}
