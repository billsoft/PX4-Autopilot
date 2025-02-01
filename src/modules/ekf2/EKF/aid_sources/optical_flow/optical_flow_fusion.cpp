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
 * @file optical_flow_fusion.cpp
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_flow_y_innov_var_and_h.h>

/**
 * @brief 融合光流数据
 *
 * 该函数用于融合光流传感器的数据，更新状态估计。根据光流的观测值和状态协方差，计算创新值和创新方差。
 *
 * @param H 观测矩阵，用于更新状态
 * @param update_terrain 是否更新地形信息的标志
 * @return 如果融合成功返回true，否则返回false
 */
bool Ekf::fuseOptFlow(VectorState &H, const bool update_terrain)
{
	const auto state_vector = _state.vector(); // 获取当前状态向量

	// 检查光流的测试比率是否超过阈值，若超过则标记为拒绝
	_innov_check_fail_status.flags.reject_optflow_X = (_aid_src_optical_flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (_aid_src_optical_flow.test_ratio[1] > 1.f);

	// 如果任一轴的创新被拒绝，则中止融合
	if (_aid_src_optical_flow.innovation_rejected) {
		return false;
	}

	// 顺序融合观测轴
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// 第一轴的计算已在上方完成

		} else if (index == 1) {
			// 由于先前的融合导致状态协方差发生变化，因此重新计算创新方差
			const float R_LOS = _aid_src_optical_flow.observation_variance[1]; // 获取观测方差
			const float epsilon = 1e-3f; // 设置小的常数以避免数值不稳定
			// 调用计算函数，更新创新方差和观测矩阵H
			sym::ComputeFlowYInnovVarAndH(state_vector, P, R_LOS, epsilon, &_aid_src_optical_flow.innovation_variance[1], &H);

			// 使用更新后的状态重新计算创新值
			const Vector3f flow_gyro_corrected = _flow_sample_delayed.gyro_rate - _flow_gyro_bias; // 修正陀螺仪数据
			_aid_src_optical_flow.innovation[1] = predictFlow(flow_gyro_corrected)(1) - static_cast<float>
							      (_aid_src_optical_flow.observation[1]);
		}

		// 检查创新方差是否小于观测方差
		if (_aid_src_optical_flow.innovation_variance[index] < _aid_src_optical_flow.observation_variance[index]) {
			// 需要重新初始化协方差矩阵并中止此融合步骤
			ECL_ERR("Opt flow error - covariance reset");
			initialiseCovariance(); // 初始化协方差
			return false;
		}

		// 计算融合增益
		VectorState Kfusion = P * H / _aid_src_optical_flow.innovation_variance[index];

		// 如果不更新地形信息，则将地形状态的增益设置为0
		if (!update_terrain) {
			Kfusion(State::terrain.idx) = 0.f;
		}

		// 执行测量更新
		measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index],
				  _aid_src_optical_flow.innovation[index]);
	}

	// 标记光流状态为正常
	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_aid_src_optical_flow.time_last_fuse = _time_delayed_us; // 更新最后融合时间
	_aid_src_optical_flow.fused = true; // 标记为已融合

	_time_last_hor_vel_fuse = _time_delayed_us; // 更新最后水平速度融合时间

	// 如果需要更新地形信息，则更新最后地形融合时间
	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true; // 返回融合成功
}

/**
 * @brief 预测光流范围
 *
 * 该函数计算光流传感器相对于IMU的距离，返回光流的预测范围。
 *
 * @return 预测的光流范围
 */
float Ekf::predictFlowRange() const
{
	// 计算传感器相对于IMU的位置偏移
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// 计算传感器在地球坐标系中的位置偏移
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// 计算光流相机的高度，正偏移会导致地面高度估计减小
	const float height_above_gnd_est = getHagl() - pos_offset_earth(2);

	// 计算从焦点到图像中心的范围
	float flow_range = height_above_gnd_est / _R_to_earth(2, 2); // 计算视野中框架区域的绝对距离

	// 避免在范围为0时的光流预测奇异性
	if (fabsf(flow_range) < FLT_EPSILON) {
		flow_range = signNoZero(flow_range) * FLT_EPSILON; // 确保范围不为0
	}

	return flow_range; // 返回预测的光流范围
}

/**
 * @brief 预测光流
 *
 * 该函数根据修正的陀螺仪数据预测光流。
 *
 * @param flow_gyro 修正后的陀螺仪数据
 * @return 预测的光流
 */
Vector2f Ekf::predictFlow(const Vector3f &flow_gyro) const
{
	// 计算传感器相对于IMU的位置偏移
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// 计算传感器相对于IMU的速度（在机体坐标系中）
	// 注意：光流陀螺仪是机体角速度的负值，因此使用负号
	const Vector3f vel_rel_imu_body = -flow_gyro % pos_offset_body;

	// 计算传感器在地球坐标系中的速度
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// 将速度旋转到机体坐标系
	const Vector2f vel_body = _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();

	// 计算从焦点到图像中心的范围
	const float range = predictFlowRange();

	// 返回预测的光流
	return Vector2f(vel_body(1) / range, -vel_body(0) / range);
}

/**
 * @brief 计算光流测量方差
 *
 * 该函数根据光流样本计算观测噪声方差，噪声在光流质量范围内线性缩放。
 *
 * @param flow_sample 光流样本
 * @return 计算得到的观测噪声方差
 */
float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample) const
{
	// 计算最佳和最差情况下的观测噪声方差
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f); // 最佳情况下的噪声方差
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f); // 最差情况下的噪声方差

	// 计算一个权重，最佳流质量时为1，最差时为0
	float weighting = (255.f - (float)_params.flow_qual_min);

	if (weighting >= 1.f) {
		// 计算权重
		weighting = math::constrain((float)(flow_sample.quality - _params.flow_qual_min) / weighting, 0.f, 1.f);
	} else {
		weighting = 0.0f; // 若权重小于1，则设置为0
	}

	// 计算最佳和最差流质量的观测噪声的加权平均
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS; // 返回计算得到的观测噪声方差
}
