/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 *    used to endorse或推广基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因）引起的，均不对使用本软件的任何方式负责，即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

#include "ekf.h"  // 引入EKF头文件

#include <ekf_derivation/generated/compute_yaw_innov_var_and_h.h>  // 引入计算航向创新方差和H的生成文件

#include <mathlib/mathlib.h>  // 引入数学库

// 融合航向信息的函数
bool Ekf::fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW)
{
	// 检查创新方差计算是否条件不良
	if (aid_src_status.innovation_variance >= aid_src_status.observation_variance) {
		// 状态协方差的创新方差贡献不是负值，没有故障
		_fault_status.flags.bad_hdg = false;

	} else {
		// 状态协方差的创新方差贡献为负，这意味着协方差矩阵条件不良
		_fault_status.flags.bad_hdg = true;

		// 重新初始化协方差矩阵并中止此融合步骤
		initialiseCovariance();
		ECL_ERR("航向融合数值错误 - 协方差重置");

		return false;  // 返回失败
	}

	// 计算卡尔曼增益
	// 仅计算我们正在使用的状态的增益
	VectorState Kfusion;  // 定义融合增益的状态向量
	const float heading_innov_var_inv = 1.f / aid_src_status.innovation_variance;  // 计算创新方差的倒数

	for (uint8_t row = 0; row < State::size; row++) {
		for (uint8_t col = 0; col <= 3; col++) {
			Kfusion(row) += P(row, col) * H_YAW(col);  // 计算增益
		}

		Kfusion(row) *= heading_innov_var_inv;  // 乘以创新方差的倒数
	}

	// 如果测试失败，则将航向标记为不健康
	if (aid_src_status.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;

		// 如果我们在空中，则不希望融合测量值
		// 我们允许在地面使用它，因为大的创新可能是由干扰或大的初始陀螺仪偏置引起的
		if (!_control_status.flags.in_air
		    && isTimedOut(_time_last_in_air, (uint64_t)5e6)  // 检查是否在空中超时
		    && isTimedOut(aid_src_status.time_last_fuse, (uint64_t)1e6)  // 检查融合超时
		   ) {
			// 将创新限制在由门限设置的最大值
			// 我们需要延迟这个强制融合，以避免在着陆后立即开始
			const float gate_sigma = math::max(_params.heading_innov_gate, 1.f);  // 获取门限
			const float gate_limit = sqrtf((sq(gate_sigma) * aid_src_status.innovation_variance));  // 计算门限限制
			aid_src_status.innovation = math::constrain(aid_src_status.innovation, -gate_limit, gate_limit);  // 限制创新值

			// 还重置航向陀螺仪方差以更快收敛，避免停留在先前的错误估计上
			resetGyroBiasZCov();

		} else {
			return false;  // 返回失败
		}

	} else {
		_innov_check_fail_status.flags.reject_yaw = false;  // 如果没有拒绝，则标记为正常
	}

	// 更新测量值
	measurementUpdate(Kfusion, H_YAW, aid_src_status.observation_variance, aid_src_status.innovation);

	_time_last_heading_fuse = _time_delayed_us;  // 更新最后航向融合时间

	aid_src_status.time_last_fuse = _time_delayed_us;  // 更新最后融合时间
	aid_src_status.fused = true;  // 标记为已融合

	_fault_status.flags.bad_hdg = false;  // 重置航向故障标志

	return true;  // 返回成功
}

// 计算航向创新方差和H的函数
void Ekf::computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const
{
	sym::ComputeYawInnovVarAndH(_state.vector(), P, variance, &innovation_variance, &H_YAW);  // 调用计算函数
}

// 重置四元数状态航向的函数
void Ekf::resetQuatStateYaw(float yaw, float yaw_variance)
{
	// 保存四元数状态的副本，以便后续计算重置变化量
	const Quatf quat_before_reset = _state.quat_nominal;

	// 更新航向角方差
	if (PX4_ISFINITE(yaw_variance) && (yaw_variance > FLT_EPSILON)) {
		P.uncorrelateCovarianceSetVariance<1>(2, yaw_variance);  // 设置航向方差
	}

	// 使用当前估计更新从机体到世界坐标系的变换矩阵
	// 使用新的航向值更新旋转矩阵
	_R_to_earth = updateYawInRotMat(yaw, Dcmf(_state.quat_nominal));

	// 计算四元数变化量
	const Quatf quat_after_reset(_R_to_earth);  // 更新后的四元数
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());  // 计算四元数误差

	// 更新四元数状态
	_state.quat_nominal = quat_after_reset;

	// 将重置量添加到输出观察者缓冲数据中
	_output_predictor.resetQuaternion(q_error);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	// 更新外部视觉姿态误差滤波器
	if (_ev_q_error_initialized) {
		const Quatf ev_q_error_updated = (q_error * _ev_q_error_filt.getState()).normalized();  // 更新外部视觉误差
		_ev_q_error_filt.reset(ev_q_error_updated);  // 重置外部视觉滤波器
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 记录状态变化
	if (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) {
		_state_reset_status.quat_change = q_error;  // 如果没有重置，记录变化量

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.quat_change = q_error * _state_reset_status.quat_change;  // 计算总变化量
		_state_reset_status.quat_change.normalize();  // 归一化变化量
	}

	_state_reset_status.reset_count.quat++;  // 更新重置计数

	_time_last_heading_fuse = _time_delayed_us;  // 更新最后航向融合时间
}
