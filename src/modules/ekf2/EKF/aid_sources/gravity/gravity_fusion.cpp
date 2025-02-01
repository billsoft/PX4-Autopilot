/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 开发团队。保留所有权利。
 *
 * 以源代码和二进制形式重新分发和使用，或有或没有修改，均可，前提是满足以下条件：
 *
 * 1. 重新分发的源代码必须保留上述版权声明、此列表和以下免责声明。
 * 2. 以二进制形式重新分发必须在分发的文档或其他材料中复制上述版权声明、此列表和以下免责声明。
 * 3. 除非事先获得书面许可，否则不得使用 PX4 的名称或其贡献者的名称来支持或推广衍生自本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的担保，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权拥有者或贡献者均不对任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于采购替代商品或服务、使用、数据或利润损失或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他）引起的，均不对使用本软件的可能性负责。
 *
 ****************************************************************************/

/**
 * @file gravity_fusion.cpp
 * 融合重力向量的观测值，以约束滚转和俯仰（类似于互补滤波）。
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gravity_xyz_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_gravity_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_gravity_z_innov_var_and_h.h>

#include <mathlib/mathlib.h>

void Ekf::controlGravityFusion(const imuSample &imu)
{
	// 获取延迟时间内的原始加速度计读数，并计算期望的测量噪声（高斯噪声）
	const Vector3f measurement = Vector3f(imu.delta_vel / imu.delta_vel_dt - _state.accel_bias).unit(); // 计算单位重力向量
	const float measurement_var = math::max(sq(_params.gravity_noise), sq(0.01f)); // 计算测量方差，确保不小于0.01的平方

	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f; // 设置加速度上限为1.1g
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f; // 设置加速度下限为0.9g
	const bool accel_lpf_norm_good = (_accel_magnitude_filt > lower_accel_limit) // 检查低通滤波后的加速度是否在合理范围内
					 && (_accel_magnitude_filt < upper_accel_limit);

	// 如果整体加速度不太大，则融合重力观测
	_control_status.flags.gravity_vector = (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector)) // 检查IMU控制参数是否允许重力向量
					       && (accel_lpf_norm_good || _control_status.flags.vehicle_at_rest) // 检查加速度是否在合理范围内或车辆是否静止
					       && !isHorizontalAidingActive(); // 确保没有水平辅助激活

	// 计算卡尔曼增益和创新方差
	Vector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement; // 计算创新值
	Vector3f innovation_variance; // 初始化创新方差
	const auto state_vector = _state.vector(); // 获取当前状态向量
	VectorState H; // 初始化观测矩阵H
	sym::ComputeGravityXyzInnovVarAndHx(state_vector, P, measurement_var, &innovation_variance, &H); // 计算创新方差和观测矩阵H

	// 填充估计器辅助源状态
	updateAidSourceStatus(_aid_src_gravity,
			      imu.time_us,                                                 // 采样时间戳
			      measurement,                                                 // 观测值
			      Vector3f{measurement_var, measurement_var, measurement_var}, // 观测方差
			      innovation,                                                  // 创新值
			      innovation_variance,                                         // 创新方差
			      0.25f);                                                      // 创新门限

	// 使用顺序融合更新状态和协方差
	for (uint8_t index = 0; index <= 2; index++) {
		// 计算卡尔曼增益和观测雅可比矩阵
		if (index == 0) {
			// 第一轴的计算已在上方完成

		} else if (index == 1) {
			// 由于先前的融合导致状态协方差发生变化，因此重新计算创新方差（使用相同的初始状态对所有轴进行线性化）
			sym::ComputeGravityYInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// 使用更新后的状态重新计算创新值
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);

		} else if (index == 2) {
			// 由于先前的融合导致状态协方差发生变化，因此重新计算创新方差（使用相同的初始状态对所有轴进行线性化）
			sym::ComputeGravityZInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// 使用更新后的状态重新计算创新值
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);
		}

		VectorState K = P * H / _aid_src_gravity.innovation_variance[index]; // 计算卡尔曼增益

		const bool accel_clipping = imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2]; // 检查加速度是否被裁剪

		if (_control_status.flags.gravity_vector && !_aid_src_gravity.innovation_rejected && !accel_clipping) { // 如果重力向量有效且创新未被拒绝且没有加速度裁剪
			measurementUpdate(K, H, _aid_src_gravity.observation_variance[index], _aid_src_gravity.innovation[index]); // 更新测量值
		}
	}

	_aid_src_gravity.fused = true; // 标记重力融合为已完成
	_aid_src_gravity.time_last_fuse = imu.time_us; // 更新最后融合时间戳
}
