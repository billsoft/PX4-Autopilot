/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file wind.cpp
 * 风状态的辅助函数
 */

#include "ekf.h" // 引入EKF头文件
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_wind_speed_and_direction.h> // 引入计算风速和风向初始化及协方差的生成文件

// 将风状态重置为外部观测值
void Ekf::resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
		float wind_direction_accuracy)
{
	// 检查当前状态是否不在空中
	if (!_control_status.flags.in_air) {

		// 限制风速为非负值
		const float wind_speed_constrained = math::max(wind_speed, 0.0f);
		// 计算风向的方差
		const float wind_direction_var = sq(wind_direction_accuracy);
		// 计算风速的方差
		const float wind_speed_var = sq(wind_speed_accuracy);

		Vector2f wind; // 定义风的二维向量
		Vector2f wind_var; // 定义风方差的二维向量

		// 根据风速和风向计算风的初始化和协方差
		sym::ComputeWindInitAndCovFromWindSpeedAndDirection(wind_speed_constrained, wind_direction, wind_speed_var,
				wind_direction_var, &wind, &wind_var);

		// 输出信息，重置风状态为外部观测值
		ECL_INFO("reset wind states to external observation");
		_information_events.flags.reset_wind_to_ext_obs = true; // 设置标志，表示已重置风状态
		_external_wind_init = true; // 设置外部风初始化标志

		// 调用重置风状态的函数
		resetWindTo(wind, wind_var);
	}
}

// 将风状态重置为给定的风和风方差
void Ekf::resetWindTo(const Vector2f &wind, const Vector2f &wind_var)
{
	_state.wind_vel = wind; // 更新状态中的风速

	// 如果风方差的第一个分量是有限的
	if (PX4_ISFINITE(wind_var(0))) {
		// 设置风速的方差，确保不超过初始风不确定性的平方
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(0)));
	}

	// 如果风方差的第二个分量是有限的
	if (PX4_ISFINITE(wind_var(1))) {
		// 设置风速的方差，确保不超过初始风不确定性的平方
		P.uncorrelateCovarianceSetVariance<1>(State::wind_vel.idx + 1,
						      math::min(sq(_params.initial_wind_uncertainty), wind_var(1)));
	}
}

// 重置风的协方差
void Ekf::resetWindCov()
{
	// 从一个小的初始不确定性开始，以改善初始估计
	P.uncorrelateCovarianceSetVariance<State::wind_vel.dof>(State::wind_vel.idx, sq(_params.initial_wind_uncertainty));
}

// 将风状态重置为零
void Ekf::resetWindToZero()
{
	// 输出信息，重置风为零
	ECL_INFO("reset wind to zero");
	_state.wind_vel.setZero(); // 将风速设置为零
	resetWindCov(); // 重置风的协方差
}
