/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
 *
 * 版权声明：在源代码和二进制形式中进行再分发和使用，允许进行修改，前提是满足以下条件：
 *
 * 1. 再分发源代码必须保留上述版权声明、条件列表和免责声明。
 * 2. 再分发二进制形式必须在分发的文档或其他材料中重现上述版权声明、条件列表和免责声明。
 * 3. PX4的名称或其贡献者的名称不得用于推广或宣传基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因）引起的，均不对使用本软件的任何方式负责，即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

#include "ekf.h"  // 引入EKF头文件

// 融合水平速度的函数
bool Ekf::fuseHorizontalVelocity(estimator_aid_source2d_s &aid_src)
{
	// 检查创新是否被拒绝
	if (!aid_src.innovation_rejected) {
		// 对于水平速度的两个分量（vx, vy）
		for (unsigned i = 0; i < 2; i++) {
			// 直接融合状态测量
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::vel.idx + i);
		}

		// 标记为已融合
		aid_src.fused = true;
		// 记录最后融合的时间
		aid_src.time_last_fuse = _time_delayed_us;

		// 更新最后水平速度融合的时间
		_time_last_hor_vel_fuse = _time_delayed_us;

	} else {
		// 如果创新被拒绝，标记为未融合
		aid_src.fused = false;
	}

	// 返回融合状态
	return aid_src.fused;
}

// 融合三维速度的函数
bool Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	// 检查创新是否被拒绝
	if (!aid_src.innovation_rejected) {
		// 对于三维速度的三个分量（vx, vy, vz）
		for (unsigned i = 0; i < 3; i++) {
			// 直接融合状态测量
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::vel.idx + i);
		}

		// 标记为已融合
		aid_src.fused = true;
		// 记录最后融合的时间
		aid_src.time_last_fuse = _time_delayed_us;

		// 更新最后水平和垂直速度融合的时间
		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;

	} else {
		// 如果创新被拒绝，标记为未融合
		aid_src.fused = false;
	}

	// 返回融合状态
	return aid_src.fused;
}

// 将水平速度重置为新的值
void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	// 计算新的水平速度与当前速度的差值
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	// 更新当前水平速度
	_state.vel.xy() = new_horz_vel;

	// 如果新的水平速度方差有效
	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		// 更新协方差矩阵的方差
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	// 如果新的水平速度方差有效
	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		// 更新协方差矩阵的方差
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	// 更新位置的协方差方差
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, P(State::pos.idx, State::pos.idx));
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, P(State::pos.idx + 1, State::pos.idx + 1));

	// 更新输出预测器的水平速度
	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// 记录状态变化
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	// 更新重置计数
	_state_reset_status.reset_count.velNE++;

	// 重置超时计时器
	_time_last_hor_vel_fuse = _time_delayed_us;
}

// 将垂直速度重置为新的值
void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	// 计算新的垂直速度与当前速度的差值
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	// 更新当前垂直速度
	_state.vel(2) = new_vert_vel;

	// 如果新的垂直速度方差有效
	if (PX4_ISFINITE(new_vert_vel_var)) {
		// 更新协方差矩阵的方差
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	// 更新位置的协方差方差
	P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, P(State::pos.idx + 2, State::pos.idx + 2));

	// 更新输出预测器的垂直速度
	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// 记录状态变化
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.velD_change += delta_vert_vel;
	}

	// 更新重置计数
	_state_reset_status.reset_count.velD++;

	// 重置超时计时器
	_time_last_ver_vel_fuse = _time_delayed_us;
}

// 将水平速度重置为零
void Ekf::resetHorizontalVelocityToZero()
{
	ECL_INFO("将速度重置为零");
	_information_events.flags.reset_vel_to_zero = true;

	// 当回退到非辅助模式时使用
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

// 将垂直速度重置为零
void Ekf::resetVerticalVelocityToZero()
{
	// 我们不知道垂直速度是多少，因此将其设置为零
	// 将方差设置为一个足够大的值，以便状态能够快速收敛
	// 这不会使滤波器不稳定
	resetVerticalVelocityTo(0.0f, 10.f);
}

// 将速度重置为新的值
void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	// 重置水平速度
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	// 重置垂直速度
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}
