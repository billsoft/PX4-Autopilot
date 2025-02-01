/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * 该软件的源代码和二进制形式的再分发和使用，均可在以下条件下进行，且不论是否进行修改：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随附的文档或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、惩戒性或后果性损害（包括但不限于采购替代商品或服务；使用、数据或利润的损失；或业务中断）承担责任，无论是基于合同、严格责任或侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file fake_pos_control.cpp
 * 控制函数，用于扩展卡尔曼滤波器（EKF）的假位置融合
 */

#include "ekf.h"

void Ekf::controlFakePosFusion()
{
	auto &aid_src = _aid_src_fake_pos; // 获取假位置辅助源

	// 如果没有进行任何辅助，则在最后已知位置进行假位置测量，以限制漂移
	// 在初始倾斜对齐期间，假位置用于执行EKF的“准静态”水平调整
	const bool fake_pos_data_ready = !isHorizontalAidingActive() // 检查是否未激活水平辅助
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // 检查假位置是否可以以有限速率融合

	if (fake_pos_data_ready) {

		Vector2f obs_var; // 定义观察方差的二维向量

		if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
			// 如果在空中并且倾斜对齐，则设置较大的观察方差
			obs_var(0) = obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, 1.f));

		} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
			// 当车辆静止时，通过更积极地融合来加速倾斜精细对齐
			obs_var(0) = obs_var(1) = sq(0.01f); // 设置较小的观察方差以加快融合

		} else {
			// 其他情况下，设置中等的观察方差
			obs_var(0) = obs_var(1) = sq(0.5f);
		}

		// 计算创新值，即当前假位置与最后已知假位置的差
		const Vector2f innovation = (_gpos - _last_known_gpos).xy();

		const float innov_gate = 3.f; // 创新门限值

		// 更新辅助源状态
		updateAidSourceStatus(aid_src,
				      _time_delayed_us, // 延迟的时间戳
				      Vector2f(_gpos.latitude_deg(), _gpos.longitude_deg()), // 观察值，假位置的经纬度
				      obs_var,                                               // 观察方差
				      innovation,                       // 创新值
				      Vector2f(getStateVariance<State::pos>()) + obs_var,    // 创新方差，包含状态方差和观察方差
				      innov_gate);                                           // 创新门限

		// 检查是否启用有效的假位置
		const bool enable_valid_fake_pos = _control_status.flags.constant_pos || _control_status.flags.vehicle_at_rest;
		// 检查是否启用假位置
		const bool enable_fake_pos = !enable_valid_fake_pos
					     && (getTiltVariance() > sq(math::radians(3.f))) // 检查倾斜方差
					     && !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector)) // 检查IMU控制参数
					     && _horizontal_deadreckon_time_exceeded; // 检查水平推算时间是否超限

		// 运行假位置状态机，更新假位置和有效假位置的状态标志
		_control_status.flags.fake_pos = runFakePosStateMachine(enable_fake_pos, _control_status.flags.fake_pos, aid_src);
		_control_status.flags.valid_fake_pos = runFakePosStateMachine(enable_valid_fake_pos,
						       _control_status.flags.valid_fake_pos, aid_src);

	} else if ((_control_status.flags.fake_pos || _control_status.flags.valid_fake_pos) && isHorizontalAidingActive()) {
		// 如果假位置或有效假位置标志被激活且水平辅助被激活，则停止假位置融合
		ECL_INFO("停止假位置融合");
		_control_status.flags.fake_pos = false; // 重置假位置标志
		_control_status.flags.valid_fake_pos = false; // 重置有效假位置标志
	}
}

void Ekf::resetFakePosFusion()
{
	ECL_INFO("重置假位置融合");
	_last_known_gpos.setLatLon(_gpos); // 将最后已知假位置设置为当前假位置

	resetHorizontalPositionToLastKnown(); // 重置水平位置到最后已知位置
	resetHorizontalVelocityToZero(); // 将水平速度重置为零

	_aid_src_fake_pos.time_last_fuse = _time_delayed_us; // 更新最后融合时间为当前延迟时间
}

bool Ekf::runFakePosStateMachine(const bool enable_conditions_passing, bool status_flag,
				 estimator_aid_source2d_s &aid_src)
{
	if (status_flag) { // 如果假位置状态标志为真
		if (enable_conditions_passing) { // 如果启用条件通过
			if (!aid_src.innovation_rejected) { // 如果创新未被拒绝
				for (unsigned i = 0; i < 2; i++) { // 遍历两个维度
					// 直接融合状态测量
					fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
								   State::pos.idx + i);
				}

				aid_src.fused = true; // 标记为已融合
				aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5); // 检查融合是否超时

			if (is_fusion_failing) { // 如果融合失败
				ECL_WARN("假位置融合失败，正在重置");
				resetFakePosFusion(); // 重置假位置融合
			}

		} else {
			ECL_INFO("停止假位置融合");
			status_flag = false; // 停止假位置状态
		}

	} else {
		if (enable_conditions_passing) { // 如果启用条件通过
			ECL_INFO("开始假位置融合");
			status_flag = true; // 启动假位置状态

			resetFakePosFusion(); // 重置假位置融合
		}
	}

	return status_flag; // 返回当前状态标志
}
