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
 * @file fake_height_control.cpp
 * 控制函数，用于扩展卡尔曼滤波器（EKF）的假高度融合
 */

#include "ekf.h"

void Ekf::controlFakeHgtFusion()
{
	auto &aid_src = _aid_src_fake_hgt; // 获取假高度辅助源

	// 如果没有进行任何辅助，使用最后已知的垂直位置进行假高度测量，以限制漂移
	const bool fake_hgt_data_ready = !isVerticalAidingActive() // 检查是否未激活垂直辅助
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // 检查假高度是否可以以有限速率融合

	if (fake_hgt_data_ready) {

		const float obs_var = sq(_params.pos_noaid_noise); // 观察方差，基于无辅助噪声参数的平方
		const float innov_gate = 3.f; // 创新门限，用于判断融合的有效性

		// 更新垂直位置辅助状态，传入当前时间、最后已知高度的负值、观察方差和创新门限
		updateVerticalPositionAidStatus(aid_src, _time_delayed_us, -_last_known_gpos.altitude(), obs_var, innov_gate);

		const bool continuing_conditions_passing = !isVerticalAidingActive(); // 检查是否继续满足条件
		const bool starting_conditions_passing = continuing_conditions_passing // 检查是否满足启动条件
				&& _vertical_velocity_deadreckon_time_exceeded // 垂直速度推算时间是否超过
				&& _vertical_position_deadreckon_time_exceeded; // 垂直位置推算时间是否超过

		if (_control_status.flags.fake_hgt) { // 如果假高度融合正在进行
			if (continuing_conditions_passing) { // 如果继续满足条件

				// 始终保护以防极端值导致NaN
				if (aid_src.test_ratio < sq(100.0f / innov_gate)) { // 检查测试比率是否在合理范围内
					if (!aid_src.innovation_rejected) { // 如果创新未被拒绝
						// 直接融合状态测量，传入创新值、创新方差、观察方差和状态索引
						fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
									   State::pos.idx + 2);

						aid_src.fused = true; // 标记为已融合
						aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间
					}
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5); // 检查融合是否超时

				if (is_fusion_failing) { // 如果融合失败
					resetFakeHgtFusion(); // 重置假高度融合
				}

			} else {
				stopFakeHgtFusion(); // 停止假高度融合
			}

		} else { // 如果假高度融合未进行
			if (starting_conditions_passing) { // 如果满足启动条件
				ECL_INFO("开始假高度融合"); // 记录信息
				_control_status.flags.fake_hgt = true; // 设置假高度标志为真
				resetFakeHgtFusion(); // 重置假高度融合
			}
		}

	} else if (_control_status.flags.fake_hgt && isVerticalAidingActive()) { // 如果假高度融合正在进行且垂直辅助已激活
		stopFakeHgtFusion(); // 停止假高度融合
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("重置假高度融合"); // 记录重置信息
	_last_known_gpos.setAltitude(_gpos.altitude()); // 设置最后已知位置的高度为当前高度

	resetVerticalVelocityToZero(); // 将垂直速度重置为零
	resetHeightToLastKnown(); // 将高度重置为最后已知高度

	_aid_src_fake_hgt.time_last_fuse = _time_delayed_us; // 更新最后融合时间为当前延迟时间
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true; // 设置标志以重置位置为最后已知位置
	ECL_INFO("将高度重置为最后已知高度 (%.3f)", (double)_last_known_gpos.altitude()); // 记录重置信息
	resetAltitudeTo(_last_known_gpos.altitude(), sq(_params.pos_noaid_noise)); // 将高度重置为最后已知高度，并传入观察方差
}

void Ekf::stopFakeHgtFusion()
{
	if (_control_status.flags.fake_hgt) { // 如果假高度融合正在进行
		ECL_INFO("停止假高度融合"); // 记录停止信息
		_control_status.flags.fake_hgt = false; // 设置假高度标志为假
	}
}
