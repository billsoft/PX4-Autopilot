/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file terrain_control.cpp
 * 该文件实现了地形控制的相关功能，包括初始化地形、控制地形融合和更新地形有效性。
 */

#include "ekf.h"  // 引入EKF头文件
#include "ekf_derivation/generated/compute_hagl_innov_var.h"  // 引入计算地面高度创新方差的头文件

#include <mathlib/mathlib.h>  // 引入数学库

// 初始化地形信息
void Ekf::initTerrain()
{
	// 假设一个地面间隙
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;  // 计算当前地形高度

	// 使用地面间隙值作为我们的不确定性
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, sq(_params.rng_gnd_clearance));  // 设置协方差矩阵的方差
}

// 控制地形的虚假融合
void Ekf::controlTerrainFakeFusion()
{
	// 如果我们在地面上，存储本地位置和时间以用作参考
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = -_gpos.altitude();  // 记录地面高度
		_control_status.flags.rng_fault = false;  // 重置范围传感器故障标志

	} else if (!_control_status_prev.flags.in_air) {
		// 在准备进行基准测试之前，让估计器自由运行，但在起飞时重置
		// 因为在使用光流测量时，安全起见，最好从小于地面的距离开始
		// 过高的估计距离会导致过高的速度估计，从而引发危险行为。
		initTerrain();  // 初始化地形
	}

	if (!_control_status.flags.in_air) {
		// 检查是否没有地形辅助
		bool no_terrain_aiding = !_control_status.flags.rng_terrain
					 && !_control_status.flags.opt_flow_terrain
					 && isTimedOut(_time_last_terrain_fuse, (uint64_t)1e6);  // 检查地形融合超时

		if (no_terrain_aiding && (_height_sensor_ref != HeightSensor::RANGE)) {
			initTerrain();  // 如果没有地形辅助，重新初始化地形
		}
	}
}

// 更新地形有效性
void Ekf::updateTerrainValidity()
{
	bool valid_opt_flow_terrain = false;  // 光流地形有效性标志
	bool valid_rng_terrain = false;  // 范围传感器地形有效性标志
	bool positive_hagl_var = false;  // 正的地面高度方差标志
	bool small_relative_hagl_var = false;  // 小的相对地面高度方差标志

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// 检查光流地形是否有效
	if (_control_status.flags.opt_flow_terrain
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_opt_flow_terrain = true;  // 设置光流地形有效性标志
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)

	// 检查范围传感器地形是否有效
	if (_control_status.flags.rng_terrain
	    && isRecent(_aid_src_rng_hgt.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_rng_terrain = true;  // 设置范围传感器地形有效性标志
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	if (_time_last_terrain_fuse != 0) {
		// 假设当不确定性相对于地面高度较小时有效
		float hagl_var = INFINITY;  // 初始化地面高度方差
		sym::ComputeHaglInnovVar(P, 0.f, &hagl_var);  // 计算地面高度创新方差

		positive_hagl_var = hagl_var > 0.f;  // 检查地面高度方差是否为正

		if (positive_hagl_var
		    && (hagl_var < sq(fmaxf(0.1f * getHagl(), 0.5f)))  // 检查地面高度方差是否小于阈值
		   ) {
			small_relative_hagl_var = true;  // 设置小的相对地面高度方差标志
		}
	}

	const bool positive_hagl = getHagl() >= 0.f;  // 检查地面高度是否为正

	if (!_terrain_valid) {
		// 要求有效的范围传感器或光流（加上有效方差）才能初步认为地形有效
		if (positive_hagl
		    && positive_hagl_var
		    && (valid_rng_terrain
			|| (valid_opt_flow_terrain && small_relative_hagl_var))
		   ) {
			_terrain_valid = true;  // 设置地形有效性标志
		}

	} else {
		// 地形之前有效，如果方差良好则继续认为有效
		_terrain_valid = positive_hagl && positive_hagl_var && small_relative_hagl_var;  // 更新地形有效性
	}
}
