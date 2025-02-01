/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file control.cpp
 * 控制函数用于EKF姿态和位置估计器。
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"  // 引入EKF头文件
#include <mathlib/mathlib.h>  // 引入数学库

// 控制融合模式的函数
void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
	// 存储状态以启用变化检测
	_control_status_prev.value = _control_status.value;  // 保存当前控制状态
	_state_reset_count_prev = _state_reset_status.reset_count;  // 保存状态重置计数

	if (_system_flag_buffer) {  // 检查系统标志缓冲区是否存在
		systemFlagUpdate system_flags_delayed;  // 定义系统标志更新结构

		// 从缓冲区中弹出时间戳早于imu_delayed.time_us的第一个系统标志
		if (_system_flag_buffer->pop_first_older_than(imu_delayed.time_us, &system_flags_delayed)) {
			// 设置车辆静止状态
			set_vehicle_at_rest(system_flags_delayed.at_rest);
			// 设置在空中状态
			set_in_air_status(system_flags_delayed.in_air);
			// 设置是否为固定翼
			set_is_fixed_wing(system_flags_delayed.is_fixed_wing);

			// 如果地面效应标志为真，则设置地面效应
			if (system_flags_delayed.gnd_effect) {
				set_gnd_effect();
			}

			// 设置恒定位置状态
			set_constant_pos(system_flags_delayed.constant_pos);
		}
	}

	// 监控倾斜对齐状态
	if (!_control_status.flags.tilt_align) {  // 如果尚未对齐
		// 在对齐倾斜时，监控方差
		// 一旦倾斜方差降低到相当于3度的不确定性，声明倾斜对齐完成
		if (getTiltVariance() < sq(math::radians(3.f))) {
			_control_status.flags.tilt_align = true;  // 设置倾斜对齐标志为真

			// 发送对齐状态消息到控制台
			const char *height_source = "unknown";  // 初始化高度源为未知

			// 根据不同的高度源设置高度源名称
			if (_control_status.flags.baro_hgt) {
				height_source = "baro";  // 气压高度
			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";  // 外部视觉高度
			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";  // GPS高度
			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";  // 距离传感器高度
			}

			// 打印对齐信息，包括时间戳和高度源
			ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
				 (unsigned long long)imu_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);

			// 打印倾斜对齐的角度信息
			ECL_DEBUG("tilt aligned, roll: %.3f, pitch %.3f, yaw: %.3f",
				  (double)matrix::Eulerf(_state.quat_nominal).phi(),
				  (double)matrix::Eulerf(_state.quat_nominal).theta(),
				  (double)matrix::Eulerf(_state.quat_nominal).psi()
				 );
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 控制观测数据的使用以进行辅助
	controlMagFusion(imu_delayed);  // 控制磁力计融合
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlOpticalFlowFusion(imu_delayed);  // 控制光流融合
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
	controlGpsFusion(imu_delayed);  // 控制GNSS融合
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	_aux_global_position.update(*this, imu_delayed);  // 更新辅助全局位置
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#if defined(CONFIG_EKF2_AIRSPEED)
	controlAirDataFusion(imu_delayed);  // 控制气流数据融合
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	controlBetaFusion(imu_delayed);  // 控制侧滑融合
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	controlDragFusion(imu_delayed);  // 控制阻力融合
#endif // CONFIG_EKF2_DRAG_FUSION

	controlHeightFusion(imu_delayed);  // 控制高度融合

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	controlGravityFusion(imu_delayed);  // 控制重力融合
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 从外部估计器融合额外的里程计数据
	controlExternalVisionFusion(imu_delayed);  // 控制外部视觉融合
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// 从辅助传感器融合额外的水平速度数据
	controlAuxVelFusion(imu_delayed);  // 控制辅助速度融合
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_TERRAIN)
	controlTerrainFakeFusion();  // 控制地形假融合
	updateTerrainValidity();  // 更新地形有效性
#endif // CONFIG_EKF2_TERRAIN

	controlZeroInnovationHeadingUpdate();  // 控制零创新航向更新

	_zero_velocity_update.update(*this, imu_delayed);  // 更新零速度

	if (_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias)) {
		_zero_gyro_update.update(*this, imu_delayed);  // 更新零陀螺仪偏置
	}

	// 当没有其他速度或位置测量时，进行假位置测量以限制漂移
	controlFakePosFusion();  // 控制假位置融合
	controlFakeHgtFusion();  // 控制假高度融合

	// 检查我们是否不再融合直接约束速度漂移的测量
	updateDeadReckoningStatus();  // 更新死 reckoning 状态
}
