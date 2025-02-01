/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include "ZeroVelocityUpdate.hpp" // 引入零速度更新的头文件

#include "../ekf.h" // 引入扩展卡尔曼滤波器的头文件

ZeroVelocityUpdate::ZeroVelocityUpdate() // 构造函数
{
	reset(); // 调用重置函数
}

void ZeroVelocityUpdate::reset() // 重置函数
{
	_time_last_fuse = 0; // 将最后融合时间初始化为0
}

bool ZeroVelocityUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed) // 更新函数，返回布尔值
{
	// 以有限的速率融合零速度（每200毫秒）
	const bool zero_velocity_update_data_ready = (_time_last_fuse + 200'000 < imu_delayed.time_us); // 检查是否可以进行零速度更新

	if (zero_velocity_update_data_ready) { // 如果可以进行更新
		// 检查持续条件是否满足
		const bool continuing_conditions_passing = ekf.control_status_flags().vehicle_at_rest // 车辆是否静止
				&& ekf.control_status_prev_flags().vehicle_at_rest // 上一状态车辆是否静止
				&& (!ekf.isVerticalVelocityAidingActive() // 垂直速度辅助是否激活
				    || !ekf.control_status_flags().tilt_align); // 如果未对齐，滤波器将“过于刚性”以跟随位置漂移

		if (continuing_conditions_passing) { // 如果持续条件满足
			Vector3f vel_obs{0.f, 0.f, 0.f}; // 观察到的速度初始化为零向量

			// 初始设置较低的方差以加快水平调整，后期提高方差以让状态跟随测量
			const float obs_var = ekf.control_status_flags().tilt_align ? sq(0.2f) : sq(0.001f); // 根据是否对齐设置观察方差
			Vector3f innov_var = ekf.getVelocityVariance() + obs_var; // 获取当前速度方差并加上观察方差

			for (unsigned i = 0; i < 3; i++) { // 遍历三个维度
				const float innovation = ekf.state().vel(i) - vel_obs(i); // 计算创新值，即当前速度与观察速度的差
				ekf.fuseDirectStateMeasurement(innovation, innov_var(i), obs_var, State::vel.idx + i); // 直接融合状态测量
			}

			_time_last_fuse = imu_delayed.time_us; // 更新最后融合时间为当前IMU时间

			return true; // 返回更新成功
		}
	}

	return false; // 返回更新失败
}
