/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ZeroGyroUpdate.hpp" // 引入零陀螺仪更新的头文件

#include "../ekf.h" // 引入扩展卡尔曼滤波器的头文件

ZeroGyroUpdate::ZeroGyroUpdate() // 构造函数
{
	reset(); // 调用重置函数，初始化状态
}

void ZeroGyroUpdate::reset() // 重置函数
{
	_zgup_delta_ang.setZero(); // 将角度增量初始化为零
	_zgup_delta_ang_dt = 0.f; // 将角度增量时间初始化为零
}

bool ZeroGyroUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed) // 更新函数，返回布尔值
{
	// 当车辆静止时，将陀螺仪数据融合为陀螺仪偏置的直接观测
	if (ekf.control_status_flags().vehicle_at_rest) {
		// 将陀螺仪数据下采样，以较低的频率进行融合
		_zgup_delta_ang += imu_delayed.delta_ang; // 累加延迟IMU样本的角度增量
		_zgup_delta_ang_dt += imu_delayed.delta_ang_dt; // 累加延迟IMU样本的角度增量时间

		static constexpr float zgup_dt = 0.2f; // 定义更新周期为0.2秒
		const bool zero_gyro_update_data_ready = _zgup_delta_ang_dt >= zgup_dt; // 检查是否达到更新条件

		if (zero_gyro_update_data_ready) { // 如果满足更新条件

			Vector3f gyro_bias = _zgup_delta_ang / _zgup_delta_ang_dt; // 计算陀螺仪偏置

			const float obs_var = sq(math::constrain(ekf.getGyroNoise(), 0.f, 1.f)); // 计算观测噪声方差，限制在0到1之间

			for (unsigned i = 0; i < 3; i++) { // 遍历三个维度
				const float innovation = ekf.state().gyro_bias(i) - gyro_bias(i); // 计算创新值，即当前陀螺仪偏置与计算得到的偏置之差
				const float innov_var = ekf.getGyroBiasVariance()(i) + obs_var; // 计算创新方差，包含当前偏置方差和观测噪声方差
				ekf.fuseDirectStateMeasurement(innovation, innov_var, obs_var, State::gyro_bias.idx + i); // 直接融合状态测量
			}

			// 重置积分器
			_zgup_delta_ang.setZero(); // 将角度增量重置为零
			_zgup_delta_ang_dt = 0.f; // 将角度增量时间重置为零

			return true; // 返回更新成功
		}

	} else if (ekf.control_status_prev_flags().vehicle_at_rest) { // 如果之前车辆是静止状态
		// 重置积分器
		_zgup_delta_ang.setZero(); // 将角度增量重置为零
		_zgup_delta_ang_dt = 0.f; // 将角度增量时间重置为零
	}

	return false; // 返回更新失败
}
