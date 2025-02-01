/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 开发团队。保留所有权利。
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
 * @file optical_flow_control.cpp
 * 光流融合的控制函数
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	// 检查光流缓冲区是否存在以及光流控制参数是否启用
	if (!_flow_buffer || (_params.flow_ctrl != 1)) {
		stopFlowFusion(); // 停止光流融合
		return; // 退出函数
	}

	VectorState H; // 定义状态转移矩阵 H

	// 当新的光流数据可用且样本的中点时间早于融合时间范围时，准备进行融合
	if (_flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed)) {

		// 光流陀螺仪的符号约定相反
		_ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - getGyroBias());

		// 确保光流样本的陀螺仪速率有效
		switch (static_cast<FlowGyroSource>(_params.flow_gyro_src)) {
		default:
		/* FALLTHROUGH */
		case FlowGyroSource::Auto:
			// 如果光流样本的前两个陀螺仪速率无效，则使用参考体速率
			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_rate(1))) {
				_flow_sample_delayed.gyro_rate = _ref_body_rate; // 使用参考体速率
			}

			// 如果光流样本的第三个陀螺仪速率无效，则使用参考体速率的 Z 分量
			if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate(2))) {
				// 某些光流模块仅提供 X 和 Y 角速率。如果是这种情况，则用我们的 Z 陀螺仪速率补全向量
				_flow_sample_delayed.gyro_rate(2) = _ref_body_rate(2);
			}
			break;

		case FlowGyroSource::Internal:
			_flow_sample_delayed.gyro_rate = _ref_body_rate; // 使用参考体速率
			break;
		}

		const flowSample &flow_sample = _flow_sample_delayed; // 获取延迟的光流样本

		// 根据当前状态判断光流质量的最小阈值
		const int32_t min_quality = _control_status.flags.in_air
					    ? _params.flow_qual_min // 在空中时的最小质量
					    : _params.flow_qual_min_gnd; // 在地面时的最小质量

		const bool is_quality_good = (flow_sample.quality >= min_quality); // 判断光流质量是否良好

		bool is_tilt_good = true; // 初始化倾斜状态为良好

#if defined(CONFIG_EKF2_RANGE_FINDER)
		is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt); // 检查倾斜是否在允许范围内
#endif // CONFIG_EKF2_RANGE_FINDER

		calcOptFlowBodyRateComp(flow_sample); // 计算光流体速率补偿

		// 使用去除体角速率贡献的光流速率计算光学 LOS 速率
		// 修正用于运动补偿的数据中的陀螺仪偏差错误
		// 注意符号约定：正的 LOS 速率表示场景绕该轴的右手旋转
		const Vector3f flow_gyro_corrected = flow_sample.gyro_rate - _flow_gyro_bias; // 修正后的光流陀螺仪速率
		const Vector2f flow_compensated = flow_sample.flow_rate - flow_gyro_corrected.xy(); // 补偿后的光流速率

		// 计算光流观测方差
		const float R_LOS = calcOptFlowMeasVar(flow_sample); // 观测方差

		const float epsilon = 1e-3f; // 设置一个小的常数以避免数值不稳定
		Vector2f innov_var; // 创新方差
		sym::ComputeFlowXyInnovVarAndHx(_state.vector(), P, R_LOS, epsilon, &innov_var, &H); // 计算创新方差和 H 矩阵

		// 运行创新一致性检查并记录结果
		updateAidSourceStatus(_aid_src_optical_flow,
				      flow_sample.time_us,                                 // 样本时间戳
				      flow_compensated,                                    // 观测值
				      Vector2f{R_LOS, R_LOS},                              // 观测方差
				      predictFlow(flow_gyro_corrected) - flow_compensated, // 创新
				      innov_var,                                           // 创新方差
				      math::max(_params.flow_innov_gate, 1.f));            // 创新门限

		// 记录补偿后的光流速率
		_flow_rate_compensated = flow_compensated;

		// 从修正的光流测量中计算体和局部框架中的速度，仅用于记录
		const float range = predictFlowRange(); // 预测光流范围
		_flow_vel_body(0) = -flow_compensated(1) * range; // 计算体框架中的 X 速度
		_flow_vel_body(1) =  flow_compensated(0) * range; // 计算体框架中的 Y 速度

		if (_flow_counter == 0) {
			_flow_vel_body_lpf.reset(_flow_vel_body); // 初始化低通滤波器
			_flow_counter = 1; // 更新计数器

		} else {
			_flow_vel_body_lpf.update(_flow_vel_body); // 更新低通滤波器
			_flow_counter++; // 增加计数器
		}

		// 检查是否在空中并需要光流来控制位置漂移
		bool is_flow_required = _control_status.flags.in_air
					&& (_control_status.flags.inertial_dead_reckoning // 正在进行惯性死 reckoning，因此必须紧急约束漂移
					    || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)); // 检查是否是唯一的水平辅助源

		const bool is_within_sensor_dist = (getHagl() >= _flow_min_distance) && (getHagl() <= _flow_max_distance); // 检查高度是否在传感器距离范围内

		const bool is_magnitude_good = flow_sample.flow_rate.isAllFinite() // 检查光流速率是否有限
					       && !flow_sample.flow_rate.longerThan(_flow_max_rate) // 检查光流速率是否超过最大速率
					       && !flow_compensated.longerThan(_flow_max_rate); // 检查补偿后的光流速率是否超过最大速率

		const bool continuing_conditions_passing = (_params.flow_ctrl == 1) // 检查光流控制参数是否启用
				&& _control_status.flags.tilt_align // 检查倾斜是否对齐
				&& is_within_sensor_dist; // 检查是否在传感器距离范围内

		const bool starting_conditions_passing = continuing_conditions_passing // 检查启动条件是否满足
				&& is_quality_good // 检查光流质量是否良好
				&& is_magnitude_good // 检查光流幅度是否良好
				&& is_tilt_good // 检查倾斜是否良好
				&& (_flow_counter > 10) // 检查光流计数器是否大于 10
				&& (isTerrainEstimateValid() || isHorizontalAidingActive()) // 检查地形估计是否有效或水平辅助是否激活
				&& isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)2e6); // 防止快速切换

		// 如果高度相对于地面，则无法观察地形高度。
		_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE); // 更新地形光流标志

		if (_control_status.flags.opt_flow) {
			if (continuing_conditions_passing) {

				if (is_quality_good && is_magnitude_good && is_tilt_good) {
					fuseOptFlow(H, _control_status.flags.opt_flow_terrain); // 融合光流数据
				}

				// 处理在依赖光流但长时间未使用的情况
				if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)) {
					if (is_flow_required && is_quality_good && is_magnitude_good) {
						resetFlowFusion(flow_sample); // 重置光流融合

						if (_control_status.flags.opt_flow_terrain && !isTerrainEstimateValid()) {
							resetTerrainToFlow(); // 重置地形到光流
						}

					} else {
						stopFlowFusion(); // 停止光流融合
					}
				}

			} else {
				stopFlowFusion(); // 停止光流融合
			}

		} else {
			if (starting_conditions_passing) {
				// 如果高度相对于地面，则无法观察地形高度。
				_control_status.flags.opt_flow_terrain = (_height_sensor_ref != HeightSensor::RANGE); // 更新地形光流标志

				if (isHorizontalAidingActive()) {
					if (fuseOptFlow(H, _control_status.flags.opt_flow_terrain)) {
						ECL_INFO("开始光流融合");
						_control_status.flags.opt_flow = true; // 启用光流标志

					} else if (_control_status.flags.opt_flow_terrain && !_control_status.flags.rng_terrain) {
						ECL_INFO("开始光流融合，重置地形");
						resetTerrainToFlow(); // 重置地形
						_control_status.flags.opt_flow = true; // 启用光流标志
					}

				} else {
					if (isTerrainEstimateValid() || (_height_sensor_ref == HeightSensor::RANGE)) {
						ECL_INFO("开始光流融合，重置");
						resetFlowFusion(flow_sample); // 重置光流融合
						_control_status.flags.opt_flow = true; // 启用光流标志

					} else if (_control_status.flags.opt_flow_terrain) {
						ECL_INFO("开始光流融合，重置地形");
						resetTerrainToFlow(); // 重置地形
						_control_status.flags.opt_flow = true; // 启用光流标志
					}
				}

				_control_status.flags.opt_flow_terrain = _control_status.flags.opt_flow && !(_height_sensor_ref == HeightSensor::RANGE); // 更新地形光流标志
			}
		}

	} else if (_control_status.flags.opt_flow && isTimedOut(_flow_sample_delayed.time_us, _params.reset_timeout_max)) {
		stopFlowFusion(); // 停止光流融合
	}
}

void Ekf::resetFlowFusion(const flowSample &flow_sample)
{
	ECL_INFO("重置速度到光流");
	_information_events.flags.reset_vel_to_flow = true; // 设置重置速度标志

	const float flow_vel_var = sq(predictFlowRange()) * calcOptFlowMeasVar(flow_sample); // 计算光流速度方差
	resetHorizontalVelocityTo(getFilteredFlowVelNE(), flow_vel_var); // 重置水平速度

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow); // 重置辅助源状态为零创新

	_innov_check_fail_status.flags.reject_optflow_X = false; // 重置光流 X 方向的创新检查失败状态
	_innov_check_fail_status.flags.reject_optflow_Y = false; // 重置光流 Y 方向的创新检查失败状态
}

void Ekf::resetTerrainToFlow()
{
	ECL_INFO("重置地形到光流");

	// TODO: 使用光流数据
	const float new_terrain = -_gpos.altitude() + _params.rng_gnd_clearance; // 计算新的地形高度
	const float delta_terrain = new_terrain - _state.terrain; // 计算地形高度变化
	_state.terrain = new_terrain; // 更新地形状态
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 100.f); // 设置地形状态的协方差方差

	resetAidSourceStatusZeroInnovation(_aid_src_optical_flow); // 重置辅助源状态为零创新

	_innov_check_fail_status.flags.reject_optflow_X = false; // 重置光流 X 方向的创新检查失败状态
	_innov_check_fail_status.flags.reject_optflow_Y = false; // 重置光流 Y 方向的创新检查失败状态

	// 记录状态变化
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain; // 更新地形变化

	} else {
		// 如果在此更新中已经有重置，则累积总变化
		_state_reset_status.hagl_change += delta_terrain; // 累积地形变化
	}

	_state_reset_status.reset_count.hagl++; // 增加重置计数
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("停止光流融合");
		_control_status.flags.opt_flow = false; // 禁用光流标志
		_control_status.flags.opt_flow_terrain = false; // 禁用地形光流标志

		_fault_status.flags.bad_optflow_X = false; // 重置光流 X 方向的故障状态
		_fault_status.flags.bad_optflow_Y = false; // 重置光流 Y 方向的故障状态

		_innov_check_fail_status.flags.reject_optflow_X = false; // 重置光流 X 方向的创新检查失败状态
		_innov_check_fail_status.flags.reject_optflow_Y = false; // 重置光流 Y 方向的创新检查失败状态

		_flow_counter = 0; // 重置光流计数器
	}
}

void Ekf::calcOptFlowBodyRateComp(const flowSample &flow_sample)
{
	// 使用组合低通滤波器和尖峰滤波器计算偏差估计
	_flow_gyro_bias = 0.99f * _flow_gyro_bias // 以 99% 的权重保留先前的偏差
			  + 0.01f * matrix::constrain(flow_sample.gyro_rate - _ref_body_rate, -0.1f, 0.1f); // 计算当前偏差并限制在 -0.1 到 0.1 之间
}
