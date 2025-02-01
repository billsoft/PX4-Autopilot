/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "EKF2Selector.hpp" // 引入EKF2选择器的头文件

using namespace time_literals; // 使用时间字面量
using matrix::Quatf; // 使用四元数类型
using matrix::Vector2f; // 使用二维向量类型
using math::constrain; // 使用约束函数
using math::radians; // 使用弧度转换函数

// EKF2Selector类的构造函数
EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr), // 初始化模块参数为nullptr
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers) // 调度工作项，名称为"ekf2_selector"
{
	_estimator_selector_status_pub.advertise(); // 广播估计器选择器状态
	_sensor_selection_pub.advertise(); // 广播传感器选择
	_vehicle_attitude_pub.advertise(); // 广播车辆姿态
	_vehicle_global_position_pub.advertise(); // 广播车辆全球位置
	_vehicle_local_position_pub.advertise(); // 广播车辆局部位置
	_vehicle_odometry_pub.advertise(); // 广播车辆里程计
	_wind_pub.advertise(); // 广播风速
}

// EKF2Selector类的析构函数
EKF2Selector::~EKF2Selector()
{
	Stop(); // 调用停止函数
}

// 停止函数，注销所有实例的回调
void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) { // 遍历所有实例
		_instance[i].estimator_attitude_sub.unregisterCallback(); // 注销姿态估计的回调
		_instance[i].estimator_status_sub.unregisterCallback(); // 注销状态估计的回调
	}

	ScheduleClear(); // 清除调度
}

// 打印实例变化的函数
void EKF2Selector::PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance)
{
	const char *old_reason = nullptr; // 旧实例的原因

	if (_instance[old_instance].filter_fault) { // 如果旧实例有滤波故障
		old_reason = " (filter fault)"; // 设置原因为滤波故障

	} else if (_instance[old_instance].timeout) { // 如果旧实例超时
		old_reason = " (timeout)"; // 设置原因为超时

	} else if (_gyro_fault_detected) { // 如果检测到陀螺仪故障
		old_reason = " (gyro fault)"; // 设置原因为陀螺仪故障

	} else if (_accel_fault_detected) { // 如果检测到加速度计故障
		old_reason = " (accel fault)"; // 设置原因为加速度计故障

	} else if (!_instance[_selected_instance].healthy.get_state() && (_instance[_selected_instance].healthy_count > 0)) {
		// 如果之前的实例从未健康（例如初始化时）
		old_reason = " (unhealthy)"; // 设置原因为不健康
	}

	const char *new_reason = nullptr; // 新实例的原因

	if (_request_instance.load() == new_instance) { // 如果请求的实例是新实例
		new_reason = " (user selected)"; // 设置原因为用户选择
	}

	if (old_reason || new_reason) { // 如果有旧原因或新原因
		if (old_reason == nullptr) {
			old_reason = ""; // 如果旧原因为空，设置为空字符串
		}

		if (new_reason == nullptr) {
			new_reason = ""; // 如果新原因为空，设置为空字符串
		}

		PX4_WARN("primary EKF changed %" PRIu8 "%s -> %" PRIu8 "%s", old_instance, old_reason, new_instance, new_reason); // 打印实例变化的警告信息
	}
}

// 选择实例的函数
bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != _selected_instance) && (ekf_instance < _available_instances)) { // 如果选择的实例不是当前实例且在可用实例范围内
		// 立即更新传感器选择
		sensor_selection_s sensor_selection{}; // 创建传感器选择结构体
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id; // 设置加速度计设备ID
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id; // 设置陀螺仪设备ID
		sensor_selection.timestamp = hrt_absolute_time(); // 设置时间戳
		_sensor_selection_pub.publish(sensor_selection); // 发布传感器选择

		if (_selected_instance != INVALID_INSTANCE) { // 如果当前实例有效
			// 切换回调注册
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback(); // 注销当前实例的姿态估计回调
			_instance[_selected_instance].estimator_status_sub.unregisterCallback(); // 注销当前实例的状态估计回调

			PrintInstanceChange(_selected_instance, ekf_instance); // 打印实例变化
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback(); // 注册新实例的姿态估计回调
		_instance[ekf_instance].estimator_status_sub.registerCallback(); // 注册新实例的状态估计回调

		_selected_instance = ekf_instance; // 更新当前选择的实例
		_instance_changed_count++; // 实例变化计数加一
		_last_instance_change = sensor_selection.timestamp; // 更新最后实例变化时间
		_instance[ekf_instance].time_last_selected = _last_instance_change; // 更新新实例的最后选择时间

		// 重置所有相对测试比率
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0; // 将每个实例的相对测试比率重置为0
		}

		return true; // 返回选择成功
	}

	return false; // 返回选择失败
}

// 更新错误评分的函数
bool EKF2Selector::UpdateErrorScores()
{
	// 首先检查IMU不一致性
	_gyro_fault_detected = false; // 初始化陀螺仪故障检测为假
	uint32_t faulty_gyro_id = 0; // 初始化故障陀螺仪ID
	_accel_fault_detected = false; // 初始化加速度计故障检测为假
	uint32_t faulty_accel_id = 0; // 初始化故障加速度计ID

	if (_sensors_status_imu.updated()) { // 如果IMU传感器状态更新
		sensors_status_imu_s sensors_status_imu; // 创建IMU传感器状态结构体

		if (_sensors_status_imu.copy(&sensors_status_imu)) { // 复制传感器状态

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f); // 计算时间步长，限制在0到0.02秒之间
			_last_update_us = sensors_status_imu.timestamp; // 更新最后更新时间戳

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get()); // 获取角速度阈值并转换为弧度
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get()); // 获取角度阈值并转换为弧度
				uint8_t n_gyros = 0; // 初始化陀螺仪数量
				uint8_t n_gyro_exceedances = 0; // 初始化超出阈值的陀螺仪数量
				float largest_accumulated_gyro_error = 0.0f; // 初始化最大累计陀螺仪误差
				uint8_t largest_gyro_error_index = 0; // 初始化最大误差索引

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) { // 遍历IMU状态数组
					// 检查陀螺仪与均值的过大差异，使用累计误差
					if (sensors_status_imu.gyro_device_ids[i] != 0) { // 如果陀螺仪设备ID有效
						n_gyros++; // 增加陀螺仪数量
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s; // 更新累计误差
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f); // 确保累计误差不小于0

						if (_accumulated_gyro_error[i] > angle_threshold) { // 如果累计误差超过角度阈值
							n_gyro_exceedances++; // 增加超出数量
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) { // 如果当前累计误差大于最大累计误差
							largest_accumulated_gyro_error = _accumulated_gyro_error[i]; // 更新最大累计误差
							largest_gyro_error_index = i; // 更新最大误差索引
						}

					} else {
						// 没有传感器
						_accumulated_gyro_error[i] = NAN; // 设置为NAN表示没有传感器
					}
				}

				if (n_gyro_exceedances > 0) { // 如果有超出阈值的陀螺仪
					if (n_gyros >= 3) { // 如果陀螺仪数量大于等于3
						// 如果有3个或更多传感器，具有最大累计误差的传感器被认为是故障
						_gyro_fault_detected = true; // 检测到陀螺仪故障
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index]; // 记录故障陀螺仪ID

					} else if (n_gyros == 2) { // 如果陀螺仪数量为2
						// 存在故障，但无法确定故障传感器的身份
						_gyro_fault_detected = true; // 检测到陀螺仪故障
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get(); // 获取加速度阈值
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get(); // 获取速度阈值
				uint8_t n_accels = 0; // 初始化加速度计数量
				uint8_t n_accel_exceedances = 0; // 初始化超出阈值的加速度计数量
				float largest_accumulated_accel_error = 0.0f; // 初始化最大累计加速度误差
				uint8_t largest_accel_error_index = 0; // 初始化最大加速度误差索引

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) { // 遍历IMU状态数组
					// 检查加速度计与均值的过大差异，使用累计误差
					if (sensors_status_imu.accel_device_ids[i] != 0) { // 如果加速度计设备ID有效
						n_accels++; // 增加加速度计数量
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s; // 更新累计误差
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f); // 确保累计误差不小于0

						if (_accumulated_accel_error[i] > velocity_threshold) { // 如果累计误差超过速度阈值
							n_accel_exceedances++; // 增加超出数量
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) { // 如果当前累计误差大于最大累计误差
							largest_accumulated_accel_error = _accumulated_accel_error[i]; // 更新最大累计误差
							largest_accel_error_index = i; // 更新最大误差索引
						}

					} else {
						// 没有传感器
						_accumulated_accel_error[i] = NAN; // 设置为NAN表示没有传感器
					}
				}

				if (n_accel_exceedances > 0) { // 如果有超出阈值的加速度计
					if (n_accels >= 3) { // 如果加速度计数量大于等于3
						// 如果有3个或更多传感器，具有最大累计误差的传感器被认为是故障
						_accel_fault_detected = true; // 检测到加速度计故障
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index]; // 记录故障加速度计ID

					} else if (n_accels == 2) { // 如果加速度计数量为2
						// 存在故障，但无法确定故障传感器的身份
						_accel_fault_detected = true; // 检测到加速度计故障
					}
				}
			}
		}
	}

	bool updated = false; // 初始化更新标志为假
	bool primary_updated = false; // 初始化主更新标志为假

	// 默认估计器超时
	const hrt_abstime status_timeout = 50_ms; // 设置状态超时为50毫秒

	// 计算各个实例的错误评分
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) { // 遍历所有实例
		const bool prev_healthy = _instance[i].healthy.get_state(); // 获取之前的健康状态

		estimator_status_s status; // 创建估计器状态结构体

		if (_instance[i].estimator_status_sub.update(&status)) { // 更新估计器状态

			_instance[i].timestamp_last = status.timestamp; // 更新最后时间戳

			_instance[i].accel_device_id = status.accel_device_id; // 更新加速度计设备ID
			_instance[i].gyro_device_id = status.gyro_device_id; // 更新陀螺仪设备ID
			_instance[i].baro_device_id = status.baro_device_id; // 更新气压计设备ID
			_instance[i].mag_device_id = status.mag_device_id; // 更新磁力计设备ID

			if ((i + 1) > _available_instances) { // 如果当前实例数量超过可用实例
				_available_instances = i + 1; // 更新可用实例数量
				updated = true; // 设置更新标志为真
			}

			if (i == _selected_instance) { // 如果当前实例是选择的实例
				primary_updated = true; // 设置主更新标志为真
			}

			// 测试比率为0时无效，>=1为失败
			if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) { // 如果速度测试比率无效或小于等于0
				status.vel_test_ratio = 1.f; // 设置速度测试比率为1
			}

			if (!PX4_ISFINITE(status.pos_test_ratio) || (status.pos_test_ratio <= 0.f)) { // 如果位置测试比率无效或小于等于0
				status.pos_test_ratio = 1.f; // 设置位置测试比率为1
			}

			if (!PX4_ISFINITE(status.hgt_test_ratio) || (status.hgt_test_ratio <= 0.f)) { // 如果高度测试比率无效或小于等于0
				status.hgt_test_ratio = 1.f; // 设置高度测试比率为1
			}

			float combined_test_ratio = fmaxf(0.5f * (status.vel_test_ratio + status.pos_test_ratio), status.hgt_test_ratio); // 计算综合测试比率

			_instance[i].combined_test_ratio = combined_test_ratio; // 更新综合测试比率

			const bool healthy = (status.filter_fault_flags == 0) && (combined_test_ratio > 0.f); // 判断当前实例是否健康
			_instance[i].healthy.set_state_and_update(healthy, status.timestamp); // 更新健康状态

			_instance[i].warning = (combined_test_ratio >= 1.f); // 设置警告标志
			_instance[i].filter_fault = (status.filter_fault_flags != 0); // 设置滤波故障标志
			_instance[i].timeout = false; // 设置超时标志为假

			if (!_instance[i].warning) { // 如果没有警告
				_instance[i].time_last_no_warning = status.timestamp; // 更新最后无警告时间
			}

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) { // 如果相对测试比率无效
				_instance[i].relative_test_ratio = 0; // 将相对测试比率重置为0
			}

		} else if (!_instance[i].timeout && (hrt_elapsed_time(&_instance[i].timestamp_last) > status_timeout)) { // 如果未超时且最后时间超过超时
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time()); // 设置健康状态为不健康
			_instance[i].timeout = true; // 设置超时标志为真
		}

		// 如果EKF使用的陀螺仪故障，立即声明EKF不健康
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time()); // 设置健康状态为不健康
		}

		// 如果EKF使用的加速度计故障，立即声明EKF不健康
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time()); // 设置健康状态为不健康
		}

		if (prev_healthy != _instance[i].healthy.get_state()) { // 如果健康状态发生变化
			updated = true; // 设置更新标志为真
			_selector_status_publish = true; // 设置选择器状态发布标志为真

			if (!prev_healthy) { // 如果之前不健康
				_instance[i].healthy_count++; // 健康计数加一
			}
		}
	}

	// 如果主实例已更新，则更新相对测试比率
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) { // 遍历所有可用实例
			if (i != _selected_instance) { // 如果不是当前选择的实例

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio; // 计算误差差值

				// 只有在相对测试比率比主实例好时才减少误差，以防止不必要的选择变化
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f); // 设置阈值

				if (error_delta > 0 || error_delta < -threshold) { // 如果误差差值大于0或小于负阈值
					_instance[i].relative_test_ratio += error_delta; // 更新相对测试比率
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim); // 限制相对测试比率在范围内

					if ((error_delta < -threshold) && (_instance[i].relative_test_ratio < 1.f)) { // 如果误差差值小于负阈值且相对测试比率小于1
						// 如果有向潜在实例变化的趋势，增加状态发布频率
						_selector_status_publish = true; // 设置选择器状态发布标志为真
					}
				}
			}
		}
	}

	return (primary_updated || updated); // 返回主更新或更新标志
}

// 发布车辆姿态的函数
void EKF2Selector::PublishVehicleAttitude()
{
	// 选择的姿态估计器 -> 车辆姿态
	vehicle_attitude_s attitude; // 创建车辆姿态结构体

	if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) { // 更新姿态估计
		bool instance_change = false; // 初始化实例变化标志为假

		if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) { // 如果当前实例与之前的实例不同
			_attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance(); // 更新之前的实例
			instance_change = true; // 设置实例变化标志为真
		}

		if (_attitude_last.timestamp != 0) { // 如果上一次姿态的时间戳不为0
			if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) { // 如果没有实例变化且四元数重置计数增加
				// 从估计器数据传播增量，同时保持整体重置计数
				++_quat_reset_counter; // 四元数重置计数加一
				_delta_q_reset = Quatf{attitude.delta_q_reset}; // 更新增量四元数

			} else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) { // 如果实例变化或四元数重置计数不同
				// 在重置时计算从最后发布数据的增量
				++_quat_reset_counter; // 四元数重置计数加一
				_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized(); // 计算增量四元数
			}

		} else {
			_quat_reset_counter = attitude.quat_reset_counter; // 更新四元数重置计数
			_delta_q_reset = Quatf{attitude.delta_q_reset}; // 更新增量四元数
		}

		bool publish = true; // 初始化发布标志为真

		// 确保时间戳样本单调递增，通过重置，不发布
		// 估计器的姿态对于系统（车辆姿态）如果过时
		if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample) // 如果时间戳样本小于等于上一次的时间戳样本
		    || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) { // 或者时间戳过期
			publish = false; // 设置发布标志为假
		}

		// 保存最后的主要姿态估计，带有原始重置
		_attitude_last = attitude; // 更新最后的姿态

		if (publish) { // 如果发布标志为真
			// 重新发布，带有总重置计数和当前时间戳
			attitude.quat_reset_counter = _quat_reset_counter; // 设置四元数重置计数
			_delta_q_reset.copyTo(attitude.delta_q_reset); // 复制增量四元数到姿态

			attitude.timestamp = hrt_absolute_time(); // 设置当前时间戳
			_vehicle_attitude_pub.publish(attitude); // 发布车辆姿态
		}
	}
}

// 发布车辆局部位置的函数
void EKF2Selector::PublishVehicleLocalPosition()
{
	// 选择的局部位置估计器 -> 车辆局部位置
	vehicle_local_position_s local_position; // 创建车辆局部位置结构体

	if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) { // 更新局部位置估计
		bool instance_change = false; // 初始化实例变化标志为假

		if (_instance[_selected_instance].estimator_local_position_sub.get_instance() != _local_position_instance_prev) { // 如果当前实例与之前的实例不同
			_local_position_instance_prev = _instance[_selected_instance].estimator_local_position_sub.get_instance(); // 更新之前的实例
			instance_change = true; // 设置实例变化标志为真
		}

		if (_local_position_last.timestamp != 0) { // 如果上一次局部位置的时间戳不为0
			// XY重置
			if (!instance_change && (local_position.xy_reset_counter == _local_position_last.xy_reset_counter + 1)) { // 如果没有实例变化且XY重置计数增加
				++_xy_reset_counter; // XY重置计数加一
				_delta_xy_reset = Vector2f{local_position.delta_xy}; // 更新增量XY

			} else if (instance_change || (local_position.xy_reset_counter != _local_position_last.xy_reset_counter)) { // 如果实例变化或XY重置计数不同
				++_xy_reset_counter; // XY重置计数加一
				_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y}; // 计算增量XY
			}

			// Z重置
			if (!instance_change && (local_position.z_reset_counter == _local_position_last.z_reset_counter + 1)) { // 如果没有实例变化且Z重置计数增加
				++_z_reset_counter; // Z重置计数加一
				_delta_z_reset = local_position.delta_z; // 更新增量Z

			} else if (instance_change || (local_position.z_reset_counter != _local_position_last.z_reset_counter)) { // 如果实例变化或Z重置计数不同
				++_z_reset_counter; // Z重置计数加一
				_delta_z_reset = local_position.z - _local_position_last.z; // 计算增量Z
			}

			// VXY重置
			if (!instance_change && (local_position.vxy_reset_counter == _local_position_last.vxy_reset_counter + 1)) { // 如果没有实例变化且VXY重置计数增加
				++_vxy_reset_counter; // VXY重置计数加一
				_delta_vxy_reset = Vector2f{local_position.delta_vxy}; // 更新增量VXY

			} else if (instance_change || (local_position.vxy_reset_counter != _local_position_last.vxy_reset_counter)) { // 如果实例变化或VXY重置计数不同
				++_vxy_reset_counter; // VXY重置计数加一
				_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy}; // 计算增量VXY
			}

			// VZ重置
			if (!instance_change && (local_position.vz_reset_counter == _local_position_last.vz_reset_counter + 1)) { // 如果没有实例变化且VZ重置计数增加
				++_vz_reset_counter; // VZ重置计数加一
				_delta_vz_reset = local_position.delta_vz; // 更新增量VZ

			} else if (instance_change || (local_position.vz_reset_counter != _local_position_last.vz_reset_counter)) { // 如果实例变化或VZ重置计数不同
				++_vz_reset_counter; // VZ重置计数加一
				_delta_vz_reset = local_position.vz - _local_position_last.vz; // 计算增量VZ
			}

			// 方向重置
			if (!instance_change && (local_position.heading_reset_counter == _local_position_last.heading_reset_counter + 1)) { // 如果没有实例变化且方向重置计数增加
				++_heading_reset_counter; // 方向重置计数加一
				_delta_heading_reset = local_position.delta_heading; // 更新增量方向

			} else if (instance_change || (local_position.heading_reset_counter != _local_position_last.heading_reset_counter)) { // 如果实例变化或方向重置计数不同
				++_heading_reset_counter; // 方向重置计数加一
				_delta_heading_reset = matrix::wrap_pi(local_position.heading - _local_position_last.heading); // 计算增量方向
			}

			// HAGL（底部距离）重置
			if (!instance_change
			    && (local_position.dist_bottom_reset_counter == _local_position_last.dist_bottom_reset_counter + 1)) { // 如果没有实例变化且底部距离重置计数增加
				++_hagl_reset_counter; // HAGL重置计数加一
				_delta_hagl_reset = local_position.delta_dist_bottom; // 更新增量底部距离

			} else if (instance_change
				   || (local_position.dist_bottom_reset_counter != _local_position_last.dist_bottom_reset_counter)) { // 如果实例变化或底部距离重置计数不同
				++_hagl_reset_counter; // HAGL重置计数加一
				_delta_hagl_reset = local_position.dist_bottom - _local_position_last.dist_bottom; // 计算增量底部距离
			}

		} else {
			_xy_reset_counter = local_position.xy_reset_counter; // 更新XY重置计数
			_z_reset_counter = local_position.z_reset_counter; // 更新Z重置计数
			_vxy_reset_counter = local_position.vxy_reset_counter; // 更新VXY重置计数
			_vz_reset_counter = local_position.vz_reset_counter; // 更新VZ重置计数
			_heading_reset_counter = local_position.heading_reset_counter; // 更新方向重置计数
			_hagl_reset_counter = local_position.dist_bottom_reset_counter; // 更新HAGL重置计数

			_delta_xy_reset = Vector2f{local_position.delta_xy}; // 更新增量XY
			_delta_z_reset = local_position.delta_z; // 更新增量Z
			_delta_vxy_reset = Vector2f{local_position.delta_vxy}; // 更新增量VXY
			_delta_vz_reset = local_position.delta_vz; // 更新增量VZ
			_delta_heading_reset = local_position.delta_heading; // 更新增量方向
			_delta_hagl_reset = local_position.dist_bottom; // 更新增量底部距离
		}

		bool publish = true; // 初始化发布标志为真

		// 确保时间戳样本单调递增，通过重置，不发布
		// 估计器的局部位置对于系统（车辆局部位置）如果过时
		if ((local_position.timestamp_sample <= _local_position_last.timestamp_sample) // 如果时间戳样本小于等于上一次的时间戳样本
		    || (hrt_elapsed_time(&local_position.timestamp) > 20_ms)) { // 或者时间戳过期
			publish = false; // 设置发布标志为假
		}

		// 保存最后的主要局部位置估计，带有原始重置
		_local_position_last = local_position; // 更新最后的局部位置

		if (publish) { // 如果发布标志为真
			// 重新发布，带有总重置计数和当前时间戳
			local_position.xy_reset_counter = _xy_reset_counter; // 设置XY重置计数
			local_position.z_reset_counter = _z_reset_counter; // 设置Z重置计数
			local_position.vxy_reset_counter = _vxy_reset_counter; // 设置VXY重置计数
			local_position.vz_reset_counter = _vz_reset_counter; // 设置VZ重置计数
			local_position.heading_reset_counter = _heading_reset_counter; // 设置方向重置计数
			local_position.dist_bottom_reset_counter = _hagl_reset_counter; // 设置HAGL重置计数

			_delta_xy_reset.copyTo(local_position.delta_xy); // 复制增量XY到局部位置
			local_position.delta_z = _delta_z_reset; // 设置增量Z
			_delta_vxy_reset.copyTo(local_position.delta_vxy); // 复制增量VXY到局部位置
			local_position.delta_vz = _delta_vz_reset; // 设置增量VZ
			local_position.delta_heading = _delta_heading_reset; // 设置增量方向

			local_position.timestamp = hrt_absolute_time(); // 设置当前时间戳
			_vehicle_local_position_pub.publish(local_position); // 发布车辆局部位置
		}
	}
}

// 发布车辆里程计的函数
void EKF2Selector::PublishVehicleOdometry()
{
	// 选择的里程计估计器 -> 车辆里程计
	vehicle_odometry_s odometry; // 创建车辆里程计结构体

	if (_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) { // 更新里程计估计
		bool instance_change = false; // 初始化实例变化标志为假

		if (_instance[_selected_instance].estimator_odometry_sub.get_instance() != _odometry_instance_prev) { // 如果当前实例与之前的实例不同
			_odometry_instance_prev = _instance[_selected_instance].estimator_odometry_sub.get_instance(); // 更新之前的实例
			instance_change = true; // 设置实例变化标志为真
		}

		if (_odometry_last.timestamp != 0) { // 如果上一次里程计的时间戳不为0
			// 重置
			if (instance_change || (odometry.reset_counter != _odometry_last.reset_counter)) { // 如果实例变化或重置计数不同
				++_odometry_reset_counter; // 里程计重置计数加一
			}

		} else {
			_odometry_reset_counter = odometry.reset_counter; // 更新里程计重置计数
		}

		bool publish = true; // 初始化发布标志为真

		// 确保时间戳样本单调递增，通过重置，不发布
		// 估计器的里程计对于系统（车辆里程计）如果过时
		if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample) // 如果时间戳样本小于等于上一次的时间戳样本
		    || (hrt_elapsed_time(&odometry.timestamp) > 20_ms)) { // 或者时间戳过期
			publish = false; // 设置发布标志为假
		}

		// 保存最后的主要里程计估计，带有原始重置
		_odometry_last = odometry; // 更新最后的里程计

		if (publish) { // 如果发布标志为真
			// 重新发布，带有总重置计数和当前时间戳
			odometry.reset_counter = _odometry_reset_counter; // 设置里程计重置计数

			odometry.timestamp = hrt_absolute_time(); // 设置当前时间戳
			_vehicle_odometry_pub.publish(odometry); // 发布车辆里程计
		}
	}
}

// 发布车辆全球位置的函数
void EKF2Selector::PublishVehicleGlobalPosition()
{
	// 选择的全球位置估计器 -> 车辆全球位置
	vehicle_global_position_s global_position; // 创建车辆全球位置结构体

	if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) { // 更新全球位置估计
		bool instance_change = false; // 初始化实例变化标志为假

		if (_instance[_selected_instance].estimator_global_position_sub.get_instance() != _global_position_instance_prev) { // 如果当前实例与之前的实例不同
			_global_position_instance_prev = _instance[_selected_instance].estimator_global_position_sub.get_instance(); // 更新之前的实例
			instance_change = true; // 设置实例变化标志为真
		}

		if (_global_position_last.timestamp != 0) { // 如果上一次全球位置的时间戳不为0
			// 纬度/经度重置
			if (!instance_change && (global_position.lat_lon_reset_counter == _global_position_last.lat_lon_reset_counter + 1)) { // 如果没有实例变化且纬度/经度重置计数增加
				++_lat_lon_reset_counter; // 纬度/经度重置计数加一

				// TODO: 增量纬度/经度
				_delta_lat_reset = global_position.lat - _global_position_last.lat; // 计算增量纬度
				_delta_lon_reset = global_position.lon - _global_position_last.lon; // 计算增量经度

			} else if (instance_change || (global_position.lat_lon_reset_counter != _global_position_last.lat_lon_reset_counter)) { // 如果实例变化或纬度/经度重置计数不同
				++_lat_lon_reset_counter; // 纬度/经度重置计数加一

				_delta_lat_reset = global_position.lat - _global_position_last.lat; // 计算增量纬度
				_delta_lon_reset = global_position.lon - _global_position_last.lon; // 计算增量经度
			}

			// 高度重置
			if (!instance_change && (global_position.alt_reset_counter == _global_position_last.alt_reset_counter + 1)) { // 如果没有实例变化且高度重置计数增加
				++_alt_reset_counter; // 高度重置计数加一
				_delta_alt_reset = global_position.delta_alt; // 更新增量高度

			} else if (instance_change || (global_position.alt_reset_counter != _global_position_last.alt_reset_counter)) { // 如果实例变化或高度重置计数不同
				++_alt_reset_counter; // 高度重置计数加一
				_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt; // 计算增量高度
			}

			// 地形重置
			if (!instance_change && (global_position.terrain_reset_counter == _global_position_last.terrain_reset_counter + 1)) { // 如果没有实例变化且地形重置计数增加
				++_terrain_reset_counter; // 地形重置计数加一
				_delta_terrain_reset = global_position.delta_terrain; // 更新增量地形

			} else if (instance_change || (global_position.terrain_reset_counter != _global_position_last.terrain_reset_counter)) { // 如果实例变化或地形重置计数不同
				++_terrain_reset_counter; // 地形重置计数加一
				_delta_terrain_reset = global_position.delta_terrain - _global_position_last.delta_terrain; // 计算增量地形
			}

		} else {
			_lat_lon_reset_counter = global_position.lat_lon_reset_counter; // 更新纬度/经度重置计数
			_alt_reset_counter = global_position.alt_reset_counter; // 更新高度重置计数

			_delta_alt_reset = global_position.delta_alt; // 更新增量高度
		}

		bool publish = true; // 初始化发布标志为真

		// 确保时间戳样本单调递增，通过重置，不发布
		// 估计器的全球位置对于系统（车辆全球位置）如果过时
		if ((global_position.timestamp_sample <= _global_position_last.timestamp_sample) // 如果时间戳样本小于等于上一次的时间戳样本
		    || (hrt_elapsed_time(&global_position.timestamp) > 20_ms)) { // 或者时间戳过期
			publish = false; // 设置发布标志为假
		}

		// 保存最后的主要全球位置估计，带有原始重置
		_global_position_last = global_position; // 更新最后的全球位置

		if (publish) { // 如果发布标志为真
			// 重新发布，带有总重置计数和当前时间戳
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter; // 设置纬度/经度重置计数
			global_position.alt_reset_counter = _alt_reset_counter; // 设置高度重置计数
			global_position.delta_alt = _delta_alt_reset; // 设置增量高度

			global_position.timestamp = hrt_absolute_time(); // 设置当前时间戳
			_vehicle_global_position_pub.publish(global_position); // 发布车辆全球位置
		}
	}
}

// 发布风速估计的函数
void EKF2Selector::PublishWindEstimate()
{
	// 选择的风速估计器 -> 风速
	wind_s wind; // 创建风速结构体

	if (_instance[_selected_instance].estimator_wind_sub.update(&wind)) { // 更新风速估计
		bool publish = true; // 初始化发布标志为真

		// 确保时间戳样本单调递增，通过重置，不发布
		// 估计器的风速对于系统（风速）如果过时
		if ((wind.timestamp_sample <= _wind_last.timestamp_sample) // 如果时间戳样本小于等于上一次的时间戳样本
		    || (hrt_elapsed_time(&wind.timestamp) > 100_ms)) { // 或者时间戳过期
			publish = false; // 设置发布标志为假
		}

		// 保存最后的主要风速
		_wind_last = wind; // 更新最后的风速

		// 发布估计器的风速对于系统，除非它过时
		if (publish) { // 如果发布标志为真
			// 重新发布，带有当前时间戳
			wind.timestamp = hrt_absolute_time(); // 设置当前时间戳
			_wind_pub.publish(wind); // 发布风速
		}
	}
}

// 运行函数
void EKF2Selector::Run()
{
	// 检查参数更新
	if (_parameter_update_sub.updated()) { // 如果参数更新
		// 清除更新
		parameter_update_s pupdate; // 创建参数更新结构体
		_parameter_update_sub.copy(&pupdate); // 复制参数更新

		// 从存储中更新参数
		updateParams(); // 更新参数
	}

	// 更新所有估计器的综合测试比率
	const bool updated = UpdateErrorScores(); // 更新错误评分并获取更新标志

	// 如果没有有效实例，则强制选择第一个有效IMU实例
	if (_selected_instance == INVALID_INSTANCE) { // 如果当前选择的实例无效
		for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) { // 遍历所有实例
			if ((_instance[i].accel_device_id != 0) // 如果加速度计设备ID有效
			    && (_instance[i].gyro_device_id != 0)) { // 且陀螺仪设备ID有效

				if (SelectInstance(i)) { // 尝试选择当前实例
					break; // 选择成功则跳出循环
				}
			}
		}

		// 如果仍然无效，则提前返回并在下一个调度运行时检查
		if (_selected_instance == INVALID_INSTANCE) { // 如果当前选择的实例仍然无效
			ScheduleDelayed(100_ms); // 延迟100毫秒后重新调度
			return; // 返回
		}
	}

	if (updated) { // 如果有更新
		const uint8_t available_instances_prev = _available_instances; // 保存之前的可用实例数量
		const uint8_t selected_instance_prev = _selected_instance; // 保存之前选择的实例
		const uint32_t instance_changed_count_prev = _instance_changed_count; // 保存之前的实例变化计数
		const hrt_abstime last_instance_change_prev = _last_instance_change; // 保存之前的最后实例变化时间

		bool lower_error_available = false; // 初始化较低错误可用标志为假
		float alternative_error = 0.f; // 寻找错误低于当前主实例的实例
		float best_test_ratio = FLT_MAX; // 初始化最佳测试比率为最大值

		uint8_t best_ekf = _selected_instance; // 初始化最佳EKF为当前选择的实例
		uint8_t best_ekf_alternate = INVALID_INSTANCE; // 初始化最佳替代EKF为无效实例
		uint8_t best_ekf_different_imu = INVALID_INSTANCE; // 初始化最佳不同IMU的EKF为无效实例

		// 遍历所有可用实例，寻找是否有替代实例
		for (int i = 0; i < _available_instances; i++) { // 遍历所有可用实例
			// 如果健康且最近更新
			// 且
			// 相对错误小于选择的实例且未在至少10秒内被选择
			// 或者
			// 选择的实例已停止更新
			if (_instance[i].healthy.get_state() && (i != _selected_instance)) { // 如果当前实例健康且不是选择的实例
				const float test_ratio = _instance[i].combined_test_ratio; // 获取当前实例的综合测试比率
				const float relative_error = _instance[i].relative_test_ratio; // 获取当前实例的相对测试比率

				if (relative_error < alternative_error) { // 如果相对错误小于替代错误
					best_ekf_alternate = i; // 更新最佳替代EKF
					alternative_error = relative_error; // 更新替代错误

					// 相对错误小于选择的实例且未在至少10秒内被选择
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) { // 如果相对错误小于阈值且距离上次选择时间超过10秒
						lower_error_available = true; // 设置较低错误可用标志为真
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) { // 如果测试比率大于0且小于最佳测试比率
					best_ekf = i; // 更新最佳EKF
					best_test_ratio = test_ratio; // 更新最佳测试比率

					// 还检查使用不同IMU的下一个最佳可用EKF
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) { // 如果当前实例的加速度计ID与选择的实例不同
						best_ekf_different_imu = i; // 更新最佳不同IMU的EKF
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy.get_state()) { // 如果当前选择的实例不健康
			// 优先选择最佳健康实例，使用不同的IMU
			if (!SelectInstance(best_ekf_different_imu)) { // 尝试选择最佳不同IMU的EKF
				// 否则切换到健康实例，具有最佳整体测试比率
				SelectInstance(best_ekf); // 选择最佳EKF
			}

		} else if (lower_error_available // 如果较低错误可用
			   && ((hrt_elapsed_time(&_last_instance_change) > 10_s) // 且距离上次实例变化超过10秒
			       || (_instance[_selected_instance].warning // 或者当前选择的实例有警告
				   && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) { // 且距离上次无警告时间超过1秒

			// 如果此实例的相对错误显著低于活动主实例，我们认为它是更好的实例
			// 并希望切换到它，即使当前主实例健康
			SelectInstance(best_ekf_alternate); // 选择最佳替代EKF

		} else if (_request_instance.load() != INVALID_INSTANCE) { // 如果请求的实例有效

			const uint8_t new_instance = _request_instance.load(); // 加载请求的实例

			// 尝试切换到用户手动选择的实例
			if (!SelectInstance(new_instance)) { // 如果选择失败
				PX4_ERR("unable to switch to user selected instance %d", new_instance); // 打印错误信息
			}

			// 重置请求实例
			_request_instance.store(INVALID_INSTANCE); // 将请求实例重置为无效
		}

		// 每秒发布选择器状态，或在任何变化时立即发布
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s) // 如果选择器状态发布标志为真或距离上次发布超过1秒
		    || (available_instances_prev != _available_instances) // 或者可用实例数量变化
		    || (selected_instance_prev != _selected_instance) // 或者选择的实例变化
		    || (instance_changed_count_prev != _instance_changed_count) // 或者实例变化计数变化
		    || (last_instance_change_prev != _last_instance_change) // 或者最后实例变化时间变化
		    || _accel_fault_detected || _gyro_fault_detected) { // 或者检测到加速度计或陀螺仪故障

			PublishEstimatorSelectorStatus(); // 发布估计器选择器状态
			_selector_status_publish = false; // 重置选择器状态发布标志
		}
	}

	// 重新发布选择的估计器数据到系统
	PublishVehicleAttitude(); // 发布车辆姿态
	PublishVehicleLocalPosition(); // 发布车辆局部位置
	PublishVehicleGlobalPosition(); // 发布车辆全球位置
	PublishVehicleOdometry(); // 发布车辆里程计
	PublishWindEstimate(); // 发布风速估计

	// 重新调度作为备份超时
	ScheduleDelayed(FILTER_UPDATE_PERIOD); // 调度下一个更新周期
}

// 发布估计器选择器状态的函数
void EKF2Selector::PublishEstimatorSelectorStatus()
{
	estimator_selector_status_s selector_status{}; // 创建估计器选择器状态结构体
	selector_status.primary_instance = _selected_instance; // 设置主要实例为当前选择的实例
	selector_status.instances_available = _available_instances; // 设置可用实例数量
	selector_status.instance_changed_count = _instance_changed_count; // 设置实例变化计数
	selector_status.last_instance_change = _last_instance_change; // 设置最后实例变化时间
	selector_status.accel_device_id = _instance[_selected_instance].accel_device_id; // 设置加速度计设备ID
	selector_status.baro_device_id = _instance[_selected_instance].baro_device_id; // 设置气压计设备ID
	selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id; // 设置陀螺仪设备ID
	selector_status.mag_device_id = _instance[_selected_instance].mag_device_id; // 设置磁力计设备ID
	selector_status.gyro_fault_detected = _gyro_fault_detected; // 设置陀螺仪故障检测标志
	selector_status.accel_fault_detected = _accel_fault_detected; // 设置加速度计故障检测标志

	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) { // 遍历所有实例
		selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio; // 设置综合测试比率
		selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio; // 设置相对测试比率
		selector_status.healthy[i] = _instance[i].healthy.get_state(); // 设置健康状态
	}

	for (int i = 0; i < IMU_STATUS_SIZE; i++) { // 遍历IMU状态数组
		selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i]; // 设置累计陀螺仪误差
		selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i]; // 设置累计加速度误差
	}

	selector_status.timestamp = hrt_absolute_time(); // 设置当前时间戳
	_estimator_selector_status_pub.publish(selector_status); // 发布估计器选择器状态
	_last_status_publish = selector_status.timestamp; // 更新最后状态发布的时间戳
}

// 打印状态的函数
void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %" PRIu8, _available_instances); // 打印可用实例数量

	if (_selected_instance == INVALID_INSTANCE) { // 如果当前选择的实例无效
		PX4_WARN("selected instance: None"); // 打印警告信息
	}

	for (int i = 0; i < _available_instances; i++) { // 遍历所有可用实例
		const EstimatorInstance &inst = _instance[i]; // 获取当前实例

		PX4_INFO("%" PRIu8 ": ACC: %" PRIu32 ", GYRO: %" PRIu32 ", MAG: %" PRIu32 ", %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id, // 打印实例信息
			 inst.healthy.get_state() ? "healthy" : "unhealthy", // 打印健康状态
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio, // 打印测试比率
			 (_selected_instance == i) ? "*" : ""); // 如果是选择的实例，打印星号
	}
}
