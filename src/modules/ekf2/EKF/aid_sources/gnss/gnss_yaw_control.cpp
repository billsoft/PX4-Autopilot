/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，无论是否修改，均允许，前提是满足以下条件：
 *
 * 1. 再分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 再分发的二进制形式必须在随分发的文档和/或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. 未经特定的事先书面许可，PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于对适销性和特定用途的隐含保证均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file gnss_yaw_control.cpp
 * 定义了使用GNSS双天线测量获得的航向所需的函数。
 * 方程由src/modules/ekf2/EKF/python/ekf_derivation/derivation.py生成。
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

#include <ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h>

// 控制GNSS航向融合的函数
void Ekf::controlGnssYawFusion(const gnssSample &gnss_sample)
{
	// 检查GNSS控制参数是否启用航向控制，或是否存在航向故障
	if (!(_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
	    || _control_status.flags.gnss_yaw_fault) {

		stopGnssYawFusion(); // 停止GNSS航向融合
		return; // 退出函数
	}

	// 检查GNSS样本中的航向数据是否有效
	const bool is_new_data_available = PX4_ISFINITE(gnss_sample.yaw);

	if (is_new_data_available) {

		updateGnssYaw(gnss_sample); // 更新GNSS航向

		const bool continuing_conditions_passing = _control_status.flags.tilt_align; // 检查是否通过倾斜对齐条件

		// 检查GNSS航向数据是否间歇性可用
		const bool is_gnss_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		// 检查启动条件是否通过
		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& !is_gnss_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gnss_yaw) {
			if (continuing_conditions_passing) {

				fuseGnssYaw(gnss_sample.yaw_offset); // 融合GNSS航向偏移

				// 检查融合是否超时
				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					stopGnssYawFusion(); // 停止GNSS航向融合

					// 在起飞前，如果停止了融合，不希望继续依赖当前航向
					if (!_control_status.flags.in_air) {
						ECL_INFO("清除航向对齐");
						_control_status.flags.yaw_align = false; // 清除航向对齐标志
					}
				}

			} else {
				// 停止GNSS航向融合，但不声明其故障
				stopGnssYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// 尝试激活GNSS航向融合
				const bool not_using_ne_aiding = !_control_status.flags.gps && !_control_status.flags.aux_gpos;

				if (!_control_status.flags.in_air
				    || !_control_status.flags.yaw_align
				    || not_using_ne_aiding) {

					// 在开始融合之前重置
					if (resetYawToGnss(gnss_sample.yaw, gnss_sample.yaw_offset)) {

						resetAidSourceStatusZeroInnovation(_aid_src_gnss_yaw); // 重置辅助源状态为零创新

						_control_status.flags.gnss_yaw = true; // 设置GNSS航向标志为真
						_control_status.flags.yaw_align = true; // 设置航向对齐标志为真
					}

				} else if (!_aid_src_gnss_yaw.innovation_rejected) {
					// 不强制重置，而是等待一致性检查通过
					_control_status.flags.gnss_yaw = true; // 设置GNSS航向标志为真
					fuseGnssYaw(gnss_sample.yaw_offset); // 融合GNSS航向偏移
				}

				if (_control_status.flags.gnss_yaw) {
					ECL_INFO("开始GNSS航向融合");
				}
			}
		}

	} else if (_control_status.flags.gnss_yaw
		   && !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push, _params.reset_timeout_max)) {

		// 消息中不再有航向数据。停止直到数据回来。
		stopGnssYawFusion();
	}
}

// 更新GNSS航向的函数
void Ekf::updateGnssYaw(const gnssSample &gnss_sample)
{
	// 计算天线阵列的观测航向角，将其从机体坐标系转换为天线航向测量
	const float measured_hdg = wrap_pi(gnss_sample.yaw + gnss_sample.yaw_offset);

	// 检查航向加速度是否有效，若无效则设为0
	const float yaw_acc = PX4_ISFINITE(gnss_sample.yaw_acc) ? gnss_sample.yaw_acc : 0.f;
	const float R_YAW = sq(fmaxf(yaw_acc, _params.gnss_heading_noise)); // 计算航向观测噪声方差

	float heading_pred; // 预测的航向
	float heading_innov_var; // 创新方差

	VectorState H; // 观测矩阵
	// 计算航向预测创新和观测矩阵
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, gnss_sample.yaw_offset, R_YAW, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	// 更新辅助源状态
	updateAidSourceStatus(_aid_src_gnss_yaw,
			      gnss_sample.time_us,                          // 样本时间戳
			      measured_hdg,                                // 观测值
			      R_YAW,                                       // 观测方差
			      wrap_pi(heading_pred - measured_hdg),        // 创新
			      heading_innov_var,                           // 创新方差
			      math::max(_params.heading_innov_gate, 1.f)); // 创新门限
}

// 融合GNSS航向的函数
void Ekf::fuseGnssYaw(float antenna_yaw_offset)
{
	auto &aid_src = _aid_src_gnss_yaw; // 获取GNSS航向辅助源

	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true; // 标记航向创新被拒绝
		return; // 退出函数
	}

	if (!PX4_ISFINITE(antenna_yaw_offset)) {
		antenna_yaw_offset = 0.f; // 如果天线航向偏移无效，则设为0
	}

	float heading_pred; // 预测的航向
	float heading_innov_var; // 创新方差
	VectorState H; // 观测矩阵

	// 注意：我们重新计算创新和创新方差，因为这不会比仅计算H消耗更多的资源
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, antenna_yaw_offset, aid_src.observation_variance, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	// 检查创新方差计算是否条件不良
	if (aid_src.innovation_variance < aid_src.observation_variance) {
		// 状态协方差的创新方差贡献为负，意味着协方差矩阵条件不良
		_fault_status.flags.bad_hdg = true;

		// 重新初始化协方差矩阵并中止此融合步骤
		initialiseCovariance();
		ECL_ERR("GNSS航向数值错误 - 协方差重置");
		stopGnssYawFusion(); // 停止GNSS航向融合
		return; // 退出函数
	}

	_fault_status.flags.bad_hdg = false; // 标记航向状态正常
	_innov_check_fail_status.flags.reject_yaw = false; // 标记航向创新未被拒绝

	// 检查测试比率是否过大，可能表示陀螺仪偏差错误
	if ((fabsf(aid_src.test_ratio_filtered) > 0.2f)
	    && !_control_status.flags.in_air && isTimedOut(aid_src.time_last_fuse, (uint64_t)1e6)
	   ) {
		// 重置航向陀螺仪方差以加快收敛，避免停留在之前的错误估计上
		resetGyroBiasZCov();
	}

	// 计算卡尔曼增益
	// 仅计算我们正在使用的状态的增益
	VectorState Kfusion = P * H / aid_src.innovation_variance;

	// 执行测量更新
	measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);

	_fault_status.flags.bad_hdg = false; // 标记航向状态正常
	aid_src.fused = true; // 标记GNSS航向已融合
	aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间

	_time_last_heading_fuse = _time_delayed_us; // 更新最后航向融合时间
}

// 将航向重置为GNSS的函数
bool Ekf::resetYawToGnss(const float gnss_yaw, const float gnss_yaw_offset)
{
	// 定义预测的天线阵列向量并旋转到地球坐标系
	const Vector3f ant_vec_bf = {cosf(gnss_yaw_offset), sinf(gnss_yaw_offset), 0.0f}; // 天线阵列在机体坐标系中的向量
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf; // 将天线阵列向量转换到地球坐标系

	// 检查天线阵列向量是否在垂直方向30度以内，若超出则无法提供可靠的航向
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false; // 返回false，表示无法重置航向
	}

	// GNSS航向测量已在驱动程序中补偿了天线偏移
	const float measured_yaw = gnss_yaw;

	const float yaw_variance = sq(fmaxf(_params.gnss_heading_noise, 1.e-2f)); // 计算航向方差
	resetQuatStateYaw(measured_yaw, yaw_variance); // 重置四元数状态的航向

	return true; // 返回true，表示成功重置航向
}

// 停止GNSS航向融合的函数
void Ekf::stopGnssYawFusion()
{
	if (_control_status.flags.gnss_yaw) {

		_control_status.flags.gnss_yaw = false; // 设置GNSS航向标志为假

		ECL_INFO("停止GNSS航向融合");
	}
}
