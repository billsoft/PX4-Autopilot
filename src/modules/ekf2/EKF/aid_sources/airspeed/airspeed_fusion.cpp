/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4开发团队。保留所有权利。
 *
 * 以源代码和二进制形式重新分发和使用，无论是否修改，均可在满足以下条件的情况下进行：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随附的文档和/或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. 除非事先获得书面许可，否则不得使用PX4的名称或其贡献者的名称来支持或推广从本软件派生的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，且不提供任何明示或暗示的担保，包括但不限于对适销性和特定用途适用性的暗示担保。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file airspeed_fusion.cpp
 * 速度融合方法。
 * 方程由EKF/python/ekf_derivation/main.py生成。
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_airspeed_h.h>
#include <ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_airspeed.h>

#include <mathlib/mathlib.h>

void Ekf::controlAirDataFusion(const imuSample &imu_delayed)
{
	// 控制激活和初始化/重置空气速度融合所需的风状态

	// 如果空气速度和侧滑融合都超时，并且我们没有使用拖曳观测模型，则我们不再有有效的风估计
	const bool airspeed_timed_out = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

	if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && (_params.drag_ctrl == 0))) {
		_control_status.flags.wind = false; // 设置风状态为无效
	}

	if (_control_status.flags.wind && _external_wind_init) {
		_external_wind_init = false; // 如果风状态有效，重置外部风初始化标志
	}

#if defined(CONFIG_EKF2_GNSS)

	// 清除偏航估计器的空气速度（如果空气速度融合处于活动状态，将稍后用真实空气速度更新）
	if (_control_status.flags.fixed_wing) {
		if (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest) {
			if (!_control_status.flags.fuse_aspd) {
				_yawEstimator.setTrueAirspeed(_params.EKFGSF_tas_default); // 设置默认的真实空气速度
			}

		} else {
			_yawEstimator.setTrueAirspeed(0.f); // 如果不在空中，设置真实空气速度为0
		}
	}

#endif // CONFIG_EKF2_GNSS

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion(); // 如果空气速度阈值小于等于0，停止空气速度融合
		return;
	}

	if (_airspeed_buffer && _airspeed_buffer->pop_first_older_than(imu_delayed.time_us, &_airspeed_sample_delayed)) {

		const airspeedSample &airspeed_sample = _airspeed_sample_delayed; // 获取延迟的空气速度样本

		updateAirspeed(airspeed_sample, _aid_src_airspeed); // 更新空气速度

		_innov_check_fail_status.flags.reject_airspeed =
			_aid_src_airspeed.innovation_rejected; // TODO: 移除这个冗余标志

		const bool continuing_conditions_passing = _control_status.flags.in_air
				&& !_control_status.flags.fake_pos; // 检查持续条件是否满足

		const bool is_airspeed_significant = airspeed_sample.true_airspeed > _params.arsp_thr; // 检查空气速度是否显著
		const bool is_airspeed_consistent = (_aid_src_airspeed.test_ratio > 0.f && _aid_src_airspeed.test_ratio < 1.f); // 检查空气速度是否一致
		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_airspeed_significant
				&& (is_airspeed_consistent || !_control_status.flags.wind || _control_status.flags.inertial_dead_reckoning); // 检查启动条件是否满足

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed(airspeed_sample, _aid_src_airspeed); // 融合空气速度
				}

#if defined(CONFIG_EKF2_GNSS)
				_yawEstimator.setTrueAirspeed(airspeed_sample.true_airspeed); // 更新真实空气速度
#endif // CONFIG_EKF2_GNSS

				const bool is_fusion_failing = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6); // 检查融合是否超时

				if (is_fusion_failing) {
					stopAirspeedFusion(); // 如果融合超时，停止空气速度融合
				}

			} else {
				stopAirspeedFusion(); // 如果持续条件不满足，停止空气速度融合
			}

		} else if (starting_conditions_passing) {
			ECL_INFO("starting airspeed fusion"); // 记录开始空气速度融合的信息

			if (_control_status.flags.inertial_dead_reckoning && !is_airspeed_consistent) {
				resetVelUsingAirspeed(airspeed_sample); // 使用空气速度重置速度

			} else if (!_external_wind_init
				   && (!_control_status.flags.wind
				       || getWindVelocityVariance().longerThan(sq(_params.initial_wind_uncertainty)))) {
				resetWindUsingAirspeed(airspeed_sample); // 使用空气速度重置风
				_aid_src_airspeed.time_last_fuse = _time_delayed_us; // 更新最后融合时间
			}

			_control_status.flags.wind = true; // 设置风状态为有效
			_control_status.flags.fuse_aspd = true; // 设置空气速度融合标志为有效
		}

	} else if (_control_status.flags.fuse_aspd && !isRecent(_airspeed_sample_delayed.time_us, (uint64_t)1e6)) {
		ECL_WARN("Airspeed data stopped"); // 记录空气速度数据停止的警告
		stopAirspeedFusion(); // 停止空气速度融合
	}
}

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const
{
	// 真实空气速度测量的方差 - (米/秒)^2
	const float R = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f)); // 计算空气速度测量的方差

	float innov = 0.f; // 初始化创新值
	float innov_var = 0.f; // 初始化创新方差
	sym::ComputeAirspeedInnovAndInnovVar(_state.vector(), P, airspeed_sample.true_airspeed, R, FLT_EPSILON,
					     &innov, &innov_var); // 计算创新和创新方差

	updateAidSourceStatus(aid_src,
			      airspeed_sample.time_us,                 // 样本时间戳
			      airspeed_sample.true_airspeed,           // 观测值
			      R,                                       // 观测方差
			      innov,                                   // 创新值
			      innov_var,                               // 创新方差
			      math::max(_params.tas_innov_gate, 1.f)); // 创新门限
}

void Ekf::fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src)
{
	if (aid_src.innovation_rejected) {
		return; // 如果创新被拒绝，直接返回
	}

	// 确定是否需要空气速度融合来修正风以外的状态
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning; // 检查是否仅更新风状态

	const float innov_var = aid_src.innovation_variance; // 获取创新方差

	if (innov_var < aid_src.observation_variance || innov_var < FLT_EPSILON) {
		// 重置估计器协方差矩阵
		// 如果我们正在从其他源获取辅助，警告并仅重置风状态和协方差
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed(airspeed_sample); // 使用空气速度重置风
			action_string = "wind"; // 设置操作字符串为"wind"

		} else {
			initialiseCovariance(); // 初始化协方差
			_state.wind_vel.setZero(); // 将风速度设置为零
			action_string = "full"; // 设置操作字符串为"full"
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string); // 记录空气速度条件不良的错误信息

		_fault_status.flags.bad_airspeed = true; // 设置空气速度状态为不良

		return; // 返回
	}

	_fault_status.flags.bad_airspeed = false; // 设置空气速度状态为良好

	const VectorState H = sym::ComputeAirspeedH(_state.vector(), FLT_EPSILON); // 计算空气速度的H矩阵
	VectorState K = P * H / aid_src.innovation_variance; // 计算卡尔曼增益

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0); // 获取风的卡尔曼增益
		K.setZero(); // 将K矩阵清零
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind; // 仅保留风的卡尔曼增益
	}

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation); // 执行测量更新

	aid_src.fused = true; // 设置融合标志为真
	aid_src.time_last_fuse = _time_delayed_us; // 更新最后融合时间

	if (!update_wind_only) {
		_time_last_hor_vel_fuse = _time_delayed_us; // 更新最后水平速度融合时间
	}
}

void Ekf::stopAirspeedFusion()
{
	if (_control_status.flags.fuse_aspd) {
		ECL_INFO("stopping airspeed fusion"); // 记录停止空气速度融合的信息
		_control_status.flags.fuse_aspd = false; // 设置空气速度融合标志为无效

#if defined(CONFIG_EKF2_GNSS)
		_yawEstimator.setTrueAirspeed(NAN); // 设置真实空气速度为NAN
#endif // CONFIG_EKF2_GNSS
	}
}

void Ekf::resetWindUsingAirspeed(const airspeedSample &airspeed_sample)
{
	constexpr float sideslip_var = sq(math::radians(15.0f)); // 侧滑角方差

	const float euler_yaw = getEulerYaw(_R_to_earth); // 获取地球坐标系下的偏航角
	const float airspeed_var = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f)
				      * math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f)); // 计算空气速度方差

	matrix::SquareMatrix<float, State::wind_vel.dof> P_wind; // 风状态的协方差矩阵
	sym::ComputeWindInitAndCovFromAirspeed(_state.vel, euler_yaw, airspeed_sample.true_airspeed, getVelocityVariance(),
					       getYawVar(), sideslip_var, airspeed_var, &_state.wind_vel, &P_wind); // 从空气速度计算风的初始值和协方差

	resetStateCovariance<State::wind_vel>(P_wind); // 重置风状态的协方差

	ECL_INFO("reset wind using airspeed to (%.3f, %.3f)", (double)_state.wind_vel(0), (double)_state.wind_vel(1)); // 记录使用空气速度重置风的值

	resetAidSourceStatusZeroInnovation(_aid_src_airspeed); // 重置空气速度辅助源状态为零创新
}

void Ekf::resetVelUsingAirspeed(const airspeedSample &airspeed_sample)
{
	const float euler_yaw = getEulerYaw(_R_to_earth); // 获取地球坐标系下的偏航角

	// 使用零侧滑假设和空气速度测量估计速度
	Vector2f horizontal_velocity; // 水平速度向量
	horizontal_velocity(0) = _state.wind_vel(0) + airspeed_sample.true_airspeed * cosf(euler_yaw); // 计算水平速度的X分量
	horizontal_velocity(1) = _state.wind_vel(1) + airspeed_sample.true_airspeed * sinf(euler_yaw); // 计算水平速度的Y分量

	float vel_var = NAN; // 不重置速度方差，因为风方差估计可能不正确
	resetHorizontalVelocityTo(horizontal_velocity, vel_var); // 重置水平速度

	_aid_src_airspeed.time_last_fuse = _time_delayed_us; // 更新最后融合时间
}
