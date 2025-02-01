/****************************************************************************
 *
 *   版权所有 (c) 2015-2023 PX4 开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，无论是否修改，均需遵守以下条件：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随附的文档和/或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. 未经特定的书面许可，PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的担保，包括但不限于对适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于采购替代商品或服务；使用、数据或利润的损失；或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file sideslip_fusion.cpp
 * 侧滑融合方法。
 * 方程由 EKF/python/ekf_derivation/main.py 生成。
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_sideslip_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_sideslip_h.h>

#include <mathlib/mathlib.h>

// 控制侧滑融合的函数，接收延迟的IMU样本
void Ekf::controlBetaFusion(const imuSample &imu_delayed)
{
	// 检查是否启用侧滑融合，条件包括：侧滑融合参数启用、固定翼状态或气速融合启用、在空中状态且未使用虚假位置
	_control_status.flags.fuse_beta = _params.beta_fusion_enabled
					  && (_control_status.flags.fixed_wing || _control_status.flags.fuse_aspd)
					  && _control_status.flags.in_air
					  && !_control_status.flags.fake_pos;

	if (_control_status.flags.fuse_beta) {

		// 在空中且外部启用了侧滑融合时，定期执行合成侧滑融合
		const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

		if (beta_fusion_time_triggered) {

			// 更新侧滑信息
			updateSideslip(_aid_src_sideslip);
			// 检查创新是否被拒绝
			_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

			// 如果侧滑融合成功，标记风状态为真
			if (fuseSideslip(_aid_src_sideslip)) {
				_control_status.flags.wind = true;

			// 如果未初始化外部风且当前未标记风状态，则重置风协方差
			} else if (!_external_wind_init && !_control_status.flags.wind) {
				resetWindCov();
			}
		}
	}
}

// 更新侧滑信息的函数，接收估计辅助源
void Ekf::updateSideslip(estimator_aid_source1d_s &aid_src) const
{
	float observation = 0.f; // 观测值初始化为0
	const float R = math::max(sq(_params.beta_noise), sq(0.01f)); // 观测噪声方差，取beta噪声和0.01的平方的最大值
	const float epsilon = 1e-3f; // 用于计算的微小值
	float innov; // 创新值
	float innov_var; // 创新方差
	// 计算侧滑创新和创新方差
	sym::ComputeSideslipInnovAndInnovVar(_state.vector(), P, R, epsilon, &innov, &innov_var);

	// 更新辅助源状态
	updateAidSourceStatus(aid_src,
			      _time_delayed_us,                         // 样本时间戳
			      observation,                              // 观测值
			      R,                                        // 观测方差
			      innov,                                    // 创新值
			      innov_var,                                // 创新方差
			      math::max(_params.beta_innov_gate, 1.f)); // 创新门限
}

// 融合侧滑信息的函数，接收侧滑估计
bool Ekf::fuseSideslip(estimator_aid_source1d_s &sideslip)
{
	// 如果创新被拒绝，返回false
	if (sideslip.innovation_rejected) {
		return false;
	}

	// 确定是否仅需要侧滑融合来修正风以外的状态
	bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// 如果计算条件不良，重置协方差和状态
	if ((sideslip.innovation_variance < sideslip.observation_variance)
	    || (sideslip.innovation_variance < FLT_EPSILON)) {
		_fault_status.flags.bad_sideslip = true;

		// 如果我们正在从其他源获取辅助，发出警告并仅重置风状态和协方差
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindCov(); // 仅重置风协方差
			action_string = "wind";

		} else {
			initialiseCovariance(); // 重置所有协方差
			_state.wind_vel.setZero(); // 将风速度设置为零
			action_string = "full";
		}

		ECL_ERR("侧滑条件不良 - %s 协方差重置", action_string);

		return false; // 返回false，表示融合失败
	}

	_fault_status.flags.bad_sideslip = false; // 标记侧滑状态为良好

	const float epsilon = 1e-3f; // 用于计算的微小值

	// 计算侧滑的H矩阵
	const VectorState H = sym::ComputeSideslipH(_state.vector(), epsilon);
	// 计算卡尔曼增益K
	VectorState K = P * H / sideslip.innovation_variance;

	if (update_wind_only) {
		// 如果仅更新风，提取风的卡尔曼增益
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero(); // 将K矩阵清零
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind; // 仅保留风的增益
	}

	// 执行测量更新
	measurementUpdate(K, H, sideslip.observation_variance, sideslip.innovation);

	// 标记侧滑为已融合
	sideslip.fused = true;
	// 更新最后融合时间
	sideslip.time_last_fuse = _time_delayed_us;

	return true; // 返回true，表示融合成功
}
