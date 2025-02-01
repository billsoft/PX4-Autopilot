/****************************************************************************
 *
 *   版权所有 (c) 2015-2023 PX4 开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，无论是否修改，均可在满足以下条件的情况下进行：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随分发的文档或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. 未经特定的书面许可，PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于对适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于采购替代商品或服务、使用损失、数据或利润损失或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file mag_fusion.cpp
 * 磁力计融合方法。
 * 方程由 EKF/python/ekf_derivation/main.py 生成。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_mag_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_mag_z_innov_var_and_h.h>

#include <ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h>

#include <mathlib/mathlib.h>

// 磁力计融合函数
// 参数：
// - mag: 输入的磁力计测量值（3D向量）
// - R_MAG: 磁力计的测量噪声方差
// - H: 观测雅可比矩阵
// - aid_src: 估计辅助源结构体，包含创新和方差等信息
// - update_all_states: 是否更新所有状态
// - update_tilt: 是否更新倾斜角
bool Ekf::fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, estimator_aid_source3d_s &aid_src,
		  bool update_all_states, bool update_tilt)
{
	// 如果任何轴的创新被拒绝，则中止磁力计融合
	if (aid_src.innovation_rejected) {
		return false;
	}

	const auto state_vector = _state.vector(); // 获取当前状态向量

	// 使用磁力计分量的顺序融合更新状态和协方差
	for (uint8_t index = 0; index <= 2; index++) {
		// 计算卡尔曼增益和观测雅可比矩阵
		if (index == 0) {
			// 第一个轴的计算已经完成，无需重新计算

		} else if (index == 1) {
			// 由于之前的融合，状态协方差已发生变化，因此需要重新计算创新方差
			// 使用相同的初始状态对所有轴进行线性化
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// 使用更新后的状态重新计算创新
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

		} else if (index == 2) {
			// 在进行3D融合时，不融合合成的磁力计测量值
			if (_control_status.flags.synthetic_mag_z) {
				continue; // 跳过此循环
			}

			// 由于之前的融合，状态协方差已发生变化，因此需要重新计算创新方差
			// 使用相同的初始状态对所有轴进行线性化
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// 使用更新后的状态重新计算创新
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);
		}

		// 检查创新方差是否小于测量噪声方差
		if (aid_src.innovation_variance[index] < R_MAG) {
			ECL_ERR("磁力计数值的数值误差协方差重置");

			// 需要重新初始化协方差并中止此融合步骤
			if (update_all_states) {
				resetQuatCov(_params.mag_heading_noise); // 重置四元数协方差
			}

			resetMagEarthCov(); // 重置地球磁场协方差
			resetMagBiasCov(); // 重置磁偏差协方差

			return false; // 返回失败
		}

		// 计算卡尔曼增益
		VectorState Kfusion = P * H / aid_src.innovation_variance[index];

		if (update_all_states) {
			if (!update_tilt) {
				// 如果不更新倾斜角，则将四元数的增益置为零
				Kfusion(State::quat_nominal.idx + 0) = 0.f;
				Kfusion(State::quat_nominal.idx + 1) = 0.f;
			}

		} else {
			// 如果不更新所有状态，则将非磁力计的卡尔曼增益置为零

			// 复制磁力计的卡尔曼增益
			const Vector3f K_mag_I = Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0);
			const Vector3f K_mag_B = Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0);

			// 将所有卡尔曼增益置为零，然后恢复磁力计的增益
			Kfusion.setZero();
			Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = K_mag_I;
			Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = K_mag_B;
		}

		// 执行测量更新
		measurementUpdate(Kfusion, H, aid_src.observation_variance[index], aid_src.innovation[index]);
	}

	// 重置故障状态标志
	_fault_status.flags.bad_mag_x = false;
	_fault_status.flags.bad_mag_y = false;
	_fault_status.flags.bad_mag_z = false;

	// 标记为已融合，并记录最后一次融合的时间
	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	if (update_all_states) {
		_time_last_heading_fuse = _time_delayed_us; // 更新最后一次航向融合的时间
	}

	return true; // 返回成功
}

// 磁偏角融合函数
// 参数：
// - decl_measurement_rad: 磁偏角测量值（弧度）
// - R: 测量噪声方差
// - update_all_states: 是否更新所有状态
bool Ekf::fuseDeclination(float decl_measurement_rad, float R, bool update_all_states)
{
	VectorState H; // 观测雅可比矩阵
	float decl_pred; // 预测的磁偏角
	float innovation_variance; // 创新方差

	// 计算磁偏角的预测创新方差和雅可比矩阵
	sym::ComputeMagDeclinationPredInnovVarAndH(_state.vector(), P, R, FLT_EPSILON,
			&decl_pred, &innovation_variance, &H);

	// 计算创新值
	const float innovation = wrap_pi(decl_pred - decl_measurement_rad);

	// 检查创新方差是否小于测量噪声方差
	if (innovation_variance < R) {
		// 方差计算条件不良
		_fault_status.flags.bad_mag_decl = true; // 设置故障标志
		return false; // 返回失败
	}

	// 计算卡尔曼增益
	VectorState Kfusion = P * H / innovation_variance;

	if (!update_all_states) {
		// 如果不更新所有状态，则将非磁力计的卡尔曼增益置为零

		// 复制磁力计的卡尔曼增益
		const Vector3f K_mag_I = Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0);
		const Vector3f K_mag_B = Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0);

		// 将所有卡尔曼增益置为零，然后恢复磁力计的增益
		Kfusion.setZero();
		Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = K_mag_I;
		Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = K_mag_B;
	}

	// 执行测量更新
	measurementUpdate(Kfusion, H, R, innovation);

	_fault_status.flags.bad_mag_decl = false; // 重置故障标志

	return true; // 返回成功
}

// 计算合成磁力计Z分量测量值的函数
// 参数：
// - mag_meas: 磁力计测量值（3D向量）
// - mag_earth_predicted: 预测的地球磁场（3D向量）
float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// 根据X和Y传感器测量值以及我们对当前位置地球磁场向量的了解，计算理论的磁力计Z分量的绝对值
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// 根据预测的磁力计Z分量的符号计算合成磁力计Z分量的符号
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs; // 返回合成的Z分量测量值
}
