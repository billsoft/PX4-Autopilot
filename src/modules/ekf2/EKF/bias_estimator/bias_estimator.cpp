/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file bias_estimator.cpp
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#include "bias_estimator.hpp" // 引入偏置估计器的头文件

void BiasEstimator::predict(const float dt)
{
	// 状态保持不变
	// 仅预测状态协方差
	float delta_state_var = _process_psd * dt; // 计算状态方差的增量

	if (isOffsetDetected()) { // 检测到偏置
		// 通过创新序列检查检测到状态中的偏置
		// 在偏置被移除之前增强过程噪声
		delta_state_var *= _process_var_boost_gain; // 增加状态方差的增量
	}

	_state_var += delta_state_var; // 更新状态方差
	constrainStateVar(); // 限制状态方差在合理范围内

	if (dt > FLT_EPSILON && fabsf(_dt - dt) > 0.001f) { // 检查时间间隔是否有效
		_signed_innov_test_ratio_lpf.setParameters(dt, _innov_sequence_monitnoring_time_constant); // 设置低通滤波器参数
		_dt = dt; // 更新时间间隔
	}

	_status.bias_var = _state_var; // 更新状态中的偏置方差
}

void BiasEstimator::constrainStateVar()
{
	_state_var = math::constrain(_state_var, 1e-8f, _state_var_max); // 限制状态方差在最小值和最大值之间
}

void BiasEstimator::fuseBias(const float measurement, const float measurement_var)
{
	const float innov_var = _state_var + math::max(sq(0.01f), measurement_var); // 计算创新方差
	const float innov = measurement - _state; // 计算创新
	const float K = _state_var / innov_var; // 计算卡尔曼增益
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var); // 计算创新测试比率

	if (isTestRatioPassing(innov_test_ratio)) { // 检查测试比率是否通过
		updateState(K, innov); // 更新状态
		updateStateCovariance(K); // 更新状态协方差
	}

	updateOffsetDetection(innov, innov_test_ratio); // 更新偏置检测

	_status = packStatus(innov, innov_var, innov_test_ratio); // 打包状态以供记录
}

inline float BiasEstimator::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var); // 计算创新测试比率
}

inline bool BiasEstimator::isTestRatioPassing(const float innov_test_ratio) const
{
	return innov_test_ratio < 1.f; // 检查创新测试比率是否小于1
}

inline void BiasEstimator::updateState(const float K, const float innov)
{
	_state = _state + K * innov; // 更新状态
}

inline void BiasEstimator::updateStateCovariance(const float K)
{
	_state_var -= K * _state_var; // 更新状态方差
	constrainStateVar(); // 限制状态方差在合理范围内
}

inline void BiasEstimator::updateOffsetDetection(const float innov, const float innov_test_ratio)
{
	const float signed_innov_test_ratio = matrix::sign(innov) * innov_test_ratio; // 计算带符号的创新测试比率
	_signed_innov_test_ratio_lpf.update(math::constrain(signed_innov_test_ratio, -1.f, 1.f)); // 更新低通滤波器状态

	if (innov > 0.f) { // 如果创新为正
		_time_since_last_positive_innov = 0.f; // 重置正创新计时
		_time_since_last_negative_innov += _dt; // 增加负创新计时
	} else { // 如果创新为负
		_time_since_last_negative_innov = 0.f; // 重置负创新计时
		_time_since_last_positive_innov += _dt; // 增加正创新计时
	}
}

inline bool BiasEstimator::isOffsetDetected() const
{
	// 如果创新的平均值在统计上过大
	// 或者创新的符号始终相同，则认为存在偏置
	return fabsf(_signed_innov_test_ratio_lpf.getState()) > 0.2f // 检查创新测试比率的绝对值
	       || (_time_since_last_positive_innov > _innov_sequence_monitnoring_time_constant) // 检查正创新持续时间
	       || (_time_since_last_negative_innov > _innov_sequence_monitnoring_time_constant); // 检查负创新持续时间
}

inline BiasEstimator::status BiasEstimator::packStatus(const float innov, const float innov_var,
		const float innov_test_ratio) const
{
	// 返回状态以供记录
	status ret{};
	ret.bias = _state; // 偏置
	ret.bias_var = _state_var; // 偏置方差
	ret.innov = innov; // 创新
	ret.innov_var = innov_var; // 创新方差
	ret.innov_test_ratio = innov_test_ratio; // 创新测试比率

	return ret; // 返回状态
}
