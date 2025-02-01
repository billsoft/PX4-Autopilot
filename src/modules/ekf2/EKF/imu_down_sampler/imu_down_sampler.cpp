#include "imu_down_sampler/imu_down_sampler.hpp" // 引入IMU下采样器的头文件

#include <lib/mathlib/mathlib.h> // 引入数学库

// IMU下采样器构造函数，接收目标时间间隔（微秒）
ImuDownSampler::ImuDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
	reset(); // 调用重置函数，初始化IMU下采样器
}

// 更新IMU样本，直到达到目标时间间隔
// 假设陀螺仪的时间间隔接近加速度计的时间间隔
// 如果达到目标时间间隔则返回true
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	// 使用加权平均更新角速度时间间隔
	_delta_ang_dt_avg = 0.9f * _delta_ang_dt_avg + 0.1f * imu_sample_new.delta_ang_dt;

	// 累加时间增量
	_imu_down_sampled.time_us = imu_sample_new.time_us; // 更新下采样时间
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt; // 累加角速度时间增量
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt; // 累加速度时间增量
	_imu_down_sampled.delta_vel_clipping[0] |= imu_sample_new.delta_vel_clipping[0]; // 更新速度剪切标志
	_imu_down_sampled.delta_vel_clipping[1] |= imu_sample_new.delta_vel_clipping[1]; // 更新速度剪切标志
	_imu_down_sampled.delta_vel_clipping[2] |= imu_sample_new.delta_vel_clipping[2]; // 更新速度剪切标志

	// 使用四元数累加角度数据
	// 该四元数表示从累加周期开始到结束的旋转
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang)); // 创建表示角度变化的四元数
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q; // 更新累积的角度
	_delta_angle_accumulated.normalize(); // 归一化累积的角度

	// 每次将累积的速度数据旋转到更新的旋转框架中
	const Dcmf delta_R(delta_q.inversed()); // 计算四元数的逆
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel; // 更新速度数据

	// 在更新的旋转框架中累加最近的速度数据
	// 假设有效采样时间在前一个和当前旋转框架之间的中间
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f; // 更新速度数据

	_accumulated_samples++; // 增加累积样本计数

	// 检查是否已累积所需数量的样本，并且总时间至少达到目标的一半
	// 或者总时间已经超过目标
	if ((_accumulated_samples >= _required_samples && _imu_down_sampled.delta_ang_dt > _min_dt_s)
	    || (_imu_down_sampled.delta_ang_dt > _target_dt_s)) {

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated); // 更新角度数据
		return true; // 返回true，表示已达到目标时间间隔
	}

	return false; // 返回false，表示未达到目标时间间隔
}

// 重置IMU下采样器的状态
void ImuDownSampler::reset()
{
	_imu_down_sampled = {}; // 清空下采样数据
	_delta_angle_accumulated.setIdentity(); // 将累积角度初始化为单位四元数
	_accumulated_samples = 0; // 重置累积样本计数

	// 安全约束目标时间间隔（秒）
	float target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1e-6f; // 将微秒转换为秒并限制范围

	_required_samples = math::max((int)roundf(target_dt_s / _delta_ang_dt_avg), 1); // 计算所需样本数量

	_target_dt_s = _required_samples * _delta_ang_dt_avg; // 计算目标时间间隔

	// 最小角度变化时间间隔（除了样本数量之外）
	_min_dt_s = math::max(_delta_ang_dt_avg * (_required_samples - 1.f), _delta_ang_dt_avg * 0.5f); // 计算最小时间间隔
}
