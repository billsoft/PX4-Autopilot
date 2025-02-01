/*
 * ****************************************************************************
 *
 *   版权所有 (c) 2015-2023 PX4 开发团队。保留所有权利。
 *
 * 允许在源代码和二进制形式中重新发布和使用，无论是否进行修改，只要满足以下条件：
 *
 * 1. 源码形式的再发布必须保留上述版权声明、本条件列表以及以下免责声明。
 * 2. 二进制形式的再发布必须在随附的文档和/或其它材料中，复制上述版权声明、本条件列表以及以下免责声明。
 * 3. 未经事先书面许可，不得使用PX4的名称或其贡献者的名称来推广或认可从本软件衍生的产品。
 *
 * 本软件由版权所有者和贡献者以“按现状(AS IS)”提供，不提供任何明示或暗示的担保，包括但不限于对适销性和特定用途适用性的暗示担保。无论是在任何情况下，版权所有者或贡献者都不对任何直接、间接、附带、特殊、惩罚性或间接损害（包括但不限于采购替代商品或服务；使用损失、数据或利润；或业务中断）承担责任，无论是因何种原因并根据何种责任理论（无论是合同责任、严格责任或侵权行为，包括疏忽或其他），即使已被告知可能会发生此类损害。
 *
 * ****************************************************************************/

/**
 * @file covariance.cpp
 * @brief 该文件包含了用于初始化、预测和更新状态协方差矩阵的函数。
 * 在此文件中使用的公式是通过 EKF/python/ekf_derivation/main.py 所生成。
 *
 * @author Roman Bast <bastroman@gmail.com>
 */

#include "ekf.h"
#include <ekf_derivation/generated/predict_covariance.h>

#include <math.h>
#include <mathlib/mathlib.h>

/*
 * 用于初始化状态协方差矩阵的函数。
 * 需要注意的是，四元数状态必须先行初始化后再调用此函数。
 * 因为这里会初始化姿态相关的协方差，保证初始状态有合适的协方差值。
 */
void Ekf::initialiseCovariance()
{
	P.zero();

	// 先将姿态四元数部分的协方差设置为0，以便通过零速度/位置融合来实现更精细的水平校准
	resetQuatCov(0.f);

	// 设置速度协方差
	#if defined(CONFIG_EKF2_GNSS)
	const float vel_var = sq(fmaxf(_params.gps_vel_noise, 0.01f));
	#else
	const float vel_var = sq(0.5f);
	#endif

	// 这里分别为速度三个分量设定初始方差，其中z轴速度的方差在这里乘以1.5倍
	P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx, Vector3f(vel_var, vel_var, sq(1.5f) * vel_var));

	// 设置位置协方差
	#if defined(CONFIG_EKF2_BAROMETER)
	float z_pos_var = sq(fmaxf(_params.baro_noise, 0.01f));
	#else
	float z_pos_var = sq(1.f);
	#endif // CONFIG_EKF2_BAROMETER

	#if defined(CONFIG_EKF2_GNSS)
	const float xy_pos_var = sq(fmaxf(_params.gps_pos_noise, 0.01f));

	// 如果当前高度源是GPS，则在z_pos_var中考虑一定放大
	if (_control_status.flags.gps_hgt) {
		z_pos_var = sq(fmaxf(1.5f * _params.gps_pos_noise, 0.01f));
	}

	#else
	const float xy_pos_var = sq(fmaxf(_params.pos_noaid_noise, 0.01f));
	#endif

	#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 如果当前高度源是测距仪（Range Finder），则将z方向位置协方差设置为测距仪的噪声
	if (_control_status.flags.rng_hgt) {
		z_pos_var = sq(fmaxf(_params.range_noise, 0.01f));
	}

	#endif // CONFIG_EKF2_RANGE_FINDER

	P.uncorrelateCovarianceSetVariance<State::pos.dof>(State::pos.idx, Vector3f(xy_pos_var, xy_pos_var, z_pos_var));

	// 重置陀螺仪偏置协方差
	resetGyroBiasCov();

	// 重置加速度计偏置协方差
	resetAccelBiasCov();

	#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 如果使用磁力计，则重置地磁场和磁偏置的协方差
	resetMagEarthCov();
	resetMagBiasCov();
	#endif // CONFIG_EKF2_MAGNETOMETER

	#if defined(CONFIG_EKF2_WIND)
	// 如果使用风估计，则重置风协方差
	resetWindCov();
	#endif // CONFIG_EKF2_WIND

	#if defined(CONFIG_EKF2_TERRAIN)
	// 对地形估计的初始化处理：将地形状态的协方差设置为地面高度余量的平方
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, sq(_params.rng_gnd_clearance));
	#endif // CONFIG_EKF2_TERRAIN
}

/*
 * 使用imuSample中的增量来预测协方差矩阵。
 * 该步骤主要基于离散化的运动方程，进行状态协方差在时间上的传播，
 * 并在必要时添加相应的过程噪声Q。
 */
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
	// 预测协方差时使用的时间增量，取加速度和角速度增量的平均采样时间
	const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

	// 陀螺仪噪声方差
	float gyro_noise = _params.gyro_noise;
	const float gyro_var = sq(gyro_noise);

	// 加速度计噪声方差（可能会根据不良数据或剪切数据进行放大）
	float accel_noise = _params.accel_noise;
	Vector3f accel_var;

	for (unsigned i = 0; i < 3; i++) {
		// 如果检测到垂直加速度异常或者该轴发生了加速度剪切，则采用一个较大的加速度过程噪声
		if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
			accel_var(i) = sq(BADACC_BIAS_PNOISE);

		} else {
			accel_var(i) = sq(accel_noise);
		}
	}

	// 调用自动生成的符号函数PredictCovariance来计算协方差P的预测部分
	// 这里内部相当于实现了：P = F * P * F' + G * Q * G' 的离散形式
	// 其中F为雅可比矩阵，Q为过程噪声矩阵
	P = sym::PredictCovariance(
		_state.vector(),
		P,
		imu_delayed.delta_vel / imu_delayed.delta_vel_dt,
		accel_var,
		imu_delayed.delta_ang / imu_delayed.delta_ang_dt,
		gyro_var,
		dt);

	// 对某些状态（如陀螺仪偏置和加速度计偏置），我们还要在此处额外添加过程噪声
	// 因为它们属于随机游走模型或更复杂的过程模型

	// 陀螺仪偏置：添加过程噪声
	{
		const float gyro_bias_sig = dt * _params.gyro_bias_p_noise;
		const float gyro_bias_process_noise = sq(gyro_bias_sig);

		for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
			const unsigned i = State::gyro_bias.idx + index;

			// 对角线上的偏置方差在极小值范围内时，会额外增加过程噪声保证偏置可被估计
			if (P(i, i) < gyro_var) {
				P(i, i) += gyro_bias_process_noise;
			}
		}
	}

	// 加速度计偏置：添加过程噪声
	{
		const float accel_bias_sig = dt * _params.accel_bias_p_noise;
		const float accel_bias_process_noise = sq(accel_bias_sig);

		for (unsigned index = 0; index < State::accel_bias.dof; index++) {
			const unsigned i = State::accel_bias.idx + index;

			if (P(i, i) < accel_var(index)) {
				P(i, i) += accel_bias_process_noise;
			}
		}
	}


#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 对地磁场向量(机体外部的地磁分量)和磁偏置进行过程噪声注入
	// mag_I: 与地球磁场方向相关的状态
	{
		float mag_I_sig = dt * _params.mage_p_noise;
		float mag_I_process_noise = sq(mag_I_sig);

		for (unsigned index = 0; index < State::mag_I.dof; index++) {
			const unsigned i = State::mag_I.idx + index;

			// 如果当前P(i, i)过小，则加一些过程噪声
			if (P(i, i) < sq(_params.mag_noise)) {
				P(i, i) += mag_I_process_noise;
			}
		}
	}

	// mag_B: 与机体自己的磁偏置相关
	{
		float mag_B_sig = dt * _params.magb_p_noise;
		float mag_B_process_noise = sq(mag_B_sig);

		for (unsigned index = 0; index < State::mag_B.dof; index++) {
			const unsigned i = State::mag_B.idx + index;

			if (P(i, i) < sq(_params.mag_noise)) {
				P(i, i) += mag_B_process_noise;
			}
		}
	}

#endif // CONFIG_EKF2_MAGNETOMETER


#if defined(CONFIG_EKF2_WIND)
	// 风速估计：根据高度变化率来适度增加过程噪声
	// wind_vel_nsd_scaled 根据高度变化率进行比例放大
	{
		const float height_rate = _height_rate_lpf.update(_state.vel(2), imu_delayed.delta_vel_dt);
		const float wind_vel_nsd_scaled = _params.wind_vel_nsd * (1.f + _params.wind_vel_nsd_scaler * fabsf(height_rate));
		const float wind_vel_process_noise = sq(wind_vel_nsd_scaled) * dt;

		for (unsigned index = 0; index < State::wind_vel.dof; index++) {
			const unsigned i = State::wind_vel.idx + index;

			if (P(i, i) < sq(_params.initial_wind_uncertainty)) {
				P(i, i) += wind_vel_process_noise;
			}
		}
	}

#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	// 地形（terrain）状态协方差的预测更新
	// 当高度传感器参考不是测距仪时，估计地面高度这一状态
	// 由于车辆高度估计误差以及地形斜率，会导致对地形高度的误差增长
	if (_height_sensor_ref != HeightSensor::RANGE) {
		// 车辆高度估计带来的过程噪声
		float terrain_process_noise = sq(imu_delayed.delta_vel_dt * _params.terrain_p_noise);

		// 由于地形梯度引入的不确定性，与车辆水平速度vx、vy相关
		terrain_process_noise += sq(imu_delayed.delta_vel_dt * _params.terrain_gradient) * (sq(_state.vel(0)) + sq(_state.vel(1)));
		P(State::terrain.idx, State::terrain.idx) += terrain_process_noise;
	}

#endif // CONFIG_EKF2_TERRAIN

	// 协方差矩阵对称化：将上三角元素映射到下三角，确保数值对称
	for (unsigned row = 0; row < State::size; row++) {
		for (unsigned column = 0; column < row; column++) {
			P(row, column) = P(column, row);
		}
	}

	// 对可能过大的状态协方差进行限制，避免数值发散
	constrainStateVariances();
}

/*
 * 该函数对过大的协方差进行收敛性约束，以防止数值溢出或发散。
 * 如果协方差超过一定范围，会通过融合零创新的方式进行限制。
 */
void Ekf::constrainStateVariances()
{
	// 注意：此限制仅在最后手段情况下使用，不应被依赖。
	// 在这里对每个状态的协方差进行上下界的限制

	// 对姿态四元数的协方差进行限制
	constrainStateVar(State::quat_nominal, 1e-9f, 1.f);
	// 速度
	constrainStateVar(State::vel, 1e-6f, 1e6f);
	// 位置
	constrainStateVar(State::pos, 1e-6f, 1e6f);
	// 陀螺仪偏置
	constrainStateVarLimitRatio(State::gyro_bias, kGyroBiasVarianceMin, 1.f);
	// 加速度计偏置
	constrainStateVarLimitRatio(State::accel_bias, kAccelBiasVarianceMin, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 如果使用磁力计，则对磁场和磁偏置进行限制
	if (_control_status.flags.mag) {
		constrainStateVarLimitRatio(State::mag_I, kMagVarianceMin, 1.f);
		constrainStateVarLimitRatio(State::mag_B, kMagVarianceMin, 1.f);
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// 如果在估计风，则限制风速状态的协方差
	if (_control_status.flags.wind) {
		constrainStateVarLimitRatio(State::wind_vel, 1e-6f, 1e6f);
	}
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	// 地形状态
	constrainStateVarLimitRatio(State::terrain, 0.f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

/*
 * 对某一段状态向量的对角线元素进行最小值和最大值的限制，如果超过范围，会调用融合零创新的方法进行减缩。
 */
void Ekf::constrainStateVar(const IdxDof &state, float min, float max)
{
	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		if (P(i, i) < min) {
			P(i, i) = min;

		} else if (P(i, i) > max) {
			// 如果超过上限，则通过融合零创新的方式（相当于测量残差为0），
			// 在测量方差=当前过大的协方差10倍情况下，适度降低协方差。
			const float innov = 0.f;
			const float R = 10.f * P(i, i); // 这里K = P/(P+R)，约10%的减缩
			const float innov_var = P(i, i) + R;
			fuseDirectStateMeasurement(innov, innov_var, R, i);
		}
	}
}

/*
 * 与上面类似，但增加了一个 ratio 的概念，用于控制最大和最小协方差的比值不超过一定倍数。
 */
void Ekf::constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio)
{
	float state_var_max = 0.f;

	// 找到这个状态维度中最大的协方差值
	for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
		if (P(i, i) > state_var_max) {
			state_var_max = P(i, i);
		}
	}

	// 对最大协方差进行限制，同时根据max_ratio计算最小协方差限制
	float limited_max = math::constrain(state_var_max, min, max);
	float limited_min = math::constrain(limited_max / max_ratio, min, max);

	constrainStateVar(state, limited_min, limited_max);
}

/*
 * 重置姿态四元数相关的协方差，假设我们对初始姿态的倾斜有一定置信度但对偏航有单独设定。
 * yaw_noise 表示对偏航角的噪声或不确定性。
 */
void Ekf::resetQuatCov(const float yaw_noise)
{
	const float tilt_var = sq(math::max(_params.initial_tilt_err, 0.01f));
	float yaw_var = sq(0.01f);

	// 如果外部传入了一个有效的yaw噪声，则使用之
	if (PX4_ISFINITE(yaw_noise)) {
		yaw_var = sq(yaw_noise);
	}

	resetQuatCov(Vector3f(tilt_var, tilt_var, yaw_var));
}

/*
 * 重置姿态四元数的协方差，传入的Vector3f包含了俯仰、横滚和偏航的方差。
 */
void Ekf::resetQuatCov(const Vector3f &rot_var_ned)
{
	P.uncorrelateCovarianceSetVariance<State::quat_nominal.dof>(State::quat_nominal.idx, rot_var_ned);
}

/*
 * 重置陀螺仪偏置的协方差，将其与其它状态解耦并设置一个初始方差。
 */
void Ekf::resetGyroBiasCov()
{
	P.uncorrelateCovarianceSetVariance<State::gyro_bias.dof>(State::gyro_bias.idx, sq(_params.switch_on_gyro_bias));
}

/*
 * 只重置z轴陀螺仪偏置的协方差
 */
void Ekf::resetGyroBiasZCov()
{
	P.uncorrelateCovarianceSetVariance<1>(State::gyro_bias.idx + 2, sq(_params.switch_on_gyro_bias));
}

/*
 * 重置加速度计偏置的协方差，将其与其它状态解耦并设置为初始方差。
 */
void Ekf::resetAccelBiasCov()
{
	P.uncorrelateCovarianceSetVariance<State::accel_bias.dof>(State::accel_bias.idx, sq(_params.switch_on_accel_bias));
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
/*
 * 重置地磁场在地理坐标系下的协方差，表示我们再次对地磁向量不确定性重新初始化。
 */
void Ekf::resetMagEarthCov()
{
	ECL_INFO("reset mag earth covariance");

	P.uncorrelateCovarianceSetVariance<State::mag_I.dof>(State::mag_I.idx, sq(_params.mag_noise));
}

/*
 * 重置磁偏置的协方差，表示对机体磁偏置不确定性重新初始化。
 */
void Ekf::resetMagBiasCov()
{
	ECL_INFO("reset mag bias covariance");

	P.uncorrelateCovarianceSetVariance<State::mag_B.dof>(State::mag_B.idx, sq(_params.mag_noise));
}
#endif // CONFIG_EKF2_MAGNETOMETER
