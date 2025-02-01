/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，无论是否修改，均可在满足以下条件的情况下进行：
 *
 * 1. 再分发的源代码必须保留上述版权
 *    声明、此列表中的条件和以下免责声明。
 * 2. 再分发的二进制形式必须在
 *    分发的文档或其他材料中重现上述版权
 *    声明和此免责声明。
 * 3. PX4的名称或其贡献者的名称不得
 *    用于支持或推广从本软件派生的产品
 *    未经特定的事先书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于
 * 对适销性和特定用途的隐含保证均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、
 * 偶然、特殊、示范性或后果性损害负责（包括但不限于采购替代商品或服务；使用、数据或利润的损失；
 * 或业务中断）无论是如何引起的，无论是在合同、严格责任还是侵权（包括疏忽或其他）中，均不对因使用本软件而引起的任何损害负责，即使已被告知
 * 可能发生此类损害。
 *
 ****************************************************************************/

#include "EKFGSF_yaw.h" // 引入EKFGSF_yaw类的头文件

#include <cstdlib> // 引入C标准库

#include <lib/geo/geo.h> // 引入地理常量库，包含常量CONSTANTS_ONE_G

#include "derivation/generated/yaw_est_predict_covariance.h" // 引入预测协方差的生成文件
#include "derivation/generated/yaw_est_compute_measurement_update.h" // 引入测量更新的生成文件

using matrix::AxisAnglef; // 使用AxisAnglef类
using matrix::Dcmf; // 使用Dcmf类
using matrix::Eulerf; // 使用Eulerf类
using matrix::Matrix3f; // 使用Matrix3f类
using matrix::Quatf; // 使用Quatf类
using matrix::Vector2f; // 使用Vector2f类
using matrix::Vector3f; // 使用Vector3f类
using matrix::wrap_pi; // 使用wrap_pi函数
using math::Utilities::getEulerYaw; // 使用获取欧拉角的函数
using math::Utilities::updateYawInRotMat; // 使用更新旋转矩阵中偏航的函数

EKFGSF_yaw::EKFGSF_yaw() // EKFGSF_yaw类的构造函数
{
	reset(); // 调用重置函数
}

void EKFGSF_yaw::reset() // 重置函数
{
	_ekf_gsf_vel_fuse_started = false; // 初始化速度融合状态为false

	_gsf_yaw_variance = INFINITY; // 初始化偏航方差为无穷大
}

void EKFGSF_yaw::predict(const matrix::Vector3f &delta_ang, const float delta_ang_dt, const matrix::Vector3f &delta_vel,
			 const float delta_vel_dt, bool in_air) // 预测函数
{
	const Vector3f accel = delta_vel / delta_vel_dt; // 计算加速度

	if (delta_vel_dt > 0.001f) { // 如果时间间隔大于0.001秒
		// 为了减少振动的影响，使用一个低通滤波器进行滤波，其时间常数为AHRS倾斜修正时间常数的1/10
		const float filter_coef = fminf(10.f * delta_vel_dt * _tilt_gain, 1.f); // 计算滤波系数
		_ahrs_accel = _ahrs_accel * (1.f - filter_coef) + accel * filter_coef; // 更新加速度

	} else {
		return; // 如果时间间隔太小，直接返回
	}

	// 初始化状态
	if (!_ahrs_ekf_gsf_tilt_aligned) { // 如果尚未对齐倾斜
		// 检查加速度是否过大，以减少由于车辆运动导致的大初始滚转/俯仰误差的可能性
		const float accel_norm_sq = accel.norm_squared(); // 计算加速度的平方范数
		const float accel_lpf_norm_sq = _ahrs_accel.norm_squared(); // 计算低通滤波后的加速度的平方范数

		static constexpr float upper_accel_limit = CONSTANTS_ONE_G * 1.1f; // 定义加速度上限
		static constexpr float lower_accel_limit = CONSTANTS_ONE_G * 0.9f; // 定义加速度下限

		// 检查是否可以对齐
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit))
					 && (accel_lpf_norm_sq > sq(lower_accel_limit)) && (accel_lpf_norm_sq < sq(upper_accel_limit));

		if (ok_to_align) { // 如果可以对齐
			ahrsAlignTilt(delta_vel); // 对齐倾斜
			_ahrs_ekf_gsf_tilt_aligned = true; // 设置倾斜对齐状态为true

		} else {
			return; // 如果不可以对齐，直接返回
		}
	}

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) { // 遍历所有模型
		predictEKF(model_index, delta_ang, delta_ang_dt, delta_vel, delta_vel_dt, in_air); // 进行EKF预测
	}
}

void EKFGSF_yaw::fuseVelocity(const Vector2f &vel_NE, const float vel_accuracy, const bool in_air) // 融合速度函数
{
	// 在有规律的速度观测之前，不开始运行EKF算法
	if (!_ekf_gsf_vel_fuse_started) { // 如果速度融合尚未开始

		initialiseEKFGSF(vel_NE, vel_accuracy); // 初始化EKF

		ahrsAlignYaw(); // 对齐偏航

		// 只有在空中或速度不微不足道时才开始
		if (in_air || vel_NE.longerThan(vel_accuracy)) {
			_ekf_gsf_vel_fuse_started = true; // 设置速度融合状态为true
		}

	} else {
		bool bad_update = false; // 初始化坏更新标志

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) { // 遍历所有模型
			// 后续测量作为直接状态观测进行融合
			if (!updateEKF(model_index, vel_NE, vel_accuracy)) { // 更新EKF
				bad_update = true; // 如果更新失败，设置坏更新标志为true
			}
		}

		if (!bad_update) { // 如果没有坏更新
			float total_weight = 0.0f; // 初始化总权重
			// 假设正态分布，计算每个模型的权重
			const float min_weight = 1e-5f; // 定义最小权重

			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) { // 遍历所有模型
				_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index); // 更新模型权重

				if (_model_weights(model_index) < min_weight) { // 如果权重小于最小权重
					_model_weights(model_index) = min_weight; // 设置为最小权重
				}

				total_weight += _model_weights(model_index); // 累加总权重
			}

			// 归一化权重函数
			_model_weights /= total_weight; // 归一化权重
		}

		// 计算复合偏航向量，作为每个模型状态的加权平均。
		// 为了避免角度包裹问题，偏航状态在求和之前被转换为长度等于权重值的向量。
		Vector2f yaw_vector; // 初始化偏航向量

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) { // 遍历所有模型
			yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2)); // 计算偏航向量的x分量
			yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2)); // 计算偏航向量的y分量
		}

		_gsf_yaw = atan2f(yaw_vector(1), yaw_vector(0)); // 计算复合偏航

		// 计算偏航状态的复合方差，基于每个模型方差的加权平均
		// 创新较大的模型权重较小
		_gsf_yaw_variance = 0.0f; // 初始化偏航方差

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) { // 遍历所有模型
			const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw); // 计算偏航差
			_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2, 2) + yaw_delta * yaw_delta); // 更新偏航方差
		}

		if (_gsf_yaw_variance <= 0.f || !PX4_ISFINITE(_gsf_yaw_variance)) { // 如果偏航方差无效
			reset(); // 重置
		}
	}
}

void EKFGSF_yaw::ahrsPredict(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt) // AHRS预测函数
{
	// 使用简单的互补滤波器生成所选模型的姿态解
	const Vector3f ang_rate = delta_ang / fmaxf(delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias; // 计算角速度

	const Vector3f gravity_direction_bf = _ahrs_ekf_gsf[model_index].q.inversed().dcm_z(); // 计算重力方向

	const float ahrs_accel_norm = _ahrs_accel.norm(); // 计算AHRS加速度的范数

	// 从加速度向量倾斜误差到速率陀螺仪修正的增益，用于AHRS计算
	const float ahrs_accel_fusion_gain = ahrsCalcAccelGain(); // 计算加速度融合增益

	// 使用加速度数据进行角速率修正，并在加速度幅度远离1g时减少修正（减少在车辆被提起和移动时的漂移）。
	// 在固定翼飞行期间，假设协调转弯并假设X轴向前，补偿向心加速度
	Vector3f tilt_correction{}; // 初始化倾斜修正

	if (ahrs_accel_fusion_gain > 0.f) { // 如果加速度融合增益有效

		Vector3f accel = _ahrs_accel; // 获取当前加速度

		if (PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON)) { // 如果真实空速有效且大于零
			// 计算机体框架的向心加速度，假设X轴与空速向量对齐
			// 使用机体速率和机体框架空速向量的叉积
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// 修正测量的加速度以考虑向心加速度
			accel -= centripetal_accel_bf; // 更新加速度
		}

		tilt_correction = (gravity_direction_bf % accel) * ahrs_accel_fusion_gain / ahrs_accel_norm; // 计算倾斜修正
	}

	// 陀螺仪偏差估计
	constexpr float gyro_bias_limit = 0.05f; // 定义陀螺仪偏差限制
	const float spin_rate = ang_rate.length(); // 计算旋转速率

	if (spin_rate < math::radians(10.f)) { // 如果旋转速率小于10度
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * delta_ang_dt); // 更新陀螺仪偏差
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias,
						       -gyro_bias_limit, gyro_bias_limit); // 限制陀螺仪偏差
	}

	// 从前一帧到当前帧的增量角度
	const Vector3f delta_angle_corrected = delta_ang
					       + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) * delta_ang_dt; // 计算修正后的增量角度

	// 将增量角度应用于姿态
	const Quatf dq(AxisAnglef{delta_angle_corrected}); // 创建增量四元数
	_ahrs_ekf_gsf[model_index].q = (_ahrs_ekf_gsf[model_index].q * dq).normalized(); // 更新姿态
}

void EKFGSF_yaw::ahrsAlignTilt(const Vector3f &delta_vel) // AHRS对齐倾斜函数
{
	// 旋转矩阵直接从加速度测量构造，对所有模型都是相同的，因此只需计算一次。假设：
	// 1）偏航角为零 - 偏航稍后在每个模型中对齐，当速度融合开始时。
	// 2）车辆没有加速，因此所有测量的加速度都归因于重力。

	// 倾斜是测量的重力与垂直轴之间的旋转
	Quatf q(delta_vel, Vector3f(0.f, 0.f, -1.f)); // 创建四元数

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) { // 遍历所有模型
		_ahrs_ekf_gsf[model_index].q = q; // 更新模型的姿态
	}
}

void EKFGSF_yaw::ahrsAlignYaw() // AHRS对齐偏航函数
{
	// 对每个模型对齐偏航角
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) { // 遍历所有模型
		const float yaw = wrap_pi(_ekf_gsf[model_index].X(2)); // 获取偏航角并进行包裹
		const Dcmf R(_ahrs_ekf_gsf[model_index].q); // 获取旋转矩阵
		_ahrs_ekf_gsf[model_index].q = Quatf(updateYawInRotMat(yaw, R)); // 更新模型的姿态
	}
}

void EKFGSF_yaw::predictEKF(const uint8_t model_index, const Vector3f &delta_ang, const float delta_ang_dt,
			    const Vector3f &delta_vel, const float delta_vel_dt, bool in_air) // EKF预测函数
{
	// 使用IMU数据生成姿态参考
	ahrsPredict(model_index, delta_ang, delta_ang_dt); // 调用AHRS预测

	// 在有规律的速度观测之前，不开始运行EKF算法
	if (!_ekf_gsf_vel_fuse_started) {
		return; // 如果速度融合尚未开始，直接返回
	}

	const Dcmf R(_ahrs_ekf_gsf[model_index].q); // 获取当前模型的旋转矩阵

	// 通过投影到水平面来计算偏航状态，避免万向节锁定
	_ekf_gsf[model_index].X(2) = getEulerYaw(R); // 获取偏航角

	// 计算水平前右框架中的增量速度
	const Vector3f del_vel_NED = R * delta_vel; // 计算在NED框架中的增量速度
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2)); // 计算偏航角的余弦值
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2)); // 计算偏航角的正弦值
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw; // 计算x方向的增量速度
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw; // 计算y方向的增量速度
	const float daz = Vector3f(R * delta_ang)(2); // 计算z方向的增量角度

	// 如果不在空中，增量速度过程噪声加倍
	const float accel_noise = in_air ? _accel_noise : 2.f * _accel_noise; // 根据是否在空中设置加速度噪声
	const float d_vel_var = sq(accel_noise * delta_vel_dt); // 计算增量速度方差

	// 使用固定值作为增量角度过程噪声方差
	const float d_ang_var = sq(_gyro_noise * delta_ang_dt); // 计算增量角度方差

	// 预测协方差
	_ekf_gsf[model_index].P = sym::YawEstPredictCovariance(_ekf_gsf[model_index].X, _ekf_gsf[model_index].P, Vector2f(dvx,
				  dvy), d_vel_var, daz, d_ang_var); // 更新协方差

	// 协方差矩阵是对称的，因此将上半部分复制到下半部分
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1); // 复制协方差
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2); // 复制协方差
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2); // 复制协方差

	// 限制方差
	const float min_var = 1e-6f; // 定义最小方差

	for (unsigned index = 0; index < 3; index++) { // 遍历协方差矩阵的对角线
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var); // 限制方差
	}

	// 在地球框架中累加增量速度：
	_ekf_gsf[model_index].X(0) += del_vel_NED(0); // 更新x方向的状态
	_ekf_gsf[model_index].X(1) += del_vel_NED(1); // 更新y方向的状态
}

bool EKFGSF_yaw::updateEKF(const uint8_t model_index, const Vector2f &vel_NE, const float vel_accuracy) // 更新EKF函数
{
	// 根据GPS提供的精度估计设置观测方差，并应用合理的最小值检查
	const float vel_obs_var = sq(fmaxf(vel_accuracy, 0.01f)); // 计算观测方差

	// 计算速度观测创新
	_ekf_gsf[model_index].innov = _ekf_gsf[model_index].X.xy() - vel_NE; // 计算创新

	matrix::Matrix<float, 3, 2> K; // 初始化卡尔曼增益矩阵
	matrix::SquareMatrix<float, 3> P_new; // 初始化新的协方差矩阵
	matrix::SquareMatrix<float, 2> S_inverse; // 初始化S的逆矩阵

	sym::YawEstComputeMeasurementUpdate(_ekf_gsf[model_index].P,
					    vel_obs_var,
					    FLT_EPSILON,
					    &S_inverse,
					    &_ekf_gsf[model_index].S_det_inverse,
					    &K,
					    &P_new); // 计算测量更新

	_ekf_gsf[model_index].P = P_new; // 更新协方差矩阵

	// 将上半部分复制到下半部分
	_ekf_gsf[model_index].P(1, 0) = _ekf_gsf[model_index].P(0, 1); // 复制协方差
	_ekf_gsf[model_index].P(2, 0) = _ekf_gsf[model_index].P(0, 2); // 复制协方差
	_ekf_gsf[model_index].P(2, 1) = _ekf_gsf[model_index].P(1, 2); // 复制协方差

	// 限制方差
	const float min_var = 1e-6f; // 定义最小方差

	for (unsigned index = 0; index < 3; index++) { // 遍历协方差矩阵的对角线
		_ekf_gsf[model_index].P(index, index) = fmaxf(_ekf_gsf[model_index].P(index, index), min_var); // 限制方差
	}

	// 归一化创新平方 = 转置(创新) * 逆(创新方差) * 创新 = [1x2] * [2,2] * [2,1] = [1,1]
	_ekf_gsf[model_index].nis = _ekf_gsf[model_index].innov * (S_inverse * _ekf_gsf[model_index].innov); // 计算归一化创新

	// 执行卡方创新一致性测试，并计算压缩缩放因子
	// 限制创新的大小为5个标准差
	// 如果归一化创新平方大于25（5个标准差），则减少创新向量的长度，将其限制在5个标准差
	// 这可以防止大测量尖峰
	if (_ekf_gsf[model_index].nis > sq(5.f)) { // 如果创新超出范围
		_ekf_gsf[model_index].innov *= sqrtf(sq(5.f) / _ekf_gsf[model_index].nis); // 限制创新
		_ekf_gsf[model_index].nis = sq(5.f); // 更新创新平方
	}

	// 修正状态向量
	const Vector3f delta_state = -K * _ekf_gsf[model_index].innov; // 计算状态增量
	const float yawDelta = delta_state(2); // 获取偏航增量

	_ekf_gsf[model_index].X.xy() += delta_state.xy(); // 更新状态
	_ekf_gsf[model_index].X(2) = wrap_pi(_ekf_gsf[model_index].X(2) + yawDelta); // 更新偏航状态

	// 使用左乘法将偏航角的变化应用于AHRS，以围绕地球下轴旋转姿态
	const Quatf dq(cosf(yawDelta / 2.f), 0.f, 0.f, sinf(yawDelta / 2.f)); // 创建增量四元数
	_ahrs_ekf_gsf[model_index].q = (dq * _ahrs_ekf_gsf[model_index].q).normalized(); // 更新姿态

	return true; // 返回成功
}

void EKFGSF_yaw::initialiseEKFGSF(const Vector2f &vel_NE, const float vel_accuracy) // 初始化EKFGSF函数
{
	_gsf_yaw = 0.0f; // 初始化复合偏航为0
	_gsf_yaw_variance = sq(M_PI_F / 2.f); // 初始化复合偏航方差
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // 所有滤波模型以相同的权重开始

	const float yaw_increment = 2.f * M_PI_F / (float)N_MODELS_EKFGSF; // 计算偏航增量

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) { // 遍历所有模型
		_ekf_gsf[model_index] = {}; // 初始化模型状态

		// 在+-Pi之间均匀分布初始偏航估计
		_ekf_gsf[model_index].X(2) = -M_PI_F + (0.5f * yaw_increment) + ((float)model_index * yaw_increment); // 设置初始偏航

		// 从最后一次测量中获取速度状态和相应的方差
		_ekf_gsf[model_index].X(0) = vel_NE(0); // 设置x方向速度
		_ekf_gsf[model_index].X(1) = vel_NE(1); // 设置y方向速度

		_ekf_gsf[model_index].P(0, 0) = sq(fmaxf(vel_accuracy, 0.01f)); // 设置x方向方差
		_ekf_gsf[model_index].P(1, 1) = _ekf_gsf[model_index].P(0, 0); // 设置y方向方差

		// 使用偏航增量的一半作为偏航不确定性
		_ekf_gsf[model_index].P(2, 2) = sq(0.5f * yaw_increment); // 设置偏航方差
	}
}

float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const // 计算高斯密度函数
{
	return (1.f / (2.f * M_PI_F)) * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * _ekf_gsf[model_index].nis); // 返回高斯密度
}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			    float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const // 获取日志数据函数
{
	if (_ekf_gsf_vel_fuse_started) { // 如果速度融合已开始
		*yaw_composite = _gsf_yaw; // 获取复合偏航
		*yaw_variance = _gsf_yaw_variance; // 获取复合偏航方差

		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) { // 遍历所有模型
			yaw[model_index] = _ekf_gsf[model_index].X(2); // 获取每个模型的偏航
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0); // 获取每个模型的北向创新
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1); // 获取每个模型的东向创新
			weight[model_index] = _model_weights(model_index); // 获取每个模型的权重
		}

		return true; // 返回成功
	}

	return false; // 返回失败
}

float EKFGSF_yaw::ahrsCalcAccelGain() const // 计算加速度融合增益函数
{
	// 使用一个连续函数计算加速度融合增益，该函数在1g时为1，在最小和最大g值时为0。
	// 在飞行时允许更多的加速度，使用向心加速度修正，因为会经历更高和更持续的g。
	// 使用二次函数而不是线性函数，以防止在1g附近的振动降低倾斜修正的有效性。
	// 参见 https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f; // 初始化衰减因子
	const bool centripetal_accel_compensation_enabled = PX4_ISFINITE(_true_airspeed) && (_true_airspeed > FLT_EPSILON); // 检查向心加速度补偿是否有效

	const float ahrs_accel_norm = _ahrs_accel.norm(); // 计算AHRS加速度的范数

	if (centripetal_accel_compensation_enabled && (ahrs_accel_norm > CONSTANTS_ONE_G)) { // 如果向心加速度补偿有效且加速度大于1g
		attenuation = 1.f; // 设置衰减因子为1
	}

	const float delta_accel_g = (ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G; // 计算加速度差
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f)); // 返回加速度融合增益
}
