/****************************************************************************
 * ImuAhrs.cpp - IMU 姿态融合模块实现文件
 *
 * 本模块主要功能：
 *  - 订阅 uORB 上的 sensor_combined 消息，获取 IMU 原始数据（加速度与角速度）
 *  - 调用核心滤波器 (core::IMUFilter) 对原始数据进行预处理，得到滤波后的数据
 *  - 利用陀螺仪积分计算增量四元数，并与当前姿态四元数相乘得到预测姿态
 *  - 利用滤波后的加速度数据计算理论重力向量，并与实际数据比较获得残差 residual_xy，
 *    从而计算 yaw 融合权重；同时调用 core::YawDriftCompensator 对 yaw 进行补偿
 *  - 采用互补滤波修正 roll 和 pitch（yaw 保持预测值），更新最新姿态四元数
 *  - 发布 vehicle_attitude 和自定义的 imu_ahrs_status 消息到 uORB
 *
 * 模块接口符合 PX4 自定义模块要求，继承自 ModuleBase、ModuleParams 与 ScheduledWorkItem
 ****************************************************************************/

#include "ImuAhrs.hpp"
#include <px4_platform_common/px4_log.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/imu_ahrs_status.h>
#include <math.h>

// 引入 core 和 math 层相关头文件（注意：实际项目中应包含对应的头文件路径）
#include "core/IMUFilter.hpp"
#include "core/YawDriftCompensator.hpp"
#include "math/QuaternionMath.hpp"
#include "math/EulerMath.hpp"

using namespace QuaternionMath;
using namespace EulerMath;
using matrix::Vector3f;

// 为方便使用，将核心对象作为成员变量
// 注：在 ImuAhrs.hpp 中未声明 _q 与 _initialized，此处我们在构造函数中进行初始化
// 并将其作为本模块的内部状态

// 构造函数
ImuAhrs::ImuAhrs() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_last_update(0),
	_dt(0.0f)
{
	// 初始化内部姿态状态为单位四元数
	_q[0] = 1.0f;
	_q[1] = 0.0f;
	_q[2] = 0.0f;
	_q[3] = 0.0f;
	_initialized = false;
}

// 析构函数
ImuAhrs::~ImuAhrs()
{
	// 清理工作：取消调度
	ScheduleClear();
}

// 初始化模块
bool ImuAhrs::init()
{
	// 根据需要设置调度周期，此处设置为 10ms (100Hz)
	ScheduleOnInterval(10_ms);
	return true;
}

// 主循环
void ImuAhrs::Run()
{
	// 判断是否退出
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 如果有新的 IMU 数据，执行更新
	sensor_combined_s imu;
	if (_sensor_combined_sub.update(&imu)) {
		// 更新采样间隔
		hrt_abstime now = hrt_absolute_time();
		if (_last_update != 0) {
			_dt = (now - _last_update) * 1e-6f;  // 转换为秒
		}
		_last_update = now;

		// 更新 IMU 数据与姿态融合
		updateIMU();
		updateAHRS();
	}
}

// 更新 IMU 数据，主要用于预处理（本例中直接调用 updateAHRS() 实现融合处理）
void ImuAhrs::updateIMU()
{
	// 本例中不做额外处理，所有数据在 updateAHRS() 中处理
}

// 更新姿态融合与发布
void ImuAhrs::updateAHRS()
{
	// 1. 从 sensor_combined 消息中获取原始数据
	sensor_combined_s imu;
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub.get(), &imu);

	// 提取原始数据
	float ax_raw = imu.accelerometer_m_s2[0];
	float ay_raw = imu.accelerometer_m_s2[1];
	float az_raw = imu.accelerometer_m_s2[2];
	float gx_raw = imu.gyro_rad[0];
	float gy_raw = imu.gyro_rad[1];
	float gz_raw = imu.gyro_rad[2];

	// 若尚未初始化，则使用加速度初始化姿态
	if (!_initialized) {
		// 调用辅助函数 _initialize_by_accel()，利用加速度数据计算初始 roll, pitch（yaw = 0）
		// 此处实现见下文私有辅助函数 _initialize_by_accel
		// 更新 _q 数组，并设置 _initialized 为 true
		// 注意：本辅助函数基于 math/EulerMath 与 QuaternionMath 实现
		// 先计算 roll, pitch
		float roll, pitch;
		{
			float norm = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
			norm = fmaxf(norm, 1e-12f);
			roll = atan2f(ay_raw / norm, az_raw / norm);
			pitch = atan2f(-ax_raw / norm, sqrtf((ay_raw/ norm)*(ay_raw/ norm) + (az_raw/ norm)*(az_raw/ norm)));
		}
		float yaw = 0.0f;
		// 通过 EulerMath::eulerToQuaternion 得到初始四元数
		eulerToQuaternion(roll, pitch, yaw, _q);
		_initialized = true;
	}

	// 2. 使用 core::IMUFilter 对原始数据进行预处理滤波
	float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
	// _imuFilter 为核心滤波器对象，调用 update_all()，采样周期 _dt
	// 注意：_dt 可能在第一次调用时为0，可加保护
	if (_dt <= 0.0f) {
		_dt = 0.01f; // 默认设置 10ms
	}
	_imuFilter.update_all(ax_raw, ay_raw, az_raw,
	                      gx_raw, gy_raw, gz_raw,
	                      _dt,
	                      ax_f, ay_f, az_f,
	                      gx_f, gy_f, gz_f);

	// 3. 利用滤波后的陀螺仪数据计算增量四元数 dq1（未修正 yaw）
	float dq1[4];
	_gyro_to_quat_delta(gx_f, gy_f, gz_f, _dt, dq1);

	// 4. 预测姿态： q_pred = _q * dq1
	float q_pred[4];
	_quat_multiply(_q, dq1, q_pred);

	// 5. 计算预测姿态的欧拉角
	float pred_roll, pred_pitch, pred_yaw;
	_quat_to_euler(q_pred, &pred_roll, &pred_pitch, &pred_yaw);

	// 6. 根据预测的 roll、pitch（忽略 yaw=0）计算理论重力向量
	float gpx, gpy, gpz;
	_compute_gravity_ignore_yaw(pred_roll, pred_pitch, &gpx, &gpy, &gpz);

	// 7. 计算加速度滤波结果与理论重力的 XY 残差
	float residual_xy = sqrtf((ax_f - gpx)*(ax_f - gpx) + (ay_f - gpy)*(ay_f - gpy));

	// 8. 根据残差计算 yaw 融合权重，设定阈值 T = 0.5
	float T = 0.5f;
	float w_yaw = fmaxf(0.0f, 1.0f - (residual_xy / T));

	// 9. ZUPT 补偿：计算陀螺仪滤波后角速度模长
	float gyro_norm = sqrtf(gx_f*gx_f + gy_f*gy_f + gz_f*gz_f);
	// 判断是否静止：若陀螺仪角速度小于 0.05 且残差小于 0.05，则认为静止
	bool is_static = (gyro_norm < 0.05f && residual_xy < 0.05f);
	// 调用 YawDriftCompensator 更新 yaw 偏置
	float gz_comp = _yawDriftComp.update_bias(gz_f, _dt, is_static);

	// 10. 计算融合后的 yaw 角速度：用权重 w_yaw 融合原始 gz_f 与补偿后值 gz_comp
	float gz_final = w_yaw * gz_comp + (1.0f - w_yaw) * gz_f;

	// 11. 利用修正后的陀螺仪数据 (gx_f, gy_f, gz_final) 重新计算增量四元数 dq2
	float dq2[4];
	_gyro_to_quat_delta(gx_f, gy_f, gz_final, _dt, dq2);

	// 12. 用当前姿态更新： q_new = _q * dq2
	float q_new[4];
	_quat_multiply(_q, dq2, q_new);

	// 13. 互补滤波：利用加速度数据修正 q_new 的 roll 和 pitch（yaw 保持 q_new 中的值）
	float q_fused[4];
	_complementary_accel(q_new, ax_f, ay_f, az_f, q_fused);

	// 14. 归一化 q_fused，并更新内部姿态 _q
	_normalize_quat(q_fused);
	_q[0] = q_fused[0];
	_q[1] = q_fused[1];
	_q[2] = q_fused[2];
	_q[3] = q_fused[3];

	// 15. 构造 vehicle_attitude 消息并发布
	vehicle_attitude_s att{};
	att.timestamp = hrt_absolute_time();
	att.q[0] = _q[0];
	att.q[1] = _q[1];
	att.q[2] = _q[2];
	att.q[3] = _q[3];
	_attitude_pub.publish(att);

	// 16. 构造 imu_ahrs_status 消息并发布
	imu_ahrs_status_s status{};
	status.timestamp = att.timestamp;
	{
		// 利用 EulerMath 将当前四元数转换为欧拉角
		float roll, pitch, yaw;
		_quat_to_euler(_q, &roll, &pitch, &yaw);
		status.euler_angle[0] = roll;
		status.euler_angle[1] = pitch;
		status.euler_angle[2] = yaw;
	}
	// 复制四元数数据
	status.quaternion[0] = _q[0];
	status.quaternion[1] = _q[1];
	status.quaternion[2] = _q[2];
	status.quaternion[3] = _q[3];
	// 此处 confidence 暂设为1.0
	status.confidence = 1.0f;
	_ahrs_status_pub.publish(status);
}

// ---------------------- 私有辅助函数 ----------------------

// 以下辅助函数均实现了与 Python 代码类似的功能

/**
 * @brief 计算陀螺仪数据产生的增量四元数
 *
 * 根据陀螺仪角速度 (gx, gy, gz) 和采样周期 dt_s 计算增量四元数 dq。
 * 当角速度较小时，使用泰勒展开优化积分精度。
 */
void ImuAhrs::_gyro_to_quat_delta(float gx, float gy, float gz, float dt_s, float dq[4])
{
    float omega_sq = gx*gx + gy*gy + gz*gz;
    if (omega_sq < 1e-8f) {
        float scale = 0.5f * dt_s * (1.0f - omega_sq * dt_s * dt_s / 24.0f);
        dq[0] = 1.0f - omega_sq * dt_s * dt_s / 8.0f;
        dq[1] = gx * scale;
        dq[2] = gy * scale;
        dq[3] = gz * scale;
    } else {
        float omega = sqrtf(omega_sq);
        float theta = omega * dt_s;
        float half_theta = 0.5f * theta;
        float sin_ht = sinf(half_theta);
        float cos_ht = cosf(half_theta);
        float s = sin_ht / omega;
        dq[0] = cos_ht;
        dq[1] = gx * s;
        dq[2] = gy * s;
        dq[3] = gz * s;
    }
}

/**
 * @brief 四元数乘法，result = q1 * q2，并归一化结果
 */
void ImuAhrs::_quat_multiply(const float q1[4], const float q2[4], float result[4])
{
    float w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    float x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    float y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    float z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    result[0] = w;
    result[1] = x;
    result[2] = y;
    result[3] = z;
    _normalize_quat(result);
}

/**
 * @brief 将四元数转换为欧拉角 (roll, pitch, yaw)
 */
void ImuAhrs::_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    norm = fmaxf(norm, 1e-12f);
    float w = q[0] / norm;
    float x = q[1] / norm;
    float y = q[2] / norm;
    float z = q[3] / norm;

    // 计算 roll
    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // 计算 pitch
    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI/2.0f, sinp);
    } else {
        *pitch = asinf(sinp);
    }

    // 计算 yaw
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

/**
 * @brief 归一化四元数，使其模长为1
 */
void ImuAhrs::_normalize_quat(float q[4])
{
    float mag = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    mag = fmaxf(mag, 1e-12f);
    q[0] /= mag;
    q[1] /= mag;
    q[2] /= mag;
    q[3] /= mag;
}

/**
 * @brief 互补滤波融合，加速度数据用于修正预测姿态中的 roll 与 pitch（yaw 不修正）
 *
 * 输入：
 *   pred_q: 预测姿态四元数 (数组长度 4)
 *   ax_f, ay_f, az_f: 滤波后的加速度数据
 * 输出：
 *   fused_q: 融合后的四元数
 */
void ImuAhrs::_complementary_accel(const float pred_q[4], float ax_f, float ay_f, float az_f, float fused_q[4])
{
    // 将预测四元数转换为欧拉角
    float pred_roll, pred_pitch, pred_yaw;
    _quat_to_euler(pred_q, &pred_roll, &pred_pitch, &pred_yaw);

    // 利用加速度数据计算 roll 和 pitch（假设重力加速度为 9.81 m/s^2）
    float norm = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    norm = fmaxf(norm, 1e-12f);
    float axn = ax_f / norm;
    float ayn = ay_f / norm;
    float azn = az_f / norm;
    float acc_roll = atan2f(ayn, azn);
    float acc_pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));

    // 根据加速度与理论值的偏差构造互补权重 alpha
    // 这里采用简单方法：假设加速度稳定时 alpha 较小（更多依赖加速度），不稳定时 alpha 较大（依赖预测）
    float diff = fabsf(norm - 9.81f);
    float alpha = 0.02f + (0.98f - 0.02f) * fminf(diff / 9.81f, 1.0f);

    // 仅修正 roll 和 pitch（yaw 保持预测值）
    float fused_roll  = alpha * pred_roll + (1.0f - alpha) * acc_roll;
    float fused_pitch = alpha * pred_pitch + (1.0f - alpha) * acc_pitch;
    float fused_yaw   = pred_yaw;

    // 利用 EulerMath 将融合后的欧拉角转换回四元数
    eulerToQuaternion(fused_roll, fused_pitch, fused_yaw, fused_q);
}

/**
 * @brief 用加速度数据初始化姿态（roll, pitch）; yaw 置0
 */
void ImuAhrs::_initialize_by_accel(float ax, float ay, float az)
{
    float roll, pitch;
    {
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        norm = fmaxf(norm, 1e-12f);
        roll = atan2f(ay / norm, az / norm);
        pitch = atan2f(-ax / norm, sqrtf((ay/ norm)*(ay/ norm) + (az/ norm)*(az/ norm)));
    }
    float yaw = 0.0f;
    eulerToQuaternion(roll, pitch, yaw, _q);
    _initialized = true;
}

/**
 * @brief 计算忽略 yaw 时的理论重力向量
 *
 * 输入：预测的 roll, pitch
 * 输出：gpx, gpy, gpz 重力向量分量（单位 m/s^2）
 */
void ImuAhrs::_compute_gravity_ignore_yaw(float roll, float pitch, float *gpx, float *gpy, float *gpz)
{
    const float g = 9.81f;
    // 先计算 Rx(roll) 作用在 [0, 0, g] 上
    float sr = sinf(roll);
    float cr = cosf(roll);
    float gx1 = 0.0f;
    float gy1 = -sr * g;
    float gz1 = cr * g;
    // 再计算 Ry(pitch) 作用：Ry(pitch) * [gx1, gy1, gz1]
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    *gpx = cp * gx1 + sp * gz1;
    *gpy = gy1;
    *gpz = -sp * gx1 + cp * gz1;
}

/**
 * @brief 通过陀螺仪数据计算增量四元数 dq
 */
void ImuAhrs::_gyro_to_quat_delta(float gx, float gy, float gz, float dt_s, float dq[4])
{
    float omega_sq = gx*gx + gy*gy + gz*gz;
    if (omega_sq < 1e-8f) {
        float scale = 0.5f * dt_s * (1.0f - omega_sq * dt_s * dt_s / 24.0f);
        dq[0] = 1.0f - omega_sq * dt_s * dt_s / 8.0f;
        dq[1] = gx * scale;
        dq[2] = gy * scale;
        dq[3] = gz * scale;
    } else {
        float omega = sqrtf(omega_sq);
        float theta = omega * dt_s;
        float half_theta = 0.5f * theta;
        float sin_ht = sinf(half_theta);
        float cos_ht = cosf(half_theta);
        float s = sin_ht / omega;
        dq[0] = cos_ht;
        dq[1] = gx * s;
        dq[2] = gy * s;
        dq[3] = gz * s;
    }
}

/**
 * @brief 四元数乘法，result = q * dq
 */
void ImuAhrs::_quat_multiply(const float q[4], const float dq[4], float result[4])
{
    float w = q[0]*dq[0] - q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3];
    float x = q[0]*dq[1] + q[1]*dq[0] + q[2]*dq[3] - q[3]*dq[2];
    float y = q[0]*dq[2] - q[1]*dq[3] + q[2]*dq[0] + q[3]*dq[1];
    float z = q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1] + q[3]*dq[0];
    result[0] = w;
    result[1] = x;
    result[2] = y;
    result[3] = z;
    _normalize_quat(result);
}

/**
 * @brief 将四元数转换为欧拉角 (roll, pitch, yaw)
 */
void ImuAhrs::_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    norm = fmaxf(norm, 1e-12f);
    float w = q[0] / norm;
    float x = q[1] / norm;
    float y = q[2] / norm;
    float z = q[3] / norm;

    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w*y - z*x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI/2.0f, sinp);
    else
        *pitch = asinf(sinp);

    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

/**
 * @brief 对四元数进行归一化处理
 */
void ImuAhrs::_normalize_quat(float q[4])
{
    float mag = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    mag = fmaxf(mag, 1e-12f);
    q[0] /= mag;
    q[1] /= mag;
    q[2] /= mag;
    q[3] /= mag;
}

/**
 * @brief 互补滤波，将加速度数据用于修正预测姿态的 roll 与 pitch（yaw 保持预测值）
 */
void ImuAhrs::_complementary_accel(const float pred_q[4], float ax_f, float ay_f, float az_f, float fused_q[4])
{
    // 将预测四元数转换为欧拉角
    float pred_roll, pred_pitch, pred_yaw;
    _quat_to_euler(pred_q, &pred_roll, &pred_pitch, &pred_yaw);

    // 利用加速度计算 roll 和 pitch（假设加速度主要由重力产生）
    float norm = sqrtf(ax_f*ax_f + ay_f*ay_f + az_f*az_f);
    norm = fmaxf(norm, 1e-12f);
    float axn = ax_f / norm;
    float ayn = ay_f / norm;
    float azn = az_f / norm;
    float acc_roll = atan2f(ayn, azn);
    float acc_pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));

    // 计算加速度可靠性：越接近 9.81 效果越好
    float diff = fabsf(norm - 9.81f);
    float reliability = 1.0f - fminf(diff / 9.81f, 1.0f);
    // 互补滤波权重：可靠性高时更多依赖加速度数据
    float alpha = 0.02f + (0.98f - 0.02f) * (1.0f - reliability);

    // 融合 roll 和 pitch
    float fused_roll  = alpha * acc_roll + (1.0f - alpha) * pred_roll;
    float fused_pitch = alpha * acc_pitch + (1.0f - alpha) * pred_pitch;
    float fused_yaw   = pred_yaw; // yaw 保持不变

    // 由融合后的欧拉角生成四元数
    eulerToQuaternion(fused_roll, fused_pitch, fused_yaw, fused_q);
}

/**
 * @brief 初始化：利用加速度数据计算初始 roll 与 pitch，yaw 置 0
 */
void ImuAhrs::_initialize_by_accel(float ax, float ay, float az)
{
    float roll, pitch;
    {
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        norm = fmaxf(norm, 1e-12f);
        roll = atan2f(ay / norm, az / norm);
        pitch = atan2f(-ax / norm, sqrtf((ay/ norm)*(ay/ norm) + (az/ norm)*(az/ norm)));
    }
    float yaw = 0.0f;
    eulerToQuaternion(roll, pitch, yaw, _q);
    _initialized = true;
}

// ---------------------- ImuAhrs 主模块 Run() 内部逻辑 ----------------------

/**
 * @brief ImuAhrs 主更新入口
 *
 * 本函数由调度工作队列周期性调用，执行以下步骤：
 *  1) 获取 sensor_combined 消息的原始 IMU 数据；
 *  2) 若未初始化，则利用加速度数据初始化姿态；
 *  3) 调用 core::IMUFilter 对原始数据进行滤波，得到滤波后数据；
 *  4) 计算陀螺仪积分产生的增量四元数 dq1，预测姿态 q_pred = _q * dq1；
 *  5) 将 q_pred 转换为欧拉角，利用加速度数据计算理论重力向量，
 *     得到残差 residual_xy 及 yaw 融合权重 w_yaw；
 *  6) 通过 YawDriftCompensator 补偿 gz 得到 gz_comp，并融合原始 gz 得到 gz_final；
 *  7) 重新计算增量四元数 dq2（采用修正后的 gz_final），更新姿态 q_new = _q * dq2；
 *  8) 利用互补滤波将加速度数据融合修正 roll 与 pitch，得到融合后四元数 fused_q；
 *  9) 归一化 fused_q 并更新内部姿态 _q；
 * 10) 构造 vehicle_attitude 与 imu_ahrs_status 消息并发布。
 */
void ImuAhrs::updateAHRS()
{
    // 获取 sensor_combined 消息
    sensor_combined_s imu;
    orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub.get(), &imu);

    // 提取原始数据
    float ax_raw = imu.accelerometer_m_s2[0];
    float ay_raw = imu.accelerometer_m_s2[1];
    float az_raw = imu.accelerometer_m_s2[2];
    float gx_raw = imu.gyro_rad[0];
    float gy_raw = imu.gyro_rad[1];
    float gz_raw = imu.gyro_rad[2];

    // 若未初始化，利用加速度数据初始化姿态
    if (!_initialized) {
        _initialize_by_accel(ax_raw, ay_raw, az_raw);
    }

    // 使用采样周期 _dt（若为0则默认 10ms）
    float dt_s = (_dt > 0.0f) ? _dt : 0.01f;

    // 1. 调用 core::IMUFilter 对原始数据进行滤波
    float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
    _imuFilter.update_all(ax_raw, ay_raw, az_raw,
                          gx_raw, gy_raw, gz_raw,
                          dt_s,
                          ax_f, ay_f, az_f,
                          gx_f, gy_f, gz_f);

    // 2. 利用滤波后的陀螺仪数据计算增量四元数 dq1（未修正 yaw）
    float dq1[4];
    _gyro_to_quat_delta(gx_f, gy_f, gz_f, dt_s, dq1);

    // 3. 预测姿态： q_pred = _q * dq1
    float q_pred[4];
    _quat_multiply(_q, dq1, q_pred);

    // 4. 计算预测姿态的欧拉角
    float pred_roll, pred_pitch, pred_yaw;
    _quat_to_euler(q_pred, &pred_roll, &pred_pitch, &pred_yaw);

    // 5. 根据预测 roll 和 pitch（忽略 yaw = 0），计算理论重力向量
    float gpx, gpy, gpz;
    _compute_gravity_ignore_yaw(pred_roll, pred_pitch, &gpx, &gpy, &gpz);

    // 6. 计算加速度滤波后数据与理论重力在 XY 平面的残差
    float residual_xy = sqrtf((ax_f - gpx) * (ax_f - gpx) + (ay_f - gpy) * (ay_f - gpy));

    // 7. 根据残差计算 yaw 融合权重 w_yaw（阈值 T = 0.5）
    float T = 0.5f;
    float w_yaw = fmaxf(0.0f, 1.0f - (residual_xy / T));

    // 8. 判断静止状态：若陀螺仪数据模 < 0.05 且 residual_xy < 0.05，则认为静止
    float gyro_norm = sqrtf(gx_f*gx_f + gy_f*gy_f + gz_f*gz_f);
    bool is_static = (gyro_norm < 0.05f && residual_xy < 0.05f);

    // 9. 通过 YawDriftCompensator 更新 yaw 偏置，并获得补偿后的 gz 值
    float gz_comp = _yawDriftComp.update_bias(gz_f, dt_s, is_static);

    // 10. 融合原始 gz 与补偿值： gz_final = w_yaw * gz_comp + (1 - w_yaw) * gz_f
    float gz_final = w_yaw * gz_comp + (1.0f - w_yaw) * gz_f;

    // 11. 利用修正后的陀螺仪数据 (gx_f, gy_f, gz_final) 重新计算增量四元数 dq2
    float dq2[4];
    _gyro_to_quat_delta(gx_f, gy_f, gz_final, dt_s, dq2);

    // 12. 更新姿态： q_new = _q * dq2
    float q_new[4];
    _quat_multiply(_q, dq2, q_new);

    // 13. 互补滤波：利用加速度数据修正 q_new 的 roll 与 pitch（yaw 保持不变）
    float q_fused[4];
    _complementary_accel(q_new, ax_f, ay_f, az_f, q_fused);

    // 14. 归一化 fused 四元数并更新内部姿态 _q
    _normalize_quat(q_fused);
    _q[0] = q_fused[0];
    _q[1] = q_fused[1];
    _q[2] = q_fused[2];
    _q[3] = q_fused[3];

    // 15. 构造 vehicle_attitude 消息并发布
    vehicle_attitude_s att{};
    att.timestamp = hrt_absolute_time();
    att.q[0] = _q[0];
    att.q[1] = _q[1];
    att.q[2] = _q[2];
    att.q[3] = _q[3];
    _attitude_pub.publish(att);

    // 16. 构造 imu_ahrs_status 消息并发布
    imu_ahrs_status_s status{};
    status.timestamp = att.timestamp;
    {
        float roll, pitch, yaw;
        _quat_to_euler(_q, &roll, &pitch, &yaw);
        status.euler_angle[0] = roll;
        status.euler_angle[1] = pitch;
        status.euler_angle[2] = yaw;
    }
    status.quaternion[0] = _q[0];
    status.quaternion[1] = _q[1];
    status.quaternion[2] = _q[2];
    status.quaternion[3] = _q[3];
    status.confidence = 1.0f;  // 此处固定为1.0，可根据实际情况计算
    _ahrs_status_pub.publish(status);
}

// -------------------- ImuAhrs 模块管理函数 --------------------

int ImuAhrs::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
ImuAhrs 模块：
  该模块订阅 sensor_combined 消息，获取 IMU 原始数据，利用内部滤波器与姿态融合算法，
  计算并发布 vehicle_attitude 与 imu_ahrs_status 消息，实现 IMU 姿态解算功能。

运行命令：
  imu_ahrs start
  imu_ahrs stop
  imu_ahrs status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("imu_ahrs", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ImuAhrs::print_status()
{
	PX4_INFO("ImuAhrs模块运行中");
	float roll, pitch, yaw;
	_quat_to_euler(_q, &roll, &pitch, &yaw);
	PX4_INFO("当前姿态: Roll=%.2f deg, Pitch=%.2f deg, Yaw=%.2f deg",
	         (double)math::degrees(roll),
	         (double)math::degrees(pitch),
	         (double)math::degrees(yaw));
	return 0;
}

int ImuAhrs::custom_command(int argc, char *argv[])
{
	return print_usage("未知命令");
}

ImuAhrs *ImuAhrs::instantiate(int argc, char *argv[])
{
	ImuAhrs *instance = new ImuAhrs();
	if (!instance) {
		PX4_ERR("实例分配失败");
	}
	return instance;
}

int ImuAhrs::task_spawn(int argc, char *argv[])
{
	ImuAhrs *instance = new ImuAhrs();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("imu_ahrs",
		                              SCHED_DEFAULT,
		                              SCHED_PRIORITY_ATTITUDE_ESTIMATION,
		                              2048,
		                              (px4_main_t)&run_trampoline,
		                              (char *const *)argv);

		if (_task_id < 0) {
			_object.store(nullptr);
			return -1;
		}

		return 0;
	} else {
		PX4_ERR("分配实例失败");
		_task_id = -1;
		return -1;
	}
}

// 模块主入口函数
extern "C" __EXPORT int imu_ahrs_main(int argc, char *argv[])
{
	return ImuAhrs::main(argc, argv);
}
