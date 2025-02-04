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
 *  - 订阅 parameter_update 消息以支持参数动态更新
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
#include <uORB/topics/parameter_update.h>
#include <math.h>

// 引入 core 和 math 层相关头文件
#include "core/IMUFilter.hpp"
#include "core/YawDriftCompensator.hpp"
#include "math/QuaternionMath.hpp"
#include "math/EulerMath.hpp"

using namespace QuaternionMath;
using namespace EulerMath;
using matrix::Vector3f;

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

// 参数更新处理函数
void ImuAhrs::parameters_update(bool force)
{
	// 更新模块参数（由 ModuleParams 提供的接口）
	updateParameters(force);
}

// 主任务入口
void ImuAhrs::Run()
{
	// 判断是否退出
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 检查是否有参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		parameters_update();
	}

	// 获取新的 sensor_combined 数据
	sensor_combined_s imu;
	if (_sensor_combined_sub.update(&imu)) {
		hrt_abstime now = hrt_absolute_time();
		if (_last_update != 0) {
			_dt = (now - _last_update) * 1e-6f;  // 转换为秒
		}
		_last_update = now;

		updateIMU();
		updateAHRS();
	}
}

// 更新 IMU 数据（本例中不做额外处理，所有数据在 updateAHRS() 中处理）
void ImuAhrs::updateIMU()
{
	// 此处预留处理代码，如需对原始数据进行预处理，可在此添加
}

// 更新姿态融合与发布
void ImuAhrs::updateAHRS()
{
	// 1. 从 sensor_combined 消息中获取原始数据
	sensor_combined_s imu;
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub.get(), &imu);

	// 2. 提取原始数据
	float ax_raw = imu.accelerometer_m_s2[0];
	float ay_raw = imu.accelerometer_m_s2[1];
	float az_raw = imu.accelerometer_m_s2[2];
	float gx_raw = imu.gyro_rad[0];
	float gy_raw = imu.gyro_rad[1];
	float gz_raw = imu.gyro_rad[2];

	// 3. 若尚未初始化，则利用加速度数据初始化姿态
	if (!_initialized) {
		float roll, pitch;
		{
			float norm = sqrtf(ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw);
			norm = fmaxf(norm, 1e-12f);
			roll = atan2f(ay_raw / norm, az_raw / norm);
			pitch = atan2f(-ax_raw / norm, sqrtf((ay_raw / norm) * (ay_raw / norm) + (az_raw / norm) * (az_raw / norm)));
		}
		float yaw = 0.0f;
		eulerToQuaternion(roll, pitch, yaw, _q);
		_initialized = true;
	}

	// 4. 使用 core::IMUFilter 对原始数据进行滤波
	float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
	if (_dt <= 0.0f) {
		_dt = 0.01f; // 默认 10ms
	}
	_imuFilter.update_all(ax_raw, ay_raw, az_raw,
	                      gx_raw, gy_raw, gz_raw,
	                      _dt,
	                      ax_f, ay_f, az_f,
	                      gx_f, gy_f, gz_f);

	// 5. 利用滤波后的陀螺仪数据计算增量四元数 dq1（未修正 yaw）
	float dq1[4];
	_gyro_to_quat_delta(gx_f, gy_f, gz_f, _dt, dq1);

	// 6. 预测姿态： q_pred = _q * dq1
	float q_pred[4];
	_quat_multiply(_q, dq1, q_pred);

	// 7. 计算预测姿态的欧拉角
	float pred_roll, pred_pitch, pred_yaw;
	_quat_to_euler(q_pred, &pred_roll, &pred_pitch, &pred_yaw);

	// 8. 根据预测的 roll、pitch（忽略 yaw=0）计算理论重力向量
	float gpx, gpy, gpz;
	_compute_gravity_ignore_yaw(pred_roll, pred_pitch, &gpx, &gpy, &gpz);

	// 9. 计算加速度滤波结果与理论重力在 XY 平面的残差
	float residual_xy = sqrtf((ax_f - gpx) * (ax_f - gpx) + (ay_f - gpy) * (ay_f - gpy));

	// 10. 根据残差计算 yaw 融合权重（阈值 T = 0.5）
	float T = 0.5f;
	float w_yaw = fmaxf(0.0f, 1.0f - (residual_xy / T));

	// 11. 判断是否静止：若陀螺仪模 < 0.05 且残差 < 0.05，则认为静止
	float gyro_norm = sqrtf(gx_f * gx_f + gy_f * gy_f + gz_f * gz_f);
	bool is_static = (gyro_norm < 0.05f && residual_xy < 0.05f);

	// 12. 通过 YawDriftCompensator 更新 yaw 偏置，并获得补偿后的 gz
	float gz_comp = _yawDriftComp.update_bias(gz_f, _dt, is_static);

	// 13. 融合原始 gz 与补偿后的值： gz_final = w_yaw * gz_comp + (1 - w_yaw) * gz_f
	float gz_final = w_yaw * gz_comp + (1.0f - w_yaw) * gz_f;

	// 14. 利用修正后的陀螺仪数据 (gx_f, gy_f, gz_final) 重新计算增量四元数 dq2
	float dq2[4];
	_gyro_to_quat_delta(gx_f, gy_f, gz_final, _dt, dq2);

	// 15. 更新姿态： q_new = _q * dq2
	float q_new[4];
	_quat_multiply(_q, dq2, q_new);

	// 16. 互补滤波：利用加速度数据修正 q_new 的 roll 与 pitch（yaw 保持不变）
	float q_fused[4];
	_complementary_accel(q_new, ax_f, ay_f, az_f, q_fused);

	// 17. 归一化融合后的四元数并更新内部状态 _q
	_normalize_quat(q_fused);
	_q[0] = q_fused[0];
	_q[1] = q_fused[1];
	_q[2] = q_fused[2];
	_q[3] = q_fused[3];

	// 18. 构造并发布 vehicle_attitude 消息
	vehicle_attitude_s att{};
	att.timestamp = hrt_absolute_time();
	att.q[0] = _q[0];
	att.q[1] = _q[1];
	att.q[2] = _q[2];
	att.q[3] = _q[3];
	_attitude_pub.publish(att);

	// 19. 构造并发布 imu_ahrs_status 消息
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
	status.confidence = 1.0f;  // 固定为1.0，可根据需要计算
	_ahrs_status_pub.publish(status);
}

// ---------------------- 私有辅助函数 ----------------------

void ImuAhrs::_gyro_to_quat_delta(float gx, float gy, float gz, float dt_s, float dq[4])
{
    float omega_sq = gx * gx + gy * gy + gz * gz;
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

void ImuAhrs::_quat_multiply(const float q1[4], const float q2[4], float result[4])
{
    float w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    float x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    float y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    float z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    result[0] = w;
    result[1] = x;
    result[2] = y;
    result[3] = z;
    _normalize_quat(result);
}

void ImuAhrs::_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    norm = fmaxf(norm, 1e-12f);
    float w = q[0] / norm;
    float x = q[1] / norm;
    float y = q[2] / norm;
    float z = q[3] / norm;

    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(M_PI / 2.0f, sinp);
    else
        *pitch = asinf(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void ImuAhrs::_normalize_quat(float q[4])
{
    float mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    mag = fmaxf(mag, 1e-12f);
    q[0] /= mag;
    q[1] /= mag;
    q[2] /= mag;
    q[3] /= mag;
}

void ImuAhrs::_complementary_accel(const float pred_q[4], float ax_f, float ay_f, float az_f, float fused_q[4])
{
    float pred_roll, pred_pitch, pred_yaw;
    _quat_to_euler(pred_q, &pred_roll, &pred_pitch, &pred_yaw);

    float norm = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    norm = fmaxf(norm, 1e-12f);
    float axn = ax_f / norm;
    float ayn = ay_f / norm;
    float azn = az_f / norm;
    float acc_roll = atan2f(ayn, azn);
    float acc_pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    float diff = fabsf(norm - 9.81f);
    float reliability = 1.0f - fminf(diff / 9.81f, 1.0f);
    float alpha = 0.02f + (0.98f - 0.02f) * (1.0f - reliability);

    float fused_roll = alpha * acc_roll + (1.0f - alpha) * pred_roll;
    float fused_pitch = alpha * acc_pitch + (1.0f - alpha) * pred_pitch;
    float fused_yaw = pred_yaw;

    eulerToQuaternion(fused_roll, fused_pitch, fused_yaw, fused_q);
}

void ImuAhrs::_initialize_by_accel(float ax, float ay, float az)
{
    float roll, pitch;
    {
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        norm = fmaxf(norm, 1e-12f);
        roll = atan2f(ay / norm, az / norm);
        pitch = atan2f(-ax / norm, sqrtf((ay / norm) * (ay / norm) + (az / norm) * (az / norm)));
    }
    float yaw = 0.0f;
    eulerToQuaternion(roll, pitch, yaw, _q);
    _initialized = true;
}

void ImuAhrs::_compute_gravity_ignore_yaw(float roll, float pitch, float *gpx, float *gpy, float *gpz)
{
    const float g = 9.81f;
    float sr = sinf(roll);
    float cr = cosf(roll);
    float gx1 = 0.0f;
    float gy1 = -sr * g;
    float gz1 = cr * g;
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    *gpx = cp * gx1 + sp * gz1;
    *gpy = gy1;
    *gpz = -sp * gx1 + cp * gz1;
}

extern "C" __EXPORT int imu_ahrs_main(int argc, char *argv[])
{
    return ImuAhrs::main(argc, argv);
}
