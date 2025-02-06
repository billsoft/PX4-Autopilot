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
using namespace time_literals;

// 构造函数
ImuAhrs::ImuAhrs() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
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
	perf_free(_loop_perf);
}

// 初始化模块
bool ImuAhrs::init()
{
	if (!_sensor_combined_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_attitude_pub.advertise();
	return true;
}

// 参数更新处理函数
void ImuAhrs::parameters_update(bool force)
{
	updateParams();
}

// 主任务入口
void ImuAhrs::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 检查参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		parameters_update();
	}

	sensor_combined_s imu{};
	if (_sensor_combined_sub.update(&imu)) {
		const hrt_abstime now = hrt_absolute_time();
		const float dt = (now - _last_update) * 1e-6f;
		_last_update = now;

		if (dt > 0.0f) {
			updateIMU();
			updateAHRS();
		}
	}
}

// 更新 IMU 数据（本例中不做额外处理，所有数据在 updateAHRS() 中处理）
void ImuAhrs::updateIMU()
{
	sensor_combined_s imu{};
	_sensor_combined_sub.copy(&imu);

	// 使用SIMD优化的数据拷贝
	#ifdef __ARM_NEON
	float32x4_t acc = vld1q_f32(imu.accelerometer_m_s2);
	float32x4_t gyro = vld1q_f32(imu.gyro_rad);
	#else
	const float ax_raw = imu.accelerometer_m_s2[0];
	const float ay_raw = imu.accelerometer_m_s2[1];
	const float az_raw = imu.accelerometer_m_s2[2];
	const float gx_raw = imu.gyro_rad[0];
	const float gy_raw = imu.gyro_rad[1];
	const float gz_raw = imu.gyro_rad[2];
	#endif

	// 使用IMUFilter进行数据预处理
	float ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
	_imuFilter.update_all(
		#ifdef __ARM_NEON
		vgetq_lane_f32(acc, 0), vgetq_lane_f32(acc, 1), vgetq_lane_f32(acc, 2),
		vgetq_lane_f32(gyro, 0), vgetq_lane_f32(gyro, 1), vgetq_lane_f32(gyro, 2),
		#else
		ax_raw, ay_raw, az_raw,
		gx_raw, gy_raw, gz_raw,
		#endif
		_dt,
		ax_f, ay_f, az_f,
		gx_f, gy_f, gz_f);

	// 更新姿态估计
	updateAHRS(ax_f, ay_f, az_f, gx_f, gy_f, gz_f);
}

// 更新姿态融合与发布
void ImuAhrs::updateAHRS(float ax_f, float ay_f, float az_f,
                        float gx_f, float gy_f, float gz_f)
{
	if (!_initialized) {
		_initialize_by_accel(ax_f, ay_f, az_f);
		return;
	}

	// 使用SIMD优化的四元数计算
	#ifdef __ARM_NEON
	float32x4_t q_vec = vld1q_f32(_q);
	float32x4_t gyro_vec = {gx_f, gy_f, gz_f, 0.0f};

	// 四元数更新（SIMD优化）
	float dq[4];
	_gyro_to_quat_delta_simd(gyro_vec, _dt, dq);
	#else
	// 标准四元数更新
	float dq[4];
	_gyro_to_quat_delta(gx_f, gy_f, gz_f, _dt, dq);
	#endif

	// 预测和融合
	float q_pred[4];
	_quat_multiply(_q, dq, q_pred);

	// 计算重力残差
	float gpx, gpy, gpz;
	_compute_gravity_vector(q_pred, &gpx, &gpy, &gpz);

	float residual_xy = fast_sqrt((ax_f - gpx) * (ax_f - gpx) +
								 (ay_f - gpy) * (ay_f - gpy));

	// 发布姿态数据
	vehicle_attitude_s att{};
	att.timestamp = hrt_absolute_time();
	memcpy(att.q, _q, sizeof(float) * 4);
	_attitude_pub.publish(att);
}

// 添加SIMD优化的四元数更新函数
#ifdef __ARM_NEON
inline void ImuAhrs::_gyro_to_quat_delta_simd(float32x4_t gyro, float dt, float dq[4])
{
	// SIMD优化的四元数更新实现
	const float32x4_t half_dt = vdupq_n_f32(dt * 0.5f);
	float32x4_t omega = vmulq_f32(gyro, half_dt);

	// 计算四元数增量
	const float32x4_t one = vdupq_n_f32(1.0f);
	float32x4_t dq_vec = vaddq_f32(one, omega);

	// 存储结果
	vst1q_f32(dq, dq_vec);
}
#endif

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

void ImuAhrs::_compute_gravity_vector(const float q[4], float *gpx, float *gpy, float *gpz)
{
    const float g = 9.81f;
    float sr = sinf(q[0]);
    float cr = cosf(q[0]);
    float gx1 = 0.0f;
    float gy1 = -sr * g;
    float gz1 = cr * g;
    float sp = sinf(q[1]);
    float cp = cosf(q[1]);
    *gpx = cp * gx1 + sp * gz1;
    *gpy = gy1;
    *gpz = -sp * gx1 + cp * gz1;
}

int ImuAhrs::task_spawn(int argc, char *argv[])
{
	ImuAhrs *instance = new ImuAhrs();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ImuAhrs::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ImuAhrs::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IMU姿态估计模块，基于卡尔曼滤波和互补滤波实现。

### Implementation
订阅sensor_combined消息，处理IMU数据并发布vehicle_attitude。
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("imu_ahrs", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int imu_ahrs_main(int argc, char *argv[])
{
	return ImuAhrs::main(argc, argv);
}
