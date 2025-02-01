/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4开发团队。保留所有权利。
 *
 * 以源代码和二进制形式重新分发和使用，无论是否修改，均可在满足以下条件的情况下进行：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随分发的文档或其他材料中复制上述版权声明、此条件列表和以下免责声明。
 * 3. 未经事先书面许可，PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，且不提供任何明示或暗示的担保，包括但不限于对适销性和特定用途适用性的暗示担保。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file gps_checks.cpp
 * 执行起飞前和飞行中的GPS质量检查
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

// GPS起飞前检查位位置定义
#define MASK_GPS_NSATS   (1<<0)  // 卫星数量检查标志
#define MASK_GPS_PDOP    (1<<1)  // 精度衰减检查标志
#define MASK_GPS_HACC    (1<<2)  // 水平精度检查标志
#define MASK_GPS_VACC    (1<<3)  // 垂直精度检查标志
#define MASK_GPS_SACC    (1<<4)  // 速度精度检查标志
#define MASK_GPS_HDRIFT  (1<<5)  // 水平漂移检查标志
#define MASK_GPS_VDRIFT  (1<<6)  // 垂直漂移检查标志
#define MASK_GPS_HSPD    (1<<7)  // 水平速度检查标志
#define MASK_GPS_VSPD    (1<<8)  // 垂直速度检查标志
#define MASK_GPS_SPOOFED (1<<9)  // GPS欺骗检查标志

// 运行GNSS检查的函数
bool Ekf::runGnssChecks(const gnssSample &gps)
{
	_gps_check_fail_status.flags.spoofed = gps.spoofed; // 设置GPS欺骗状态

	// 检查定位类型
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3); // 如果定位类型小于3，则标记为未定位

	// 检查卫星数量
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats); // 如果卫星数量小于要求的数量，则标记为失败

	// 检查位置精度衰减
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop); // 如果PDOP值超过要求，则标记为失败

	// 检查报告的水平和垂直位置精度
	_gps_check_fail_status.flags.hacc = (gps.hacc > _params.req_hacc); // 如果水平精度超过要求，则标记为失败
	_gps_check_fail_status.flags.vacc = (gps.vacc > _params.req_vacc); // 如果垂直精度超过要求，则标记为失败

	// 检查报告的速度精度
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc); // 如果速度精度超过要求，则标记为失败

	// 计算自上次更新以来经过的时间，限制以防止数值错误并计算低通滤波系数
	constexpr float filt_time_const = 10.0f; // 低通滤波时间常数
	const float dt = math::constrain(float(int64_t(gps.time_us) - int64_t(_gps_pos_prev.getProjectionReferenceTimestamp()))
					 * 1e-6f, 0.001f, filt_time_const); // 计算时间差，限制在0.001到10秒之间
	const float filter_coef = dt / filt_time_const; // 计算滤波系数

	// 以下检查仅在车辆静止时有效
	const double lat = gps.lat; // 获取GPS纬度
	const double lon = gps.lon; // 获取GPS经度

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// 计算自上次测量以来的位置移动
		float delta_pos_n = 0.0f; // 北向位置变化
		float delta_pos_e = 0.0f; // 东向位置变化

		// 计算自上次GPS定位以来的位置变化
		if (_gps_pos_prev.getProjectionReferenceTimestamp() > 0) {
			_gps_pos_prev.project(lat, lon, delta_pos_n, delta_pos_e); // 计算位置变化

		} else {
			// 如果没有设置先前位置
			_gps_pos_prev.initReference(lat, lon, gps.time_us); // 初始化参考位置
			_gps_alt_prev = gps.alt; // 保存上次GPS高度
		}

		// 计算水平和垂直漂移速度分量，并限制在阈值的10倍以内
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift); // 定义速度限制
		Vector3f delta_pos(delta_pos_n, delta_pos_e, (_gps_alt_prev - gps.alt)); // 计算位置变化向量

		// 应用低通滤波器
		_gps_pos_deriv_filt = delta_pos / dt * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef); // 计算滤波后的速度

		// 应用反饱和处理，避免在不对称信号上产生偏差
		_gps_pos_deriv_filt = matrix::constrain(_gps_pos_deriv_filt, -10.0f * vel_limit, 10.0f * vel_limit); // 限制速度在合理范围内

		// hdrift: 计算水平漂移速度，如果过高则标记为失败
		_gps_horizontal_position_drift_rate_m_s = Vector2f(_gps_pos_deriv_filt.xy()).norm(); // 计算水平漂移速度的大小
		_gps_check_fail_status.flags.hdrift = (_gps_horizontal_position_drift_rate_m_s > _params.req_hdrift); // 检查水平漂移速度是否超过阈值

		// vdrift: 如果垂直漂移速度过高则标记为失败
		_gps_vertical_position_drift_rate_m_s = fabsf(_gps_pos_deriv_filt(2)); // 计算垂直漂移速度的大小
		_gps_check_fail_status.flags.vdrift = (_gps_vertical_position_drift_rate_m_s > _params.req_vdrift); // 检查垂直漂移速度是否超过阈值

		// hspeed: 检查滤波后的水平GNSS速度的大小
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel.xy()),
					   -10.0f * _params.req_hdrift,
					   10.0f * _params.req_hdrift); // 限制水平速度在合理范围内
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef); // 计算滤波后的水平速度
		_gps_filtered_horizontal_velocity_m_s = _gps_velNE_filt.norm(); // 计算滤波后的水平速度的大小
		_gps_check_fail_status.flags.hspeed = (_gps_filtered_horizontal_velocity_m_s > _params.req_hdrift); // 检查水平速度是否超过阈值

		// vspeed: 检查滤波后的垂直GNSS速度的大小
		const float gnss_vz_limit = 10.f * _params.req_vdrift; // 定义垂直速度限制
		const float gnss_vz = math::constrain(gps.vel(2), -gnss_vz_limit, gnss_vz_limit); // 限制垂直速度在合理范围内
		_gps_vel_d_filt = gnss_vz * filter_coef + _gps_vel_d_filt * (1.f - filter_coef); // 计算滤波后的垂直速度

		_gps_check_fail_status.flags.vspeed = (fabsf(_gps_vel_d_filt) > _params.req_vdrift); // 检查垂直速度是否超过阈值

	} else if (_control_status.flags.in_air) {
		// 当飞行时，这些检查始终被视为通过
		// 如果在地面并且正在移动，则保留移动开始前的最后结果
		_gps_check_fail_status.flags.hdrift = false; // 水平漂移标志重置
		_gps_check_fail_status.flags.vdrift = false; // 垂直漂移标志重置
		_gps_check_fail_status.flags.hspeed = false; // 水平速度标志重置
		_gps_check_fail_status.flags.vspeed = false; // 垂直速度标志重置

		resetGpsDriftCheckFilters(); // 重置GPS漂移检查滤波器

	} else {
		// 车辆在地面且IMU运动阻碍漂移计算的情况
		resetGpsDriftCheckFilters(); // 重置GPS漂移检查滤波器
	}

	// 如果水平速度超过限制，则强制标记为失败
	if (gps.vel.xy().longerThan(_params.velocity_limit)) {
		_gps_check_fail_status.flags.hspeed = true; // 水平速度标志标记为失败
	}

	// 如果垂直速度超过限制，则强制标记为失败
	if (fabsf(gps.vel(2)) > _params.velocity_limit) {
		_gps_check_fail_status.flags.vspeed = true; // 垂直速度标志标记为失败
	}

	// 保存GPS定位以备下次使用
	_gps_pos_prev.initReference(lat, lon, gps.time_us); // 初始化参考位置
	_gps_alt_prev = gps.alt; // 保存上次GPS高度

	// 假设第一次通过时失败
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_delayed_us; // 记录失败时间
	}

	// 如果任何用户选择的检查失败，记录失败时间
	if (
		_gps_check_fail_status.flags.fix || // 检查定位状态
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) || // 检查卫星数量
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) || // 检查精度衰减
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) || // 检查水平精度
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) || // 检查垂直精度
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) || // 检查速度精度
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) || // 检查水平漂移
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) || // 检查垂直漂移
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) || // 检查水平速度
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD)) || // 检查垂直速度
		(_gps_check_fail_status.flags.spoofed && (_params.gps_check_mask & MASK_GPS_SPOOFED)) // 检查GPS欺骗
	) {
		_last_gps_fail_us = _time_delayed_us; // 记录失败时间
		return false; // 返回失败
	} else {
		_last_gps_pass_us = _time_delayed_us; // 记录成功时间
		return true; // 返回成功
	}
}

// 重置GPS漂移检查滤波器的函数
void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero(); // 将水平速度滤波器设置为零
	_gps_vel_d_filt = 0.f; // 将垂直速度滤波器设置为零

	_gps_pos_deriv_filt.setZero(); // 将位置导数滤波器设置为零

	_gps_horizontal_position_drift_rate_m_s = NAN; // 水平位置漂移速率初始化为非数值
	_gps_vertical_position_drift_rate_m_s = NAN; // 垂直位置漂移速率初始化为非数值
	_gps_filtered_horizontal_velocity_m_s = NAN; // 滤波后的水平速度初始化为非数值
}
