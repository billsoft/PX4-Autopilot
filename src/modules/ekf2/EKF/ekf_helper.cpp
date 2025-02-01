#include "ekf.h"

#include <mathlib/mathlib.h>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <cstdlib>

/****************************************************************************
 *
 *   版权声明 (c) 2015-2023 PX4开发团队 (PX4 Development Team). 保留所有权利.
 *
 *   无论是否进行修改，以下条件都必须得到满足，才能使用本软件源代码和二进制形式：
 *
 *   1. 源代码的再发布必须保留上述版权声明、本条件列表以及下面的免责声明。
 *   2. 以二进制形式再发布时，必须在随附的文档和/或其它材料中，
 *      包含上述版权声明、本条件列表以及下面的免责声明。
 *   3. 未经事先书面许可，不得使用PX4或者其贡献者的名称来为本软件衍生品做推广或背书。
 *
 *   本软件由版权拥有者和贡献者按“原样”提供，并且不提供任何明示或暗示的保证，
 *   包括但不限于适销性和适用性的暗示保证。在任何情况下，版权拥有者或贡献者都不对使用本软件
 *   而产生的任何直接、间接、附带、特殊、惩罚性或衍生性损害（包括但不限于获取替代产品或服务，
 *   使用损失，数据或利润损失，或业务中断）承担责任，即使在事先已被告知该等损害的可能性。
 *
 ****************************************************************************/

/**
 * @file ekf_helper.cpp
 * 该文件定义了与EKF辅助功能相关的实现，包含若干帮助函数。
 */

bool Ekf::isHeightResetRequired() const
{
	// 检查垂直方向的加速度传感器数据是否持续不可靠，如果超时则需要重置高度
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// 检查高度融合是否超时，没有在预期时间内进行高度融合，也需要重置
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max);

	// 当连续不良的加速度数据或高度融合超时时，需要重置高度
	return (continuous_bad_accel_hgt || hgt_fusion_timeout);
}

Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	// 计算地球自转在NED坐标系下的角速度向量，lat_rad为当前纬度（弧度）
	// 地球自转速率约为0.000072722052 rad/s (CONSTANTS_EARTH_SPIN_RATE)
	// 在NED下，x轴为北向，y轴为东向，z轴朝地心方向
	// cos(lat)*地球自转速度在N轴分量，sin(lat)*地球自转速度在Z轴分量
	return Vector3f(
		CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
		0.0f,
		-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad)
	);
}

void Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	// 获取EKF全局原点的信息
	// origin_time：原点对应的时间戳
	// latitude, longitude：原点的纬度、经度
	// origin_alt：原点的高度

	origin_time = _local_origin_lat_lon.getProjectionReferenceTimestamp();
	latitude = _local_origin_lat_lon.getProjectionReferenceLat();
	longitude = _local_origin_lat_lon.getProjectionReferenceLon();
	origin_alt  = getEkfGlobalOriginAltitude();
}

bool Ekf::checkLatLonValidity(const double latitude, const double longitude)
{
	// 检查纬度和经度数值是否在合理范围内，并且是有限数
	// 纬度范围：[-90, 90]，经度范围：[-180,180]
	const bool lat_valid = (PX4_ISFINITE(latitude) && (abs(latitude) <= 90));
	const bool lon_valid = (PX4_ISFINITE(longitude) && (abs(longitude) <= 180));

	return (lat_valid && lon_valid);
}

bool Ekf::checkAltitudeValidity(const float altitude)
{
	// 检查高度数值是否在合理范围内，并且是有限数
	// 给定一个极限范围，如-12000m(海底)到100000m(接近太空边缘)
	return (PX4_ISFINITE(altitude) && ((altitude > -12000.f) && (altitude < 100000.f)));
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude, const float hpos_var,
		     const float vpos_var)
{
	// 设置EKF的全局原点信息，包含经纬度和高度。如果设置失败则返回false
	// hpos_var：水平方向位置融合的方差，vpos_var：垂直方向位置融合的方差

	if (!setLatLonOrigin(latitude, longitude, hpos_var)) {
		return false;
	}

	// 海拔高度是可选参数
	setAltOrigin(altitude, vpos_var);

	return true;
}

bool Ekf::setLatLonOrigin(const double latitude, const double longitude, const float hpos_var)
{
	// 设置EKF的经纬度原点，如果初始就已经在导航中，需要根据原先的局部坐标做对应平移
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	if (!_local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid()) {
		// 如果之前已经在局部坐标系下导航，那么需要将原点转到新的全球坐标经纬度，
		// 同时对已有的局部位置信息进行校正
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		double new_latitude;
		double new_longitude;
		_local_origin_lat_lon.reproject(pos_prev(0), pos_prev(1), new_latitude, new_longitude);
		resetHorizontalPositionTo(new_latitude, new_longitude, hpos_var);

	} else {
		// 如果还没有初始化或者不满足条件，则只是简单地移动原点，
		// 并计算在局部坐标下的位置偏移
		const Vector2f pos_prev = getLocalHorizontalPosition();
		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);
		const Vector2f pos_new = getLocalHorizontalPosition();
		const Vector2f delta_pos = pos_new - pos_prev;
		updateHorizontalPositionResetStatus(delta_pos);
	}

	return true;
}

bool Ekf::setAltOrigin(const float altitude, const float vpos_var)
{
	// 设置EKF的高度原点，如果该高度数值不合理，则返回false
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	ECL_INFO("EKF origin altitude %.1fm -> %.1fm", (double)_local_origin_alt,
		 (double)altitude);

	if (!PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid()) {
		// 如果之前还没设置过局部高度原点，但是系统已经在进行垂直方向导航，
		// 则在当前状态的基础上进行高度重置
		const float local_alt_prev = _gpos.altitude();
		_local_origin_alt = altitude;
		resetAltitudeTo(local_alt_prev + _local_origin_alt);

	} else {
		// 否则，只需要更新本地原点高度并做好相应的重置标记
		const float delta_origin_alt = altitude - _local_origin_alt;
		_local_origin_alt = altitude;
		updateVerticalPositionResetStatus(-delta_origin_alt);

#if defined(CONFIG_EKF2_TERRAIN)
		updateTerrainResetStatus(-delta_origin_alt);
#endif // CONFIG_EKF2_TERRAIN
	}

	return true;
}

bool Ekf::resetGlobalPositionTo(const double latitude, const double longitude, const float altitude,
			  const float hpos_var, const float vpos_var)
{
	// 将全局位置重置到指定经纬度和高度（如果有效），并且更新相应的协方差
	if (!resetLatLonTo(latitude, longitude, hpos_var)) {
		return false;
	}

	// 可选重置海拔高度
	initialiseAltitudeTo(altitude, vpos_var);

	return true;
}

bool Ekf::resetLatLonTo(const double latitude, const double longitude, const float hpos_var)
{
	// 将当前位置重置到指定的经纬度，更新EKF的全局位置（经纬度），并相应修改局部坐标系
	if (!checkLatLonValidity(latitude, longitude)) {
		return false;
	}

	Vector2f pos_prev;

	if (!_local_origin_lat_lon.isInitialized()) {
		// 如果本地原点尚未初始化，则构造一个临时的(0,0)投影，然后获得之前的pos_prev
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		pos_prev = zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());

		_local_origin_lat_lon.initReference(latitude, longitude, _time_delayed_us);

		// 如果已经在进行定位融合，则矫正当前位置使其与新原点相一致
		if (isLocalHorizontalPositionValid()) {
			double est_lat;
			double est_lon;
			_local_origin_lat_lon.reproject(-pos_prev(0), -pos_prev(1), est_lat, est_lon);
			_local_origin_lat_lon.initReference(est_lat, est_lon, _time_delayed_us);
		}

		ECL_INFO("Origin set to lat=%.6f, lon=%.6f",
			 _local_origin_lat_lon.getProjectionReferenceLat(), _local_origin_lat_lon.getProjectionReferenceLon());

	} else {
		// 如果已经有了原点，则先记录当前pos_prev
		pos_prev = _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}

	// 更新全局位置经纬度
	_gpos.setLatLonDeg(latitude, longitude);
	_output_predictor.resetLatLonTo(latitude, longitude);

	// 计算新的局部位置，并更新重置状态
	const Vector2f delta_horz_pos = getLocalHorizontalPosition() - pos_prev;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION

	updateHorizontalPositionResetStatus(delta_horz_pos);

	// 如果传入了水平方向的方差，则进行协方差矩阵更新
	if (PX4_ISFINITE(hpos_var)) {
		P.uncorrelateCovarianceSetVariance<2>(State::pos.idx, math::max(sq(0.01f), hpos_var));
	}

	// 更新水平位置融合时间戳
	_time_last_hor_pos_fuse = _time_delayed_us;

	return true;
}

bool Ekf::initialiseAltitudeTo(const float altitude, const float vpos_var)
{
	// 如果海拔高度有效，则用该值初始化垂直方向位置
	if (!checkAltitudeValidity(altitude)) {
		return false;
	}

	if (!PX4_ISFINITE(_local_origin_alt)) {
		// 如果还没有本地原点的海拔信息
		const float local_alt_prev = _gpos.altitude();

		if (isLocalVerticalPositionValid()) {
			_local_origin_alt = altitude - local_alt_prev;

		} else {
			_local_origin_alt = altitude;
		}

		ECL_INFO("Origin alt=%.3f", (double)_local_origin_alt);
	}

	// 重置到指定altitude并更新协方差
	resetAltitudeTo(altitude, vpos_var);

	return true;
}

void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// 获取EKF全局位置（GPS坐标系）在水平方向和垂直方向的精度，
	// 如果没有全局原点，则返回无穷大
	if (global_origin_valid()) {
		get_ekf_lpos_accuracy(ekf_eph, ekf_epv);

	} else {
		*ekf_eph = INFINITY;
		*ekf_epv = INFINITY;
	}
}

void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// 获取EKF在局部坐标系下的位置精度
	// 水平精度通过状态协方差矩阵的pos部分（前2维）计算
	// 垂直精度通过pos的第三维（对应高度）

	float hpos_err = sqrtf(P.trace<2>(State::pos.idx));

	// 如果水平方向长时间处于纯惯性推算（deadreckoning），那么可能是因为存在陀螺或加速度传感器不可靠问题
	// 此时仅用状态协方差来估计误差可能过于乐观，可将位置创新作为另一种保守的估计
	if (_horizontal_deadreckon_time_exceeded) {
#if defined(CONFIG_EKF2_GNSS)
		if (_control_status.flags.gps) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			hpos_err = math::max(hpos_err, Vector2f(_aid_src_ev_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(State::pos.idx + 2, State::pos.idx + 2));
}

void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	// 获取EKF在局部坐标系下的速度精度
	// 水平速度误差通过状态协方差矩阵的vel部分（前2维）计算
	// 垂直速度误差通过vel的第三维

	float hvel_err = sqrtf(P.trace<2>(State::vel.idx));

	// 如果水平方向长时间处于纯惯性推算（deadreckoning），则同样考虑保守的基于创新的估计
	if (_horizontal_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max(getHagl(), gndclearance) * Vector2f(_aid_src_optical_flow.innovation).norm();
		}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_GNSS)
		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_gnss_pos.innovation).norm());
		}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_pos.innovation).norm());
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, Vector2f(_aid_src_ev_vel.innovation).norm());
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(State::vel.idx + 2, State::vel.idx + 2));
}

void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max_z,
	      float *hagl_max_xy) const
{
	// 该函数用于获取系统在控制层面所可能需要的限制，如最大速度、最小或最大高度等
	// 初始置为NAN表示不受限制

	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max_z = NAN;
	*hagl_max_xy = NAN;

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 如果使用了激光或超声波测距仪，可提供距离地面信息（HAGL）
	// 这里给出测距仪的最小和最大测距，以便飞控做限高
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();
	// 为了保留一定余量，最大距离只用测距仪满量程的90%
	const float rangefinder_hagl_max = 0.9f * _range_sensor.getValidMaxVal();

	// 标记仅依赖测距仪进行垂直高度融合的状态
	const bool relying_on_rangefinder = isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.rng_hgt);

	if (relying_on_rangefinder) {
		*hagl_min = rangefinder_hagl_min;
		*hagl_max_z = rangefinder_hagl_max;
	}

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 如果使用光流，结合测距仪进行高度和速度限制
	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	if (relying_on_optical_flow) {
		// 计算光流能够正常工作的高度范围
		float flow_hagl_min = _flow_min_distance;
		float flow_hagl_max = _flow_max_distance;

		// 如果没有地形估计或者地形信息不可靠，则取测距仪的范围作为补偿
		if ((!_control_status.flags.opt_flow_terrain && _control_status.flags.rng_terrain)
		    || !isTerrainEstimateValid()
		   ) {
			flow_hagl_min = math::max(flow_hagl_min, rangefinder_hagl_min);
			flow_hagl_max = math::min(flow_hagl_max, rangefinder_hagl_max);
		}

		const float flow_constrained_height = math::constrain(getHagl(), flow_hagl_min, flow_hagl_max);

		// 允许地面相对速度使用传感器量程50%的范围，保留部分余量应对角度变化
		float flow_vxy_max = 0.5f * _flow_max_rate * flow_constrained_height;
		flow_hagl_max = math::max(flow_hagl_max * 0.9f, flow_hagl_max - 1.0f);

		*vxy_max = flow_vxy_max;
		*hagl_min = flow_hagl_min;
		*hagl_max_xy = flow_hagl_max;
	}

# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_RANGE_FINDER
}

void Ekf::resetGyroBias()
{
	// 将陀螺仪偏置置零
	// 该函数用于重置陀螺仪的偏置值，确保陀螺仪在后续的测量中不受之前偏置的影响
	_state.gyro_bias.zero(); // 调用状态对象的零函数将陀螺仪偏置清零

	resetGyroBiasCov(); // 重置陀螺仪偏置的协方差
}

void Ekf::resetAccelBias()
{
	// 将加速度计偏置置零
	// 该函数用于重置加速度计的偏置值，确保加速度计在后续的测量中不受之前偏置的影响
	_state.accel_bias.zero(); // 调用状态对象的零函数将加速度计偏置清零

	resetAccelBiasCov(); // 重置加速度计偏置的协方差
}

float Ekf::getHeadingInnovationTestRatio() const
{
	// 获取航向角创新向量的最大检验比值(innovation test ratio)
	// 如果没有可用的航向数据来源，则返回NaN
	float test_ratio = -1.f; // 初始化检验比值为-1，表示尚未计算有效值

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 如果使用了磁力计进行航向测量
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		// 遍历磁力计的检验比值数组
		for (auto &test_ratio_filtered : _aid_src_mag.test_ratio_filtered) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)
	// 如果使用了GNSS航向测量
	if (_control_status.flags.gnss_yaw) {
		// 更新检验比值为当前最大值
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_yaw.test_ratio_filtered));
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 如果使用了外部视觉进行航向测量
	if (_control_status.flags.ev_yaw) {
		// 更新检验比值为当前最大值
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_yaw.test_ratio_filtered));
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 检查检验比值是否有效且大于等于0
	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		// 返回检验比值的平方根
		return sqrtf(test_ratio);
	}

	// 如果没有有效的检验比值，返回NaN
	return NAN;
}

float Ekf::getHorizontalVelocityInnovationTestRatio() const
{
	// 返回与水平速度相关的创新检验比值的最大值
	float test_ratio = -1.f; // 初始化检验比值为-1，表示尚未计算有效值

#if defined(CONFIG_EKF2_GNSS)
	// 如果使用了GNSS进行水平速度测量
	if (_control_status.flags.gps) {
		// 遍历GNSS速度的检验比值数组
		for (int i = 0; i < 2; i++) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[i]));
		}
	}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 如果使用了外部视觉进行水平速度测量
	if (_control_status.flags.ev_vel) {
		// 遍历外部视觉速度的检验比值数组
		for (int i = 0; i < 2; i++) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[i]));
		}
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 如果光流是唯一的水平辅助源
	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		// 遍历光流的检验比值数组
		for (auto &test_ratio_filtered : _aid_src_optical_flow.test_ratio_filtered) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// 检查检验比值是否有效且大于等于0
	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		// 返回检验比值的平方根
		return sqrtf(test_ratio);
	}

	// 如果没有有效的检验比值，返回NaN
	return NAN;
}

float Ekf::getVerticalVelocityInnovationTestRatio() const
{
	// 返回与垂直速度相关的创新检验比值的最大值
	float test_ratio = -1.f; // 初始化检验比值为-1，表示尚未计算有效值

#if defined(CONFIG_EKF2_GNSS)
	// 如果使用了GNSS进行垂直速度测量
	if (_control_status.flags.gps) {
		// 更新检验比值为当前最大值
		test_ratio = math::max(test_ratio, fabsf(_aid_src_gnss_vel.test_ratio_filtered[2]));
	}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 如果使用了外部视觉进行垂直速度测量
	if (_control_status.flags.ev_vel) {
		// 更新检验比值为当前最大值
		test_ratio = math::max(test_ratio, fabsf(_aid_src_ev_vel.test_ratio_filtered[2]));
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 检查检验比值是否有效且大于等于0
	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		// 返回检验比值的平方根
		return sqrtf(test_ratio);
	}

	// 如果没有有效的检验比值，返回NaN
	return NAN;
}

float Ekf::getHorizontalPositionInnovationTestRatio() const
{
	// 返回与水平位置相关的创新检验比值的最大值
	float test_ratio = -1.f; // 初始化检验比值为-1，表示尚未计算有效值

#if defined(CONFIG_EKF2_GNSS)
	// 如果使用了GNSS进行水平位置测量
	if (_control_status.flags.gps) {
		// 遍历GNSS位置的检验比值数组
		for (auto &test_ratio_filtered : _aid_src_gnss_pos.test_ratio_filtered) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 如果使用了外部视觉进行水平位置测量
	if (_control_status.flags.ev_pos) {
		// 遍历外部视觉位置的检验比值数组
		for (auto &test_ratio_filtered : _aid_src_ev_pos.test_ratio_filtered) {
			// 更新检验比值为当前最大值
			test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
		}
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	// 如果使用了辅助全局位置
	if (_control_status.flags.aux_gpos) {
		// 更新检验比值为当前最大值
		test_ratio = math::max(test_ratio, fabsf(_aux_global_position.test_ratio_filtered()));
	}
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

	// 检查检验比值是否有效且大于等于0
	if (PX4_ISFINITE(test_ratio) && (test_ratio >= 0.f)) {
		// 返回检验比值的平方根
		return sqrtf(test_ratio);
	}

	// 如果没有有效的检验比值，返回NaN
	return NAN;
}

float Ekf::getVerticalPositionInnovationTestRatio() const
{
	// 返回垂直方向位置相关的平均创新检验比值
	// 当多个高度来源（气压、高度、光学、视觉等）时，对它们的创新进行综合考虑
	float hgt_sum = 0.f; // 初始化高度检验比值总和为0
	int n_hgt_sources = 0; // 初始化有效高度来源计数为0

#if defined(CONFIG_EKF2_BAROMETER)
	// 如果使用了气压计进行高度测量
	if (_control_status.flags.baro_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_baro_hgt.test_ratio_filtered)); // 累加气压计的检验比值
		n_hgt_sources++; // 有效高度来源计数加1
	}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	// 如果使用了GNSS进行高度测量
	if (_control_status.flags.gps_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_gnss_hgt.test_ratio_filtered)); // 累加GNSS的检验比值
		n_hgt_sources++; // 有效高度来源计数加1
	}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 如果使用了测距仪进行高度测量
	if (_control_status.flags.rng_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered)); // 累加测距仪的检验比值
		n_hgt_sources++; // 有效高度来源计数加1
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 如果使用了外部视觉进行高度测量
	if (_control_status.flags.ev_hgt) {
		hgt_sum += sqrtf(fabsf(_aid_src_ev_hgt.test_ratio_filtered)); // 累加外部视觉的检验比值
		n_hgt_sources++; // 有效高度来源计数加1
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// 如果有有效的高度来源
	if (n_hgt_sources > 0) {
		// 返回平均检验比值，确保不小于FLT_MIN
		return math::max(hgt_sum / static_cast<float>(n_hgt_sources), FLT_MIN);
	}

	// 如果没有有效的高度来源，返回NaN
	return NAN;
}

float Ekf::getAirspeedInnovationTestRatio() const
{
	// 返回空速融合相关的创新检验比值，如果未融合空速，则返回NAN
#if defined(CONFIG_EKF2_AIRSPEED)
	// 如果空速融合被激活
	if (_control_status.flags.fuse_aspd) {
		// 返回空速的检验比值的平方根
		return sqrtf(fabsf(_aid_src_airspeed.test_ratio_filtered));
	}
#endif // CONFIG_EKF2_AIRSPEED

	// 如果未融合空速，返回NaN
	return NAN;
}

float Ekf::getSyntheticSideslipInnovationTestRatio() const
{
	// 返回合成侧滑角融合相关的创新检验比值
#if defined(CONFIG_EKF2_SIDESLIP)
	// 如果侧滑角融合被激活
	if (_control_status.flags.fuse_beta) {
		// 返回侧滑角的检验比值的平方根
		return sqrtf(fabsf(_aid_src_sideslip.test_ratio_filtered));
	}
#endif // CONFIG_EKF2_SIDESLIP

	// 如果未融合侧滑角，返回NaN
	return NAN;
}

float Ekf::getHeightAboveGroundInnovationTestRatio() const
{
	// 返回地面高度（HAGL）相关的创新检验比值
	float hagl_sum = 0.f; // 初始化地面高度检验比值总和为0
	int n_hagl_sources = 0; // 初始化有效地面高度来源计数为0

#if defined(CONFIG_EKF2_TERRAIN)

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 如果使用了光流进行地面高度测量
	if (_control_status.flags.opt_flow_terrain) {
		// 累加光流的检验比值
		hagl_sum += sqrtf(math::max(fabsf(_aid_src_optical_flow.test_ratio_filtered[0]),
			    _aid_src_optical_flow.test_ratio_filtered[1]));
		n_hagl_sources++; // 有效地面高度来源计数加1
	}
# endif // CONFIG_EKF2_OPTICAL_FLOW

# if defined(CONFIG_EKF2_RANGE_FINDER)
	// 如果使用了测距仪进行地面高度测量
	if (_control_status.flags.rng_terrain) {
		hagl_sum += sqrtf(fabsf(_aid_src_rng_hgt.test_ratio_filtered)); // 累加测距仪的检验比值
		n_hagl_sources++; // 有效地面高度来源计数加1
	}
# endif // CONFIG_EKF2_RANGE_FINDER

#endif // CONFIG_EKF2_TERRAIN

	// 如果有有效的地面高度来源
	if (n_hagl_sources > 0) {
		// 返回平均检验比值，确保不小于FLT_MIN
		return math::max(hagl_sum / static_cast<float>(n_hagl_sources), FLT_MIN);
	}

	// 如果没有有效的地面高度来源，返回NaN
	return NAN;
}

uint16_t Ekf::get_ekf_soln_status() const
{
	// 这里返回一个16位的状态标志，用于表示EKF不同维度的解是否可靠
	// 与Mavlink中的ESTIMATOR_STATUS_FLAGS对应
	union ekf_solution_status_u {
		struct {
			uint16_t attitude           : 1; // 姿态是否可靠
			uint16_t velocity_horiz     : 1; // 水平速度是否可靠
			uint16_t velocity_vert      : 1; // 垂直速度是否可靠
			uint16_t pos_horiz_rel      : 1; // 相对水平位置是否可靠
			uint16_t pos_horiz_abs      : 1; // 绝对水平位置是否可靠
			uint16_t pos_vert_abs       : 1; // 绝对垂直位置是否可靠
			uint16_t pos_vert_agl       : 1; // 距地高度是否可靠
			uint16_t const_pos_mode     : 1; // 是否维持静止模式
			uint16_t pred_pos_horiz_rel : 1; // 相对水平位置预测是否可靠
			uint16_t pred_pos_horiz_abs : 1; // 绝对水平位置预测是否可靠
			uint16_t gps_glitch         : 1; // 是否检测到GPS故障
			uint16_t accel_error        : 1; // 是否检测到加速度计故障
		} flags; // 状态标志结构体
		uint16_t value; // 状态标志的值
	} soln_status{}; // 初始化状态标志

	// attitude：是否获得可靠的姿态估计
	soln_status.flags.attitude = attitude_valid();

	// velocity_horiz：是否获得可靠的水平速度估计
	soln_status.flags.velocity_horiz = isLocalHorizontalPositionValid();

	// velocity_vert：是否获得可靠的垂直速度估计
	soln_status.flags.velocity_vert = isLocalVerticalVelocityValid() || isLocalVerticalPositionValid();

	// pos_horiz_rel：是否获得相对水平位置估计
	soln_status.flags.pos_horiz_rel = isLocalHorizontalPositionValid();

	// pos_horiz_abs：是否获得绝对水平位置估计
	soln_status.flags.pos_horiz_abs = isGlobalHorizontalPositionValid();

	// pos_vert_abs：是否有绝对的垂直位置估计
	soln_status.flags.pos_vert_abs = isVerticalAidingActive();

	// pos_vert_agl：是否估计了距地高度
#if defined(CONFIG_EKF2_TERRAIN)
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
#endif // CONFIG_EKF2_TERRAIN

	// const_pos_mode：当外部传感器不可用时是否维持了一个静止的模式
	soln_status.flags.const_pos_mode = _control_status.flags.fake_pos || _control_status.flags.valid_fake_pos
				       || _control_status.flags.vehicle_at_rest;

	// pred_pos_horiz_rel：是否有足够的数据来输出相对水平位置预测
	soln_status.flags.pred_pos_horiz_rel = isHorizontalAidingActive();

	// pred_pos_horiz_abs：是否有足够的数据来输出绝对水平位置预测
	soln_status.flags.pred_pos_horiz_abs = _control_status.flags.gps || _control_status.flags.aux_gpos;

	// gps_glitch：是否检测到GPS故障
#if defined(CONFIG_EKF2_GNSS)
	const bool gps_vel_innov_bad = Vector3f(_aid_src_gnss_vel.test_ratio).max() > 1.f;
	const bool gps_pos_innov_bad = Vector2f(_aid_src_gnss_pos.test_ratio).max() > 1.f;
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad);
#endif // CONFIG_EKF2_GNSS

	// accel_error：是否检测到加速度计故障
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical || _fault_status.flags.bad_acc_clipping;

	return soln_status.value;
}

void Ekf::fuse(const VectorState &K, float innovation)
{
	// 进行单个观测量的融合更新，K为增益，innovation为该观测量与状态预测的残差
	// 这里逐个状态进行更新，包括姿态、速度、位置、陀螺偏置、加速度偏置等

	// 姿态更新：通过旋转向量(小量)delta_quat对四元数进行修正
	Quatf delta_quat(matrix::AxisAnglef(K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx,
				    0) * (-1.f * innovation)));
	_state.quat_nominal = delta_quat * _state.quat_nominal;
	_state.quat_nominal.normalize();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// 速度更新
	_state.vel = matrix::constrain(_state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation, -1.e3f, 1.e3f);

	// 位置更新
	const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);

	// 将位置累积到全局坐标的_gpos上
	_gpos += pos_correction;
	_state.pos.zero();
	// 由于在NED坐标系中z为负值，因此用_gpos.altitude()对pos(2)进行更新
	_state.pos(2) = -_gpos.altitude();

	// 陀螺仪偏置更新
	_state.gyro_bias = matrix::constrain(
		_state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation,
		-getGyroBiasLimit(), getGyroBiasLimit()
	);

	// 加速度计偏置更新
	_state.accel_bias = matrix::constrain(
		_state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation,
		-getAccelBiasLimit(), getAccelBiasLimit()
	);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁场在惯性坐标系（地理坐标系）和机体坐标系中的偏置更新
	if (_control_status.flags.mag) {
		_state.mag_I = matrix::constrain(
			_state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation,
			-1.f,
			1.f);
		_state.mag_B = matrix::constrain(
			_state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
			-getMagBiasLimit(), getMagBiasLimit()
		);
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	// 风速的更新
	if (_control_status.flags.wind) {
		_state.wind_vel = matrix::constrain(
			_state.wind_vel - K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) * innovation,
			-1.e2f, 1.e2f);
	}
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_TERRAIN)
	// 地形高度更新
	_state.terrain = math::constrain(_state.terrain - K(State::terrain.idx) * innovation, -1e4f, 1e4f);
#endif // CONFIG_EKF2_TERRAIN
}

void Ekf::updateDeadReckoningStatus()
{
	// 更新水平和垂直方向是否处于纯惯性推算(deadrk)的状态
	updateHorizontalDeadReckoningstatus();
	updateVerticalDeadReckoningStatus();
}

void Ekf::updateHorizontalDeadReckoningstatus()
{
	// 如果在水平速度或位置上无任何外部传感器的有效融合，则标记为惯性推算
	bool inertial_dead_reckoning = true;
	bool aiding_expected_in_air = false;

	// 如果存在GPS或外部视觉的速度融合，并且没有超时，则不处于纯惯性推算
	if ((_control_status.flags.gps || _control_status.flags.ev_vel)
	    && isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

	// 如果存在GPS或外部视觉的位置信息融合，并且没有超时，则不处于纯惯性推算
	if ((_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.aux_gpos)
	    && isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 如果只依赖光流来提供水平信息，并且光流数据没有超时，则不处于纯惯性推算
	if (_control_status.flags.opt_flow
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
	   ) {
		inertial_dead_reckoning = false;

	} else {
		// 如果着陆状态下，但参数(flow_ctrl==1)，说明离地起飞后会尝试使用光流
		if (!_control_status.flags.in_air && (_params.flow_ctrl == 1)
		    && isRecent(_aid_src_optical_flow.timestamp_sample, _params.no_aid_timeout_max)
		   ) {
			aiding_expected_in_air = true;
		}
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
	// 如果通过空速和侧滑角来进行风速的估计，可以在没有其他水平传感器时依赖它们
	if ((_control_status.flags.fuse_aspd && isRecent(_aid_src_airspeed.time_last_fuse, _params.no_aid_timeout_max))
	    && (_control_status.flags.fuse_beta && isRecent(_aid_src_sideslip.time_last_fuse, _params.no_aid_timeout_max))
	   ) {
		// 此时wind_dead_reckoning表示只有风信息但无其他水平位置或速度来源
		_control_status.flags.wind_dead_reckoning = inertial_dead_reckoning;
		inertial_dead_reckoning = false;

	} else {
		_control_status.flags.wind_dead_reckoning = false;

		if (!_control_status.flags.in_air && _control_status.flags.fixed_wing
	    && (_params.beta_fusion_enabled == 1)
	    && (_params.arsp_thr > 0.f) && isRecent(_aid_src_airspeed.timestamp_sample, _params.no_aid_timeout_max)
	   ) {
			// 若在地面且是固定翼，说明离地后可能使用空速融合
			aiding_expected_in_air = true;
		}
	}
#endif // CONFIG_EKF2_AIRSPEED

	// 零速度更新，如果只依赖地面状态而缺少其他的飞行中传感器，不能视为在空中可用
	if (isRecent(_zero_velocity_update.time_last_fuse(), _params.no_aid_timeout_max)) {
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	// 假位置融合
	if (_control_status.flags.valid_fake_pos && isRecent(_aid_src_fake_pos.time_last_fuse, _params.no_aid_timeout_max)) {
		if (aiding_expected_in_air) {
			inertial_dead_reckoning = false;
		}
	}

	// 如果一直是惯性推算，则检查时间是否超过阈值
	if (inertial_dead_reckoning) {
		if (isTimedOut(_time_last_horizontal_aiding, (uint64_t)_params.valid_timeout_max)) {
			// 水平惯性导航时间过长
			if (!_horizontal_deadreckon_time_exceeded) {
				ECL_WARN("horizontal dead reckon time exceeded");
				_horizontal_deadreckon_time_exceeded = true;
			}
		}

	} else {
		// 如果有传感器数据，则刷新_time_last_horizontal_aiding
		if (_time_delayed_us > _params.no_aid_timeout_max) {
			_time_last_horizontal_aiding = _time_delayed_us - _params.no_aid_timeout_max;
		}

		_horizontal_deadreckon_time_exceeded = false;
	}

	_control_status.flags.inertial_dead_reckoning = inertial_dead_reckoning;
}

void Ekf::updateVerticalDeadReckoningStatus()
{
	// 判断垂直方向是否只有惯性推算
	if (isVerticalPositionAidingActive()) {
		_time_last_v_pos_aiding = _time_last_hgt_fuse;
		_vertical_position_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_pos_aiding, (uint64_t)_params.valid_timeout_max)) {
		_vertical_position_deadreckon_time_exceeded = true;
	}

	if (isVerticalVelocityAidingActive()) {
		_time_last_v_vel_aiding = _time_last_ver_vel_fuse;
		_vertical_velocity_deadreckon_time_exceeded = false;

	} else if (isTimedOut(_time_last_v_vel_aiding, (uint64_t)_params.valid_timeout_max)
		   && _vertical_position_deadreckon_time_exceeded) {

		_vertical_velocity_deadreckon_time_exceeded = true;
	}
}

Vector3f Ekf::getRotVarBody() const
{
	// 获取机体坐标系下的姿态协方差对角线，首先从状态中提取姿态的协方差(在NED坐标系)，
	// 然后转换到机体系下得到机体系的姿态方差
	const matrix::SquareMatrix3f rot_cov_body = getStateCovariance<State::quat_nominal>();
	return matrix::SquareMatrix3f(_R_to_earth.T() * rot_cov_body * _R_to_earth).diag();
}

Vector3f Ekf::getRotVarNed() const
{
	// 获取在NED坐标系下的姿态方差对角项
	const matrix::SquareMatrix3f rot_cov_ned = getStateCovariance<State::quat_nominal>();
	return rot_cov_ned.diag();
}

float Ekf::getYawVar() const
{
	// 返回偏航方向的方差
	return getRotVarNed()(2);
}

float Ekf::getTiltVariance() const
{
	// 返回俯仰和横滚（倾斜）的总方差，即NED姿态方差的前两个分量之和
	const Vector3f rot_var_ned = getRotVarNed();
	return rot_var_ned(0) + rot_var_ned(1);
}

#if defined(CONFIG_EKF2_BAROMETER)
void Ekf::updateGroundEffect()
{
	// 更新地效标志，当飞行器在多旋翼悬停时，如果飞得够低，可能触发地效补偿
	if (_control_status.flags.in_air && !_control_status.flags.fixed_wing) {
#if defined(CONFIG_EKF2_TERRAIN)
		if (isTerrainEstimateValid()) {
			// 如果地形估计有效，就可以基于HAGL判断是否处于地效
			float height = getHagl();
			_control_status.flags.gnd_effect = (height < _params.gnd_effect_max_hgt);

		} else
#endif // CONFIG_EKF2_TERRAIN
			if (_control_status.flags.gnd_effect) {
			// 如果当前标记处于地效，但已经超时，则关闭地效标记
			if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {
				_control_status.flags.gnd_effect = false;
			}
		}

	} else {
		_control_status.flags.gnd_effect = false;
	}
}
#endif // CONFIG_EKF2_BAROMETER


void Ekf::updateIMUBiasInhibit(const imuSample &imu_delayed)
{
	// 当机动状态过于剧烈时，或者加速度传感器数据异常时，需要禁止更新IMU偏置(陀螺仪和加速度计)
	// 以免导致在极端状态下的融合产生错误

	{
	// 计算校正后的陀螺仪角速度，得到其幅值，并进行低通或滑窗滤波
	// 如果陀螺仪数据过大，说明在进行剧烈机动
	const Vector3f gyro_corrected = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - _state.gyro_bias;

	const float alpha = math::constrain((imu_delayed.delta_ang_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
	const float beta = 1.f - alpha;

	_ang_rate_magnitude_filt = fmaxf(gyro_corrected.norm(), beta * _ang_rate_magnitude_filt);
}
{
	// 同样计算校正后的加速度幅值
	const Vector3f accel_corrected = imu_delayed.delta_vel / imu_delayed.delta_vel_dt - _state.accel_bias;

	const float alpha = math::constrain((imu_delayed.delta_vel_dt / _params.acc_bias_learn_tc), 0.f, 1.f);
	const float beta = 1.f - alpha;

		_accel_magnitude_filt = fmaxf(accel_corrected.norm(), beta * _accel_magnitude_filt);
	}

	// 如果机动剧烈超过阈值，认为此时偏置估计不可靠
	const bool is_manoeuvre_level_high = (_ang_rate_magnitude_filt > _params.acc_bias_learn_gyr_lim)
				     || (_accel_magnitude_filt > _params.acc_bias_learn_acc_lim);

	// 对陀螺仪偏置的抑制条件
	const bool do_inhibit_all_gyro_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias));

	for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
		bool is_bias_observable = true; // TODO: 将来可根据其他条件判断某个轴是否可观测
		_gyro_bias_inhibit[index] = do_inhibit_all_gyro_axes || !is_bias_observable;
	}

	// 对加速度计偏置的抑制条件
	const bool do_inhibit_all_accel_axes = !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::AccelBias))
				       || is_manoeuvre_level_high
				       || _fault_status.flags.bad_acc_vertical;

	for (unsigned index = 0; index < State::accel_bias.dof; index++) {
		bool is_bias_observable = true;

		// 如果在地面且只在竖直方向上可观测，也要进行相应的逻辑
		if (_control_status.flags.vehicle_at_rest) {
			is_bias_observable = true;

		} else if (_control_status.flags.fake_hgt) {
			is_bias_observable = false;

		} else if (_control_status.flags.fake_pos) {
			// 当只假设位置时，只有与重力方向平行的加速度可观测
			is_bias_observable = (fabsf(_R_to_earth(2, index)) > 0.966f);
		}

		_accel_bias_inhibit[index] = do_inhibit_all_accel_axes || imu_delayed.delta_vel_clipping[index] || !is_bias_observable;
	}
}

void Ekf::fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index)
{
	// 直接对某个状态分量进行观测更新，比如高度或其他单一维度量
	// innvo：创新（残差）， innov_var：创新方差， R：测量噪声方差，state_index：状态量的索引

	VectorState K;  // 卡尔曼增益向量

	// 计算卡尔曼增益 K = PHS，其中 S = 1/innov_var
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	// 根据抑制标记，清除对应状态的增益
	clearInhibitedStateKalmanGains(K);

#if false
	// 以下是Joseph形式的更新，计算开销非常大，仅用于调试
	auto A = matrix::eye<float, State::size>();
	VectorState H;
	H(state_index) = 1.f;
	A -= K.multiplyByTranspose(H);
	P = A * P;
	P = P.multiplyByTranspose(A);

	const VectorState KR = K * R;
	P += KR.multiplyByTranspose(K);
#else
	// 更高效的Joseph稳定形式
	VectorState PH = P.row(state_index);
	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j);
		}
	}

	PH = P.col(state_index);
	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
			P(j, i) = P(i, j);
		}
	}
#endif
	// 限制状态方差，确保方差在合理范围内
	constrainStateVariances();

	// 应用状态修正，将卡尔曼增益 K 和创新值 innov 结合，更新状态
	fuse(K, innov);
}

bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
	// 这一函数用于一般性的一维观测卡尔曼更新
	// K为增益向量，H为观测模型向量，R为测量噪声，innovation为观测残差

	// 清除被抑制的状态卡尔曼增益
	clearInhibitedStateKalmanGains(K);

#if false
	// Joseph稳定形式，计算代价很大，仅用于调试
	auto A = matrix::eye<float, State::size>(); // 创建单位矩阵
	A -= K.multiplyByTranspose(H); // 更新矩阵 A
	P = A * P; // 更新协方差矩阵 P
	P = P.multiplyByTranspose(A); // 进行转置乘法更新
	const VectorState KR = K * R; // 计算增益与噪声的乘积
	P += KR.multiplyByTranspose(K); // 更新协方差矩阵 P
#else
	// 更高效的Joseph稳定形式
	VectorState PH = P * H; // 计算 P 和 H 的乘积
	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j < State::size; j++) {
			P(i, j) -= K(i) * PH(j); // 更新协方差矩阵 P
		}
	}

	PH = P * H; // 再次计算 P 和 H 的乘积
	for (unsigned i = 0; i < State::size; i++) {
		for (unsigned j = 0; j <= i; j++) {
			P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j); // 更新协方差矩阵 P
			P(j, i) = P(i, j); // 确保协方差矩阵是对称的
		}
	}
#endif

	// 限制状态方差，确保方差在合理范围内
	constrainStateVariances();

	// 应用状态修正，将卡尔曼增益 K 和创新值结合，更新状态
	fuse(K, innovation);
	return true; // 返回更新成功
}

void Ekf::resetAidSourceStatusZeroInnovation(estimator_aid_source1d_s &status) const
{
	// 重置观测源的创新标记，设定为零创新（例如强制对齐）

	status.time_last_fuse = _time_delayed_us; // 记录最后一次融合的时间

	status.innovation = 0.f; // 设置创新为零
	status.innovation_filtered = 0.f; // 设置过滤后的创新为零
	status.innovation_variance = status.observation_variance; // 设置创新方差为观测方差

	status.test_ratio = 0.f; // 设置检验比为零
	status.test_ratio_filtered = 0.f; // 设置过滤后的检验比为零

	status.innovation_rejected = false; // 设置创新未被拒绝
	status.fused = true; // 设置为已融合状态
}

void Ekf::updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				const float &observation, const float &observation_variance,
				const float &innovation, const float &innovation_variance,
				float innovation_gate) const
{
	// 更新给定观测源的状态，包括创新、方差和检验比等
	// 如果检验比超过阈值，认为创新被拒绝

	bool innovation_rejected = false; // 初始化创新拒绝标记为假

	const float test_ratio = sq(innovation) / (sq(innovation_gate) * innovation_variance); // 计算检验比

	if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {
		// 如果时间戳有效且新时间戳大于上一个时间戳
		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f); // 计算时间间隔

		static constexpr float tau = 0.5f; // 设置时间常数
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f); // 计算平滑因子
		if (PX4_ISFINITE(status.test_ratio_filtered)) {
			// 如果过滤后的检验比是有限的
			status.test_ratio_filtered += alpha * (matrix::sign(innovation) * test_ratio - status.test_ratio_filtered); // 更新过滤后的检验比

		} else {
			status.test_ratio_filtered = test_ratio; // 否则直接设置为当前检验比
		}

		// 更新过滤后的创新
		if (PX4_ISFINITE(status.innovation_filtered)) {
			status.innovation_filtered += alpha * (innovation - status.innovation_filtered); // 更新过滤后的创新

		} else {
			status.innovation_filtered = innovation; // 否则直接设置为当前创新
		}

		// 防止数值过大
		static constexpr float kNormalizedInnovationLimit = 2.f; // 设置创新限制
		static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit); // 设置检验比限制

		if (test_ratio > kTestRatioLimit) {
			// 如果检验比超过限制
			status.test_ratio_filtered = math::constrain(status.test_ratio_filtered, -kTestRatioLimit, kTestRatioLimit); // 限制过滤后的检验比

			const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance); // 计算创新限制
			status.innovation_filtered = math::constrain(status.innovation_filtered, -innov_limit, innov_limit); // 限制过滤后的创新
		}

	} else {
		// 如果时间戳无效或回跳，则直接重置过滤后的值
		status.test_ratio_filtered = test_ratio; // 设置过滤后的检验比为当前检验比
		status.innovation_filtered = innovation; // 设置过滤后的创新为当前创新
	}

	status.test_ratio = test_ratio; // 更新检验比

	status.observation = observation; // 更新观测值
	status.observation_variance = observation_variance; // 更新观测方差

	status.innovation = innovation; // 更新创新
	status.innovation_variance = innovation_variance; // 更新创新方差

	if ((test_ratio > 1.f) // 如果检验比大于1
	    || !PX4_ISFINITE(test_ratio) // 或者检验比不是有限值
	    || !PX4_ISFINITE(status.innovation) // 或者创新不是有限值
	    || !PX4_ISFINITE(status.innovation_variance) // 或者创新方差不是有限值
	   ) {
		innovation_rejected = true; // 标记创新被拒绝
	}

	status.timestamp_sample = timestamp_sample; // 更新时间戳

	status.innovation_rejected = innovation_rejected; // 更新创新拒绝状态

	status.fused = false; // 设置为未融合状态
}

void Ekf::clearInhibitedStateKalmanGains(VectorState &K) const
{
	// 根据标记，清除对于某些状态不可信时的卡尔曼增益，
	// 比如当不允许学习陀螺仪偏置或加速度偏置时
	for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
		if (_gyro_bias_inhibit[i]) {
			K(State::gyro_bias.idx + i) = 0.f; // 如果陀螺仪偏置被抑制，设置对应增益为零
		}
	}

	for (unsigned i = 0; i < State::accel_bias.dof; i++) {
		if (_accel_bias_inhibit[i]) {
			K(State::accel_bias.idx + i) = 0.f; // 如果加速度偏置被抑制，设置对应增益为零
		}
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_I.dof; i++) {
			K(State::mag_I.idx + i) = 0.f; // 如果磁力计未启用，设置对应增益为零
		}
	}

	if (!_control_status.flags.mag) {
		for (unsigned i = 0; i < State::mag_B.dof; i++) {
			K(State::mag_B.idx + i) = 0.f; // 如果磁力计未启用，设置对应增益为零
		}
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	if (!_control_status.flags.wind) {
		for (unsigned i = 0; i < State::wind_vel.dof; i++) {
			K(State::wind_vel.idx + i) = 0.f; // 如果风速未启用，设置对应增益为零
		}
	}
#endif // CONFIG_EKF2_WIND
}

float Ekf::getHeadingInnov() const
{
	// 获取当前有效航向观测的创新
	// 如果多种航向观测同时存在，则返回其中的某一个或其组合

#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation).max(); // 返回磁力计的最大创新
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation; // 返回GNSS航向的创新
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation; // 返回外部视觉的航向创新
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f; // 如果没有有效观测，返回0
}

float Ekf::getHeadingInnovVar() const
{
	// 获取当前有效航向观测的创新方差
#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.innovation_variance).max(); // 返回磁力计的最大创新方差
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.innovation_variance; // 返回GNSS航向的创新方差
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.innovation_variance; // 返回外部视觉的航向创新方差
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f; // 如果没有有效观测，返回0
}

float Ekf::getHeadingInnovRatio() const
{
	// 返回当前航向观测的创新检验比值
#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		return Vector3f(_aid_src_mag.test_ratio).max(); // 返回磁力计的最大检验比
	}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)
	if (_control_status.flags.gnss_yaw) {
		return _aid_src_gnss_yaw.test_ratio; // 返回GNSS航向的检验比
	}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_yaw) {
		return _aid_src_ev_yaw.test_ratio; // 返回外部视觉的航向检验比
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	return 0.f; // 如果没有有效观测，返回0
}
