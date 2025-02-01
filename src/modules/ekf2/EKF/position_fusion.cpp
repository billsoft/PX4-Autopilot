/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，允许进行修改，前提是满足以下条件：
 *
 * 1. 再分发源代码必须保留上述版权声明、条件列表和免责声明。
 * 2. 再分发二进制形式必须在分发的文档或其他材料中重现上述版权声明、条件列表和免责声明。
 * 3. PX4的名称或其贡献者的名称不得用于推广或宣传基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于替代商品或服务的采购、使用、数据或利润的损失，或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因）引起的，均不对使用本软件的任何方式负责，即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

#include "ekf.h"  // 引入EKF头文件

// 更新垂直位置辅助状态的函数
void Ekf::updateVerticalPositionAidStatus(estimator_aid_source1d_s &aid_src, const uint64_t &time_us,
		const float observation, const float observation_variance, const float innovation_gate) const
{
	// 计算创新值，创新值为当前高度与观测值的差
	float innovation = -_gpos.altitude() - observation;
	// 计算创新方差
	float innovation_variance = getStateVariance<State::pos>()(2) + observation_variance;

	// 更新辅助源状态
	updateAidSourceStatus(aid_src, time_us,
			      observation, observation_variance,
			      innovation, innovation_variance,
			      innovation_gate);

	// 特殊情况：如果垂直加速度数据不良，则不拒绝测量，
	// 但限制创新值以防止可能导致滤波器不稳定的尖峰
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innovation_gate * sqrtf(aid_src.innovation_variance);
		// 限制创新值在指定范围内
		aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
		// 将创新拒绝标志设置为false
		aid_src.innovation_rejected = false;
	}
}

// 融合水平位置的函数
bool Ekf::fuseHorizontalPosition(estimator_aid_source2d_s &aid_src)
{
	// 处理x和y方向
	if (!aid_src.innovation_rejected) {
		// 如果创新未被拒绝，进行状态测量融合
		for (unsigned i = 0; i < 2; i++) {
			fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::pos.idx + i);
		}

		// 标记为已融合
		aid_src.fused = true;
		// 记录最后融合的时间
		aid_src.time_last_fuse = _time_delayed_us;

		// 更新最后水平位置融合的时间
		_time_last_hor_pos_fuse = _time_delayed_us;

	} else {
		// 如果创新被拒绝，标记为未融合
		aid_src.fused = false;
	}

	// 返回融合状态
	return aid_src.fused;
}

// 融合垂直位置的函数
bool Ekf::fuseVerticalPosition(estimator_aid_source1d_s &aid_src)
{
	// 处理z方向
	if (!aid_src.innovation_rejected) {
		// 如果创新未被拒绝，进行状态测量融合
		fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
					   State::pos.idx + 2);

		// 标记为已融合
		aid_src.fused = true;
		// 记录最后融合的时间
		aid_src.time_last_fuse = _time_delayed_us;

		// 更新最后高度融合的时间
		_time_last_hgt_fuse = _time_delayed_us;

	} else {
		// 如果创新被拒绝，标记为未融合
		aid_src.fused = false;
	}

	// 返回融合状态
	return aid_src.fused;
}

// 将水平位置重置为新的值
void Ekf::resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				    const Vector2f &new_horz_pos_var)
{
	// 计算新的水平位置与当前位置的差值
	const Vector2f delta_horz_pos = computeDeltaHorizontalPosition(new_latitude, new_longitude);

	// 更新水平位置重置状态
	updateHorizontalPositionResetStatus(delta_horz_pos);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 更新外部视觉的偏差
	_ev_pos_b_est.setBias(_ev_pos_b_est.getBias() - delta_horz_pos);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	//_gps_pos_b_est.setBias(_gps_pos_b_est.getBias() + _state_reset_status.posNE_change);

	// 设置新的经纬度
	_gpos.setLatLonDeg(new_latitude, new_longitude);
	// 重置输出预测器的经纬度
	_output_predictor.resetLatLonTo(new_latitude, new_longitude);

	// 如果新的水平位置方差有效，更新协方差矩阵的方差
	if (PX4_ISFINITE(new_horz_pos_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx, math::max(sq(0.01f), new_horz_pos_var(0)));
	}

	if (PX4_ISFINITE(new_horz_pos_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 1, math::max(sq(0.01f), new_horz_pos_var(1)));
	}

	// 重置超时计时器
	_time_last_hor_pos_fuse = _time_delayed_us;
}

// 计算新的水平位置与当前水平位置的差值
Vector2f Ekf::computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const
{
	Vector2f pos;      // 当前水平位置
	Vector2f pos_new;  // 新的水平位置

	if (_local_origin_lat_lon.isInitialized()) {
		// 如果本地原点已初始化，进行投影计算
		_local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		_local_origin_lat_lon.project(new_latitude, new_longitude, pos_new(0), pos_new(1));

	} else {
		// 如果本地原点未初始化，使用零参考进行投影计算
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos(0), pos(1));
		zero_ref.project(new_latitude, new_longitude, pos_new(0), pos_new(1));
	}

	// 返回新的水平位置与当前水平位置的差值
	return pos_new - pos;
}

// 获取本地水平位置的函数
Vector2f Ekf::getLocalHorizontalPosition() const
{
	if (_local_origin_lat_lon.isInitialized()) {
		// 如果本地原点已初始化，返回投影后的本地水平位置
		return _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg());

	} else {
		// 如果本地原点未初始化，使用零参考返回本地水平位置
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		return zero_ref.project(_gpos.latitude_deg(), _gpos.longitude_deg());
	}
}

// 更新水平位置重置状态的函数
void Ekf::updateHorizontalPositionResetStatus(const Vector2f &delta)
{
	if (_state_reset_status.reset_count.posNE == _state_reset_count_prev.posNE) {
		// 如果当前重置计数与上次相同，记录位置变化
		_state_reset_status.posNE_change = delta;

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.posNE_change += delta;
	}

	// 更新重置计数
	_state_reset_status.reset_count.posNE++;
}

// 将水平位置重置为新的值
void Ekf::resetHorizontalPositionTo(const Vector2f &new_pos,
				    const Vector2f &new_horz_pos_var)
{
	double new_latitude;  // 新的纬度
	double new_longitude; // 新的经度

	if (_local_origin_lat_lon.isInitialized()) {
		// 如果本地原点已初始化，进行反投影计算
		_local_origin_lat_lon.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);

	} else {
		// 如果本地原点未初始化，使用零参考进行反投影计算
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.reproject(new_pos(0), new_pos(1), new_latitude, new_longitude);
	}

	// 重置水平位置
	resetHorizontalPositionTo(new_latitude, new_longitude, new_horz_pos_var);
}

// 将高度重置为新的值
void Ekf::resetAltitudeTo(const float new_altitude, float new_vert_pos_var)
{
	const float old_altitude = _gpos.altitude(); // 获取当前高度
	_gpos.setAltitude(new_altitude); // 设置新的高度

	if (PX4_ISFINITE(new_vert_pos_var)) {
		// 如果新的垂直位置方差有效，更新协方差矩阵的方差
		P.uncorrelateCovarianceSetVariance<1>(State::pos.idx + 2, math::max(sq(0.01f), new_vert_pos_var));
	}

	const float delta_z = -(new_altitude - old_altitude); // 计算高度变化

	// 将高度变化应用于最新的高度估计
	_output_predictor.resetAltitudeTo(new_altitude, delta_z);

	// 更新垂直位置重置状态
	updateVerticalPositionResetStatus(delta_z);

#if defined(CONFIG_EKF2_BAROMETER)
	// 更新气压计的偏差
	_baro_b_est.setBias(_baro_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 更新外部视觉的偏差
	_ev_hgt_b_est.setBias(_ev_hgt_b_est.getBias() - delta_z);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	// 更新GNSS的偏差
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + delta_z);
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_TERRAIN)
	// 更新地形重置状态
	updateTerrainResetStatus(delta_z);
	_state.terrain += delta_z; // 更新地形高度
#endif // CONFIG_EKF2_TERRAIN

	// 重置超时计时器
	_time_last_hgt_fuse = _time_delayed_us;
}

// 更新垂直位置重置状态的函数
void Ekf::updateVerticalPositionResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.posD == _state_reset_count_prev.posD) {
		// 如果当前重置计数与上次相同，记录高度变化
		_state_reset_status.posD_change = delta_z;

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.posD_change += delta_z;
	}

	// 更新重置计数
	_state_reset_status.reset_count.posD++;
}

// 更新地形重置状态的函数
void Ekf::updateTerrainResetStatus(const float delta_z)
{
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		// 如果当前重置计数与上次相同，记录地形变化
		_state_reset_status.hagl_change = delta_z;

	} else {
		// 如果在此更新中已经有重置，累积总变化量
		_state_reset_status.hagl_change += delta_z;
	}

	// 更新重置计数
	_state_reset_status.reset_count.hagl++;
}

// 将水平位置重置为最后已知位置
void Ekf::resetHorizontalPositionToLastKnown()
{
	ECL_INFO("重置位置为最后已知位置 (%.3f, %.3f)", (double)_last_known_gpos.latitude_deg(),
		 (double)_last_known_gpos.longitude_deg());
	_information_events.flags.reset_pos_to_last_known = true;

	// 用于回退到非辅助模式的操作
	resetHorizontalPositionTo(_last_known_gpos.latitude_deg(), _last_known_gpos.longitude_deg(),
				  sq(_params.pos_noaid_noise));
}
