/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file estimator_interface.cpp
 * @brief 这是姿态估计器(EKF)基类定义的源文件。
 *        主要提供对不同传感器数据的接收和缓冲处理，并完成一些常规的过滤逻辑。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

// 析构函数: 在对象被销毁时，释放之前在运行时动态分配的缓冲区
EstimatorInterface::~EstimatorInterface()
{
#if defined(CONFIG_EKF2_GNSS)
	delete _gps_buffer;
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
	delete _mag_buffer;
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_BAROMETER)
	delete _baro_buffer;
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	delete _range_buffer;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AIRSPEED)
	delete _airspeed_buffer;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	delete _flow_buffer;
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	delete _ext_vision_buffer;
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_DRAG_FUSION)
	delete _drag_buffer;
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AUXVEL)
	delete _auxvel_buffer;
#endif // CONFIG_EKF2_AUXVEL
}

// setIMUData: 将IMU数据(陀螺仪增量和加速度增量等)写入到IMU缓冲区中，并执行相关的输出状态预测
// 参数 imu_sample: 包含IMU测量数据的结构体, 包括时间戳、角增量、线加速度增量等
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// 如果还没有初始化滤波器，就在收到IMU数据时进行初始化
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	// 记录最新数据的时间戳
	_time_latest_us = imu_sample.time_us;

	// _output_predictor用来进行输出预测，这里每次都会进行更新，以估计当前输出状态
	_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang, imu_sample.delta_ang_dt,
							imu_sample.delta_vel, imu_sample.delta_vel_dt);

	// 将IMU数据进行累加与降采样处理，当采样器检测到可以进行一次下采样后，就会将下采样结果push到IMU缓冲区
	if (_imu_down_sampler.update(imu_sample)) {

		_imu_updated = true;

		// 获取降采样后的IMU数据，并重置累积
		imuSample imu_downsampled = _imu_down_sampler.getDownSampledImuAndTriggerReset();

		// 对IMU的采样周期做一个限制，防止数值问题
		const float filter_update_period_s = _params.filter_update_interval_us * 1e-6f;
		const float imu_min_dt = 0.5f * filter_update_period_s;
		const float imu_max_dt = 2.0f * filter_update_period_s;

		imu_downsampled.delta_ang_dt = math::constrain(imu_downsampled.delta_ang_dt, imu_min_dt, imu_max_dt);
		imu_downsampled.delta_vel_dt = math::constrain(imu_downsampled.delta_vel_dt, imu_min_dt, imu_max_dt);

		// 将降采样后的IMU数据放入IMU环形缓冲区
		_imu_buffer.push(imu_downsampled);

		// 取出缓冲区中最旧的一条数据(时延最大的)时间戳
		_time_delayed_us = _imu_buffer.get_oldest().time_us;

		// 计算观测到最旧数据所需的最小间隔，保证我们不会在数据被覆盖前就需要使用它
		_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 如果开启了空气阻力融合，则将IMU数据传入setDragData进行处理，用于估计拖曳力
	setDragData(imu_sample);
#endif // CONFIG_EKF2_DRAG_FUSION
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
// setMagData: 将磁力计数据存入磁力计缓冲区，并根据设定的延迟进行处理
// 参数 mag_sample: 包含磁力计测量数据及时间戳
void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	// 若尚未初始化滤波器，则直接返回
	if (!_initialised) {
		return;
	}

	// 如果还没有分配磁力计缓冲区，则进行分配
	if (_mag_buffer == nullptr) {
		_mag_buffer = new RingBuffer<magSample>(_obs_buffer_length);

		if (_mag_buffer == nullptr || !_mag_buffer->valid()) {
			delete _mag_buffer;
			_mag_buffer = nullptr;
			printBufferAllocationFailed("mag");
			return;
		}
	}

	// 根据配置的mag_delay_ms以及滤波器平均周期_dt_ekf_avg，计算当前需要的时间戳
	// 这里会减去指定的延迟和一半的EKF平均预测周期
	const int64_t time_us = mag_sample.time_us
				- static_cast<int64_t>(_params.mag_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds / 2

	// 若新数据的时间戳与缓冲区最新的数据时间戳差值大于_min_obs_interval_us，说明可以存入缓冲区
	if (time_us >= static_cast<int64_t>(_mag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		magSample mag_sample_new{mag_sample};
		mag_sample_new.time_us = time_us;

		_mag_buffer->push(mag_sample_new);
		_time_last_mag_buffer_push = _time_latest_us;

	} else {
		// 如果新数据时间戳跟最新的一帧数据过近，则发出警告并丢弃，以免造成缓冲区丢失数据
		ECL_WARN("mag data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _mag_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS)
// setGpsData: 将GPS数据写入GPS缓冲区，并根据配置的延迟进行处理
// 参数 gnss_sample: 包含GPS测量数据及时间戳的结构体
void EstimatorInterface::setGpsData(const gnssSample &gnss_sample)
{
	// 若滤波器未初始化，则直接返回
	if (!_initialised) {
		return;
	}

	// 如果GPS缓冲区尚未分配，则进行分配
	if (_gps_buffer == nullptr) {
		_gps_buffer = new RingBuffer<gnssSample>(_obs_buffer_length);

		if (_gps_buffer == nullptr || !_gps_buffer->valid()) {
			delete _gps_buffer;
			_gps_buffer = nullptr;
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	// 计算融合使用的数据时间戳，减去GPS延迟和一半的EKF平均预测周期
	const int64_t time_us = gnss_sample.time_us
				- static_cast<int64_t>(_params.gps_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds / 2

	// 如果该数据的时间戳与最新缓冲区数据比，间隔大于_min_obs_interval_us，则写入缓冲区
	if (time_us >= static_cast<int64_t>(_gps_buffer->get_newest().time_us + _min_obs_interval_us)) {

		gnssSample gnss_sample_new(gnss_sample);

		gnss_sample_new.time_us = time_us;

		_gps_buffer->push(gnss_sample_new);
		_time_last_gps_buffer_push = _time_latest_us;

#if defined(CONFIG_EKF2_GNSS_YAW)
		// 如果GPS数据中包含Yaw信息且有效，则更新该时间戳
		if (PX4_ISFINITE(gnss_sample.yaw)) {
			_time_last_gnss_yaw_buffer_push = _time_latest_us;
		}

#endif // CONFIG_EKF2_GNSS_YAW

	} else {
		ECL_WARN("GPS data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _gps_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
// setBaroData: 将气压计数据写入缓冲区，并根据设定延迟进行融合
// 参数 baro_sample: 气压计测量值结构体
void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised) {
		return;
	}

	// 如果还没有分配气压缓冲区，则分配
	if (_baro_buffer == nullptr) {
		_baro_buffer = new RingBuffer<baroSample>(_obs_buffer_length);

		if (_baro_buffer == nullptr || !_baro_buffer->valid()) {
			delete _baro_buffer;
			_baro_buffer = nullptr;
			printBufferAllocationFailed("baro");
			return;
		}
	}

	// 计算融合使用的数据时间戳，减去配置的baro延迟和一半的EKF平均预测周期
	const int64_t time_us = baro_sample.time_us
				- static_cast<int64_t>(_params.baro_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 若满足最小观测间隔要求，则写入缓冲区
	if (time_us >= static_cast<int64_t>(_baro_buffer->get_newest().time_us + _min_obs_interval_us)) {

		baroSample baro_sample_new{baro_sample};
		baro_sample_new.time_us = time_us;

		_baro_buffer->push(baro_sample_new);
		_time_last_baro_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("baro data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _baro_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
// setAirspeedData: 将空速数据写入缓冲区
// 参数 airspeed_sample: 空速传感器测量值
void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised) {
		return;
	}

	// 如果空速缓冲区还没分配，则分配
	if (_airspeed_buffer == nullptr) {
		_airspeed_buffer = new RingBuffer<airspeedSample>(_obs_buffer_length);

		if (_airspeed_buffer == nullptr || !_airspeed_buffer->valid()) {
			delete _airspeed_buffer;
			_airspeed_buffer = nullptr;
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	// 计算融合使用的数据时间戳，减去空速传感器延迟和一半的EKF平均预测周期
	const int64_t time_us = airspeed_sample.time_us
				- static_cast<int64_t>(_params.airspeed_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 若满足最小观测间隔，则push进缓冲区
	if (time_us >= static_cast<int64_t>(_airspeed_buffer->get_newest().time_us + _min_obs_interval_us)) {

		airspeedSample airspeed_sample_new{airspeed_sample};
		airspeed_sample_new.time_us = time_us;

		_airspeed_buffer->push(airspeed_sample_new);

	} else {
		ECL_WARN("airspeed data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _airspeed_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
// setRangeData: 将测距仪数据写入缓冲区
// 参数 range_sample: 测距仪测量的数据结构
void EstimatorInterface::setRangeData(const sensor::rangeSample &range_sample)
{
	if (!_initialised) {
		return;
	}

	// 若尚未分配测距仪缓冲区，则分配
	if (_range_buffer == nullptr) {
		_range_buffer = new RingBuffer<sensor::rangeSample>(_obs_buffer_length);

		if (_range_buffer == nullptr || !_range_buffer->valid()) {
			delete _range_buffer;
			_range_buffer = nullptr;
			printBufferAllocationFailed("range");
			return;
		}
	}

	// 计算融合使用的数据时间戳，减去测距仪延迟和一半的EKF平均预测周期
	const int64_t time_us = range_sample.time_us
				- static_cast<int64_t>(_params.range_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 检查数据速率是否过快，若满足要求则写入缓冲区
	if (time_us >= static_cast<int64_t>(_range_buffer->get_newest().time_us + _min_obs_interval_us)) {

		sensor::rangeSample range_sample_new{range_sample};
		range_sample_new.time_us = time_us;

		_range_buffer->push(range_sample_new);
		_time_last_range_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("range data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _range_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
// setOpticalFlowData: 将光流数据写入缓冲区
// 参数 flow: 光流传感器数据
void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised) {
		return;
	}

	// 如果还没有分配光流的环形缓冲区，则分配
	if (_flow_buffer == nullptr) {
		_flow_buffer = new RingBuffer<flowSample>(_imu_buffer_length);

		if (_flow_buffer == nullptr || !_flow_buffer->valid()) {
			delete _flow_buffer;
			_flow_buffer = nullptr;
			printBufferAllocationFailed("flow");
			return;
		}
	}

	// 同样，减去光流延迟和一半的EKF平均预测周期，得到用于融合的时间戳
	const int64_t time_us = flow.time_us
				- static_cast<int64_t>(_params.flow_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 若速率正常，则写入缓冲区
	if (time_us >= static_cast<int64_t>(_flow_buffer->get_newest().time_us + _min_obs_interval_us)) {

		flowSample optflow_sample_new{flow};
		optflow_sample_new.time_us = time_us;

		_flow_buffer->push(optflow_sample_new);

	} else {
		ECL_WARN("optical flow data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _flow_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
// setExtVisionData: 将外部视觉(vision)数据写入缓冲区
// 参数 evdata: 外部视觉系统测得的位置/姿态等数据
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised) {
		return;
	}

	// 若还没分配外部视觉的缓冲区，则分配
	if (_ext_vision_buffer == nullptr) {
		_ext_vision_buffer = new RingBuffer<extVisionSample>(_obs_buffer_length);

		if (_ext_vision_buffer == nullptr || !_ext_vision_buffer->valid()) {
			delete _ext_vision_buffer;
			_ext_vision_buffer = nullptr;
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// 减去视觉延迟与半个预测周期后，得到相应的融合时间戳
	const int64_t time_us = evdata.time_us
				- static_cast<int64_t>(_params.ev_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 若满足最小间隔要求，则写入缓冲区
	if (time_us >= static_cast<int64_t>(_ext_vision_buffer->get_newest().time_us + _min_obs_interval_us)) {

		extVisionSample ev_sample_new{evdata};
		ev_sample_new.time_us = time_us;

		_ext_vision_buffer->push(ev_sample_new);
		_time_last_ext_vision_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("EV data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _ext_vision_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
// setAuxVelData: 将辅助外部速度数据写入缓冲区
// 参数 auxvel_sample: 包含辅助速度信息(如某些特定传感器或估计输出)
void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised) {
		return;
	}

	// 若辅助速度缓冲区尚未分配，则分配
	if (_auxvel_buffer == nullptr) {
		_auxvel_buffer = new RingBuffer<auxVelSample>(_obs_buffer_length);

		if (_auxvel_buffer == nullptr || !_auxvel_buffer->valid()) {
			delete _auxvel_buffer;
			_auxvel_buffer = nullptr;
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	// 减去相应的auxvel_delay_ms与半个预测周期，得到融合用的时间戳
	const int64_t time_us = auxvel_sample.time_us
				- static_cast<int64_t>(_params.auxvel_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	// 若满足最小速率要求，则写入缓冲区
	if (time_us >= static_cast<int64_t>(_auxvel_buffer->get_newest().time_us + _min_obs_interval_us)) {

		auxVelSample auxvel_sample_new{auxvel_sample};
		auxvel_sample_new.time_us = time_us;

		_auxvel_buffer->push(auxvel_sample_new);

	} else {
		ECL_WARN("aux velocity data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _auxvel_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AUXVEL

// setSystemFlagData: 设置系统标志相关的数据(例如是否着陆、是否失控等), 并存入缓冲区
// 参数 system_flags: 包含系统标志更新的数据结构
void EstimatorInterface::setSystemFlagData(const systemFlagUpdate &system_flags)
{
	if (!_initialised) {
		return;
	}

	// 如果尚未分配system flag缓冲区，则进行分配
	if (_system_flag_buffer == nullptr) {
		_system_flag_buffer = new RingBuffer<systemFlagUpdate>(_obs_buffer_length);

		if (_system_flag_buffer == nullptr || !_system_flag_buffer->valid()) {
			delete _system_flag_buffer;
			_system_flag_buffer = nullptr;
			printBufferAllocationFailed("system flag");
			return;
		}
	}

	// 减去半个EKF预测周期后得到用于融合的时间戳
	const int64_t time_us = system_flags.time_us
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds / 2

	// 检查数据写入间隔
	if (time_us >= static_cast<int64_t>(_system_flag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		systemFlagUpdate system_flags_new{system_flags};
		system_flags_new.time_us = time_us;

		_system_flag_buffer->push(system_flags_new);

	} else {
		ECL_DEBUG("system flag update too fast %" PRIi64 " < %" PRIu64 " + %d", time_us,
			  _system_flag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

#if defined(CONFIG_EKF2_DRAG_FUSION)
// setDragData: 将IMU数据用于空气阻力(Drag)的估计进行下采样累加，并写入_drag_buffer
// 当_params.drag_ctrl > 0时才进行此操作
// 参数 imu: 当前IMU采样数据
void EstimatorInterface::setDragData(const imuSample &imu)
{
	// 如果drag融合开启，则进行相关累加
	if (_params.drag_ctrl > 0) {

		// 如果还没有分配_drag_buffer，则分配
		if (_drag_buffer == nullptr) {
			_drag_buffer = new RingBuffer<dragSample>(_obs_buffer_length);

			if (_drag_buffer == nullptr || !_drag_buffer->valid()) {
				delete _drag_buffer;
				_drag_buffer = nullptr;
				printBufferAllocationFailed("drag");
				return;
			}
		}

		// 如果当前加速度计通道有饱和(剪切)现象，则不使用这些样本
		if (imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2]) {
			// 重置累加器
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;

			return;
		}

		// 累加IMU的delta_vel用于估计空气阻力
		_drag_sample_count++;
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// 计算drag特征所需的下采样比率
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// 当累计样本数超过下采样比后，将平均值写入_drag_buffer
		if (_drag_sample_count >= min_sample_ratio) {
			// 将累加的delta_vel除以总的采样时间得到平均加速度
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			_drag_buffer->push(_drag_down_sampled);

			// 重置累加器
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}
#endif // CONFIG_EKF2_DRAG_FUSION

// initialise_interface: 初始化接口的一些缓冲区大小等配置
// 参数 timestamp: 当前的时间戳，用于初始化时间变量
bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// 计算滤波器更新周期(毫秒)
	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// 根据最大延迟和一定的裕度，计算IMU缓冲区大小
	_imu_buffer_length = math::max(2, (int)ceilf(_params.delay_max_ms / filter_update_period_ms));

	// 计算观测缓冲区长度，使其可以处理最短的观测到达时间间隔和最坏情况融合时延
	// 同时考虑可能出现的时间抖动(1.5倍)
	const float ekf_delay_ms = _params.delay_max_ms * 1.5f;
	_obs_buffer_length = roundf(ekf_delay_ms / filter_update_period_ms);

	// 限制观测缓冲区不要超过IMU缓冲区大小
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	ECL_DEBUG("EKF max time delay %.1f ms, OBS length %d\n", (double)ekf_delay_ms, _obs_buffer_length);

	// 为IMU缓冲区以及_output_predictor(输出预测器)分配对应大小
	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_predictor.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_time_delayed_us = timestamp;
	_time_latest_us = timestamp;

	_fault_status.value = 0;

	return true;
}

// getPosition: 返回输出预测器计算后的当前位置(以本地坐标系为基准)
// 返回值: 三维向量(x, y, z)
Vector3f EstimatorInterface::getPosition() const
{
	LatLonAlt lla = _output_predictor.getLatLonAlt();
	float x;
	float y;

	// 如果本地原点坐标已经初始化，则根据原点投影
	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(lla.latitude_deg(), lla.longitude_deg(), x, y);

	} else {
		// 否则，使用(0,0)做一个默认投影(理论上不会太常用)
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(lla.latitude_deg(), lla.longitude_deg(), x, y);
	}

	// z坐标通过全球高度与本地高度原点相对值计算，注意PX4中Z是向下为正，所以这里需要取负号
	const float z = -(lla.altitude() - getEkfGlobalOriginAltitude());

	return Vector3f(x, y, z);
}

// isOnlyActiveSourceOfHorizontalAiding: 检查某个水平观测融合(如GPS、光流等)是否是唯一的水平观测来源
bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

// isOtherSourceOfHorizontalAidingThan: 检查是否存在除了当前aiding_flag所表示的观测源以外的其他水平观测源
bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

// getNumberOfActiveHorizontalAidingSources: 返回当前已经激活的水平观测源数量
int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       + int(_control_status.flags.aux_gpos)
	       // 如果同时融合空速和侧滑(fuse_aspd && fuse_beta)，则也视为一个水平观测源
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

// isHorizontalAidingActive: 是否存在任何水平观测源
bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

// isOtherSourceOfVerticalPositionAidingThan: 检查是否存在除了aiding_flag外的其他垂直位置观测源
bool EstimatorInterface::isOtherSourceOfVerticalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveVerticalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

// isVerticalPositionAidingActive: 是否有任何垂直位置观测源在使用
bool EstimatorInterface::isVerticalPositionAidingActive() const
{
	return getNumberOfActiveVerticalPositionAidingSources() > 0;
}

// isOnlyActiveSourceOfVerticalPositionAiding: 检查某个垂直位置观测源是否是唯一的
bool EstimatorInterface::isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfVerticalPositionAidingThan(aiding_flag);
}

// getNumberOfActiveVerticalPositionAidingSources: 获得当前激活的垂直位置观测源数量
int EstimatorInterface::getNumberOfActiveVerticalPositionAidingSources() const
{
	return int(_control_status.flags.gps_hgt)
	       + int(_control_status.flags.baro_hgt)
	       + int(_control_status.flags.rng_hgt)
	       + int(_control_status.flags.ev_hgt);
}

// isVerticalAidingActive: 是否存在任何垂直方向上的观测源(位置或速度)
bool EstimatorInterface::isVerticalAidingActive() const
{
	return isVerticalPositionAidingActive() || isVerticalVelocityAidingActive();
}

// isVerticalVelocityAidingActive: 是否有任何垂直速度观测源
bool EstimatorInterface::isVerticalVelocityAidingActive() const
{
	return getNumberOfActiveVerticalVelocityAidingSources() > 0;
}

// getNumberOfActiveVerticalVelocityAidingSources: 当前激活的垂直速度观测源数量
int EstimatorInterface::getNumberOfActiveVerticalVelocityAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.ev_vel);
}

// printBufferAllocationFailed: 打印缓冲区分配失败的错误信息
// 参数 buffer_name: 缓冲区名称
void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}
