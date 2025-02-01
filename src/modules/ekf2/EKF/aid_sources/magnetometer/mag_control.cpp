#include "ekf.h"
#include <mathlib/mathlib.h>

#include <lib/world_magnetic_model/geo_mag_declination.h>

#include <ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h>

/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
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
 * @file mag_control.cpp
 * @brief 控制EKF磁场融合的相关函数实现
 */

void Ekf::controlMagFusion(const imuSample &imu_sample)
{
    // AID_SRC_NAME 用于记录当前融合的数据来源标签
    static constexpr const char *AID_SRC_NAME = "mag";
    estimator_aid_source3d_s &aid_src = _aid_src_mag;

    // 当无人机从地面第一次切换到飞行状态时，重置标志，下一次需要在飞行高度上对齐磁场
    if (!_control_status_prev.flags.in_air && _control_status.flags.in_air) {
        _control_status.flags.mag_aligned_in_flight = false;
    }

    // 如果参数指定不进行任何磁传感器融合，直接停止并返回
    if (_params.mag_fusion_type == MagFuseType::NONE) {
        stopMagFusion();
        return;
    }

    // 存储从磁力计缓冲区读取的样本
    magSample mag_sample;

    // 尝试获取与imu_sample时间匹配或更早的第一份磁力计数据
    if (_mag_buffer && _mag_buffer->pop_first_older_than(imu_sample.time_us, &mag_sample)) {

        // 如果磁力计数据标记了reset或这是第一帧数据
        if (mag_sample.reset || (_mag_counter == 0)) {
            // 传感器或校准发生了变化，重置相应状态
            _control_status.flags.mag_fault = false;
            _state.mag_B.zero();     // 重置磁偏置
            resetMagBiasCov();       // 重置磁偏置协方差

            // 停止融合，以便后面重新初始化
            stopMagFusion();

            // 重置并更新磁力计低通滤波器
            _mag_lpf.reset(mag_sample.mag);
            _mag_counter = 1;

        } else {
            // 数据正常更新的情况，更新磁力计低通滤波器
            _mag_lpf.update(mag_sample.mag);
            _mag_counter++;
        }

        // 定期检查或当全局原点（GPS原点）更新时，是否需要更新WMM（世界磁模型）
        bool wmm_updated = false;
        if (global_origin().isInitialized()) {
            // 判断全局原点的参考时间戳是否比上一次融合磁数据的时间更晚
            bool origin_newer_than_last_mag = (global_origin().getProjectionReferenceTimestamp() > aid_src.time_last_fuse);

            // 当全局原点有效并且需要更新WMM
            if (global_origin_valid() && (origin_newer_than_last_mag || (isLocalHorizontalPositionValid() && isTimedOut(_wmm_mag_time_last_checked, 10e6)))) {
                // 根据当前位置(纬度，经度)更新WMM
                if (updateWorldMagneticModel(_gpos.latitude_deg(), _gpos.longitude_deg())) {
                    wmm_updated = true;
                }
                // 记录下此次更新检查的时间
                _wmm_mag_time_last_checked = _time_delayed_us;

            } else if (origin_newer_than_last_mag) {
                // 否则只根据原点信息来更新
                if (updateWorldMagneticModel(global_origin().getProjectionReferenceLat(), global_origin().getProjectionReferenceLon())) {
                    wmm_updated = true;
                }
            }
        }

        // 如果配置了合成磁传感器Z轴，并且WMM数据可用，则以理论值来覆盖Z测量
        if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
            && (_wmm_earth_field_gauss.isAllFinite() && _wmm_earth_field_gauss.longerThan(0.f))) {
            mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, _wmm_earth_field_gauss);
            _control_status.flags.synthetic_mag_z = true;
        } else {
            _control_status.flags.synthetic_mag_z = false;
        }

        // 每次都重置磁故障相关标志
        _fault_status.flags.bad_mag_x = false;
        _fault_status.flags.bad_mag_y = false;
        _fault_status.flags.bad_mag_z = false;

        // 为磁观测设置噪声方差，需要考虑传感器自身的噪声和采样率等因素
        const float R_MAG = math::max(sq(_params.mag_noise), sq(0.01f));

        // 定义存储创新量和创新方差的变量
        Vector3f mag_innov;
        Vector3f innov_var;

        // 观测雅可比矩阵和其他更新所需临时变量
        VectorState H;
        // 调用自动生成的函数，计算创新、创新方差以及观测雅可比
        sym::ComputeMagInnovInnovVarAndHx(_state.vector(), P, mag_sample.mag, R_MAG, FLT_EPSILON, &mag_innov, &innov_var, &H);

        // 更新观测源状态：包括时间戳、观测值、观测方差、创新量、创新方差等
        updateAidSourceStatus(
            aid_src,                             // 观测源对象
            mag_sample.time_us,                  // 本次磁力计数据的时间
            mag_sample.mag,                      // 实际观测值
            Vector3f(R_MAG, R_MAG, R_MAG),       // 观测噪声方差
            mag_innov,                           // 创新量
            innov_var,                           // 创新方差
            math::max(_params.mag_innov_gate, 1.f) // 创新卡方检验阈值
        );

        // 分别检查各轴的创新超比检测
        _innov_check_fail_status.flags.reject_mag_x = (aid_src.test_ratio[0] > 1.f);
        _innov_check_fail_status.flags.reject_mag_y = (aid_src.test_ratio[1] > 1.f);
        _innov_check_fail_status.flags.reject_mag_z = (aid_src.test_ratio[2] > 1.f);

        // 确定继续进行磁融合的条件：
        bool continuing_conditions_passing = (
            (_params.mag_fusion_type == MagFuseType::INIT)
            || (_params.mag_fusion_type == MagFuseType::AUTO)
            || (_params.mag_fusion_type == MagFuseType::HEADING)
        )
        && _control_status.flags.tilt_align                             // 已经对准了俯仰和横滚
        && (
            _control_status.flags.yaw_align || (       // 如果已经对准Yaw
                (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align) // 或者不使用EV_Yaw，则允许
            )
        )
        && mag_sample.mag.longerThan(0.f)                              // 磁测量的有效性
        && mag_sample.mag.isAllFinite();

        // 判断是否满足启动磁融合的其他条件
        const bool starting_conditions_passing = continuing_conditions_passing
                                                 && checkMagField(mag_sample.mag)   // 检查磁场强度和倾角合理性
                                                 && (_mag_counter > 3)              // 至少等待3个样本通过滤波器
                                                 && (
                                                    _control_status.flags.yaw_align == _control_status_prev.flags.yaw_align // 本帧未改变Yaw对准状态
                                                 )
                                                 && (
                                                    _state_reset_status.reset_count.quat == _state_reset_count_prev.quat // 本帧未发生四元数重置
                                                 )
                                                 && isNewestSampleRecent(_time_last_mag_buffer_push, MAG_MAX_INTERVAL);

        // 检查磁航向的一致性，比如磁航向与当前姿态计算的航向差异
        checkMagHeadingConsistency(mag_sample);

        // 如果开启GPS或其他位置辅助，则说明有NE方向上的观测
        const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;

        // 在AUTO模式下，可能开启3D磁融合或者只有航向融合
        {
            // 如果磁航向一致或没有NE方向辅助，则认为满足继续融合的条件
            const bool mag_consistent_or_no_ne_aiding = _control_status.flags.mag_heading_consistent || !using_ne_aiding;

            // 通用条件：开启了磁融合、Yaw已对准、磁传感器无故障等
            const bool common_conditions_passing = _control_status.flags.mag
                                                   && (
                                                       (
                                                           _control_status.flags.yaw_align && mag_consistent_or_no_ne_aiding
                                                       )
                                                       || (
                                                           !_control_status.flags.ev_yaw && !_control_status.flags.yaw_align
                                                       )
                                                   )
                                                   && !_control_status.flags.mag_fault
                                                   && !_control_status.flags.mag_field_disturbed
                                                   && !_control_status.flags.ev_yaw
                                                   && !_control_status.flags.gnss_yaw;

            // 如果是AUTO模式并且在飞行中已经对齐过磁场，那么可以进行3D磁融合
            _control_status.flags.mag_3D = common_conditions_passing
                                           && (_params.mag_fusion_type == MagFuseType::AUTO)
                                           && _control_status.flags.mag_aligned_in_flight;

            // 如果是HEADING或AUTO模式，但不满足3D融合条件，则仅融合航向
            _control_status.flags.mag_hdg = common_conditions_passing
                                            && (
                                                (_params.mag_fusion_type == MagFuseType::HEADING)
                                                || (
                                                    _params.mag_fusion_type == MagFuseType::AUTO && !_control_status.flags.mag_3D
                                                )
                                            );
        }

        // 状态转变的调试输出
        if (_control_status.flags.mag_3D && !_control_status_prev.flags.mag_3D) {
            ECL_INFO("starting mag 3D fusion");
        } else if (!_control_status.flags.mag_3D && _control_status_prev.flags.mag_3D) {
            ECL_INFO("stopping mag 3D fusion");
        }

        // 如果没有NE方向辅助或机体当前静止，则还需要对磁偏角进行单独的融合
        const bool no_ne_aiding_or_not_moving = !using_ne_aiding || _control_status.flags.vehicle_at_rest;
        _control_status.flags.mag_dec = _control_status.flags.mag && no_ne_aiding_or_not_moving;

        // 如果已经开始磁融合
        if (_control_status.flags.mag) {
            // 并且继续融合的条件满足，同时Yaw已经对准
            if (continuing_conditions_passing && _control_status.flags.yaw_align) {
                // 如果需要在离地后重置Yaw，或WMM刚更新且没有NE辅助
                if (checkHaglYawResetReq() || (wmm_updated && no_ne_aiding_or_not_moving)) {
                    ECL_INFO("reset to %s", AID_SRC_NAME);
                    // 重置磁状态（包含地磁场和偏置），根据当前模式判断是否重置航向
                    resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
                    aid_src.time_last_fuse = imu_sample.time_us;

                } else {
                    // 根据当前模式（3D或航向融合）来决定更新哪些状态
                    const bool update_all_states = _control_status.flags.mag_3D || _control_status.flags.mag_hdg;
                    const bool update_tilt = _control_status.flags.mag_3D;

                    // 进行磁数据融合
                    fuseMag(mag_sample.mag, R_MAG, H, aid_src, update_all_states, update_tilt);

                    // 如果包含更新姿态信息，则检查创新方差
                    if (update_all_states && update_tilt) {
                        _fault_status.flags.bad_mag_x = (aid_src.innovation_variance[0] < aid_src.observation_variance[0]);
                        _fault_status.flags.bad_mag_y = (aid_src.innovation_variance[1] < aid_src.observation_variance[1]);
                        _fault_status.flags.bad_mag_z = (aid_src.innovation_variance[2] < aid_src.observation_variance[2]);
                    }

                    // 如果需要融合磁偏角
                    if (_control_status.flags.mag_dec) {
                        // 定义偏角噪声（弧度^2）
                        const float R_DECL = sq(0.5f);

                        // 如果使用WMM提供的磁偏角
                        if ((_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
                            && PX4_ISFINITE(_wmm_declination_rad)) {

                            fuseDeclination(_wmm_declination_rad, 0.5f, update_all_states);

                        } else if ((_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)
                                   && PX4_ISFINITE(_params.mag_declination_deg)
                                   && (fabsf(_params.mag_declination_deg) > 0.f)
                                  ) {
                            // 使用保存的偏角
                            fuseDeclination(math::radians(_params.mag_declination_deg), R_DECL, update_all_states);

                        } else {
                            // 如果缺少任何来源，就融合0作为默认偏角
                            float declination_rad = 0.f;
                            fuseDeclination(declination_rad, R_DECL);
                        }
                    }
                }

                // 判断磁融合是否出现了较长时间未成功融合的情况
                const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

                if (is_fusion_failing) {
                    // 如果没有NE辅助，那么在此处尝试重置磁状态
                    if (no_ne_aiding_or_not_moving) {
                        ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
                        resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
                        aid_src.time_last_fuse = imu_sample.time_us;

                    } else {
                        // 否则，如果有NE辅助，但依旧出现故障，则停止融合
                        ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
                        stopMagFusion();
                    }
                }

            } else {
                // 如果继续融合条件失败，或者Yaw未对准，则停止但不记录故障
                ECL_DEBUG("stopping %s fusion, continuing conditions no longer passing", AID_SRC_NAME);
                stopMagFusion();
            }

        } else {
            // 如果当前未进行磁融合，但满足启动条件
            if (starting_conditions_passing) {

                // 如果尚未对准Yaw或WMM更新，或者磁场状态无效，需要重置状态
                if (!_control_status.flags.yaw_align
                    || wmm_updated
                    || !_state.mag_I.longerThan(0.f)
                    || (getStateVariance<State::mag_I>().min() < kMagVarianceMin)
                    || (getStateVariance<State::mag_B>().min() < kMagVarianceMin)) {

                    ECL_INFO("starting %s fusion, resetting states", AID_SRC_NAME);

                    // 当Yaw还未对准时，需要在重置时进行对准
                    bool reset_heading = !_control_status.flags.yaw_align;

                    resetMagStates(_mag_lpf.getState(), reset_heading);
                    aid_src.time_last_fuse = imu_sample.time_us;

                    // 如果进行了Yaw重置，则更新Yaw对准标志
                    if (reset_heading) {
                        _control_status.flags.yaw_align = true;
                        resetAidSourceStatusZeroInnovation(aid_src);
                    }

                    _control_status.flags.mag = true;

                } else {
                    // 否则直接尝试融合
                    if (fuseMag(mag_sample.mag, R_MAG, H, aid_src)) {
                        ECL_INFO("starting %s fusion", AID_SRC_NAME);
                        _control_status.flags.mag = true;
                    }
                }
            }
        }

    } else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
        // 如果长时间（2倍的最大间隔）没有新的磁传感器数据，则停止融合
        stopMagFusion();
    }
}

void Ekf::stopMagFusion()
{
    // 如果当前处于磁融合状态
    if (_control_status.flags.mag) {
        ECL_INFO("stopping mag fusion");

        // 重置磁场相关协方差
        resetMagEarthCov();
        resetMagBiasCov();

        // 如果已经对准了Yaw且在进行3D或航向融合
        if (_control_status.flags.yaw_align && (_control_status.flags.mag_3D || _control_status.flags.mag_hdg)) {
            // 如果未使用NE方向的传感器数据辅助，则可能需要清除Yaw对准
            const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;

            if (!using_ne_aiding) {
                _control_status.flags.yaw_align = false;
            }
        }

        // 关闭磁融合标志
        _control_status.flags.mag = false;
        _control_status.flags.mag_dec = false;

        if (_control_status.flags.mag_3D) {
            ECL_INFO("stopping mag 3D fusion");
            _control_status.flags.mag_3D = false;
        }

        if (_control_status.flags.mag_hdg) {
            ECL_INFO("stopping mag heading fusion");
            _control_status.flags.mag_hdg = false;
            _fault_status.flags.bad_hdg = false;
        }

        _control_status.flags.mag_aligned_in_flight = false;

        // 清除磁相关故障标志
        _fault_status.flags.bad_mag_x = false;
        _fault_status.flags.bad_mag_y = false;
        _fault_status.flags.bad_mag_z = false;
        _fault_status.flags.bad_mag_decl = false;
    }
}

bool Ekf::checkHaglYawResetReq() const
{
#if defined(CONFIG_EKF2_TERRAIN)
    // 当在空中且Yaw已对准并且尚未在飞行时对齐过磁场
    if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
        // 当飞行高度大于一定阈值，认为离地面磁干扰足够远，可以进行新的Yaw重置
        static constexpr float mag_anomalies_max_hagl = 1.5f;
        const bool above_mag_anomalies = (getTerrainVPos() + _gpos.altitude()) > mag_anomalies_max_hagl;
        return above_mag_anomalies;
    }
#endif // CONFIG_EKF2_TERRAIN

    return false;
}

void Ekf::resetMagStates(const Vector3f &mag, bool reset_heading)
{
    // 记录重置前的地磁场和偏置
    const Vector3f mag_I_before_reset = _state.mag_I;
    const Vector3f mag_B_before_reset = _state.mag_B;

    // 用于判定地磁场显著变化的阈值
    static constexpr float kMagEarthMinGauss = 0.01f;

    // 如果可用的世界磁模型信息有效，则使用它来重置
    if (_wmm_earth_field_gauss.longerThan(0.f) && _wmm_earth_field_gauss.isAllFinite()) {
        // 计算想要的地磁场向量
        const Vector3f mag_I = _wmm_earth_field_gauss;
        bool mag_I_reset = false;

        // 如果和当前估计有较大差异，则重置
        if ((_state.mag_I - mag_I).longerThan(kMagEarthMinGauss)) {
            _state.mag_I = mag_I;
            resetMagEarthCov();  // 同时重置地磁场协方差
            mag_I_reset = true;
        }

        // 决定是否重置磁偏置
        if (!reset_heading && _control_status.flags.yaw_align) {
            if (mag_I_reset) {
                // 计算新的磁偏置 = 测量 - 在机体坐标下的地磁场投影
                const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
                _state.mag_B = mag - (R_to_body * _wmm_earth_field_gauss);
                resetMagBiasCov();
            }
        } else {
            // 如果要重置航向或尚未Yaw对准，则把偏置设为0
            _state.mag_B.zero();
            resetMagBiasCov();
        }

        // 如果需要重置航向，则调用相应方法
        if (reset_heading) {
            resetMagHeading(mag);
        }

    } else {
        // 如果WMM无效，则直接将磁偏置置0
        _state.mag_B.zero();
        resetMagBiasCov();

        // 如果需要，则重置航向
        if (reset_heading) {
            resetMagHeading(mag);
        }

        // 把地磁场向量更新为当前观测投影
        const Vector3f mag_I = _R_to_earth * mag;
        if ((_state.mag_I - mag_I).longerThan(kMagEarthMinGauss)) {
            _state.mag_I = mag_I;
            resetMagEarthCov();
        }
    }

    // 打印重置前后状态的变化
    if ((_state.mag_I - mag_I_before_reset).longerThan(0.f)) {
        ECL_INFO("resetting mag I [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                 (double)mag_I_before_reset(0), (double)mag_I_before_reset(1), (double)mag_I_before_reset(2),
                 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2));
    }

    if ((_state.mag_B - mag_B_before_reset).longerThan(0.f)) {
        ECL_INFO("resetting mag B [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
                 (double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
                 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2));
    }

    // 如果当前是在飞行状态下对齐磁场，则记录相应标志
    if (_control_status.flags.in_air) {
        _control_status.flags.mag_aligned_in_flight = true;
        _flt_mag_align_start_time = _time_delayed_us;
    }
}

void Ekf::checkMagHeadingConsistency(const magSample &mag_sample)
{
    // 如果磁偏置方差小，可以用估计到的偏置修正原始测量
    Vector3f mag_bias{0.f, 0.f, 0.f};
    const Vector3f mag_bias_var = getMagBiasVariance();

    if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
        mag_bias = _state.mag_B;
    }

    // 首先构造一个在航向上为0（仅俯仰、横滚和机体-地球坐标之间正常转换）的旋转矩阵
    const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
    // 得到的矢量在水平面的投影来计算航向
    const Vector3f mag_earth_pred = R_to_earth * (mag_sample.mag - mag_bias);

    // 从WMM或其他来源获取磁偏角
    const float declination = getMagDeclination();
    // 根据投影结果和偏角，得到实际测量的航向
    const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;

    // 如果Yaw对准了
    if (_control_status.flags.yaw_align) {
        // 计算当前估计和实际测量的差异，并进行滤波
        const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
        _mag_heading_innov_lpf.update(innovation);

    } else {
        // 尚未Yaw对准，则把滤波器置为0
        _mag_heading_innov_lpf.reset(0.f);
    }

    // 当创新量足够小，则说明磁航向与当前状态一致
    if (fabsf(_mag_heading_innov_lpf.getState()) < _params.mag_heading_noise) {
        // 如果在使用NE方向上的观测（GPS或其他），并且水平加速度大于阈值，则说明Yaw是可观测的
        const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;
        if (using_ne_aiding && (_accel_horiz_lpf.getState().longerThan(_params.mag_acc_gate))) {
            _control_status.flags.mag_heading_consistent = true;
        }

    } else {
        // 否则标记为磁航向不一致
        _control_status.flags.mag_heading_consistent = false;
    }
}

bool Ekf::checkMagField(const Vector3f &mag_sample)
{
    // 默认认为磁场未被严重干扰
    _control_status.flags.mag_field_disturbed = false;

    // 如果未开启任何磁检查，直接通过
    if (_params.mag_check == 0) {
        return true;
    }

    bool is_check_failing = false;
    // 计算磁场强度
    _mag_strength = mag_sample.length();

    // 如果开启了磁场强度检查
    if (_params.mag_check & static_cast<int32_t>(MagCheckMask::STRENGTH)) {
        if (PX4_ISFINITE(_wmm_field_strength_gauss)) {
            // 如果可以获得WMM预期强度
            if (!isMeasuredMatchingExpected(_mag_strength, _wmm_field_strength_gauss, _params.mag_check_strength_tolerance_gs)) {
                _control_status.flags.mag_field_disturbed = true;
                is_check_failing = true;
            }

        } else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
            // 强制要求WMM，但数据不可用
            is_check_failing = true;

        } else {
            // 未知位置时，以一个默认地球平均值范围来做粗略检查
            constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
            constexpr float average_earth_mag_gate_size = 0.40f; // 容忍范围

            if (!isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size)) {
                _control_status.flags.mag_field_disturbed = true;
                is_check_failing = true;
            }
        }
    }

    // 计算旋转到地球坐标系后的矢量，并计算磁倾角
    const Vector3f mag_earth = _R_to_earth * mag_sample;
    _mag_inclination = asinf(mag_earth(2) / fmaxf(mag_earth.norm(), 1e-4f));

    // 如果开启了磁倾角检查
    if (_params.mag_check & static_cast<int32_t>(MagCheckMask::INCLINATION)) {
        if (PX4_ISFINITE(_wmm_inclination_rad)) {
            float inc_tol_rad = radians(_params.mag_check_inclination_tolerance_deg);
            float inc_error_rad = wrap_pi(_mag_inclination - _wmm_inclination_rad);

            if (fabsf(inc_error_rad) > inc_tol_rad) {
                _control_status.flags.mag_field_disturbed = true;
                is_check_failing = true;
            }

        } else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
            // 强制WMM倾角检查但无数据
            is_check_failing = true;
        } else {
            // 无全球位置信息时无法进行倾角检查
        }
    }

    // 如果检查失败，或是第一次进来这里，则记录下失败时间
    if (is_check_failing || (_time_last_mag_check_failing == 0)) {
        _time_last_mag_check_failing = _time_delayed_us;
    }

    // 如果距离上次失败的时间超过一定阈值，则认为磁场健康
    return ((_time_delayed_us - _time_last_mag_check_failing) > (uint64_t)_min_mag_health_time_us);
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
    // 判断是否在预期范围内
    return (measured >= expected - gate) && (measured <= expected + gate);
}

void Ekf::resetMagHeading(const Vector3f &mag)
{
    // 如果磁偏置方差较小，则使用估计到的偏置
    Vector3f mag_bias{0.f, 0.f, 0.f};
    const Vector3f mag_bias_var = getMagBiasVariance();
    if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
        mag_bias = _state.mag_B;
    }

    // 构造一个yaw=0的旋转矩阵，将测量值投影到地球系
    const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
    const Vector3f mag_earth_pred = R_to_earth * (mag - mag_bias);

    // 计算磁偏角
    const float declination = getMagDeclination();
    // 计算新的yaw
    float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;
    // 设置一个合理的方差
    float yaw_new_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));

    ECL_INFO("reset mag heading %.3f -> %.3f rad (bias:[%.3f, %.3f, %.3f], declination:%.1f)",
             (double)getEulerYaw(_R_to_earth),
             (double)yaw_new,
             (double)mag_bias(0),
             (double)mag_bias(1),
             (double)mag_bias(2),
             (double)declination);

    // 重置四元数中的yaw部分，并更新协方差
    resetQuatStateYaw(yaw_new, yaw_new_variance);

    // 更新上次航向融合时间
    _time_last_heading_fuse = _time_delayed_us;

    // 重置磁航向创新的滤波器
    _mag_heading_innov_lpf.reset(0.f);

    // 标记磁航向一致
    _control_status.flags.mag_heading_consistent = true;
}

float Ekf::getMagDeclination()
{
    // 1. 如果已经在飞行中对齐过磁场，则使用状态中的地磁场
    if (_control_status.flags.mag_aligned_in_flight) {
        return atan2f(_state.mag_I(1), _state.mag_I(0));

    } else if ((_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
               && PX4_ISFINITE(_wmm_declination_rad)) {
        // 2. 如果可使用世界磁模型提供的磁偏角
        return _wmm_declination_rad;

    } else if ((_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)
               && PX4_ISFINITE(_params.mag_declination_deg)
               && (fabsf(_params.mag_declination_deg) > 0.f)) {
        // 3. 使用参数中保存的偏角
        return math::radians(_params.mag_declination_deg);
    }

    // 若无任何信息可用，则默认返回0
    return 0.f;
}

bool Ekf::updateWorldMagneticModel(const double latitude_deg, const double longitude_deg)
{
    // 根据当前经纬度获取WMM提供的磁偏角、倾角和强度
    const float declination_rad = math::radians(get_mag_declination_degrees(latitude_deg, longitude_deg));
    const float inclination_rad = math::radians(get_mag_inclination_degrees(latitude_deg, longitude_deg));
    const float strength_gauss = get_mag_strength_gauss(latitude_deg, longitude_deg);

    // 如果数据是有效的
    if (PX4_ISFINITE(declination_rad) && PX4_ISFINITE(inclination_rad) && PX4_ISFINITE(strength_gauss)) {
        // 检查与当前存储值是否有较大差异
        const bool declination_changed = (fabsf(declination_rad - _wmm_declination_rad) > math::radians(1.f));
        const bool inclination_changed = (fabsf(inclination_rad - _wmm_inclination_rad) > math::radians(1.f));
        const bool strength_changed = (fabsf(strength_gauss - _wmm_field_strength_gauss) > 0.01f);

        // 如果之前无效或发生显著变化
        if (!PX4_ISFINITE(_wmm_declination_rad)
            || !PX4_ISFINITE(_wmm_inclination_rad)
            || !PX4_ISFINITE(_wmm_field_strength_gauss)
            || !_wmm_earth_field_gauss.longerThan(0.f)
            || !_wmm_earth_field_gauss.isAllFinite()
            || declination_changed
            || inclination_changed
            || strength_changed) {

            ECL_DEBUG("WMM declination updated %.3f -> %.3f deg (lat=%.6f, lon=%.6f)",
                      (double)math::degrees(_wmm_declination_rad),
                      (double)math::degrees(declination_rad),
                      (double)latitude_deg,
                      (double)longitude_deg);

            // 更新全局存储的WMM信息
            _wmm_declination_rad = declination_rad;
            _wmm_inclination_rad = inclination_rad;
            _wmm_field_strength_gauss = strength_gauss;

            // 通过倾角和偏角确定地球系下的磁场向量
            _wmm_earth_field_gauss = Dcmf(Eulerf(0, -inclination_rad, declination_rad)) * Vector3f(strength_gauss, 0, 0);

            return true;
        }
    }

    return false;
}
