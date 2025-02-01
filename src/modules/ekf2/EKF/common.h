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
 * @file common.h
 * 定义姿态估计器的基类
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */
#ifndef EKF_COMMON_H
#define EKF_COMMON_H

#include <cstdint>

#include <matrix/math.hpp>
#include <mathlib/math/Utilities.hpp>

namespace estimator
{

using matrix::AxisAnglef;  // 轴角表示
using matrix::Dcmf;        // 方向余弦矩阵
using matrix::Eulerf;      // 欧拉角表示
using matrix::Matrix3f;    // 3x3矩阵
using matrix::Quatf;       // 四元数表示
using matrix::Vector2f;    // 2D向量
using matrix::Vector3f;    // 3D向量
using matrix::wrap_pi;     // 将角度限制在[-π, π]范围内

using math::Utilities::getEulerYaw;                  // 获取欧拉角中的偏航角
using math::Utilities::quatToInverseRotMat;         // 将四元数转换为逆旋转矩阵
using math::Utilities::shouldUse321RotationSequence;  // 判断是否使用321旋转序列
using math::Utilities::sq;                            // 计算平方
using math::Utilities::updateYawInRotMat;           // 在旋转矩阵中更新偏航角

// 最大传感器间隔（微秒）
static constexpr uint64_t BARO_MAX_INTERVAL     =
	200e3;  ///< 最大允许的气压高度测量时间间隔（微秒）
static constexpr uint64_t EV_MAX_INTERVAL       =
	200e3;  ///< 最大允许的外部视觉系统测量时间间隔（微秒）
static constexpr uint64_t GNSS_MAX_INTERVAL     =
	500e3;  ///< 最大允许的GNSS测量时间间隔（微秒）
static constexpr uint64_t GNSS_YAW_MAX_INTERVAL =
	1500e3; ///< 最大允许的GNSS偏航测量时间间隔（微秒）
static constexpr uint64_t MAG_MAX_INTERVAL      =
	500e3;  ///< 最大允许的磁场测量时间间隔（微秒）

// 不良加速度计检测和缓解
static constexpr uint64_t BADACC_PROBATION =
	3e6; ///< 声明为不良的加速度数据必须连续通过检查的时间段，以便再次声明为良好（微秒）
static constexpr float BADACC_BIAS_PNOISE =
	4.9f;  ///< 当加速度数据被声明为不良时，增量速度过程噪声设置为此值（米/秒²）

// 地面效应补偿
static constexpr uint64_t GNDEFFECT_TIMEOUT =
	10e6; ///< 地面效应保护在最后一次开启后将保持活动的最大时间段（微秒）

enum class PositionFrame : uint8_t {
	LOCAL_FRAME_NED = 0,  ///< 本地NED坐标系
	LOCAL_FRAME_FRD = 1,  ///< 本地FRD坐标系
};

enum class VelocityFrame : uint8_t {
	LOCAL_FRAME_NED = 0,  ///< 本地NED坐标系
	LOCAL_FRAME_FRD = 1,  ///< 本地FRD坐标系
	BODY_FRAME_FRD  = 2   ///< 机体FRD坐标系
};

#if defined(CONFIG_EKF2_MAGNETOMETER)
enum GeoDeclinationMask : uint8_t {
	// 磁偏角源的位置信息
	USE_GEO_DECL  = (1 << 0), ///< 当GPS位置可用时，设置为true以使用地理库中的偏角，设置为false以始终使用EKF2_MAG_DECL值
	SAVE_GEO_DECL = (1 << 1) ///< 设置为true以将EKF2_MAG_DECL参数设置为地理库返回的值
};

enum MagFuseType : uint8_t {
	// 磁融合类型的整数定义
	AUTO    = 0,   	///< 自动选择航向或3D磁力计融合
	HEADING = 1,   	///< 始终使用简单的偏航角融合。精度较低，但不易受地球场畸变影响。应避免在-60到+60度以外的俯仰角使用
	NONE    = 5,   	///< 在任何情况下都不使用磁力计。
	INIT    = 6     ///< 仅在航向初始化时使用磁力计。
};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_TERRAIN)
enum class TerrainFusionMask : uint8_t {
	TerrainFuseRangeFinder = (1 << 0), ///< 使用测距仪进行地形融合
	TerrainFuseOpticalFlow = (1 << 1)   ///< 使用光流进行地形融合
};
#endif // CONFIG_EKF2_TERRAIN

enum class HeightSensor : uint8_t {
	BARO  = 0,  ///< 气压传感器
	GNSS  = 1,  ///< GNSS传感器
	RANGE = 2,  ///< 距离传感器
	EV    = 3,  ///< 外部视觉传感器
	UNKNOWN  = 4 ///< 未知传感器
};

enum class PositionSensor : uint8_t {
	UNKNOWN = 0, ///< 未知传感器
	GNSS    = 1, ///< GNSS传感器
	EV      = 2, ///< 外部视觉传感器
};

enum class ImuCtrl : uint8_t {
	GyroBias      = (1 << 0), ///< 陀螺仪偏差控制
	AccelBias     = (1 << 1), ///< 加速度计偏差控制
	GravityVector = (1 << 2), ///< 重力向量控制
};

enum class GnssCtrl : uint8_t {
	HPOS  = (1 << 0), ///< 水平位置控制
	VPOS  = (1 << 1), ///< 垂直位置控制
	VEL   = (1 << 2), ///< 速度控制
	YAW   = (1 << 3)  ///< 偏航控制
};

enum class RngCtrl : uint8_t {
	DISABLED    = 0, ///< 禁用
	CONDITIONAL = 1, ///< 有条件启用
	ENABLED     = 2  ///< 启用
};

enum class EvCtrl : uint8_t {
	HPOS = (1 << 0), ///< 水平位置控制
	VPOS = (1 << 1), ///< 垂直位置控制
	VEL  = (1 << 2), ///< 速度控制
	YAW  = (1 << 3)  ///< 偏航控制
};

enum class MagCheckMask : uint8_t {
	STRENGTH    = (1 << 0), ///< 磁场强度检查
	INCLINATION = (1 << 1), ///< 磁场倾斜检查
	FORCE_WMM   = (1 << 2)  ///< 强制使用世界磁场模型
};

enum class FlowGyroSource : uint8_t {
	Auto     = 0, ///< 自动选择
	Internal = 1  ///< 内部选择
};

struct imuSample {
	uint64_t    time_us{};                ///< 测量的时间戳（微秒）
	Vector3f    delta_ang{};              ///< 机体坐标系中的增量角度（积分陀螺仪测量值）（弧度）
	Vector3f    delta_vel{};              ///< 机体坐标系中的增量速度（积分加速度计测量值）（米/秒）
	float       delta_ang_dt{};           ///< 增量角度积分周期（秒）
	float       delta_vel_dt{};           ///< 增量速度积分周期（秒）
	bool        delta_vel_clipping[3] {}; ///< 如果此样本包含任何加速度计剪切，则为真（每个轴）
};

struct gnssSample {
	uint64_t    time_us{};    ///< 测量的时间戳（微秒）
	double      lat{};        ///< 纬度（度）
	double      lon{};        ///< 经度（度）
	float       alt{};        ///< GNSS海拔高度（米）
	Vector3f    vel{};        ///< NED坐标系中的GNSS速度测量（米/秒）
	float       hacc{};       ///< 1-标准水平位置误差（米）
	float       vacc{};       ///< 1-标准垂直位置误差（米）
	float       sacc{};       ///< 1-标准速度误差（米/秒）
	uint8_t     fix_type{};   ///< 0-1: 无修正，2: 2D修正，3: 3D修正，4: RTCM代码差分，5: 实时修正
	uint8_t     nsats{};      ///< 使用的卫星数量
	float       pdop{};       ///< 位置精度衰减
	float       yaw{};        ///< 偏航角。如果未设置则为NaN（用于双天线GPS），（弧度，[-PI, PI]）
	float       yaw_acc{};    ///< 1-标准偏航误差（弧度）
	float       yaw_offset{}; ///< 双天线GPS的航向/偏航偏移 - 参考GPS_YAW_OFFSET的描述
	bool        spoofed{};    ///< 如果GNSS数据被伪造，则为真
};

struct magSample {
	uint64_t    time_us{};  ///< 测量的时间戳（微秒）
	Vector3f    mag{};      ///< NED坐标系中的磁力计测量（高斯）
	bool        reset{false}; ///< 磁力计是否发生变化（不同传感器或校准变化）
};

struct baroSample {
	uint64_t    time_us{};  ///< 测量的时间戳（微秒）
	float       hgt{};      ///< 海平面以上的气压高度（米）
	bool        reset{false}; ///< 是否重置
};

struct airspeedSample {
	uint64_t    time_us{};          ///< 测量的时间戳（微秒）
	float       true_airspeed{};    ///< 真正的空速测量（米/秒）
	float       eas2tas{};          ///< 相当于真正的空速因子
};

struct flowSample {
	uint64_t    time_us{};   ///< 积分周期中点的时间戳（微秒）
	Vector2f    flow_rate{}; ///< 关于X和Y机体轴的图像测量的角速度（弧度/秒），右手旋转为正
	Vector3f    gyro_rate{}; ///< 从速率陀螺仪测量中获得的关于机体轴的惯性框架的角速度（弧度/秒），右手旋转为正
	uint8_t     quality{};   ///< 质量指示器，范围从0到255
};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
struct extVisionSample {
	uint64_t    time_us{};     ///< 测量的时间戳（微秒）
	Vector3f    pos{};         ///< 外部视觉的局部参考框架中的XYZ位置（米） - Z必须与下轴对齐
	Vector3f    vel{};         ///< 在vel_frame变量定义的参考框架中的FRD速度（米/秒） - Z必须与下轴对齐
	Quatf       quat{};        ///< 定义从机体到地球框架的旋转的四元数
	Vector3f    position_var{};    ///< XYZ位置方差（米²）
	Vector3f    velocity_var{};    ///< XYZ速度方差（米/秒²）
	Vector3f    orientation_var{}; ///< 方向方差（弧度²）
	PositionFrame pos_frame = PositionFrame::LOCAL_FRAME_FRD; ///< 位置框架
	VelocityFrame vel_frame = VelocityFrame::BODY_FRAME_FRD;   ///< 速度框架
	uint8_t     reset_counter{}; ///< 重置计数器
	int8_t     quality{};     ///< 质量指示器，范围从0到100
};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_DRAG_FUSION)
struct dragSample {
	uint64_t    time_us{};     ///< 测量的时间戳（微秒）
	Vector2f    accelXY{};     ///< 沿X和Y机体轴的测量特定力（米/秒²）
};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
struct auxVelSample {
	uint64_t    time_us{};     ///< 测量的时间戳（微秒）
	Vector2f    vel{};         ///< 相对于局部原点的NE速度测量（米/秒）
	Vector2f    velVar{};      ///< NE速度的估计误差方差（米/秒）²
};
#endif // CONFIG_EKF2_AUXVEL

struct systemFlagUpdate {
	uint64_t time_us{}; ///< 时间戳（微秒）
	bool at_rest{false}; ///< 是否处于静止状态
	bool in_air{true};   ///< 是否在空中
	bool is_fixed_wing{false}; ///< 是否为固定翼
	bool gnd_effect{false}; ///< 是否存在地面效应
	bool constant_pos{false}; ///< 是否保持恒定位置
};

struct parameters {

	int32_t filter_update_interval_us{10000}; ///< 过滤器更新间隔（微秒）

	int32_t imu_ctrl{static_cast<int32_t>(ImuCtrl::GyroBias) | static_cast<int32_t>(ImuCtrl::AccelBias)}; ///< IMU控制标志

	float velocity_limit{100.f};           ///< 速度状态限制（米/秒）

	// 测量源控制
	int32_t height_sensor_ref{static_cast<int32_t>(HeightSensor::BARO)}; ///< 高度传感器参考
	int32_t position_sensor_ref{static_cast<int32_t>(PositionSensor::GNSS)}; ///< 位置传感器参考

	float delay_max_ms{110.f};              ///< 所有辅助传感器的最大时间延迟。设置观测缓冲区的大小。（毫秒）

	// 输入噪声
	float gyro_noise{1.5e-2f};              ///< IMU角速率噪声，用于协方差预测（弧度/秒）
	float accel_noise{3.5e-1f};             ///< IMU加速度噪声，用于协方差预测（米/秒²）

	// 过程噪声
	float gyro_bias_p_noise{1.0e-3f};       ///< IMU陀螺仪偏差预测的过程噪声（弧度/秒²）
	float accel_bias_p_noise{1.0e-2f};      ///< IMU加速度计偏差预测的过程噪声（米/秒³）

#if defined(CONFIG_EKF2_WIND)
	const float initial_wind_uncertainty {1.0f};    ///< 风速的1-标准差初始不确定性（米/秒）
	float wind_vel_nsd{1.0e-2f};        ///< 风速预测的过程噪声谱密度（米/秒²/√Hz）
	const float wind_vel_nsd_scaler{0.5f};      ///< 垂直速度下风过程噪声的缩放
#endif // CONFIG_EKF2_WIND

	// 初始化误差
	float switch_on_gyro_bias{0.1f};        ///< 开机时陀螺仪偏差的不确定性（弧度/秒）
	float switch_on_accel_bias{0.2f};       ///< 开机时加速度计偏差的不确定性（米/秒²）
	float initial_tilt_err{0.1f};           ///< 使用重力向量进行初始对准后的1-标准差倾斜误差（弧度）

#if defined(CONFIG_EKF2_BAROMETER)
	int32_t baro_ctrl {1}; ///< 气压计控制标志
	float baro_delay_ms{0.0f};              ///< 气压高度测量相对于IMU的延迟（毫秒）
	float baro_noise{2.0f};                 ///< 气压高度融合的观测噪声（米）
	float baro_bias_nsd{0.13f};             ///< 气压高度偏差估计的过程噪声（米/秒/√Hz）
	float baro_innov_gate{5.0f};            ///< 气压和GPS高度创新一致性门限大小（标准差）

	float gnd_effect_deadzone{5.0f};        ///< 当地面效应补偿处于活动状态时，施加于负气压创新的死区大小（米）
	float gnd_effect_max_hgt{0.5f};         ///< 在此高度以上，气压计的地面效应变得微不足道（米）

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	// 沿机体轴的静态气压位置误差系数
	float static_pressure_coef_xp{0.0f};    // (-)
	float static_pressure_coef_xn{0.0f};    // (-)
	float static_pressure_coef_yp{0.0f};    // (-)
	float static_pressure_coef_yn{0.0f};    // (-)
	float static_pressure_coef_z{0.0f};     // (-)

	// 用于修正的空速上限（米/秒²）
	float max_correction_airspeed{20.0f};
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	int32_t gnss_ctrl {static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL)}; ///< GNSS控制标志，结合水平位置和速度信息
	float gps_delay_ms{110.0f};             ///< GPS测量相对于IMU的延迟（毫秒）

	Vector3f gps_pos_body{};                ///< GPS天线在机体坐标系中的xyz位置（米）

	// 位置和速度融合
	float gps_vel_noise{0.5f};           ///< GPS速度融合的最小允许观测噪声（米/秒）
	float gps_pos_noise{0.5f};              ///< GPS位置融合的最小允许观测噪声（米）
	float gps_hgt_bias_nsd{0.13f};          ///< GNSS高度偏差估计的过程噪声（米/秒/√Hz）
	float gps_pos_innov_gate{5.0f};         ///< GPS水平位置创新一致性门限大小（标准差）
	float gps_vel_innov_gate{5.0f};         ///< GPS速度创新一致性门限大小（标准差）

	// 这些参数控制GPS质量检查的严格程度，用于判断GPS是否足够好以设置局部原点并开始辅助
	int32_t gps_check_mask{21};             ///< 用于控制使用哪些GPS质量检查的位掩码
	float req_hacc{5.0f};                   ///< 最大可接受的水平位置误差（米）
	float req_vacc{8.0f};                   ///< 最大可接受的垂直位置误差（米）
	float req_sacc{1.0f};                   ///< 最大可接受的速度误差（米/秒）
	int32_t req_nsats{6};                   ///< 最小可接受的卫星数量
	float req_pdop{2.0f};                   ///< 最大可接受的位置精度衰减
	float req_hdrift{0.3f};                 ///< 最大可接受的水平漂移速度（米/秒）
	float req_vdrift{0.5f};                 ///< 最大可接受的垂直漂移速度（米/秒）

# if defined(CONFIG_EKF2_GNSS_YAW)
	// GNSS航向融合
	float gnss_heading_noise{0.1f};          ///< 用于GNSS航向融合的测量噪声标准差（弧度）
# endif // CONFIG_EKF2_GNSS_YAW

	// 控制何时将航向重置为EKF-GSF航向估计值的参数
	float EKFGSF_tas_default{15.0f};                ///< 在固定翼飞行中，如果没有气速测量，假定的默认空速值（米/秒）
	const unsigned EKFGSF_reset_delay{1000000};     ///< 在起飞后立即阶段，主滤波器中不良创新的微秒数，达到此值后航向将重置为EKF-GSF值
	const float EKFGSF_yaw_err_max{0.262f};         ///< 用于检查收敛的复合航向1-标准差不确定性阈值（弧度）

#endif // CONFIG_EKF2_GNSS

	float pos_noaid_noise{10.0f};           ///< 非辅助位置融合的观测噪声（米）

	float heading_innov_gate{2.6f};         ///< 航向融合创新一致性门限大小（标准差）
	float mag_heading_noise{3.0e-1f};       ///< 用于简单航向融合的测量噪声（弧度）

#if defined(CONFIG_EKF2_MAGNETOMETER)
	float mag_delay_ms {0.0f};              ///< 磁力计测量相对于IMU的延迟（毫秒）

	float mage_p_noise{1.0e-3f};            ///< 地球磁场预测的过程噪声（高斯/秒）
	float magb_p_noise{1.0e-4f};            ///< 机体磁场预测的过程噪声（高斯/秒）

	// 磁力计融合
	float mag_noise{5.0e-2f};               ///< 用于三轴磁力计融合的测量噪声（高斯）
	float mag_declination_deg{0.0f};        ///< 磁偏角（度）
	float mag_innov_gate{3.0f};             ///< 磁力计融合创新一致性门限大小（标准差）
	int32_t mag_declination_source{3};      ///< 用于控制偏角数据处理的位掩码
	int32_t mag_fusion_type{0};             ///< 用于指定磁力计融合类型的整数
	float mag_acc_gate{0.5f};               ///< 在自动选择模式下，当机动加速度低于此值时，将使用航向融合（米/秒²）

	// 如果可能，计算合成磁力计Z值
	int32_t synthesize_mag_z{0};            ///< 合成磁力计Z值的标志
	int32_t mag_check{0};                    ///< 磁力计检查标志
	float mag_check_strength_tolerance_gs{0.2f}; ///< 磁力计检查强度容忍度（g）
	float mag_check_inclination_tolerance_deg{20.f}; ///< 磁力计检查倾斜容忍度（度）
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	// 空速融合
	float airspeed_delay_ms{100.0f};        ///< 空速测量相对于IMU的延迟（毫秒）
	float tas_innov_gate{5.0f};             ///< 真空速创新一致性门限大小（标准差）
	float eas_noise{1.4f};                  ///< EAS测量噪声标准差，用于空速融合（米/秒）
	float arsp_thr{2.0f};                   ///< 空速融合阈值。值为零将停用空速融合
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// 合成侧滑融合
	int32_t beta_fusion_enabled{0};         ///< 侧滑融合启用标志
	float beta_innov_gate{5.0f};            ///< 合成侧滑创新一致性门限大小（标准差）
	float beta_noise{0.3f};                 ///< 合成侧滑噪声（弧度）
	const float beta_avg_ft_us{150000.0f};  ///< 合成侧滑测量之间的平均时间（微秒）
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN)
	float terrain_p_noise {5.0f};           ///< 地形偏移的过程噪声（米/秒）
	float terrain_gradient{0.5f};           ///< 用于估计因位置变化而导致的过程噪声的地形梯度（米/米）
	const float terrain_timeout{10.f};      ///< 在重置地形估计之前，底部距离测量无效的最大时间（秒）
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
	float rng_gnd_clearance {0.1f};         ///< 地面时范围的最小有效值（米）
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 距离传感器融合
	int32_t rng_ctrl{static_cast<int32_t>(RngCtrl::CONDITIONAL)}; ///< 距离传感器控制标志

	float range_delay_ms{5.0f};             ///< 距离传感器测量相对于IMU的延迟（毫秒）
	float range_noise{0.1f};                ///< 距离传感器测量的观测噪声（米）
	float range_innov_gate{5.0f};           ///< 距离传感器融合创新一致性门限大小（标准差）
	float rng_sens_pitch{0.0f};             ///< 距离传感器的俯仰偏移（弧度）。当偏移为零时，传感器沿Z轴指向外部。正旋转是相对于Y轴的右手旋转。
	float range_noise_scaler{0.0f};         ///< 从范围测量到噪声的缩放（米/米）
	float max_hagl_for_range_aid{5.0f};     ///< 允许使用距离传感器作为高度源的最大地面高度（如果rng_control == 1）
	float max_vel_for_range_aid{1.0f};      ///< 允许使用距离传感器作为高度源的最大地面速度（如果rng_control == 1）
	float range_aid_innov_gate{1.0f};       ///< 用于距离辅助融合的创新一致性检查的门大小
	float range_valid_quality_s{1.0f};      ///< 报告的距离传感器信号质量需要非零的最小持续时间，以被声明为有效（秒）
	float range_cos_max_tilt{0.7071f};      ///< 允许使用距离传感器和光流数据的最大倾斜角的余弦值
	float range_kin_consistency_gate{1.0f}; ///< 距离传感器运动一致性检查的门大小
	float rng_fog{0.f};                 	///< 被阻挡的距离传感器测量的最大距离（雾，污垢）（米）

	Vector3f rng_pos_body{};                ///< 距离传感器在机体坐标系中的xyz位置（米）
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 视觉位置融合
	int32_t ev_ctrl{0};                     ///< 视觉控制标志
	float ev_delay_ms{175.0f};              ///< 离线视觉测量相对于IMU的延迟（毫秒）

	float ev_vel_noise{0.1f};               ///< EV速度融合的最小允许观测噪声（米/秒）
	float ev_pos_noise{0.1f};               ///< EV位置融合的最小允许观测噪声（米）
	float ev_att_noise{0.1f};               ///< EV姿态融合的最小允许观测噪声（弧度/秒）
	int32_t ev_quality_minimum{0};          ///< 视觉的最小可接受质量整数
	float ev_vel_innov_gate{3.0f};          ///< 视觉速度融合创新一致性门限大小（标准差）
	float ev_pos_innov_gate{5.0f};          ///< 视觉位置融合创新一致性门限大小（标准差）
	float ev_hgt_bias_nsd{0.13f};           ///< 视觉高度偏差估计的过程噪声（米/秒/√Hz）

	Vector3f ev_pos_body{};                 ///< VI传感器焦点在机体坐标系中的xyz位置（米）
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// 重力融合
	float gravity_noise{1.0f};              ///< 加速度计测量的高斯噪声（米/秒²）
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	int32_t flow_ctrl {0};                  ///< 光流控制标志
	int32_t flow_gyro_src {static_cast<int32_t>(FlowGyroSource::Auto)}; ///< 光流陀螺仪源选择
	float flow_delay_ms{5.0f};              ///< 光流测量相对于IMU的延迟（毫秒） - 这是光流积分间隔的中间值

	// 光流融合
	float flow_noise{0.15f};                ///< 光流LOS速率测量的观测噪声（弧度/秒）
	float flow_noise_qual_min{0.5f};        ///< 当光流传感器质量处于最低可用时的观测噪声（弧度/秒）
	int32_t flow_qual_min{1};               ///< 光流传感器的最小可接受质量整数
	int32_t flow_qual_min_gnd{0};           ///< 当在地面时光流传感器的最小可接受质量整数
	float flow_innov_gate{3.0f};            ///< 光流融合创新一致性门限大小（标准差）

	Vector3f flow_pos_body{};               ///< 光流传感器焦点在机体坐标系中的xyz位置（米）
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// 传感器在机体坐标系中的XYZ偏移（米）
	Vector3f imu_pos_body{};                ///< IMU在机体坐标系中的xyz位置（米）

	// 加速度偏差学习控制
	float acc_bias_lim{0.4f};               ///< 最大加速度偏差幅度（米/秒²）
	float acc_bias_learn_acc_lim{25.0f};    ///< 如果IMU加速度向量的幅度大于此值，则禁用学习（米/秒²）
	float acc_bias_learn_gyr_lim{3.0f};     ///< 如果IMU角速率向量的幅度大于此值，则禁用学习（弧度/秒）
	float acc_bias_learn_tc{0.5f};          ///< 用于控制施加于加速度和陀螺仪幅度的衰减包络滤波器的时间常数（秒）

	float gyro_bias_lim{0.4f};              ///< 最大陀螺仪偏差幅度（弧度/秒）

	const unsigned reset_timeout_max{7'000'000};      ///< 在尝试重置状态到测量值或更改_control_status之前，允许的最大水平惯性死算时间（微秒）
	const unsigned no_aid_timeout_max{1'000'000};     ///< 从最后一次融合约束水平速度漂移的测量开始的最大时间，超过此时间EKF将判断传感器不再对辅助有贡献（微秒）
	const unsigned hgt_fusion_timeout_max{5'000'000}; ///< 允许高度融合失败的最大时间，超过此时间将尝试重置或停止融合辅助（微秒）

	int32_t valid_timeout_max{5'000'000};     ///< 在惯性死算期间花费的时间，超过此时间估计器将报告状态估计为无效（微秒）

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 多旋翼拖曳特定力融合
	int32_t drag_ctrl{0};                    ///< 拖曳控制标志
	float drag_noise{2.5f};                 ///< 拖曳特定力测量的观测噪声方差（米/秒²）²
	float bcoef_x{100.0f};                  ///< X轴的阻力体拖曳系数（千克/米²）
	float bcoef_y{100.0f};                  ///< Y轴的阻力体拖曳系数（千克/米²）
	float mcoef{0.1f};                      ///< X轴和Y轴的转子动量拖曳系数（1/秒）
#endif // CONFIG_EKF2_DRAG_FUSION
	// 加速度误差检测和缓解控制（IMU剪切）
	const float vert_innov_test_lim{3.0f};          ///< 触发垂直加速度故障之前，允许的垂直速度/位置创新的标准差数量
	const float vert_innov_test_min{1.0f};          ///< 触发垂直加速度故障所需的最小标准差数量
	const int bad_acc_reset_delay_us{500000};       ///< 垂直位置和速度创新测试必须连续失败的时间，超过此时间状态将被重置（微秒）

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度融合
	float auxvel_delay_ms{5.0f};            ///< 辅助速度测量相对于IMU的延迟（毫秒）
	const float auxvel_noise{0.5f};         ///< 最小观测噪声，如果报告的噪声更大则使用报告的噪声（米/秒）
	const float auxvel_gate{5.0f};          ///< 速度融合创新一致性门限大小（标准差）
#endif // CONFIG_EKF2_AUXVEL

};

union fault_status_u {
	struct {
		bool bad_mag_x         : 1; ///< 0 - 如果磁力计X轴的融合遇到数值错误，则为真
		bool bad_mag_y         : 1; ///< 1 - 如果磁力计Y轴的融合遇到数值错误，则为真
		bool bad_mag_z         : 1; ///< 2 - 如果磁力计Z轴的融合遇到数值错误，则为真
		bool bad_hdg           : 1; ///< 3 - 如果航向角的融合遇到数值错误，则为真
		bool bad_mag_decl      : 1; ///< 4 - 如果磁偏角的融合遇到数值错误，则为真
		bool bad_airspeed      : 1; ///< 5 - 如果空速的融合遇到数值错误，则为真
		bool bad_sideslip      : 1; ///< 6 - 如果合成侧滑约束的融合遇到数值错误，则为真
		bool bad_optflow_X     : 1; ///< 7 - 如果光流X轴的融合遇到数值错误，则为真
		bool bad_optflow_Y     : 1; ///< 8 - 如果光流Y轴的融合遇到数值错误，则为真
		bool __UNUSED          : 1; ///< 9 - 未使用
		bool bad_acc_vertical  : 1; ///< 10 - 如果检测到不良的垂直加速度计数据，则为真
		bool bad_acc_clipping  : 1; ///< 11 - 如果增量速度数据包含剪切（不对称限制），则为真
	} flags;
	uint32_t value;
};

// 定义用于传达创新测试失败的结构
union innovation_fault_status_u {
	struct {
		bool reject_hor_vel   : 1; ///< 0 - 如果水平速度观测被拒绝，则为真
		bool reject_ver_vel   : 1; ///< 1 - 如果垂直速度观测被拒绝，则为真
		bool reject_hor_pos   : 1; ///< 2 - 如果水平位置观测被拒绝，则为真
		bool reject_ver_pos   : 1; ///< 3 - 如果垂直位置观测被拒绝，则为真
		bool reject_mag_x     : 1; ///< 4 - 如果X磁力计观测被拒绝，则为真
		bool reject_mag_y     : 1; ///< 5 - 如果Y磁力计观测被拒绝，则为真
		bool reject_mag_z     : 1; ///< 6 - 如果Z磁力计观测被拒绝，则为真
		bool reject_yaw       : 1; ///< 7 - 如果航向观测被拒绝，则为真
		bool reject_airspeed  : 1; ///< 8 - 如果空速观测被拒绝，则为真
		bool reject_sideslip  : 1; ///< 9 - 如果合成侧滑观测被拒绝，则为真
		bool reject_hagl      : 1; ///< 10 - 未使用
		bool reject_optflow_X : 1; ///< 11 - 如果X光流观测被拒绝，则为真
		bool reject_optflow_Y : 1; ///< 12 - 如果Y光流观测被拒绝，则为真
	} flags;
	uint16_t value;
};

// 发布各种GPS质量检查的状态
union gps_check_fail_status_u {
	struct {
		uint16_t fix    : 1; ///< 0 - 如果修正类型不足（没有3D解），则为真
		uint16_t nsats  : 1; ///< 1 - 如果使用的卫星数量不足，则为真
		uint16_t pdop   : 1; ///< 2 - 如果位置精度衰减不足，则为真
		uint16_t hacc   : 1; ///< 3 - 如果报告的水平精度不足，则为真
		uint16_t vacc   : 1; ///< 4 - 如果报告的垂直精度不足，则为真
		uint16_t sacc   : 1; ///< 5 - 如果报告的速度精度不足，则为真
		uint16_t hdrift : 1; ///< 6 - 如果水平漂移过大（只能在地面静止时使用），则为真
		uint16_t vdrift : 1; ///< 7 - 如果垂直漂移过大（只能在地面静止时使用），则为真
		uint16_t hspeed : 1; ///< 8 - 如果水平速度过大（只能在地面静止时使用），则为真
		uint16_t vspeed : 1; ///< 9 - 如果垂直速度误差过大，则为真
		uint16_t spoofed: 1; ///< 10 - 如果GNSS数据被伪造，则为真
	} flags;
	uint16_t value;
};
// 位掩码，包含滤波器控制状态
union filter_control_status_u {
	struct {
		uint64_t tilt_align              : 1; ///< 0 - 如果滤波器的倾斜对齐完成，则为真
		uint64_t yaw_align               : 1; ///< 1 - 如果滤波器的航向对齐完成，则为真
		uint64_t gps                     : 1; ///< 2 - 如果打算融合GPS测量数据，则为真
		uint64_t opt_flow                : 1; ///< 3 - 如果打算融合光流测量数据，则为真
		uint64_t mag_hdg                 : 1; ///< 4 - 如果打算进行简单的磁航向融合，则为真
		uint64_t mag_3D                  : 1; ///< 5 - 如果打算融合三轴磁力计测量数据，则为真
		uint64_t mag_dec                 : 1; ///< 6 - 如果打算融合合成的磁偏角测量数据，则为真
		uint64_t in_air                  : 1; ///< 7 - 如果飞行器处于空中，则为真
		uint64_t wind                    : 1; ///< 8 - 如果正在估计风速，则为真
		uint64_t baro_hgt                : 1; ///< 9 - 如果正在融合气压高度数据，则为真
		uint64_t rng_hgt                 : 1; ///< 10 - 如果正在融合测距仪数据以辅助高度估计，则为真
		uint64_t gps_hgt                 : 1; ///< 11 - 如果正在融合GPS高度数据，则为真
		uint64_t ev_pos                  : 1; ///< 12 - 如果打算融合来自外部视觉的位置信息，则为真
		uint64_t ev_yaw                  : 1; ///< 13 - 如果打算融合来自外部视觉的航向数据，则为真
		uint64_t ev_hgt                  : 1; ///< 14 - 如果打算融合来自外部视觉的高度数据，则为真
		uint64_t fuse_beta               : 1; ///< 15 - 如果正在融合合成的侧滑测量数据，则为真
		uint64_t mag_field_disturbed     : 1; ///< 16 - 如果磁场强度与预期不符，则为真
		uint64_t fixed_wing              : 1; ///< 17 - 如果飞行器以固定翼模式操作，则为真
		uint64_t mag_fault               : 1; ///< 18 - 如果磁力计被声明为故障并不再使用，则为真
		uint64_t fuse_aspd               : 1; ///< 19 - 如果正在融合空速测量数据，则为真
		uint64_t gnd_effect              : 1; ///< 20 - 如果正在激活地面效应引起的静压上升保护，则为真
		uint64_t rng_stuck               : 1; ///< 21 - 如果测距仪数据在10秒内未准备好且新测距值变化不足，则为真
		uint64_t gnss_yaw                 : 1; ///< 22 - 如果打算融合来自GPS接收器的航向（非地面航向）数据，则为真
		uint64_t mag_aligned_in_flight   : 1; ///< 23 - 如果在飞行中完成了磁场对齐，则为真
		uint64_t ev_vel                  : 1; ///< 24 - 如果打算融合来自外部视觉的局部速度数据，则为真
		uint64_t synthetic_mag_z         : 1; ///< 25 - 如果正在使用合成的磁力计Z分量测量，则为真
		uint64_t vehicle_at_rest         : 1; ///< 26 - 如果飞行器处于静止状态，则为真
		uint64_t gnss_yaw_fault           : 1; ///< 27 - 如果GNSS航向被声明为故障并不再使用，则为真
		uint64_t rng_fault               : 1; ///< 28 - 如果测距仪被声明为故障并不再使用，则为真
		uint64_t inertial_dead_reckoning : 1; ///< 29 - 如果不再融合约束水平速度漂移的测量数据，则为真
		uint64_t wind_dead_reckoning     : 1; ///< 30 - 如果导航依赖于相对风速测量，则为真
		uint64_t rng_kin_consistent      : 1; ///< 31 - 如果测距仪的运动学一致性检查通过，则为真
		uint64_t fake_pos                : 1; ///< 32 - 如果正在融合虚假位置测量数据，则为真
		uint64_t fake_hgt                : 1; ///< 33 - 如果正在融合虚假高度测量数据，则为真
		uint64_t gravity_vector          : 1; ///< 34 - 如果正在融合重力向量测量数据，则为真
		uint64_t mag                     : 1; ///< 35 - 如果打算融合三轴磁力计测量数据（仅磁状态），则为真
		uint64_t ev_yaw_fault            : 1; ///< 36 - 如果外部视觉的航向被声明为故障并不再使用，则为真
		uint64_t mag_heading_consistent  : 1; ///< 37 - 如果从磁数据获得的航向被声明为与滤波器一致，则为真
		uint64_t aux_gpos                : 1; ///< 38 - 如果打算融合辅助全球位置测量数据，则为真
		uint64_t rng_terrain             : 1; ///< 39 - 如果正在融合测距仪数据以辅助地形估计，则为真
		uint64_t opt_flow_terrain        : 1; ///< 40 - 如果正在融合光流数据以辅助地形估计，则为真
		uint64_t valid_fake_pos          : 1; ///< 41 - 如果正在融合有效的常量位置，则为真
		uint64_t constant_pos            : 1; ///< 42 - 如果飞行器处于常量位置，则为真
		uint64_t baro_fault              : 1; ///< 43 - 如果气压计被声明为故障并不再使用，则为真

	} flags;
	uint64_t value;
};

// 定义用于传达信息事件的结构
union information_event_status_u {
	struct {
		bool gps_checks_passed          : 1; ///< 0 - 如果GPS质量检查通过，则为真
		bool reset_vel_to_gps           : 1; ///< 1 - 如果速度状态被重置为GPS测量值，则为真
		bool reset_vel_to_flow          : 1; ///< 2 - 如果速度状态被重置为光流测量值，则为真
		bool reset_vel_to_vision        : 1; ///< 3 - 如果速度状态被重置为视觉系统测量值，则为真
		bool reset_vel_to_zero          : 1; ///< 4 - 如果速度状态被重置为零，则为真
		bool reset_pos_to_last_known    : 1; ///< 5 - 如果位置状态被重置为最后已知位置，则为真
		bool reset_pos_to_gps           : 1; ///< 6 - 如果位置状态被重置为GPS测量值，则为真
		bool reset_pos_to_vision        : 1; ///< 7 - 如果位置状态被重置为视觉系统测量值，则为真
		bool starting_gps_fusion        : 1; ///< 8 - 如果滤波器开始使用GPS测量来校正状态估计，则为真
		bool starting_vision_pos_fusion : 1; ///< 9 - 如果滤波器开始使用视觉系统位置测量来校正状态估计，则为真
		bool starting_vision_vel_fusion : 1; ///< 10 - 如果滤波器开始使用视觉系统速度测量来校正状态估计，则为真
		bool starting_vision_yaw_fusion : 1; ///< 11 - 如果滤波器开始使用视觉系统航向测量来校正状态估计，则为真
		bool yaw_aligned_to_imu_gps     : 1; ///< 12 - 如果滤波器将航向重置为IMU和GPS数据推导的估计值，则为真
		bool reset_hgt_to_baro          : 1; ///< 13 - 如果垂直位置状态被重置为气压计测量值，则为真
		bool reset_hgt_to_gps           : 1; ///< 14 - 如果垂直位置状态被重置为GPS测量值，则为真
		bool reset_hgt_to_rng           : 1; ///< 15 - 如果垂直位置状态被重置为测距仪测量值，则为真
		bool reset_hgt_to_ev            : 1; ///< 16 - 如果垂直位置状态被重置为外部视觉测量值，则为真
		bool reset_pos_to_ext_obs       : 1; ///< 17 - 如果水平位置被重置为外部观测值（在惯性导航中），则为真
		bool reset_wind_to_ext_obs      : 1; ///< 18 - 如果风状态被重置为外部观测值，则为真
	} flags;
	uint32_t value;
};

}
#endif // !EKF_COMMON_H
