/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * 该软件的源代码和二进制形式的再分发和使用，均可在以下条件下进行，且不论是否进行修改：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在随附的文档或其他材料中重现上述版权声明、此条件列表和以下免责声明。
 * 3. PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品，除非事先获得书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的保证，包括但不限于适销性和特定用途的适用性均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、惩戒性或后果性损害（包括但不限于采购替代商品或服务；使用、数据或利润的损失；或业务中断）承担责任，无论是基于合同、严格责任或侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

/**
 * @file zero_innovation_heading_update.cpp
 * 控制函数，用于在静止状态或没有其他航向源可用时更新扩展卡尔曼滤波器（EKF）的航向。
 */

#include "ekf.h" // 引入扩展卡尔曼滤波器的头文件

void Ekf::controlZeroInnovationHeadingUpdate()
{
	// 检查是否有航向辅助信息，航向辅助信息包括磁航向、3D磁场、电子航向和GNSS航向
	const bool yaw_aiding = _control_status.flags.mag_hdg || _control_status.flags.mag_3D
				|| _control_status.flags.ev_yaw || _control_status.flags.gnss_yaw;

	// 如果没有航向辅助信息，并且上次航向融合的时间已超时，则进行零创新融合
	if (!yaw_aiding
	    && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {

		// 根据是否倾斜对齐设置观测方差，使用比通常更大的观测方差，但要足够小以将航向方差限制在阈值以下
		const float obs_var = _control_status.flags.tilt_align ? 0.25f : 0.001f;

		// 创建一个一维估计辅助源状态结构体
		estimator_aid_source1d_s aid_src_status{};
		// 获取当前的欧拉航向并赋值给观测值
		aid_src_status.observation = getEulerYaw(_state.quat_nominal);
		// 设置观测方差
		aid_src_status.observation_variance = obs_var;
		// 初始化创新值为0
		aid_src_status.innovation = 0.f;

		VectorState H_YAW; // 定义航向的雅可比矩阵

		// 计算航向创新方差和雅可比矩阵
		computeYawInnovVarAndH(obs_var, aid_src_status.innovation_variance, H_YAW);

		// 如果没有倾斜对齐，或者创新方差与观测方差的差值大于磁航向噪声的平方
		if (!_control_status.flags.tilt_align
		    || (aid_src_status.innovation_variance - obs_var) > sq(_params.mag_heading_noise)) {
			// 航向方差过大，融合伪测量值
			fuseYaw(aid_src_status, H_YAW);
		}
	}
}
