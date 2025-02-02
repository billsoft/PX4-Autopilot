/****************************************************************************
 *
 *   版权所有 (c) 2017-2019 PX4 开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，或有或无修改，均可，前提是满足以下条件：
 *
 * 1. 源代码的再分发必须保留上述版权
 *    声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在
 *    随附的文档和/或其他材料中重现上述版权
 *    声明、此条件列表和以下免责声明。
 * 3. PX4 的名称或其贡献者的名称不得用于支持或推广
 *    从本软件派生的产品，除非事先获得特定的书面许可。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的担保，包括但不限于
 * 对适销性和特定用途的隐含担保均被否认。在任何情况下，版权持有者或贡献者均不对任何直接、间接、
 * 偶然、特殊、示范性或后果性损害（包括但不限于采购替代商品或服务；使用、数据或利润的损失；
 * 或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知
 * 可能发生此类损害。
 *
 ****************************************************************************/

/**
 *
 * 基于相机反馈的在线和离线地理标记功能
 *
 * @作者 Mohammed Kabir <kabir@uasys.io>
 */

#pragma once

#include <lib/mathlib/mathlib.h> // 数学库的头文件，提供数学运算功能
#include <lib/parameters/param.h> // 参数管理的头文件
#include <px4_platform_common/px4_config.h> // PX4 配置的公共头文件
#include <px4_platform_common/defines.h> // 定义常量和宏的公共头文件
#include <px4_platform_common/module.h> // 模块基础类的头文件
#include <px4_platform_common/module_params.h> // 模块参数管理的头文件
#include <px4_platform_common/posix.h> // POSIX 相关的头文件
#include <px4_platform_common/px4_work_queue/WorkItem.hpp> // 工作项的头文件
#include <uORB/Publication.hpp> // 发布消息的头文件
#include <uORB/Subscription.hpp> // 订阅消息的头文件
#include <uORB/SubscriptionCallback.hpp> // 订阅回调的头文件
#include <uORB/topics/camera_capture.h> // 相机捕获主题的头文件
#include <uORB/topics/camera_trigger.h> // 相机触发主题的头文件
#include <uORB/topics/vehicle_attitude.h> // 车辆姿态主题的头文件
#include <uORB/topics/vehicle_global_position.h> // 车辆全球位置主题的头文件
#include <uORB/topics/gimbal_device_attitude_status.h> // 云台设备姿态状态主题的头文件

// CameraFeedback 类，继承自 ModuleBase、ModuleParams 和 WorkItem
class CameraFeedback : public ModuleBase<CameraFeedback>, public ModuleParams, public px4::WorkItem
{
public:
	CameraFeedback(); // 构造函数
	~CameraFeedback() override = default; // 析构函数，使用默认实现

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]); // 任务生成函数，启动模块任务

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]); // 自定义命令处理函数

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr); // 打印使用说明

	bool init(); // 初始化函数，返回初始化是否成功

private:

	void Run() override; // 重写 Run 函数，执行模块的主要逻辑

	// 订阅相机触发消息的回调工作项
	uORB::SubscriptionCallbackWorkItem _trigger_sub{this, ORB_ID(camera_trigger)};

	// 订阅车辆全球位置消息
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	// 订阅车辆姿态消息
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	// 订阅云台设备姿态状态消息
	uORB::Subscription _gimbal_sub{ORB_ID(gimbal_device_attitude_status)};

	// 发布相机捕获消息
	uORB::Publication<camera_capture_s> _capture_pub{ORB_ID(camera_capture)};

	param_t _p_cam_cap_fback; // 相机捕获反馈参数
	int32_t _cam_cap_fback{0}; // 相机捕获反馈值，初始化为0
};
