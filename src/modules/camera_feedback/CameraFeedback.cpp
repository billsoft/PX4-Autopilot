/****************************************************************************
 *
 *   版权所有 (c) 2017-2019 PX4 开发团队。保留所有权利。
 *
 * 以源代码和二进制形式进行再分发和使用，无论是否修改，均可在满足以下条件的情况下进行：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 二进制形式的再分发必须在分发的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。
 * 3. 未经特定的事先书面许可，PX4的名称或其贡献者的名称不得用于支持或推广基于本软件的产品。
 *
 * 本软件由版权持有者和贡献者提供，"按原样"提供，任何明示或暗示的担保，包括但不限于对适销性和特定用途的适用性的暗示担保均被否认。在任何情况下，版权持有者或贡献者均不对因使用本软件而导致的任何直接、间接、附带、特殊、示范性或后果性损害（包括但不限于采购替代商品或服务、使用损失、数据或利润损失或业务中断）承担责任，无论是基于合同、严格责任还是侵权（包括过失或其他原因），即使已被告知可能发生此类损害。
 *
 ****************************************************************************/

#include "CameraFeedback.hpp" // 包含CameraFeedback类的头文件

using namespace time_literals; // 使用时间字面量命名空间

// CameraFeedback类的构造函数
CameraFeedback::CameraFeedback() :
	ModuleParams(nullptr), // 初始化模块参数为nullptr
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default) // 初始化工作项，设置模块名称和默认高优先级工作队列配置
{
	_p_cam_cap_fback = param_find("CAM_CAP_FBACK"); // 查找相机捕获反馈参数

	if (_p_cam_cap_fback != PARAM_INVALID) { // 如果参数有效
		param_get(_p_cam_cap_fback, (int32_t *)&_cam_cap_fback); // 获取参数值并存储
	}
}

// 初始化函数
bool
CameraFeedback::init()
{
	if (!_trigger_sub.registerCallback()) { // 注册触发器回调
		PX4_ERR("callback registration failed"); // 如果注册失败，输出错误信息
		return false; // 返回false表示初始化失败
	}

	_capture_pub.advertise(); // 广播捕获发布者

	return true; // 返回true表示初始化成功
}

// 运行函数
void
CameraFeedback::Run()
{
	if (should_exit()) { // 检查是否应该退出
		_trigger_sub.unregisterCallback(); // 注销触发器回调
		exit_and_cleanup(); // 执行退出和清理操作
		return; // 返回
	}

	camera_trigger_s trig{}; // 定义相机触发结构体

	while (_trigger_sub.update(&trig)) { // 更新触发器订阅

		// 更新地理标记订阅
		vehicle_global_position_s gpos{}; // 定义车辆全球位置结构体
		_gpos_sub.copy(&gpos); // 复制全球位置数据

		vehicle_attitude_s att{}; // 定义车辆姿态结构体
		_att_sub.copy(&att); // 复制姿态数据

		if (trig.timestamp == 0 || // 如果触发时间戳无效
		    gpos.timestamp == 0 || // 如果全球位置时间戳无效
		    att.timestamp == 0) { // 如果姿态时间戳无效

			// 在我们拥有有效数据之前拒绝处理
			continue; // 继续下一次循环
		}

		if ((_cam_cap_fback >= 1) && !trig.feedback) { // 如果启用了相机捕获反馈且触发不是反馈
			// 忽略非反馈的触发
			continue; // 继续下一次循环
		}

		camera_capture_s capture{}; // 定义相机捕获结构体

		// 填充时间戳
		capture.timestamp = trig.timestamp; // 设置捕获时间戳
		capture.timestamp_utc = trig.timestamp_utc; // 设置UTC时间戳

		// 填充图像序列
		capture.seq = trig.seq; // 设置图像序列号

		// 填充位置信息
		capture.lat = gpos.lat; // 设置纬度
		capture.lon = gpos.lon; // 设置经度
		capture.alt = gpos.alt; // 设置高度

		if (gpos.terrain_alt_valid) { // 如果地形高度有效
			capture.ground_distance = gpos.alt - gpos.terrain_alt; // 计算地面距离

		} else {
			capture.ground_distance = -1.0f; // 如果无效，设置为-1.0f
		}

		// 填充姿态数据
		gimbal_device_attitude_status_s gimbal{}; // 定义云台设备姿态状态结构体

		if (_gimbal_sub.copy(&gimbal) && (hrt_elapsed_time(&gimbal.timestamp) < 1_s)) { // 如果成功复制云台数据且时间戳有效
			if (gimbal.device_flags & gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK) { // 如果云台锁定偏航
				// 云台偏航角是相对于北方的绝对角度
				capture.q[0] = gimbal.q[0]; // 设置四元数的第一个分量
				capture.q[1] = gimbal.q[1]; // 设置四元数的第二个分量
				capture.q[2] = gimbal.q[2]; // 设置四元数的第三个分量
				capture.q[3] = gimbal.q[3]; // 设置四元数的第四个分量

			} else {
				// 云台四元数框架在地球框架中旋转，使得x轴指向前方（相对于车辆的偏航）。从车辆姿态获取航向并与云台方向结合。
				const matrix::Eulerf euler_vehicle(matrix::Quatf(att.q)); // 将车辆姿态四元数转换为欧拉角
				const matrix::Quatf q_heading(matrix::Eulerf(0.0f, 0.0f, euler_vehicle(2))); // 获取航向四元数
				matrix::Quatf q_gimbal(gimbal.q); // 获取云台四元数
				q_gimbal = q_heading * q_gimbal; // 将航向与云台四元数相乘，得到最终的云台四元数

				capture.q[0] = q_gimbal(0); // 设置捕获四元数的第一个分量
				capture.q[1] = q_gimbal(1); // 设置捕获四元数的第二个分量
				capture.q[2] = q_gimbal(2); // 设置捕获四元数的第三个分量
				capture.q[3] = q_gimbal(3); // 设置捕获四元数的第四个分量
			}

		} else {
			// 如果没有云台方向，使用车辆姿态
			capture.q[0] = att.q[0]; // 设置捕获四元数的第一个分量
			capture.q[1] = att.q[1]; // 设置捕获四元数的第二个分量
			capture.q[2] = att.q[2]; // 设置捕获四元数的第三个分量
			capture.q[3] = att.q[3]; // 设置捕获四元数的第四个分量
		}

		capture.result = 1; // 设置捕获结果为1，表示成功

		_capture_pub.publish(capture); // 发布捕获数据
	}
}

// 任务生成函数
int
CameraFeedback::task_spawn(int argc, char *argv[])
{
	CameraFeedback *instance = new CameraFeedback(); // 创建CameraFeedback实例

	if (instance) { // 如果实例创建成功
		_object.store(instance); // 存储实例
		_task_id = task_id_is_work_queue; // 设置任务ID为工作队列ID

		if (instance->init()) { // 如果初始化成功
			return PX4_OK; // 返回成功
		}

	} else {
		PX4_ERR("alloc failed"); // 如果实例创建失败，输出错误信息
	}

	delete instance; // 删除实例
	_object.store(nullptr); // 清空存储的对象
	_task_id = -1; // 设置任务ID为-1，表示无效

	return PX4_ERROR; // 返回错误
}

// 自定义命令函数
int
CameraFeedback::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command"); // 返回未知命令的使用信息
}

// 打印使用信息函数
int
CameraFeedback::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason); // 如果有原因，输出警告信息
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述

camera_feedback模块在图像捕获被触发时发布`CameraCapture` UORB主题。

如果启用了相机捕获，则从相机捕获引脚发布触发信息；
否则发布相机被命令触发时的触发信息（来自`camera_trigger`模块）。

然后在`CameraCapture`更新后发出`CAMERA_IMAGE_CAPTURED`消息（由流代码发出）。
`CameraCapture`主题也会被记录并可用于地理标记。

### 实现

`CameraTrigger`主题由`camera_trigger`模块发布（`feedback`字段设置为`false`）
当图像捕获被触发时，并且如果相机捕获引脚被激活，可能也会由`camera_capture`驱动程序发布（`feedback`字段设置为`true`）。

`camera_feedback`模块订阅`CameraTrigger`。
如果启用了相机捕获，则丢弃来自`camera_trigger`模块的主题。
对于未被丢弃的主题，它创建一个`CameraCapture`主题，包含来自`CameraTrigger`的时间戳信息和来自车辆的位置。

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("camera_feedback", "system"); // 打印模块名称和类型
	PRINT_MODULE_USAGE_COMMAND("start"); // 打印启动命令
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); // 打印默认命令

	return 0; // 返回0
}

// 主函数
extern "C" __EXPORT int camera_feedback_main(int argc, char *argv[])
{
	return CameraFeedback::main(argc, argv); // 调用CameraFeedback的主函数
}
