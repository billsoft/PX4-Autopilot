/****************************************************************************
 * CMakeLists.txt - imu_ahrs 模块编译配置文件
 *
 * 该文件通过 px4_add_module() 宏将 imu_ahrs 模块注册到 PX4 构建系统中，
 * 指定了模块名称、入口函数、任务栈大小、编译选项、源文件列表以及依赖的库。
 ****************************************************************************/

px4_add_module(
    MODULE modules__imu_ahrs         # 模块名称，统一使用 "modules__模块名" 格式
    MAIN imu_ahrs                    # 模块主入口函数名称，要求在实现文件中定义 imu_ahrs_main()
    STACK_MAIN 2048                  # 主任务栈大小（单位字节），可根据算法复杂度调整
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}      # 编译选项，通常引用系统宏
    SRCS
        ImuAhrs.cpp                  # 模块主实现文件
        ImuAhrs.hpp                  # 头文件（如果有的话）
    DEPENDS
        px4_work_queue               # 依赖的 PX4 工作队列模块
        mathlib                      # 依赖数学库，用于矩阵与数学运算
        modules__uORB               # uORB 消息依赖
    UNITY_BUILD ON                   # 启用 Unity 构建以提高编译速度
)
