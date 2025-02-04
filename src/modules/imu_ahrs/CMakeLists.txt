px4_add_module(
    MODULE modules__imu_ahrs
    MAIN imu_ahrs
    STACK_MAIN 3072                  # 建议增加到 3072，因为涉及矩阵运算
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}
    SRCS
        ImuAhrs.cpp
        ImuAhrs.hpp
        imu_ahrs_params.c           # 需要添加参数文件
    DEPENDS
        px4_work_queue
        mathlib
        modules__uORB
        # 建议添加以下依赖
        drivers__device
        px4_platform_common
        matrix                      # 矩阵运算库
    UNITY_BUILD ON
)
