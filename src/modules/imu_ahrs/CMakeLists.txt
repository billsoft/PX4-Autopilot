if(CONFIG_MODULES_IMU_AHRS)
    px4_add_module(
        MODULE modules__imu_ahrs
        MAIN imu_ahrs
        STACK_MAIN 3072
        COMPILE_FLAGS
            ${MAX_CUSTOM_OPT_LEVEL}
            -Wno-cast-align     # 禁用类型对齐警告
            -O2                 # 优化等级
        SRCS
            ImuAhrs.cpp
            core/IMUFilter.cpp
            core/YawDriftCompensator.cpp
            math/QuaternionMath.cpp
            math/EulerMath.cpp
            imu_ahrs_params.c
        INCLUDES
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/core
            ${CMAKE_CURRENT_SOURCE_DIR}/math
        DEPENDS
            px4_work_queue
            mathlib
            modules__uORB
            drivers__device
            px4_platform_common
            matrix
            version
            parameters
            perf
            systemlib
        UNITY_BUILD ON
    )
endif()
