px4_add_module(
    MODULE modules__dual_imu_fusion
    MAIN dual_imu_fusion
    SRCS
        DualIMUFusion.cpp
    DEPENDS
        px4_work_queue
        uORB
)
