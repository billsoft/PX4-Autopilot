px4_add_module(
	MODULE modules__imu_ahrs
	MAIN imu_ahrs
	SRCS
		ImuAhrs.cpp
	DEPENDS
		px4_work_queue
		mathlib
)
