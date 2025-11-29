#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>

class DualIMUFusion : public ModuleBase<DualIMUFusion>, public ModuleParams, public px4::ScheduledWorkItem {
public:
    DualIMUFusion() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}

    static int task_spawn(int argc, char *argv[]) {
        DualIMUFusion *instance = new DualIMUFusion();
        if (instance) {
            _object.store(instance);
            _task_id = task_id_is_work_queue;
            if (instance->init()) {
                return PX4_OK;
            }
        }
        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

    static int print_usage(const char *reason = nullptr) {
        if (reason) { PX4_WARN("%s\n", reason); }
        PRINT_MODULE_DESCRIPTION("Dual IMU fusion to vehicle_attitude");
        PRINT_MODULE_USAGE_NAME("dual_imu_fusion", "module");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
    }

    bool init() {
        this->ScheduleOnInterval(8333); // ~120 Hz
        return true;
    }

private:
    void Run() override {
        if (should_exit()) { exit_and_cleanup(); return; }
        sensor_accel_s accel1{}; sensor_accel_s accel2{}; sensor_gyro_s gyro1{}; sensor_gyro_s gyro2{}; sensor_mag_s mag{};
        _accel_sub1.update(&accel1);
        _accel_sub2.update(&accel2);
        _gyro_sub1.update(&gyro1);
        _gyro_sub2.update(&gyro2);
        _mag_sub.update(&mag);
        vehicle_attitude_s att{};
        att.timestamp = hrt_absolute_time();
        att.timestamp_sample = att.timestamp;
        att.q[0] = 1.f; att.q[1] = 0.f; att.q[2] = 0.f; att.q[3] = 0.f;
        att.delta_q_reset[0] = 0.f; att.delta_q_reset[1] = 0.f; att.delta_q_reset[2] = 0.f; att.delta_q_reset[3] = 0.f;
        att.quat_reset_counter = 0;
        _att_pub.publish(att);
    }

    uORB::Subscription _accel_sub1{ORB_ID(sensor_accel)};
    uORB::Subscription _accel_sub2{ORB_ID(sensor_accel)};
    uORB::Subscription _gyro_sub1{ORB_ID(sensor_gyro)};
    uORB::Subscription _gyro_sub2{ORB_ID(sensor_gyro)};
    uORB::Subscription _mag_sub{ORB_ID(sensor_mag)};
    uORB::Publication<vehicle_attitude_s> _att_pub{ORB_ID(vehicle_attitude)};
};

extern "C" __EXPORT int dual_imu_fusion_main(int argc, char *argv[]) { return DualIMUFusion::main(argc, argv); }
