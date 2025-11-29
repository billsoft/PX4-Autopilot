#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

using namespace time_literals;

class DualIMUFusion : public ModuleBase<DualIMUFusion>, public ModuleParams, public px4::ScheduledWorkItem {
public:
    DualIMUFusion();
    ~DualIMUFusion() override;
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    bool init();
    int print_status() override;

private:
    void Run() override;

    uORB::Subscription _sensor_accel_sub0{ORB_ID(sensor_accel), 0};
    uORB::Subscription _sensor_accel_sub1{ORB_ID(sensor_accel), 1};
    uORB::Subscription _sensor_gyro_sub0{ORB_ID(sensor_gyro), 0};
    uORB::Subscription _sensor_gyro_sub1{ORB_ID(sensor_gyro), 1};
    uORB::Subscription _sensor_mag_sub{ORB_ID(sensor_mag)};

    uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

    matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
    uint64_t _last_ts{0};
};

DualIMUFusion::DualIMUFusion() : ModuleParams(nullptr), px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}

DualIMUFusion::~DualIMUFusion() { perf_free(_loop_perf); }

bool DualIMUFusion::init() { return true; }

int DualIMUFusion::print_status() { return 0; }

void DualIMUFusion::Run() {
    if (should_exit()) { exit_and_cleanup(); return; }
    perf_begin(_loop_perf);

    sensor_accel_s a0{}; sensor_accel_s a1{}; sensor_gyro_s g0{}; sensor_gyro_s g1{}; sensor_mag_s m{};
    bool ua0 = _sensor_accel_sub0.update(&a0);
    bool ua1 = _sensor_accel_sub1.update(&a1);
    bool ug0 = _sensor_gyro_sub0.update(&g0);
    bool ug1 = _sensor_gyro_sub1.update(&g1);
    _sensor_mag_sub.copy(&m);

    if (!(ua0 && ua1 && ug0 && ug1)) { perf_end(_loop_perf); return; }

    uint64_t ts = a0.timestamp;
    float dt = 0.002f;
    if (_last_ts > 0 && ts > _last_ts) {
        dt = (ts - _last_ts) * 1e-6f;
        if (dt < 0.0001f || dt > 0.05f) dt = 0.002f;
    }
    _last_ts = ts;

    matrix::Vector3f acc0(a0.x, a0.y, a0.z);
    matrix::Vector3f acc1(a1.x, a1.y, a1.z);
    matrix::Vector3f gyr0(g0.x, g0.y, g0.z);
    matrix::Vector3f gyr1(g1.x, g1.y, g1.z);
    matrix::Vector3f acc = (acc0 + acc1) * 0.5f;
    matrix::Vector3f gyr = (gyr0 + gyr1) * 0.5f;

    matrix::Vector3f axis = gyr * dt;
    matrix::AxisAnglef aa(axis);
    matrix::Quatf dq(aa);  // 正确: AxisAnglef转换为Quatf
    _q = _q * dq;
    _q.normalize();

    if (acc.norm() > 0.1f) {
        matrix::Vector3f an = acc.normalized();
        matrix::Dcmf R(_q);
        matrix::Vector3f z_body = R.col(2);
        matrix::Vector3f err = an.cross(z_body);
        matrix::Vector3f corr = err * 0.5f * dt;
        matrix::AxisAnglef aa_corr(corr);
        matrix::Quatf qc(aa_corr);  // 正确: AxisAnglef转换为Quatf
        _q = qc * _q;
        _q.normalize();
    }

    vehicle_attitude_s att{};
    att.timestamp_sample = ts;
    _q.copyTo(att.q);
    att.timestamp = hrt_absolute_time();
    _attitude_pub.publish(att);

    vehicle_angular_velocity_s w{};
    w.timestamp_sample = ts;
    w.xyz[0] = gyr(0); w.xyz[1] = gyr(1); w.xyz[2] = gyr(2);
    w.timestamp = hrt_absolute_time();
    _angular_velocity_pub.publish(w);

    perf_end(_loop_perf);
}

int DualIMUFusion::task_spawn(int argc, char *argv[]) {
    DualIMUFusion *instance = new DualIMUFusion();
    if (!instance) return PX4_ERROR;
    _object.store(instance);
    _task_id = task_id_is_work_queue;
    if (instance->init()) { instance->ScheduleOnInterval(2000_us); return PX4_OK; }
    delete instance; _object.store(nullptr); _task_id = -1; return PX4_ERROR;
}

int DualIMUFusion::custom_command(int argc, char *argv[]) {
    if (argc >= 1 && !strcmp(argv[0], "start")) {
        return task_spawn(argc, argv);
    }
    return print_usage();
}

int DualIMUFusion::print_usage(const char *reason) {
    PRINT_MODULE_USAGE_NAME("dual_imu_fusion", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    return 0;
}

extern "C" __EXPORT int dual_imu_fusion_main(int argc, char *argv[]) {
    return DualIMUFusion::main(argc, argv);
}
