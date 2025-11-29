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
#include <matrix/math.hpp>

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
        this->ScheduleOnInterval(8333);
        return true;
    }

private:
    void Run() override {
        if (should_exit()) { exit_and_cleanup(); return; }
        sensor_accel_s a1{}; sensor_accel_s a2{}; sensor_gyro_s g1{}; sensor_gyro_s g2{}; sensor_mag_s m{};
        const bool a1_new = _accel_sub1.update(&a1);
        const bool a2_new = _accel_sub2.update(&a2);
        const bool g1_new = _gyro_sub1.update(&g1);
        const bool g2_new = _gyro_sub2.update(&g2);
        const bool m_new  = _mag_sub.update(&m);

        if (!(a1_new && a2_new && g1_new && g2_new)) {
            return;
        }

        const uint64_t ts = hrt_absolute_time();
        const uint64_t ts_a1 = a1.timestamp_sample;
        const uint64_t ts_a2 = a2.timestamp_sample;
        const uint64_t ts_g1 = g1.timestamp_sample;
        const uint64_t ts_g2 = g2.timestamp_sample;
        const uint64_t dt_us = (ts_g1 > _last_ts) ? (ts_g1 - _last_ts) : 8333;
        _last_ts = ts_g1;

        // 同步检查（≤1ms）
        const uint64_t sync_thr_us = 1000;
        if ((ts_a1 > ts_a2 ? ts_a1 - ts_a2 : ts_a2 - ts_a1) > sync_thr_us) return;
        if ((ts_g1 > ts_g2 ? ts_g1 - ts_g2 : ts_g2 - ts_g1) > sync_thr_us) return;

        // 简单低通滤波
        matrix::Vector3f accel1{a1.x, a1.y, a1.z};
        matrix::Vector3f accel2{a2.x, a2.y, a2.z};
        matrix::Vector3f gyro1{g1.x, g1.y, g1.z};
        matrix::Vector3f gyro2{g2.x, g2.y, g2.z};

        const float alpha = _lpf_alpha; // 0..1
        _accel1_filt = _accel1_filt + alpha * (accel1 - _accel1_filt);
        _accel2_filt = _accel2_filt + alpha * (accel2 - _accel2_filt);
        _gyro1_filt  = _gyro1_filt  + alpha * (gyro1  - _gyro1_filt);
        _gyro2_filt  = _gyro2_filt  + alpha * (gyro2  - _gyro2_filt);

        // 二号IMU已通过驱动 -R 参数进行安装方向对齐，不在融合中再翻转轴
        matrix::Vector3f accel2_aligned = _accel2_filt;
        matrix::Vector3f gyro2_aligned  = _gyro2_filt;

        // 差噪与降噪
        matrix::Vector3f noise_accel = accel2_aligned - _accel1_filt;
        matrix::Vector3f noise_gyro  = gyro2_aligned  - _gyro1_filt;
        matrix::Vector3f accel1_denoised = accel1 - _noise_gain * noise_accel;
        matrix::Vector3f gyro1_denoised  = gyro1  - _noise_gain * noise_gyro;

        // 互补/Mahony融合
        const float dt = (float)dt_us * 1e-6f;
        if (dt <= 0.f || dt > 0.1f) return;

        // 初始化姿态（首帧用加速度+磁力计）
        if (!_initialized) {
            matrix::Vector3f acc_n = accel1_denoised.normalized();
            if (m_new) {
                matrix::Vector3f mag_n = matrix::Vector3f{m.x, m.y, m.z}.normalized();
                float roll = atan2f(-acc_n(1), -acc_n(2));
                float pitch = asinf(acc_n(0));
                // 水平面磁向量
                float mx = mag_n(0) * cosf(pitch) + mag_n(1) * sinf(roll) * sinf(pitch) + mag_n(2) * cosf(roll) * sinf(pitch);
                float my = mag_n(1) * cosf(roll) - mag_n(2) * sinf(roll);
                float yaw = atan2f(-my, mx);
                _q = matrix::Quatf(matrix::Eulerf(roll, pitch, yaw));
            } else {
                _q = matrix::Quatf(1.f, 0.f, 0.f, 0.f);
            }
            _initialized = true;
        }

        // 陀螺积分
        matrix::Vector3f omega = gyro1_denoised;
        matrix::Quatf dq{1.f, 0.5f * omega(0) * dt, 0.5f * omega(1) * dt, 0.5f * omega(2) * dt};
        _q = (_q * dq).normalized();

        // 加速度校正（重力方向误差）
        matrix::Vector3f g_body = _q.inversed().rotateVector(matrix::Vector3f{0.f, 0.f, -1.f});
        matrix::Vector3f acc_b = matrix::Vector3f{accel1_denoised(0), accel1_denoised(1), accel1_denoised(2)};
        float acc_norm = acc_b.norm();
        if (acc_norm > 1e-3f) { acc_b = acc_b / acc_norm; }
        matrix::Vector3f e_acc = acc_b % g_body;
        omega += _kp_acc * e_acc;

        // 磁力计航向校正（可选）
        if (m_new) {
            matrix::Vector3f mag_n = matrix::Vector3f{m.x, m.y, m.z};
            float mn = mag_n.norm(); if (mn > 1e-6f) { mag_n = mag_n / mn; }
            matrix::Vector3f mag_body = _q.inversed().rotateVector(mag_n);
            mag_body(2) = 0.f;
            if (mag_body.norm() > 1e-3f) {
                matrix::Vector3f ref_x{1.f, 0.f, 0.f};
                float hn = mag_body.norm(); matrix::Vector3f mag_h = (hn > 1e-6f) ? (mag_body / hn) : mag_body;
                matrix::Vector3f e_yaw = mag_h % ref_x;
                omega += _kp_mag * e_yaw;
            }
        }

        // 应用校正后的陀螺再积分
        matrix::Quatf dq2{1.f, 0.5f * omega(0) * dt, 0.5f * omega(1) * dt, 0.5f * omega(2) * dt};
        _q = (_q * dq2).normalized();

        vehicle_attitude_s att{};
        att.timestamp = ts;
        att.timestamp_sample = ts_g1;
        att.q[0] = _q(0); att.q[1] = _q(1); att.q[2] = _q(2); att.q[3] = _q(3);
        att.delta_q_reset[0] = 0.f; att.delta_q_reset[1] = 0.f; att.delta_q_reset[2] = 0.f; att.delta_q_reset[3] = 0.f;
        att.quat_reset_counter = 0;
        _att_pub.publish(att);
    }

    uORB::Subscription _accel_sub1{ORB_ID(sensor_accel), 0};
    uORB::Subscription _accel_sub2{ORB_ID(sensor_accel), 1};
    uORB::Subscription _gyro_sub1{ORB_ID(sensor_gyro), 0};
    uORB::Subscription _gyro_sub2{ORB_ID(sensor_gyro), 1};
    uORB::Subscription _mag_sub{ORB_ID(sensor_mag), 0};
    uORB::Publication<vehicle_attitude_s> _att_pub{ORB_ID(vehicle_attitude)};

    matrix::Vector3f _accel1_filt{0.f, 0.f, 0.f};
    matrix::Vector3f _accel2_filt{0.f, 0.f, 0.f};
    matrix::Vector3f _gyro1_filt{0.f, 0.f, 0.f};
    matrix::Vector3f _gyro2_filt{0.f, 0.f, 0.f};
    float _lpf_alpha{0.2f};
    float _noise_gain{1.0f};
    float _kp_acc{0.05f};
    float _kp_mag{0.02f};
    matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
    bool _initialized{false};
    uint64_t _last_ts{0};
};

extern "C" __EXPORT int dual_imu_fusion_main(int argc, char *argv[]) { return DualIMUFusion::main(argc, argv); }
