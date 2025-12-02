#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>

class SensorStub : public px4::ScheduledWorkItem {
public:
    SensorStub() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}
    bool init() {
        _pub_accel0 = new uORB::Publication<sensor_accel_s>(ORB_ID(sensor_accel));
        _pub_accel1 = new uORB::Publication<sensor_accel_s>(ORB_ID(sensor_accel));
        _pub_mag = new uORB::Publication<sensor_mag_s>(ORB_ID(sensor_mag));
        ScheduleOnInterval(5000);
        return _pub_accel0 && _pub_accel1 && _pub_mag;
    }
    void Run() override {
        const uint64_t now = hrt_absolute_time();
        _tick++;
        sensor_accel_s a{};
        a.timestamp = now;
        a.device_id = 0;
        a.accel_x = (_tick % 20) < 10 ? 0.5f : -0.5f;
        a.accel_y = (_tick % 15) < 7 ? 0.3f : -0.3f;
        a.accel_z = 9.81f;
        if (_pub_accel0) { _pub_accel0->publish(a); }
        a.device_id = 1;
        a.accel_x *= 0.8f;
        a.accel_y *= 0.8f;
        if (_pub_accel1) { _pub_accel1->publish(a); }
        if ((_tick % 4) == 0) {
            sensor_mag_s m{};
            m.timestamp = now;
            m.device_id = 0;
            m.x = 0.1f;
            m.y = 0.0f;
            m.z = 0.4f;
            if (_pub_mag) { _pub_mag->publish(m); }
        }
    }
private:
    uORB::Publication<sensor_accel_s> *_pub_accel0{nullptr};
    uORB::Publication<sensor_accel_s> *_pub_accel1{nullptr};
    uORB::Publication<sensor_mag_s> *_pub_mag{nullptr};
    uint32_t _tick{0};
};

extern "C" int sensor_stub_main(int argc, char *argv[]) {
    static SensorStub *g{};
    if (!g) {
        g = new SensorStub();
        if (!g || !g->init()) { PX4_ERR("sensor_stub init fail"); return -1; }
    }
    g->ScheduleNow();
    PX4_INFO("sensor_stub started");
    return 0;
}
